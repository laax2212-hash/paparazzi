/*
 * opticflow_avoider.c
 *
 * Obstacle avoidance module using optical flow divergence from the front camera.
 *
 * Subscribes to the OPTICAL_FLOW ABI message published by cv_opticflow,
 * and uses the size_divergence field to detect obstacles ahead.
 * When an obstacle is detected, it uses the sign of the derotated lateral
 * flow (flow_der_x) to decide which direction to turn.
 *
 * State machine:
 *   SAFE           -> move forward; if divergence > threshold -> OBSTACLE_FOUND
 *   OBSTACLE_FOUND -> choose turn direction -> TURNING
 *   TURNING        -> keep turning; when divergence drops -> SAFE
 */

#include "opticflow_avoider.h"

/* Debug printing - match orange_avoider convention */
#ifndef PRINT
#define PRINT(string, ...) fprintf(stderr, "[opticflow_avoider->%s()] " string, __FUNCTION__, ##__VA_ARGS__)
#endif
#ifndef VERBOSE_PRINT
#define VERBOSE_PRINT(...)
#endif

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Paparazzi headers */
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include "state.h"
#include "modules/core/abi.h"

/* -----------------------------------------------------------------------
 * Tunable parameters
 * ----------------------------------------------------------------------- */

#ifndef OPTICFLOW_AVOIDER_DIVERGENCE_THRESHOLD
#define OPTICFLOW_AVOIDER_DIVERGENCE_THRESHOLD 0.30f
#endif
float oa_divergence_threshold = OPTICFLOW_AVOIDER_DIVERGENCE_THRESHOLD;

#ifndef OPTICFLOW_AVOIDER_TURN_HEADING_DEG
#define OPTICFLOW_AVOIDER_TURN_HEADING_DEG 25.0f
#endif
float oa_turn_heading_deg = OPTICFLOW_AVOIDER_TURN_HEADING_DEG;

#ifndef OPTICFLOW_AVOIDER_MOVE_DISTANCE
#define OPTICFLOW_AVOIDER_MOVE_DISTANCE 1.0f
#endif
float oa_move_distance = OPTICFLOW_AVOIDER_MOVE_DISTANCE;

#ifndef OPTICFLOW_AVOIDER_FLOWY_THRESHOLD
#define OPTICFLOW_AVOIDER_FLOWY_THRESHOLD 45.0f
#endif
float oa_flow_y_threshold = OPTICFLOW_AVOIDER_FLOWY_THRESHOLD;

#ifndef OPTICFLOW_AVOIDER_HIGH_FLOW_THRESHOLD
#define OPTICFLOW_AVOIDER_HIGH_FLOW_THRESHOLD 30.0f
#endif
float oa_high_flow_threshold = OPTICFLOW_AVOIDER_HIGH_FLOW_THRESHOLD;

#define DIVERGENCE_CRITICAL_FACTOR   1.5f
#define FLOW_DER_X_DEADBAND          2.0f
#define DIVERGENCE_OBSTACLE_FACTOR   1.00f
#define DIVERGENCE_CLEAR_FACTOR      0.90f
#define SEARCH_TIMEOUT_CYCLES        24
#define TAKEOFF_GRACE_CYCLES         12
#define DIVERGENCE_MIN_MARGIN        0.04f
#define FLOWY_CLEAR_FACTOR           0.70f
#define FLOWY_CRITICAL_FACTOR        1.80f
#define FLOWY_RISE_MIN               25.0f
#define FLOWY_RISE_DELTA             1.5f
#define FLOWY_RISE_COUNT_TRIGGER     3
#define DEBUG_PRINT_DECIMATION       8
#define MIN_SEARCH_CYCLES            6
#define CLEAR_CYCLES_REQUIRED        3

/* -----------------------------------------------------------------------
 * Internal state
 * ----------------------------------------------------------------------- */

typedef enum {
  OA_SAFE = 0,
  OA_OBSTACLE_FOUND,
  OA_SEARCH_FOR_SAFE_HEADING
} oa_state_t;

static oa_state_t navigation_state = OA_SAFE;
static int8_t     last_turn_sign = 1;
static uint8_t    stale_optflow_cycles = 0;
static int16_t    obstacle_free_confidence = 0;
static const int16_t max_trajectory_confidence = 5;
static uint8_t    search_cycles = 0;
static uint16_t   in_flight_cycles = 0;
static float      div_baseline = 0.0f;
static float      flow_y_abs_lp = 0.0f;
static float      prev_flow_y_abs_lp = 0.0f;
static uint8_t    flow_obstacle_count = 0;
static uint8_t    flow_rise_count = 0;
static uint16_t   dbg_decim = 0;
static uint8_t    clear_cycles = 0;

/* -----------------------------------------------------------------------
 * Shared variables from ABI callback
 * ----------------------------------------------------------------------- */
static float g_div_size   = 0.0f;
static float g_flow_x     = 0.0f;
static float g_flow_y     = 0.0f;
static float g_flow_der_x = 0.0f;
static float g_flow_der_y = 0.0f;
static float g_quality    = 0.0f;
static bool  g_new_data   = false;

/* -----------------------------------------------------------------------
 * ABI subscription
 * ----------------------------------------------------------------------- */
#ifndef OPTICFLOW_AVOIDER_ABI_ID
#define OPTICFLOW_AVOIDER_ABI_ID ABI_BROADCAST
#endif
static abi_event opticflow_ev;

static void opticflow_cb(uint8_t  sender_id    __attribute__((unused)),
                         uint32_t stamp        __attribute__((unused)),
                         int16_t  flow_x,
                         int16_t  flow_y,
                         int16_t  flow_der_x,
                         int16_t  flow_der_y,
                         uint8_t  quality,
                         float    size_divergence)
{
  g_div_size   = size_divergence;
  g_flow_x     = (float)flow_x;
  g_flow_y     = (float)flow_y;
  g_flow_der_x = (float)flow_der_x;
  g_flow_der_y = (float)flow_der_y;
  g_quality    = (float)quality;
  g_new_data   = true;
}

/* -----------------------------------------------------------------------
 * Forward declarations
 * ----------------------------------------------------------------------- */
static uint8_t move_waypoint_forward(uint8_t waypoint, float distance_m);
static uint8_t calculate_forwards(struct EnuCoor_i *new_coor, float distance_m);
static uint8_t increase_nav_heading(float increment_degrees);

/* -----------------------------------------------------------------------
 * Module lifecycle
 * ----------------------------------------------------------------------- */

void opticflow_avoider_init(void)
{
  AbiBindMsgOPTICAL_FLOW(OPTICFLOW_AVOIDER_ABI_ID, &opticflow_ev, opticflow_cb);

  navigation_state      = OA_SAFE;
  last_turn_sign        = 1;
  stale_optflow_cycles  = 0;
  obstacle_free_confidence = 0;
  search_cycles         = 0;
  in_flight_cycles      = 0;
  div_baseline          = 0.0f;
  flow_y_abs_lp         = 0.0f;
  prev_flow_y_abs_lp    = 0.0f;
  flow_obstacle_count   = 0;
  flow_rise_count       = 0;
  dbg_decim             = 0;
  clear_cycles          = 0;
  g_div_size            = 0.0f;
  g_flow_x              = 0.0f;
  g_flow_y              = 0.0f;
  g_flow_der_x          = 0.0f;
  g_flow_der_y          = 0.0f;
  g_quality             = 0.0f;
  g_new_data            = false;

  PRINT("init done. div_threshold=%.2f turn=%.1f deg\n",
        oa_divergence_threshold, oa_turn_heading_deg);
}

void opticflow_avoider_periodic(void)
{
  if (!autopilot_in_flight()) {
    navigation_state = OA_SAFE;
    obstacle_free_confidence = 0;
    search_cycles = 0;
    in_flight_cycles = 0;
    div_baseline = 0.0f;
    flow_y_abs_lp = 0.0f;
    prev_flow_y_abs_lp = 0.0f;
    flow_obstacle_count = 0;
    flow_rise_count = 0;
    dbg_decim = 0;
    clear_cycles = 0;
    return;
  }

  in_flight_cycles++;

  if (!g_new_data) {
    stale_optflow_cycles++;
    if (stale_optflow_cycles >= 4) {
      waypoint_set_xy_i(WP_GOAL, stateGetPositionEnu_i()->x, stateGetPositionEnu_i()->y);
    }
    return;
  }

  stale_optflow_cycles = 0;

  float local_div    = g_div_size;
  float local_flow_x_raw = g_flow_x;
  float local_flow_y_raw = g_flow_y;
  float local_flow_x = g_flow_der_x;
  float local_flow_y = g_flow_der_y;
  float local_quality = g_quality;
  g_new_data = false;

  if (div_baseline == 0.0f) {
    div_baseline = local_div;
  } else {
    div_baseline = 0.95f * div_baseline + 0.05f * local_div;
  }

  float flow_y_abs = fabsf(local_flow_y_raw);
  if (flow_y_abs_lp == 0.0f) {
    flow_y_abs_lp = flow_y_abs;
  } else {
    flow_y_abs_lp = 0.85f * flow_y_abs_lp + 0.15f * flow_y_abs;
  }

  float flow_y_rise = flow_y_abs_lp - prev_flow_y_abs_lp;
  prev_flow_y_abs_lp = flow_y_abs_lp;

  float effective_threshold = fmaxf(oa_divergence_threshold, div_baseline + DIVERGENCE_MIN_MARGIN);
  bool obstacle_by_div = local_div > effective_threshold * DIVERGENCE_OBSTACLE_FACTOR;
  bool obstacle_by_flow = flow_y_abs_lp > oa_flow_y_threshold;
  bool obstacle_by_flow_rise = false;

  if (obstacle_by_flow) {
    if (flow_obstacle_count < 10) {
      flow_obstacle_count++;
    }
  } else if (flow_obstacle_count > 0) {
    flow_obstacle_count--;
  }

  if (flow_y_abs_lp > FLOWY_RISE_MIN && flow_y_rise > FLOWY_RISE_DELTA) {
    if (flow_rise_count < 10) {
      flow_rise_count++;
    }
  } else if (flow_rise_count > 0) {
    flow_rise_count--;
  }
  obstacle_by_flow_rise = flow_rise_count >= FLOWY_RISE_COUNT_TRIGGER;

  if (in_flight_cycles <= TAKEOFF_GRACE_CYCLES) {
    obstacle_free_confidence = max_trajectory_confidence;
    navigation_state = OA_SAFE;
    move_waypoint_forward(WP_GOAL, fminf(oa_move_distance, 0.3f));
    PRINT("TAKEOFF_GRACE div=%.3f base=%.3f thr_eff=%.3f cycle=%u\n",
          local_div, div_baseline, effective_threshold, in_flight_cycles);
    return;
  }

  if (local_div < effective_threshold * DIVERGENCE_CLEAR_FACTOR &&
      flow_y_abs_lp < oa_flow_y_threshold * FLOWY_CLEAR_FACTOR) {
    obstacle_free_confidence++;
  } else if (obstacle_by_div || obstacle_by_flow) {
    obstacle_free_confidence--;
  }

  if (obstacle_free_confidence < 0) {
    obstacle_free_confidence = 0;
  } else if (obstacle_free_confidence > max_trajectory_confidence) {
    obstacle_free_confidence = max_trajectory_confidence;
  }

  VERBOSE_PRINT("state=%d  div=%.3f  flow_x=%.1f  conf=%d\n",
                navigation_state, local_div, local_flow_x, obstacle_free_confidence);

        if ((dbg_decim++ % DEBUG_PRINT_DECIMATION) == 0) {
          PRINT("OF raw=(%.1f,%.1f) der=(%.1f,%.1f) div=%.3f q=%.0f thr=%.3f flowy_lp=%.1f flowy_thr=%.1f flowy_cnt=%u rise=%.1f rise_cnt=%u st=%d conf=%d\n",
                local_flow_x_raw, local_flow_y_raw,
                local_flow_x, local_flow_y,
                local_div, local_quality, effective_threshold, flow_y_abs_lp, oa_flow_y_threshold, flow_obstacle_count,
                flow_y_rise, flow_rise_count,
                navigation_state, obstacle_free_confidence);
        }

  switch (navigation_state) {

  case OA_SAFE:
    search_cycles = 0;

    if ((obstacle_by_flow && flow_obstacle_count >= 2) || obstacle_by_flow_rise ||
        (obstacle_by_div && obstacle_free_confidence == 0)) {
      PRINT("OBSTACLE DETECTED (div=%.3f thr=%.3f flowy_lp=%.1f flowy_thr=%.1f cnt=%u rise=%.1f rise_cnt=%u)\n",
            local_div, effective_threshold, flow_y_abs_lp, oa_flow_y_threshold, flow_obstacle_count,
            flow_y_rise, flow_rise_count);
      navigation_state = OA_OBSTACLE_FOUND;
      break;
    }
    float forward_step = oa_move_distance;
    move_waypoint_forward(WP_GOAL, forward_step);
    break;

  case OA_OBSTACLE_FOUND:
    waypoint_set_xy_i(WP_GOAL, stateGetPositionEnu_i()->x, stateGetPositionEnu_i()->y);
    obstacle_free_confidence = 0;
    search_cycles = 0;
    clear_cycles = 0;

    if (fabsf(local_flow_x) > FLOW_DER_X_DEADBAND) {
      last_turn_sign = (local_flow_x > 0.0f) ? -1 : 1;
    }

    PRINT("Start heading search (%s, step %.1f deg)\n",
          (last_turn_sign < 0) ? "LEFT" : "RIGHT", oa_turn_heading_deg);
    navigation_state = OA_SEARCH_FOR_SAFE_HEADING;
    break;

  case OA_SEARCH_FOR_SAFE_HEADING:
    search_cycles++;

    if (flow_y_abs_lp > oa_high_flow_threshold) {
      increase_nav_heading(last_turn_sign * oa_turn_heading_deg / 2.0f);
    } else {
      increase_nav_heading(last_turn_sign * oa_turn_heading_deg);
    }

    move_waypoint_forward(WP_GOAL, 0.15f * oa_move_distance); // Still move forward slowly to prevent getting stuck

    if (!obstacle_by_flow && !obstacle_by_flow_rise) {
      if (clear_cycles < 20) {
        clear_cycles++;
      }
    } else {
      clear_cycles = 0;
    }

    if (search_cycles >= MIN_SEARCH_CYCLES && clear_cycles >= CLEAR_CYCLES_REQUIRED) {
      PRINT("Path clear, resuming SAFE (search=%u clear=%u)\n", search_cycles, clear_cycles);
      navigation_state = OA_SAFE;
      search_cycles = 0;
      clear_cycles = 0;
      break;
    }

    if (search_cycles >= SEARCH_TIMEOUT_CYCLES) {
      PRINT("Search timeout, forcing SAFE\n");
      navigation_state = OA_SAFE;
      obstacle_free_confidence = 2;
      search_cycles = 0;
      clear_cycles = 0;
    }
    break;

  default:
    navigation_state = OA_SAFE;
    break;
  }
}

/* -----------------------------------------------------------------------
 * Helpers
 * ----------------------------------------------------------------------- */

static uint8_t calculate_forwards(struct EnuCoor_i *new_coor, float distance_m)
{
  float heading = stateGetNedToBodyEulers_f()->psi;

  new_coor->x = stateGetPositionEnu_i()->x +
                POS_BFP_OF_REAL(sinf(heading) * distance_m);
  new_coor->y = stateGetPositionEnu_i()->y +
                POS_BFP_OF_REAL(cosf(heading) * distance_m);
  new_coor->z = stateGetPositionEnu_i()->z;

  VERBOSE_PRINT("Forward target: x=%.2f y=%.2f heading=%.1f deg\n",
                POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y),
                DegOfRad(heading));
  return false;
}

static uint8_t move_waypoint_forward(uint8_t waypoint, float distance_m)
{
  struct EnuCoor_i new_coor;
  calculate_forwards(&new_coor, distance_m);
  waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);
  return false;
}

static uint8_t increase_nav_heading(float increment_degrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(increment_degrees);
  FLOAT_ANGLE_NORMALIZE(new_heading);
  nav.heading = new_heading;
  return false;
}
