#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string, ...) fprintf(stderr, "[orange_avoider->%s()] " string, __FUNCTION__, ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};

float oa_orange_obstacle_threshold = 0.15f;
float oa_green_floor_threshold     = 0.15f;
float oa_green_plant_threshold     = 0.10f;

enum navigation_state_t navigation_state = SAFE;
float heading_increment = 5.f;
float maxDistance = 2.25f;
int16_t obstacle_free_confidence = 0;

const int16_t max_trajectory_confidence = 5;

static int32_t orange_lower_count = 0;
static int32_t green_lower_count  = 0;
static int32_t green_upper_count  = 0;

#ifndef ORANGE_LOWER_VISUAL_DETECTION_ID
#define ORANGE_LOWER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

#ifndef GREEN_LOWER_VISUAL_DETECTION_ID
#define GREEN_LOWER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

#ifndef GREEN_UPPER_VISUAL_DETECTION_ID
#define GREEN_UPPER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

static abi_event orange_lower_detection_ev;
static abi_event green_lower_detection_ev;
static abi_event green_upper_detection_ev;

static void orange_lower_detection_cb(uint8_t __attribute__((unused)) sender_id,
                                      int16_t __attribute__((unused)) pixel_x,
                                      int16_t __attribute__((unused)) pixel_y,
                                      int16_t __attribute__((unused)) pixel_width,
                                      int16_t __attribute__((unused)) pixel_height,
                                      int32_t quality,
                                      int16_t __attribute__((unused)) extra)
{
  orange_lower_count = quality;
}

static void green_lower_detection_cb(uint8_t __attribute__((unused)) sender_id,
                                     int16_t __attribute__((unused)) pixel_x,
                                     int16_t __attribute__((unused)) pixel_y,
                                     int16_t __attribute__((unused)) pixel_width,
                                     int16_t __attribute__((unused)) pixel_height,
                                     int32_t quality,
                                     int16_t __attribute__((unused)) extra)
{
  green_lower_count = quality;
}

static void green_upper_detection_cb(uint8_t __attribute__((unused)) sender_id,
                                     int16_t __attribute__((unused)) pixel_x,
                                     int16_t __attribute__((unused)) pixel_y,
                                     int16_t __attribute__((unused)) pixel_width,
                                     int16_t __attribute__((unused)) pixel_height,
                                     int32_t quality,
                                     int16_t __attribute__((unused)) extra)
{
  green_upper_count = quality;
}

static int32_t lower_trap_roi_pixels(int img_w, int img_h)
{
  int col_end      = (int)(0.45f * img_w);
  int margin_start = 60;
  int margin_end   = (int)(0.20f * img_h);

  int b1 = img_h - 2 * margin_start;
  int b2 = img_h - 2 * margin_end;

  if (b1 < 0) { b1 = 0; }
  if (b2 < 0) { b2 = 0; }
  if (col_end < 0) { col_end = 0; }

  return (int32_t)(0.5f * (b1 + b2) * col_end);
}

static int32_t upper_sq_roi_pixels(int img_w, int img_h)
{
  int col_start = (int)(0.60f * img_w);
  int col_end   = img_w;
  int row_start = (int)(0.25f * img_h);
  int row_end   = (int)(0.75f * img_h);

  int col_w = col_end - col_start;
  int row_h = row_end - row_start;

  if (col_w < 0) { col_w = 0; }
  if (row_h < 0) { row_h = 0; }

  return (int32_t)(col_w * row_h);
}

static float compute_ratio(int32_t colour_count, int32_t roi_pixels)
{
  if (roi_pixels <= 0) {
    return 0.0f;
  }

  if (colour_count < 0) {
    colour_count = 0;
  }

  if (colour_count > roi_pixels) {
    colour_count = roi_pixels;
  }

  return (float)colour_count / (float)roi_pixels;
}

void orange_avoider_init(void)
{
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  AbiBindMsgVISUAL_DETECTION(ORANGE_LOWER_VISUAL_DETECTION_ID,
                             &orange_lower_detection_ev,
                             orange_lower_detection_cb);

  AbiBindMsgVISUAL_DETECTION(GREEN_LOWER_VISUAL_DETECTION_ID,
                             &green_lower_detection_ev,
                             green_lower_detection_cb);

  AbiBindMsgVISUAL_DETECTION(GREEN_UPPER_VISUAL_DETECTION_ID,
                             &green_upper_detection_ev,
                             green_upper_detection_cb);
}

void orange_avoider_periodic(void)
{
  if (!autopilot_in_flight()) {
    return;
  }

  int img_w = front_camera.output_size.w;
  int img_h = front_camera.output_size.h;

  int32_t lower_trap_total = lower_trap_roi_pixels(img_w, img_h);
  int32_t upper_sq_total   = upper_sq_roi_pixels(img_w, img_h);

  float orange_ratio      = compute_ratio(orange_lower_count, lower_trap_total);
  float green_lower_ratio = compute_ratio(green_lower_count, lower_trap_total);
  float green_upper_ratio = compute_ratio(green_upper_count, upper_sq_total);

  bool orange_obstacle = orange_ratio > oa_orange_obstacle_threshold;
  bool no_floor        = green_lower_ratio < oa_green_floor_threshold;
  bool plant_obstacle  = green_upper_ratio > oa_green_plant_threshold;

  bool obstacle_detected = orange_obstacle || no_floor || plant_obstacle;

  VERBOSE_PRINT(
    "orange_lower=%ld green_lower=%ld green_upper=%ld | "
    "orange_ratio=%.4f green_lower_ratio=%.4f green_upper_ratio=%.4f | "
    "orange_obs=%d no_floor=%d plant=%d conf=%d state=%d\n",
    (long)orange_lower_count,
    (long)green_lower_count,
    (long)green_upper_count,
    orange_ratio,
    green_lower_ratio,
    green_upper_ratio,
    orange_obstacle,
    no_floor,
    plant_obstacle,
    obstacle_free_confidence,
    navigation_state
  );

  if (!obstacle_detected) {
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;
  }

  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);

  switch (navigation_state) {
    case SAFE:
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);

      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence == 0) {
        navigation_state = OBSTACLE_FOUND;
      } else {
        moveWaypointForward(WP_GOAL, moveDistance);
        moveWaypointForward(WP_RETREAT, -1.0f * moveDistance);
      }
      break;

    case OBSTACLE_FOUND:
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_RETREAT);
      waypoint_move_here_2d(WP_TRAJECTORY);
      chooseRandomIncrementAvoidance();
      navigation_state = SEARCH_FOR_SAFE_HEADING;
      break;

    case SEARCH_FOR_SAFE_HEADING:
      increase_nav_heading(heading_increment);
      if (obstacle_free_confidence >= 2) {
        navigation_state = SAFE;
      }
      break;

    case OUT_OF_BOUNDS:
      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);
      moveWaypointForward(WP_RETREAT, -1.0f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        obstacle_free_confidence = 0;
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
      break;

    default:
      break;
  }
}

uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);
  FLOAT_ANGLE_NORMALIZE(new_heading);
  nav.heading = new_heading;
  VERBOSE_PRINT("Increasing heading to %f deg\n", DegOfRad(new_heading));
  return false;
}

uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading = stateGetNedToBodyEulers_f()->psi;

  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * distanceMeters);
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * distanceMeters);

  VERBOSE_PRINT("Calculated %f m forward. x:%f y:%f heading:%f\n",
                distanceMeters,
                POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y),
                DegOfRad(heading));
  return false;
}

uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n",
                waypoint,
                POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

uint8_t chooseRandomIncrementAvoidance(void)
{
  if (rand() % 2 == 0) {
    heading_increment = 5.f;
  } else {
    heading_increment = -5.f;
  }
  VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  return false;
}