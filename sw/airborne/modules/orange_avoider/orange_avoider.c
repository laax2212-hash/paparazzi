#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE
#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// Helper function prototypes
static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t { SAFE, OBSTACLE_FOUND, SEARCH_FOR_SAFE_HEADING, OUT_OF_BOUNDS };

// Data structure to store specific color detections
struct ColorObj {
  int16_t x, y, w, h;
  int32_t count;
};

// Global variables
static enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
static struct ColorObj orange = {0}, green = {0};
static float heading_increment = 5.f;
static int16_t obstacle_free_confidence = 0;
const int16_t max_trajectory_confidence = 5;
// Add this line back in:
float oa_color_count_frac = 0.18f;
// ABI Bindings
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
#ifndef GREEN_AVOIDER_VISUAL_DETECTION_ID
#define GREEN_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

static abi_event orange_ev, green_ev;

static void orange_detection_cb(uint8_t sender_id, int16_t x, int16_t y, int16_t w, int16_t h, int32_t count, int16_t extra) {
  orange = (struct ColorObj){x, y, w, h, count};
}

static void green_detection_cb(uint8_t sender_id, int16_t x, int16_t y, int16_t w, int16_t h, int32_t count, int16_t extra) {
  green = (struct ColorObj){x, y, w, h, count};
}

void orange_avoider_init(void) {
  srand(time(NULL));
  chooseRandomIncrementAvoidance();
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &orange_ev, orange_detection_cb);
  AbiBindMsgVISUAL_DETECTION(GREEN_AVOIDER_VISUAL_DETECTION_ID, &green_ev, green_detection_cb);
  navigation_state = SAFE; 
  obstacle_free_confidence = max_trajectory_confidence;
}

void orange_avoider_periodic(void) {
  if(!autopilot_in_flight()) return;

  int32_t iw = front_camera.output_size.w;
  int32_t ih = front_camera.output_size.h;

  // 1. Noise Filter: Let's be even more lenient (3% of screen)
  int32_t min_px = (iw * ih) * 0.03f;
  bool see_orange = (orange.count > min_px);
  bool see_green = (green.count > min_px);

  // 2. Logic A: Obstacle Near (Reliable)
  bool obstacle_near = false;
  if (see_orange) {
    // 1. Widest possible horizontal check (Ignore coordinates, just check if it's there)
    bool centric = (orange.x > iw * 0.15f && orange.x < iw * 0.85f);
    
  
    
    // 3. Size check (If orange takes up more than 8% of the screen, it's close)
    bool large_enough = (orange.count > (iw * ih * 0.08f));

    if (centric && large_enough) {
      VERBOSE_PRINT("DANGER: Orange detected! x=%d, count=%d\n", orange.x, orange.count);
      obstacle_near = true;
    }
  }

 // 3. Logic B: Floor Loss (SIMPLIFIED)
  // We remove the (y + h/2) check because it's too sensitive to drone pitch.
  // Now, we only care if the green count drops significantly.
  bool floor_lost = !see_green; 

  // 4. Update Confidence (Refined for Instant Reaction)
  if (obstacle_near) {
    // INSTANT REACTION: If we see orange, confidence is gone immediately.
    obstacle_free_confidence = 0; 
  } else if (floor_lost) {
    // SLOW DRAIN: If we lose the floor, we still wait for confidence to hit 0.
    if (obstacle_free_confidence > 0) {
      obstacle_free_confidence--; 
    }
  } else {
    // ALL CLEAR: Increase confidence when everything looks good.
    obstacle_free_confidence++;
  }
  
  Bound(obstacle_free_confidence, 0, 20);

  // 5. State Machine Logic
  switch (navigation_state) {
    case SAFE:
      // Only transition to OUT_OF_BOUNDS if confidence is ZERO
      // This means the drone must lose the floor for 20 periodic cycles
      if (floor_lost && obstacle_free_confidence == 0) {
        VERBOSE_PRINT("Transition: SAFE -> OUT_OF_BOUNDS (Low Green Count)\n");
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_near) {
        VERBOSE_PRINT("Transition: SAFE -> OBSTACLE_FOUND\n");
        navigation_state = OBSTACLE_FOUND;
      } else {
        // Reduced step size for smoother simulation movement
        moveWaypointForward(WP_GOAL, 0.3f); 
      }
      break;

    case OBSTACLE_FOUND:
      waypoint_move_here_2d(WP_GOAL);
      heading_increment = (orange.x > iw/2) ? -10.f : 10.f; 
      navigation_state = SEARCH_FOR_SAFE_HEADING;
      break;

    case SEARCH_FOR_SAFE_HEADING:
      increase_nav_heading(heading_increment);
      // RELAXED: Only need a confidence of 5 to stop spinning
      if (obstacle_free_confidence >= 5 && !obstacle_near) {
        VERBOSE_PRINT("Transition: SEARCH -> SAFE\n");
        navigation_state = SAFE;
      }
      break;

    case OUT_OF_BOUNDS:
      increase_nav_heading(60.0f);//Slower turn for better stability
      if (see_green && !floor_lost) {
        VERBOSE_PRINT("Transition: OUT_OF_BOUNDS -> SAFE\n");
        navigation_state = SAFE;
      }
      break;
  }
}

/* Original TU Delft Helper Functions */
uint8_t increase_nav_heading(float incrementDegrees) {
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);
  FLOAT_ANGLE_NORMALIZE(new_heading);
  nav.heading = new_heading;
  return false;
}

uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters) {
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters) {
  float heading = stateGetNedToBodyEulers_f()->psi;
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  return false;
}

uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor) {
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

uint8_t chooseRandomIncrementAvoidance(void) {
  heading_increment = (rand() % 2 == 0) ? 7.f : -7.f;
  return false;
}