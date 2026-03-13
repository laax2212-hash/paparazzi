/*
 * opticflow_avoider.h
 *
 * Public interface for the opticflow_avoider module.
 */

#ifndef OPTICFLOW_AVOIDER_H
#define OPTICFLOW_AVOIDER_H

#include "std.h"   /* uint8_t, bool, etc. */

/* -----------------------------------------------------------------------
 * Tunable parameters – can be overridden from the airframe XML and are
 * adjustable at run-time via the GCS 'Settings' tab.
 * ----------------------------------------------------------------------- */

/** Divergence level above which an obstacle is declared */
extern float oa_divergence_threshold;

/** How many degrees to turn when an obstacle is found */
extern float oa_turn_heading_deg;

/** Distance (m) to project the GOAL waypoint forward each periodic step */
extern float oa_move_distance;

/** Absolute forward optic-flow threshold [subpixels] to trigger obstacle detection */
extern float oa_flow_y_threshold;

/* -----------------------------------------------------------------------
 * Module entry points called by the autopilot (registered in module XML)
 * ----------------------------------------------------------------------- */

/** Called once at autopilot startup */
extern void opticflow_avoider_init(void);

/** Called periodically at the frequency set in the module XML */
extern void opticflow_avoider_periodic(void);

#endif /* OPTICFLOW_AVOIDER_H */
