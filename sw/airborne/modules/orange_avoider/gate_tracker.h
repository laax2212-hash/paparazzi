/*
 * Gate Tracker - Blue banner detection and PnP distance estimation
 *
 * Detects a gate composed of two horizontal blue banners stacked vertically.
 * Uses OpenCV solvePnP to estimate distance and horizontal offset to the gate.
 */

#ifndef GATE_TRACKER_H
#define GATE_TRACKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * Process a YUYV image to detect a gate.
 *
 * @param img        Pointer to YUYV422 image buffer
 * @param width      Image width in pixels
 * @param height     Image height in pixels
 * @param quality    Output: number of blue mask pixels (0 = nothing blue found)
 * @param distance_m Output: estimated forward distance to gate [m]
 * @param offset_m   Output: estimated horizontal offset to gate center [m] (positive = right)
 * @return 1 if a gate pair was detected and PnP solved, 0 otherwise
 */
int gate_tracker_process(char *img, int width, int height,
                         int32_t *quality, float *distance_m, float *offset_m);

#ifdef __cplusplus
}
#endif

#endif
