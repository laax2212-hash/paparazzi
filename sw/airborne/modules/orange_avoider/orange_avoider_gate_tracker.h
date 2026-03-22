/*
 * Copyright (C) Paparazzi Team
 *
 * This file is part of paparazzi
 *
 */

#ifndef ORANGE_AVOIDER_GATE_TRACKER_H
#define ORANGE_AVOIDER_GATE_TRACKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int orange_avoider_gate_tracker_process(char *img, int width, int height,
                                        int32_t *quality, float *distance_m, float *offset_m);

#ifdef __cplusplus
}
#endif

#endif
