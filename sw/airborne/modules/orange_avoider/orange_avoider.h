/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#ifndef ORANGE_AVOIDER_H
#define ORANGE_AVOIDER_H

extern float oa_orange_obstacle_threshold;
extern float oa_green_floor_threshold;
extern float oa_green_plant_threshold;
extern float oa_gate_heading_gain;
extern float oa_gate_approach_speed;

extern void orange_avoider_init(void);
extern void orange_avoider_periodic(void);

#endif

