/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#ifndef VELOCITYCOR_H
#define VELOCITYCOR_H

#include <stddef.h>
#include <stdlib.h>

void _BallToCoRvelocity(const float vel_ball[2], const float q[4], const float dq[4], float CoR, float vel_CoR[2]);
void _ConvertCoRtoBallVelocity(const float vel_CoR[2], const float q[4], const float dq[4], const float CoR, float vel_ball[2]);

#endif // VELOCITYCOR_H
