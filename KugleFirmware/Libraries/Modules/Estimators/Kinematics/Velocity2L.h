/* Copyright (C) 2018 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.com
 * e-mail   :  thomasj@tkjelectronics.com
 * ------------------------------------------
 */

#ifndef VELOCITY2L_H
#define VELOCITY2L_H

#include <stddef.h>
#include <stdlib.h>

void _BallTo2Lvelocity(const float vel_ball[2], const float q[4], const float dq[4], float l, float vel_2L[2]);
void _Convert2LtoBallVelocity(const float vel_2L[2], const float q[4], const float dq[4], const float l, float vel_ball[2]);

#endif // VELOCITY2L_H
