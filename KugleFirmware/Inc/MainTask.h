/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
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
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */
 

#ifndef MAIN_TASK_H
#define MAIN_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void MainTask(void * pvParameters);

#ifdef __cplusplus
}

#include "LSPC.hpp"
void Reboot_Callback(void * param, const std::vector<uint8_t>& payload);
void EnterBootloader_Callback(void * param, const std::vector<uint8_t>& payload);

#endif

#endif 
