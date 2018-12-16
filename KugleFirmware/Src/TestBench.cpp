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
 
#include "main.h"
#include "cmsis_os.h"

#include "UART.h"
#include <string>
#include <stdlib.h>


void TestBench(void const * argument);
osThreadId testBenchTaskHandle;
UART * COM;

void TestBench_Init()
{
	/* Create Test bench thread */
	osThreadDef(testBenchTask, TestBench, osPriorityNormal, 0, 128);
	testBenchTaskHandle = osThreadCreate(osThread(testBenchTask), NULL);
}

void UART_Callback(uint8_t * buffer, uint32_t bufLen)
{
	COM->TransmitBlocking(buffer, bufLen);
}

void TestBench(void const * argument)
{
	COM = new UART(UART::PORT_UART3, 115200, 100);
	std::string testString("This is a test");
	COM->RegisterRXcallback(UART_Callback, 10);

	while (1) {
		osDelay(1000);
		COM->TransmitBlocking(reinterpret_cast<uint8_t *>(const_cast<char *>(testString.c_str())), testString.length());
	}
}
