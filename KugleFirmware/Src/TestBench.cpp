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
#include "SPI.h"
#include "I2C.h"
#include <string>
#include <stdlib.h>


void TestBench(void const * argument);
osThreadId testBenchTaskHandle;
UART * uart;
SPI * spi;
I2C * i2c;

void TestBench_Init()
{
	/* Create Test bench thread */
	osThreadDef(testBenchTask, TestBench, osPriorityNormal, 0, 128);
	testBenchTaskHandle = osThreadCreate(osThread(testBenchTask), NULL);
}

void UART_Callback(uint8_t * buffer, uint32_t bufLen)
{
	uart->TransmitBlocking(buffer, bufLen);
}

void TestBench(void const * argument)
{
	uart = new UART(UART::PORT_UART3, 115200, 100);
	spi = new SPI(SPI::PORT_SPI6, 500000);
	i2c = new I2C(I2C::PORT_I2C1, 0x68);

	uint8_t buffer[10];

	std::string testString("This is a test");
	uart->RegisterRXcallback(UART_Callback, 10);

	while (1) {
		/*uart->TransmitBlocking(reinterpret_cast<uint8_t *>(const_cast<char *>(testString.c_str())), testString.length());
		osDelay(500);
		spi->write(0x19, 0xAB);
		osDelay(500);
		spi->read(0x73 | 0x80, buffer, 4);
		//spi->read(0x75 | 0x80);
		osDelay(500);*/
		i2c->write(0x19, 0xAB);
		osDelay(500);
		i2c->read(0x73, buffer, 4);
		osDelay(500);
	}
}
