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
#include "FreeRTOS.h"

#include <string>
#include <stdlib.h>

#include "UART.h"
#include "SPI.h"
#include "I2C.h"
#include "PWM.h"
#include "Encoder.h"
#include "Timer.h"
#include "IO.h"


void TestBench(void const * argument);
osThreadId testBenchTaskHandle;
UART * uart;
SPI * spi;
I2C * i2c;
PWM * pwm;
Encoder * encoder;
Timer * timer;
Timer * timer2;
IO * pin;

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

int32_t encoderValue;
uint32_t timerValue1, timerValue2, timerValue3;
bool oldState = false;

bool level1, level2, level3;

void TestBench(void const * argument)
{
	uart = new UART(UART::PORT_UART3, 115200, 100);
	spi = new SPI(SPI::PORT_SPI6, 500000);
	i2c = new I2C(I2C::PORT_I2C1, 0x68);
	pwm = new PWM(PWM::TIMER1, PWM::CH1, 1, 50000);
	encoder = new Encoder(Encoder::TIMER2);
	timer = new Timer(Timer::TIMER6, 10000);
	timer2 = new Timer(Timer::TIMER7, 10000);
	pin = new IO(GPIOA, GPIO_PIN_4, true);

	pwm->Set(5000);

	uint8_t buffer[10];

	std::string testString("This is a test");
	uart->RegisterRXcallback(UART_Callback, 10);

	SemaphoreHandle_t semaphore;
	//semaphore = xSemaphoreCreateCounting(100, 0);
	semaphore = xSemaphoreCreateBinary();
	//timer->RegisterInterrupt(10, semaphore);
	pin->RegisterInterrupt(IO::TRIGGER_BOTH, semaphore);

	while (1) {
		/*uart->TransmitBlocking(reinterpret_cast<uint8_t *>(const_cast<char *>(testString.c_str())), testString.length());
		osDelay(500);
		spi->Write(0x19, 0xAB);
		osDelay(500);
		spi->Read(0x73 | 0x80, buffer, 4);
		//spi->Read(0x75 | 0x80);
		osDelay(500);*/
		/*i2c->Write(0x19, 0xAB);
		pwm->Set(250);
		osDelay(500);
		i2c->Read(0x73, buffer, 4);
		pwm->Set(750);
		osDelay(500);*/
		encoderValue = encoder->Get();


		timer->Reset();
		timerValue1 = timer2->Get();
		level1 = pin->Read();
		xSemaphoreTake( semaphore, ( TickType_t ) portMAX_DELAY );
		timerValue2 = timer2->Get();
		level2 = pin->Read();
		//timerValue1 = xTaskGetTickCount();
		//timerValue1 = timer2->Get();
		//osDelay(1000);
		xSemaphoreTake( semaphore, ( TickType_t ) portMAX_DELAY );
		timerValue3 = timer2->Get();
		level3 = pin->Read();
		//timerValue2 = xTaskGetTickCount();
		//timerValue2 = timer2->Get();

		//osDelay(100);

		/*if (pin->Read() != oldState) {
			oldState = pin->Read();

			if (oldState == false)
				timerValue1 = (uint32_t)TIM1->CNT;

			osDelay(1);
		}*/
	}
}
