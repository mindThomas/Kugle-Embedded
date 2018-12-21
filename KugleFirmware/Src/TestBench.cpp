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
#include "QuadratureKnob.h"
#include "ADC.h"
#include "MPU9250.h"
#include "ESCON.h"
#include "PowerManagement.h"
#include "Parameters.h"
#include "HealthMonitor.h"
#include "VelocityEKF.h"

void TestBench(void const * argument);
osThreadId testBenchTaskHandle;
UART * uart;
I2C * i2c;
PWM * pwm;
Encoder * encoder;
Timer * timer;
Timer * timer2;
IO * pin;
QuadratureKnob * knob;
ADC * adc;

void TestBench_Init()
{
	/* Create Test bench thread */
	osThreadDef(testBenchTask, TestBench, osPriorityRealtime, 0, 128);
	testBenchTaskHandle = osThreadCreate(osThread(testBenchTask), NULL);
}

void UART_Callback(uint8_t * buffer, uint32_t bufLen)
{
	uart->Write(buffer, bufLen);
}

int32_t encoderValue;
uint32_t timerValue1, timerValue2, timerValue3;
bool oldState = false;
uint8_t value;

bool level1, level2, level3;

float ax, ay, az, gx, gy, gz, mx, my, mz;

float angle = 0;

#if 0
void TestBench(void const * argument)
{
	PowerManagement * pm = new PowerManagement(osPriorityNormal);
	ESCON * motor1 = new ESCON(0);
	ESCON * motor2 = new ESCON(1);
	ESCON * motor3 = new ESCON(2);

	pm->Enable(true, true);

	motor1->Enable();
	motor2->Enable();
	motor3->Enable();

	motor1->SetTorque(0.05);
	motor2->SetTorque(0.05);
	motor3->SetTorque(0.05);

	osDelay(1000);

	while (1)
	{
		/*angle = motor1->GetAngle();
		encoderValue = motor1->GetEncoderRaw();*/

		motor1->SetTorque(-0.05);
		motor2->SetTorque(-0.05);
		motor3->SetTorque(-0.05);
		osDelay(1000);
		motor1->SetTorque(0.0);
		motor2->SetTorque(0.0);
		motor3->SetTorque(0.0);
		osDelay(1000);
		motor1->SetTorque(0.05);
		motor2->SetTorque(0.05);
		motor3->SetTorque(0.05);
		osDelay(1000);
		motor1->SetTorque(0.0);
		motor2->SetTorque(0.0);
		motor3->SetTorque(0.0);
		osDelay(1000);
	}
}
#endif

#if 0
void TestBench(void const * argument)
{
	SPI * spi = new SPI(SPI::PORT_SPI6, MPU9250_SPI_LOW_FREQUENCY, GPIOG, GPIO_PIN_8);
	MPU9250<SPI,MPU9250_SPI> * imu = new MPU9250<SPI,MPU9250_SPI>(spi);

	imu->Configure(ACCEL_RANGE_2G, GYRO_RANGE_250DPS);
	imu->setFilt(DLPF_BANDWIDTH_250HZ, DLPF_BANDWIDTH_184HZ, 8);
	imu->ConfigureInterrupt(GPIOE, GPIO_PIN_3);

	while (1) {
		imu->WaitForNewData();
		imu->WaitForNewData();
		imu->WaitForNewData();
		imu->WaitForNewData();
	    imu->getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
	    osDelay(100);
	}
}
#endif

#if 0
void TestBench(void const * argument)
{
	uart = new UART(UART::PORT_UART3, 115200, 100);
	spi = new SPI(SPI::PORT_SPI6, 500000);
	i2c = new I2C(I2C::PORT_I2C1, 0x68);
	imu = new MPU9250<I2C,I2C::port_t>(i2c);

	pwm = new PWM(PWM::TIMER1, PWM::CH1, 1, 50000);
	encoder = new Encoder(Encoder::TIMER2);
	timer = new Timer(Timer::TIMER6, 10000);
	timer2 = new Timer(Timer::TIMER7, 10000);
	pin = new IO(GPIOA, GPIO_PIN_4, true);
	//knob = new QuadratureKnob(GPIOA, GPIO_PIN_0, GPIOA, GPIO_PIN_1);
	adc = new ADC(ADC::ADC_1, ADC_CHANNEL_8);

	pwm->Set(5000);

	uint8_t buffer[10];

	std::string testString("This is a test");
	uart->RegisterRXcallback(UART_Callback, 10);

	SemaphoreHandle_t semaphore;
	//semaphore = xSemaphoreCreateCounting(100, 0);
	semaphore = xSemaphoreCreateBinary();
	vQueueAddToRegistry(semaphore, "Test semaphore");
	//timer->RegisterInterrupt(10, semaphore);
	//pin->RegisterInterrupt(IO::TRIGGER_BOTH, semaphore);

	uint16_t adcValue;

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
		adcValue = adc->Read();


		/*timer->Reset();
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
		//timerValue2 = timer2->Get();*/

		//osDelay(100);

		/*if (pin->Read() != oldState) {
			oldState = pin->Read();

			if (oldState == false)
				timerValue1 = (uint32_t)TIM1->CNT;

			osDelay(1);
		}*/

		osDelay(100);
	}
}
#endif

#if 1
void TestBench(void const * argument)
{
	Parameters& params = Parameters::Get();
	HealthMonitor * hm = new HealthMonitor();
	VelocityEKF * vEKF = new VelocityEKF(params);

	while (1) {
		params.test.var2 += 2;
		osDelay(100);
	}
}
#endif
