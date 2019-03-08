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

#include "MainTask.h"
#include "cmsis_os.h"
#include "Priorities.h"
#include "ProcessorInit.h"

/* Include Periphiral drivers */
#include "ADC.h"
#include "EEPROM.h"
#include "Encoder.h"
#include "I2C.h"
#include "InputCapture.h"
#include "IO.h"
#include "PWM.h"
#include "SMBus.h"
#include "SPI.h"
#include "Timer.h"
#include "UART.h"
#include "USBCDC.h"
#include "Watchdog.h"

/* Include Device drivers */
#include "Battery.h"
#include "ESCON.h"
#include "IMU.h"
#include "LCD.h"
#include "LSPC.hpp"
#include "MPU9250.h"
#include "MTI200.h"
#include "QuadratureKnob.h"

/* Include Module libraries */
#include "LQR.h"
#include "SlidingMode.h"
#include "MPC.h"
#include "Debug.h"
#include "COMEKF.h"
#include "MadgwickAHRS.h"
#include "QEKF.h"
#include "VelocityEKF.h"
#include "Parameters.h"
#include "PowerManagement.h"
#include "FrontPanel.h"
#include "Joystick.h"

/* Include Application-layer libraries */
#include "BalanceController.h"
#include "Communication.h"
#include "HealthMonitor.h"
#include "PathFollowingController.h"

/* Miscellaneous includes */
#include "MATLABCoderInit.h"
#include <stdlib.h>
#include <vector>
#include "ProcessorInit.h"

void MainTask(void * pvParameters)
{
	/* Use this task to:
	 * - Create objects for each module
	 *     (OBS! It is very important that objects created with "new"
	 *      only happens within a thread due to the usage of the FreeRTOS managed heap)
	 * - Link any modules together if necessary
	 * - Create message exchange queues and/or semaphore
	 * - (Create) and start threads related to modules
	 *
	 * Basically anything related to starting the system should happen in this thread and NOT in the main() function !!!
	 */

	/* Initialize EEPROM */
	EEPROM * eeprom = new EEPROM;

	/* Initialize MATLAB coder globals */
	MATLABCoder_initialize();

	/* Initialize power management */
	PowerManagement * pm = new PowerManagement(POWER_MANAGEMENT_PRIORITY);
	pm->Enable(true, true); // enable 19V and 5V power

	/* Initialize communication */
	USBCDC * usb = new USBCDC(USBCDC_TRANSMITTER_PRIORITY);
	LSPC * lspcUSB = new LSPC(usb, LSPC_RECEIVER_PRIORITY, LSPC_TRANSMITTER_PRIORITY); // very important to use "new", otherwise the object gets placed on the stack which does not have enough memory!
	Debug * dbg = new Debug(lspcUSB); // pair debug module with configured LSPC module to enable "Debug::print" functionality

	/* Register general (system wide) LSPC callbacks */
	lspcUSB->registerCallback(lspc::MessageTypesFromPC::Reboot, &Reboot_Callback);
	lspcUSB->registerCallback(lspc::MessageTypesFromPC::EnterBootloader, &EnterBootloader_Callback);

	/* Initialize global parameters */
	Parameters& params = *(new Parameters(eeprom, lspcUSB));

	/* Initialize and configure IMU */
	SPI * spi = new SPI(SPI::PORT_SPI6, MPU9250_Bus::SPI_LOW_FREQUENCY, GPIOG, GPIO_PIN_8);
	MPU9250 * imu = new MPU9250(spi);
	imu->AttachEEPROM(eeprom);
	imu->Configure(MPU9250::ACCEL_RANGE_2G, MPU9250::GYRO_RANGE_250DPS);
	if (params.estimator.EnableSensorLPFfilters) {
		//imu->setFilt(MPU9250::DLPF_BANDWIDTH_92HZ, MPU9250::DLPF_BANDWIDTH_92HZ); // sensor bandwidths should be lower than half the sample rate to avoid aliasing problems
		imu->setFilt(MPU9250::DLPF_BANDWIDTH_92HZ, MPU9250::DLPF_BANDWIDTH_250HZ); // however best results are seen with 250 Hz gyro bandwidth, even though we sample at 200 Hz
	} else {
		imu->setFilt(MPU9250::DLPF_BANDWIDTH_OFF, MPU9250::DLPF_BANDWIDTH_OFF);
		//imu->setFilt(MPU9250::DLPF_BANDWIDTH_184HZ, MPU9250::DLPF_BANDWIDTH_184HZ);
		//imu->setFilt(MPU9250::DLPF_BANDWIDTH_41HZ, MPU9250::DLPF_BANDWIDTH_41HZ);
		//imu->setFilt(MPU9250::DLPF_BANDWIDTH_20HZ, MPU9250::DLPF_BANDWIDTH_20HZ);
		//imu->setFilt(MPU9250::DLPF_BANDWIDTH_OFF, MPU9250::DLPF_BANDWIDTH_184HZ);
	}
	imu->ConfigureInterrupt(GPIOE, GPIO_PIN_3);

	/* Initialize microseconds timer */
	Timer * microsTimer = new Timer(Timer::TIMER6, 1000000);

	/* Initialize MPC */
	MPC::MPC * mpc = new MPC::MPC;
	/* Test MPC */
	double desired_position[2] = {0, 0};
	mpc->setXYreferencePosition(desired_position);
	double position[2] = {0.1, 0.1};
	double velocity[2] = {0, 0};
	double quaternion[4] = {1,0,0,0};
	mpc->setCurrentState(position, velocity, quaternion);
	mpc->Step();


	/* Initialize motors */
	ESCON * motor1 = new ESCON(1);
	ESCON * motor2 = new ESCON(2);
	ESCON * motor3 = new ESCON(3);

	motor1->Enable();
	motor2->Enable();
	motor3->Enable();
	while (1)
	{
		motor1->SetTorque(0.15);
		motor2->SetTorque(0.15);
		motor3->SetTorque(0.15);
		osDelay(2000);
		motor1->SetTorque(-0.15);
		motor2->SetTorque(-0.15);
		motor3->SetTorque(-0.15);
		osDelay(2000);
	}


	/* Test info */
	Debug::print("Booting...\n");

	/******* APPLICATION LAYERS *******/
	/*BalanceController * balanceController = new BalanceController(*imu, *motor1, *motor2, *motor3, *lspcUSB, *microsTimer);
	if (!balanceController) ERROR("Could not initialize balance controller");*/

	/* Send CPU load every second */
	char * pcWriteBuffer = (char *)pvPortMalloc(1024);
	while (1)
	{
		vTaskGetRunTimeStats(pcWriteBuffer);
		char * endPtr = &pcWriteBuffer[strlen(pcWriteBuffer)];
		*endPtr++ = '\n'; *endPtr++ = '\n'; *endPtr++ = 0;
		lspcUSB->TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)pcWriteBuffer, strlen(pcWriteBuffer));
		osDelay(1000);
	}

	/*while (1)
	{
		vTaskSuspend(NULL); // suspend this task
	}*/
}

void Reboot_Callback(void * param, const std::vector<uint8_t>& payload)
{
	// ToDo: Need to check for magic key
	NVIC_SystemReset();
}

void EnterBootloader_Callback(void * param, const std::vector<uint8_t>& payload)
{
	// ToDo: Need to check for magic key
	USBD_Stop(&USBCDC::hUsbDeviceFS);
	USBD_DeInit(&USBCDC::hUsbDeviceFS);
	Enter_DFU_Bootloader();
}
