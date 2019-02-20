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
#include "Debug.h"
#include "COMEKF.h"
#include "MadgwickAHRS.h"
#include "QEKF.h"
#include "VelocityEKF.h"
#include "Parameters.h"
#include "PowerManagement.h"
#include "Joystick.h"

/* Include Application-layer libraries */
#include "BalanceController.h"
#include "Communication.h"
#include "FrontPanel.h"
#include "PathFollowingController.h"

/* Miscellaneous includes */
#include "MATLABCoderInit.h"
#include <stdlib.h>
#include <vector>
#include "ProcessorInit.h"

int32_t encoder1 = 0;
int32_t encoder2 = 0;
int32_t encoder3 = 0;

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

	/* Initialize global parameters */
	Parameters& params = *(new Parameters);

	/* Initialize power management */
	IO * enable19V = new IO(GPIOE, GPIO_PIN_4); // configure as output
	IO * enable5V = new IO(GPIOC, GPIO_PIN_3); // configure as output
	Battery * battery1 = new Battery();
	Battery * battery2 = new Battery();
	PWM * powerLED = new PWM(PWM::TIMER17, PWM::CH1, POWER_LED_PWM_FREQUENCY, POWER_LED_PWM_RANGE);
	PowerManagement * pm = new PowerManagement(*enable19V, *enable5V, *battery1, *battery2, *powerLED, POWER_MANAGEMENT_PRIORITY);

	/* Initialize EEPROM */
	EEPROM * eeprom = new EEPROM;
	eeprom->EnableSection(eeprom->sections.imu_calibration, sizeof(IMU::calibration_t)); // enable IMU calibration section in EEPROM
	eeprom->EnableSection(eeprom->sections.parameters, params.getParameterSizeBytes());
	eeprom->Initialize();
	params.AttachEEPROM(eeprom); // attach EEPROM to load and store parameters into EEPROM

	/* Initialize MATLAB coder globals */
	MATLABCoder_initialize();

	/* Initialize communication */
	USBCDC * usb = new USBCDC(USBCDC_TRANSMITTER_PRIORITY);
	LSPC * lspcUSB = new LSPC(usb, LSPC_RECEIVER_PRIORITY, LSPC_TRANSMITTER_PRIORITY); // very important to use "new", otherwise the object gets placed on the stack which does not have enough memory!
	Debug * dbg = new Debug(lspcUSB); // pair debug module with configured LSPC module to enable "Debug::print" functionality
	params.AttachLSPC(lspcUSB); // attach USB object to allow modification of parameters over USB

	/* Register general (system wide) LSPC callbacks */
	lspcUSB->registerCallback(lspc::MessageTypesFromPC::Reboot, &Reboot_Callback);
	lspcUSB->registerCallback(lspc::MessageTypesFromPC::EnterBootloader, &EnterBootloader_Callback);

	/* Debug boot info */
	Debug::print("Booting...\n");

	/* Initialize front panel periphirals (eg. quadrature knob, LCD and buttons */
	IO * powerButton = new IO(GPIOB, GPIO_PIN_6, IO::PULL_DOWN); // configure as input
	IO * resetButton = new IO(GPIOD, GPIO_PIN_15, IO::PULL_DOWN); // configure as input
	IO * calibrateButton = new IO(GPIOD, GPIO_PIN_14, IO::PULL_DOWN); // configure as input

	/* Prepare Xsens IMU always, since it is used for logging and comparison purposes */
	UART * uart = new UART(UART::PORT_UART3, 460800, 100);
	MTI200 * mti200 = new MTI200(uart);
	if (params.estimator.ConfigureXsensIMUatBoot) {
		if (!mti200->Configure()) { // configuration failed, so do not use/pass on to balance controller
			delete(mti200);
			mti200 = 0;
		}
	}

	/* Initialize and configure IMU */
	IMU * imu = 0;
	if (params.estimator.UseXsensIMU) {
		if (!mti200)
			ERROR("MTI200 selected but not available!");
		imu = mti200; // use Xsens MTI200 in Balance controller
		imu->AttachEEPROM(eeprom);
	}
	else {
		// Prepare and configure MPU9250 IMU
		SPI * spi = new SPI(SPI::PORT_SPI6, MPU9250_Bus::SPI_LOW_FREQUENCY, GPIOG, GPIO_PIN_8);
		MPU9250 * mpu9250 = new MPU9250(spi);
		if (mpu9250->Configure(MPU9250::ACCEL_RANGE_4G, MPU9250::GYRO_RANGE_2000DPS) != 0)
			ERROR("MPU9250 selected but not available!");

		if (params.estimator.EnableSensorLPFfilters) {
			//mpu9250->setFilt(MPU9250::DLPF_BANDWIDTH_92HZ, MPU9250::DLPF_BANDWIDTH_92HZ); // sensor bandwidths should be lower than half the sample rate to avoid aliasing problems
			mpu9250->setFilt(MPU9250::DLPF_BANDWIDTH_92HZ, MPU9250::DLPF_BANDWIDTH_250HZ); // however best results are seen with 250 Hz gyro bandwidth, even though we sample at 200 Hz
		} else {
			mpu9250->setFilt(MPU9250::DLPF_BANDWIDTH_OFF, MPU9250::DLPF_BANDWIDTH_OFF);
			//mpu9250->setFilt(MPU9250::DLPF_BANDWIDTH_184HZ, MPU9250::DLPF_BANDWIDTH_184HZ);
			//mpu9250->setFilt(MPU9250::DLPF_BANDWIDTH_41HZ, MPU9250::DLPF_BANDWIDTH_41HZ);
			//mpu9250->setFilt(MPU9250::DLPF_BANDWIDTH_20HZ, MPU9250::DLPF_BANDWIDTH_20HZ);
			//mpu9250->setFilt(MPU9250::DLPF_BANDWIDTH_OFF, MPU9250::DLPF_BANDWIDTH_184HZ);
		}
		mpu9250->ConfigureInterrupt(GPIOE, GPIO_PIN_3);
		imu = mpu9250; // use MPU9250 in Balance controller
		imu->AttachEEPROM(eeprom);
	}

	/* Initialize microseconds timer */
	Timer * microsTimer = new Timer(Timer::TIMER6, 1000000); // create a 1 MHz counting timer used for micros() timing

	/* Initialize motors */
	ESCON * motor1 = new ESCON(1, params.model.MotorMaxCurrent, params.model.MotorTorqueConstant, params.model.i_gear, params.model.EncoderTicksPrRev, params.model.MotorMaxSpeed);
	ESCON * motor2 = new ESCON(2, params.model.MotorMaxCurrent, params.model.MotorTorqueConstant, params.model.i_gear, params.model.EncoderTicksPrRev, params.model.MotorMaxSpeed);
	ESCON * motor3 = new ESCON(3, params.model.MotorMaxCurrent, params.model.MotorTorqueConstant, params.model.i_gear, params.model.EncoderTicksPrRev, params.model.MotorMaxSpeed);

	/******* APPLICATION LAYERS *******/
	BalanceController * balanceController = new BalanceController(*imu, *motor1, *motor2, *motor3, *lspcUSB, *microsTimer, mti200);
	if (!balanceController) ERROR("Could not initialize balance controller");

	FrontPanel * frontPanel = new FrontPanel(*pm, *balanceController, powerButton, resetButton, calibrateButton);
	if (!frontPanel) ERROR("Could not initialize front panel");


	/* Send CPU load every second */
	char * pcWriteBuffer = (char *)pvPortMalloc(1024);
	while (1)
	{
		vTaskGetRunTimeStats(pcWriteBuffer);
		char * endPtr = &pcWriteBuffer[strlen(pcWriteBuffer)];
		*endPtr++ = '\n'; *endPtr++ = '\n'; *endPtr++ = 0;

		// Split into multiple packages and send
		uint16_t txIdx = 0;
		uint16_t remainingLength = strlen(pcWriteBuffer);
		uint16_t txLength;

		while (remainingLength > 0) {
			txLength = remainingLength;
			if (txLength > LSPC_MAXIMUM_PACKAGE_LENGTH) {
				txLength = LSPC_MAXIMUM_PACKAGE_LENGTH;
				while (pcWriteBuffer[txIdx+txLength] != '\n' && txLength > 0) txLength--; // find and include line-break (if possible)
				if (txLength == 0) txLength = LSPC_MAXIMUM_PACKAGE_LENGTH;
				else txLength++;
			}
			lspcUSB->TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)&pcWriteBuffer[txIdx], txLength);

			txIdx += txLength;
			remainingLength -= txLength;
		}
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
