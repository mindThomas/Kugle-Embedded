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
 
#ifndef MODULES_POWERMANAGEMENT_H
#define MODULES_POWERMANAGEMENT_H

#include "stm32h7xx_hal.h"
#include "cmsis_os.h" // for processing task

#include "IO.h"
#include "PWM.h"
#include "Battery.h"
#include "Parameters.h"
#include "Timer.h"
#include "SMBus.h"

#include "BatteryBoard.h"

// CONFIGURATIONS
#define POWER_LED_PWM_FREQUENCY 	200
#define POWER_LED_PWM_RANGE			100
#define POWER_MANAGMENT_THREAD_DT	10 		// ms
#define BATTERY_POLLING_DT			1000  	// ms  choose something that adds up to POWER_MANAGMENT_THREAD_DT


class PowerManagement
{
	public:
		bool batteryAssamblyGettingOld;

		typedef enum LEDmode_t {
			LEDMODE_ON,
			LEDMODE_OFF,
			LEDMODE_PULSING,
			LEDMODE_BLINKING,
			LEDMODE_BLINKING_FAST,
		} LEDmode_t;

		typedef enum PowerMode_t {
			POWERMODE_OFF = 0,
			POWERMODE_ALL_ON = 1,
			POWERMODE_5V_ONLY = 2
		} PowerMode_t;

	private:
		// constants that can be changed through rosservice!
        uint16_t chargerCurrentLimit = 7800;	// mA - actually 8000mA however set a bit lower to be safe!
		float BATTERY_LOW_PERCENTAGE = 0.05;	// a number between 0 and 1!!!

		IO& _enable19V; // Devices on 19V bus: LiDAR, PC (Intel NUC or Jetson) and Motors
		IO& _enable5V;  // Devices on 5V bus: Xsens IMU, USB Hub
		PWM& _powerLED;
		LSPC& com;
		Timer& microsTimer;

		PowerMode_t _powerMode;

		uint8_t _PulseValue;
		bool _PulseDirectionUp;
		LEDmode_t _ledMode;
		LEDmode_t _ledMode_pending;
		bool batteryLOW = false;

		// task params
		const uint32_t POWER_MANAGEMENT_THREAD_STACK = 256;
		TaskHandle_t _powerManagementTaskHandle;

		// SMBus constants
		// According to the SMBus v 1.1. standard (which the current batteries complies to)it is required
		// to have either a Smart Battery Selector and/or the Smart Battery System Manager to have multiple
		// batteries on the same bus. Since such devices was not available on the system at the time of
		// writing this code 2 busses was used for the duel battery system.
		static const uint8_t nSMBusses = 2;
		SMBus::port_t SMBusHardwareResources[nSMBusses] = {SMBus::PORT_I2C2, SMBus::PORT_I2C4};
		Battery * smartBattery[nSMBusses];
		BatteryBoard * smartBatteryBoard[nSMBusses];

		lspc::MessageTypesToPC::powerManagment_info_t powerManagmentStaticInfo;
		lspc::MessageTypesToPC::RawSensor_Battery_t batteryStaticInfo[nSMBusses];

	public:
		PowerManagement(IO& enable19V, IO& enable5V, PWM& powerLED, uint32_t powerManagementTaskPriority, LSPC& com_, Timer& microsTimer_);
		~PowerManagement();

		void SetPowerMode(PowerMode_t powerMode);
		PowerMode_t GetPowerMode();
		void SetLEDmode(LEDmode_t ledMode);


	private:
		static void PowerButtonInterrupt(void * params);
		static void ResetButtonInterrupt(void * params);
		static void CalibrateButtonInterrupt(void * params);
		static void PowerManagementThread(void * pvParameters);
		void SendBatteryInfo();
		void BlinkLED();

		void Enable(bool enable19V, bool enable5V);
		void ButtonHandler();
};
	
	
#endif
