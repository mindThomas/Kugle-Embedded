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
 
#include "PowerManagement.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h" // for processing task
#include "SMBus.h"
#include "Battery.h"
#include "BatteryBoard.h"


#include "Debug.h"

PowerManagement::PowerManagement(IO& enable19V, IO& enable5V, PWM& powerLED, uint32_t powerManagementTaskPriority, LSPC& com_, Timer& microsTimer_) : _powerManagementTaskHandle(0), _enable19V(enable19V), _enable5V(enable5V), _powerLED(powerLED), com(com_), microsTimer(microsTimer_)
{
	_ledMode_pending = LEDMODE_PULSING;
	_PulseValue = 0;
	_PulseDirectionUp = true;

	_powerMode = POWERMODE_OFF;
	Enable(true, false); // start with 19V turned on and 5V off

	osDelay(2000);

	for(uint8_t i = 0;i<nSMBusses;i++)
	{
		smartBatteryBoard[i] = new BatteryBoard(SMBusHardwareResources[i],chargerCurrentLimit/nSMBusses);
		smartBattery[i] = new Battery(SMBusHardwareResources[i], i);
	}

	powerManagmentStaticInfo.batteryAssamblyGettingOld = false;
	powerManagmentStaticInfo.batteryAssamblyTotalCapacity = 0;

	for(int i = 0;i<nSMBusses;i++){
		// static values only requested at initialization!

		batteryStaticInfo[i].serial_number = smartBattery[i]->getSerialNumber();
		batteryStaticInfo[i].capacity = smartBattery[i]->getCapacity();
		batteryStaticInfo[i].design_capacity = smartBattery[i]->getDesign_capacity();
		batteryStaticInfo[i].power_supply_technology = smartBattery[i]->getPowerSupplyTechnology();
		batteryStaticInfo[i].present = smartBattery[i]->checkPresent();
		batteryStaticInfo[i].location = smartBattery[i]->getLocation();

		powerManagmentStaticInfo.batteryAssamblyTotalCapacity = powerManagmentStaticInfo.batteryAssamblyTotalCapacity + batteryStaticInfo[i].capacity;
		powerManagmentStaticInfo.batteryAssamblyGettingOld = (powerManagmentStaticInfo.batteryAssamblyGettingOld || smartBattery[i]->isBatteryGettingOld()); // not part of the standard ros battery msg. thus only included in the powermanagment info msg

	}


	xTaskCreate(PowerManagement::PowerManagementThread, (char *)"Power Management", POWER_MANAGEMENT_THREAD_STACK, (void*) this, powerManagementTaskPriority, &_powerManagementTaskHandle);
}

PowerManagement::~PowerManagement()
{
	if (_powerManagementTaskHandle)
		vTaskDelete(_powerManagementTaskHandle); // stop task

	for(int i = 0;i<nSMBusses;i++)
	{
		if(smartBattery[i]) delete smartBattery[i];
		if(smartBatteryBoard[i]) delete smartBatteryBoard[i];
	}
}

void PowerManagement::Enable(bool enable19V, bool enable5V)
{
	if (enable19V)
		_enable19V.Set(true);
	else
		_enable19V.Set(false);

	if (enable5V)
		_enable5V.Set(true);
	else
		_enable5V.Set(false);
}

void PowerManagement::SetLEDmode(LEDmode_t ledMode)
{
	_ledMode_pending = ledMode;
}

void PowerManagement::SetPowerMode(PowerManagement::PowerMode_t powerMode)
{
	_powerMode = powerMode;
	if (powerMode == POWERMODE_OFF)
		Enable(false, false);
	else if (powerMode == POWERMODE_ALL_ON)
		Enable(true, true);
	else if (powerMode == POWERMODE_5V_ONLY)
		Enable(false, true);
}

PowerManagement::PowerMode_t PowerManagement::GetPowerMode()
{
	return _powerMode;
}

#if 0
void PowerManagement::PowerButtonInterrupt(void * params)
{
	PowerManagement * pm = (PowerManagement *)params;

}

void PowerManagement::ResetButtonInterrupt(void * params)
{
	PowerManagement * pm = (PowerManagement *)params;

}

void PowerManagement::CalibrateButtonInterrupt(void * params)
{
	PowerManagement * pm = (PowerManagement *)params;

}
#endif

void PowerManagement::SendBatteryInfo()
{
	lspc::MessageTypesToPC::powerManagment_info_t powerManagment_msg;
	lspc::MessageTypesToPC::RawSensor_Battery_t battery_msg[nSMBusses];

	for(int i = 0;i<nSMBusses;i++)
	{

		battery_msg[i].current = smartBattery[i]->getCurrent();
		osDelay(5);
		battery_msg[i].voltage = smartBattery[i]->getVoltage();
		osDelay(5);
		battery_msg[i].charge = smartBattery[i]->getCharge();
		osDelay(5);
		battery_msg[i].percentage = smartBattery[i]->getPercentage();
		osDelay(5);

		battery_msg[i].power_supply_status = smartBattery[i]->getPowerSupplyStatus();
		osDelay(5);
		battery_msg[i].power_supply_health = smartBattery[i]->getPowerSupplyHealth();
		osDelay(5);

		battery_msg[i].serial_number = batteryStaticInfo[i].serial_number;
		battery_msg[i].capacity = batteryStaticInfo[i].capacity;
		battery_msg[i].design_capacity = batteryStaticInfo[i].design_capacity;
		battery_msg[i].power_supply_technology = batteryStaticInfo[i].power_supply_technology;
		battery_msg[i].present = batteryStaticInfo[i].present;
		battery_msg[i].location = batteryStaticInfo[i].location;


		smartBatteryBoard[i]->SetCurrentLimitRAM(chargerCurrentLimit/nSMBusses);
	}

	float batteryAssamblyChargePercentage = 0;
	float batteryAssamblyTotalCharge = 0;

	// add check of charge current and correct if it does not fit the charger.
	// consider if some of the above shold only be called once when the platform startsup? e.g. getLocation, getPowerSupplyTechnology, isBatteryGettingOld, serial_number etc
	// add function to change gettingOldCycleCount
	// add ros services to change appropriate values

	for(int i = 0;i<nSMBusses;i++){
		batteryAssamblyChargePercentage = batteryAssamblyChargePercentage + battery_msg[i].percentage;
		batteryAssamblyTotalCharge = powerManagmentStaticInfo.batteryAssamblyTotalCharge + battery_msg[i].charge;
	}
	batteryAssamblyChargePercentage = batteryAssamblyChargePercentage/(float)nSMBusses;

	if(batteryAssamblyChargePercentage<BATTERY_LOW_PERCENTAGE){
		batteryLOW = true;
		Debug::printf("battery low /n");
	}
	else{batteryLOW = false;}

	powerManagment_msg.time = microsTimer.GetTime();
	powerManagment_msg.batteryAssamblyGettingOld = powerManagmentStaticInfo.batteryAssamblyGettingOld;
	powerManagment_msg.batteryAssamblyChargePercentage = batteryAssamblyChargePercentage;
	powerManagment_msg.batteryAssamblyTotalCapacity = powerManagmentStaticInfo.batteryAssamblyTotalCapacity;
	powerManagment_msg.batteryAssamblyTotalCharge = batteryAssamblyTotalCharge;
	powerManagment_msg.nBatteries = nSMBusses;

	// add additonal data processing
	powerManagment_msg.power_supply_health = Battery::POWER_SUPPLY_HEALTH_UNKNOWN; // add some processing to get proper feedback
	powerManagment_msg.power_supply_status = Battery::POWER_SUPPLY_STATUS_UNKNOWN; // add some processing to get proper feedback


	// send msgs
	com.TransmitAsync(lspc::MessageTypesToPC::powerManagment_info, (uint8_t *)&powerManagment_msg, sizeof(powerManagment_msg));

	for(int i = 0;i<nSMBusses;i++)
	{
		com.TransmitAsync(lspc::MessageTypesToPC::RawSensor_Battery, (uint8_t *)&battery_msg[i], sizeof(battery_msg[0]));
	}
}


void PowerManagement::BlinkLED(){

	if (batteryLOW){
		_ledMode = LEDMODE_BLINKING_FAST;
	}
	else{
		_ledMode = _ledMode_pending;
	}

	if (_ledMode == PowerManagement::LEDMODE_PULSING) {
		if (_PulseDirectionUp && _PulseValue < POWER_LED_PWM_RANGE)
			_PulseValue++;
		else if (!_PulseDirectionUp && _PulseValue > 0)
			_PulseValue--;
		else
			_PulseDirectionUp = !_PulseDirectionUp;

		_powerLED.SetRaw(_PulseValue);
	}
	else if (_ledMode == PowerManagement::LEDMODE_BLINKING) {
		_PulseValue++;
		if (_PulseValue > POWER_LED_PWM_RANGE)
			_PulseValue = 0;

		if (_PulseValue > POWER_LED_PWM_RANGE/2)
			_powerLED.SetRaw(POWER_LED_PWM_RANGE);
		else
			_powerLED.SetRaw(0);
	}
	else if (_ledMode == PowerManagement::LEDMODE_BLINKING_FAST) {
		_PulseValue++;
		if (_PulseValue > POWER_LED_PWM_RANGE/2)
			_PulseValue = 0;

		if (_PulseValue > POWER_LED_PWM_RANGE/4)
			_powerLED.SetRaw(POWER_LED_PWM_RANGE);
		else
			_powerLED.SetRaw(0);
	}
	else if (_ledMode == PowerManagement::LEDMODE_ON) {
		_powerLED.SetRaw(POWER_LED_PWM_RANGE);
		_PulseValue = POWER_LED_PWM_RANGE;
	}
	else if (_ledMode == PowerManagement::LEDMODE_OFF) {
		_powerLED.SetRaw(0);
		_PulseValue = 0;
	}
}

void PowerManagement::PowerManagementThread(void * pvParameters)
{
	PowerManagement * pm = (PowerManagement *)pvParameters;

	/* Consider to implement this as a high priority 'watchdog task' (referred to as a 'check' task in all the official demos) that monitors how the cycle counters of each task to ensure they are cycling as expected */
	/* This task could possibly also poll for the real time stats - https://www.freertos.org/a00021.html#vTaskGetRunTimeStats */

	uint16_t batteryPollingRateCounter = 0;
	uint16_t batteryPollingRateCounterMax = BATTERY_POLLING_DT / POWER_MANAGMENT_THREAD_DT;

	while (1) {
		/*if (pm->_ledMode != PULSING)
			vTaskSuspend(NULL); // suspend current thread - this could also be replaced by semaphore-based waiting (flagging)
		*/

		// Blinking led
		pm->BlinkLED();

		// Battery Data Polling
		if (batteryPollingRateCounter==batteryPollingRateCounterMax){
			pm->SendBatteryInfo();
			batteryPollingRateCounter = 0;
		}
		batteryPollingRateCounter++;

		// Pause task
		osDelay(POWER_MANAGMENT_THREAD_DT); // ms delay
	}
}
