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

#include "BatteryBoard.h"
#include "SMBus.h"
#include "Debug.h"

BatteryBoard::BatteryBoard(SMBus::port_t SMBusHardwareResources, uint16_t _chargerCurrentLimit) : chargerCurrentLimit(_chargerCurrentLimit), batteryBoardRamTaskHandle(0), batteryBoardEepromTaskHandle(0)
{

	// the device can only be accessed when the charger is plugged in!
	smbus = new SMBus(SMBusHardwareResources, DeviceAddr);

	SetCurrentLimitRAM(chargerCurrentLimit); // set the input limit of the batteryboard to whatever the charger can deliver!
	//SetCurrentLimitEEPROM(chargerCurrentLimit); // set the input limit of the batteryboard to whatever the charger can deliver!

	// We do this in RAM for 2 reasons!
	// 1) the EEPROM have limited erase/write cycles
	// 2) If the change was saved to EEPROM it would only take affect after all sources of power to the battery board is removed (thus it requires physical interaction with the robot, e.g. removing the battery and charger cable)
	// Therefore a change the the setting
}

BatteryBoard::~BatteryBoard()
{
	if(smbus){delete smbus;}
	if(batteryBoardRamTaskHandle){vTaskDelete(batteryBoardRamTaskHandle);}
	if(batteryBoardEepromTaskHandle){vTaskDelete(batteryBoardEepromTaskHandle);}
}

void BatteryBoard::BatteryBoardRAMThread(void * pvParameters)
{
	BatteryBoard * thisObjPtr = (BatteryBoard *)pvParameters;

	while(1){ // loop until the device becomes available and the configuration can be changed
		if(thisObjPtr->SetCurrentLimitRAM()){ // stop task
			if (thisObjPtr->batteryBoardRamTaskHandle){ // free memory associated with the task
				vTaskSuspend(thisObjPtr->batteryBoardRamTaskHandle);
			}
		}
		osDelay(1000);
	}
}

void BatteryBoard::BatteryBoardEEPROMThread(void * pvParameters)
{
	BatteryBoard * thisObjPtr = (BatteryBoard *)pvParameters;

	while(1){ // loop until the device becomes available and the configuration can be changed
		if(thisObjPtr->SetCurrentLimitEEPROM()){ // stop task
			if (thisObjPtr->batteryBoardEepromTaskHandle){ // free memory associated with the task
				vTaskSuspend(thisObjPtr->batteryBoardEepromTaskHandle);
			}
		}
		osDelay(1000);
	}
}

bool BatteryBoard::SetCurrentLimitRAM(){
	if(smbus->devicePresent()){
		SetChargeCurrentLimit(currentLimitRAM,configuration::RAM);
		osDelay(5);
		SetInputCurrentLimit(currentLimitRAM,configuration::RAM);
		return true;
	}
	else{return false;}
}

bool BatteryBoard::SetCurrentLimitEEPROM(){
	if(smbus->devicePresent()){
		SetChargeCurrentLimit(currentLimitEEPROM,configuration::EEPROM);
		osDelay(5);
		SetInputCurrentLimit(currentLimitEEPROM,configuration::EEPROM);
		return true;
	}
	else{return false;}
}


void BatteryBoard::SetCurrentLimitRAM(uint16_t LimitmA){
	currentLimitRAM = LimitmA;
	if (!batteryBoardRamTaskHandle){
		xTaskCreate(BatteryBoard::BatteryBoardRAMThread, (char *)"Battery Board RAM Change", BATTERY_BOARD_THREAD_STACK, (void*) this, BATTERY_BOARD_PRIORITY, &batteryBoardRamTaskHandle);
	}
	else{
		vTaskResume(batteryBoardRamTaskHandle);
	}
}

void BatteryBoard::SetCurrentLimitEEPROM(uint16_t LimitmA){
	currentLimitEEPROM = LimitmA;
	if (!batteryBoardEepromTaskHandle){
		xTaskCreate(BatteryBoard::BatteryBoardEEPROMThread, (char *)"Battery Board EEPROM Change", BATTERY_BOARD_THREAD_STACK, (void*) this, BATTERY_BOARD_PRIORITY, &batteryBoardEepromTaskHandle);
	}
	else{vTaskResume(batteryBoardEepromTaskHandle);}
}

void BatteryBoard::SetChargeCurrentLimit(uint16_t LimitmA, uint8_t config){
	// Sets the Charge Current Limit
	// LimitmA: The current limit in mA

	uint8_t writeLength = 4;
	uint8_t cmdBuffer[writeLength];
	cmdBuffer[0] = writeLength-1;
	cmdBuffer[1] = config;
	cmdBuffer[2] = (uint8_t)((LimitmA & 0xFF00U) >> 8U);     // First 8 bits of LimitmA
	cmdBuffer[3] = (uint8_t)(LimitmA & 0x00FFU);           // Last 8 bit of LimitmA

	smbus->Write(CmdAddr::write::SetChargeCurrentLimit,cmdBuffer,writeLength); // we use a normal "write word" function to make a "write block" call
}

void BatteryBoard::SetInputCurrentLimit(uint16_t LimitmA, uint8_t config){
	// Sets the Charge Current Limit
	// LimitmA: The current limit in mA

	uint8_t writeLength = 4;
	uint8_t cmdBuffer[writeLength];
	cmdBuffer[0] = writeLength-1;
	cmdBuffer[1] = config;
	cmdBuffer[2] = (uint8_t)((LimitmA & 0xFF00U) >> 8U);     // First 8 bits of LimitmA
	cmdBuffer[3] = (uint8_t)(LimitmA & 0x00FFU);           // Last 8 bit of LimitmA

	smbus->Write(CmdAddr::write::SetInputCurrentLimit,cmdBuffer,writeLength);
}
