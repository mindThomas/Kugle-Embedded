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
 
#include "Battery.h"
#include <bitset>         // std::bitset
#include "Debug.h"
 
Battery::Battery(SMBus::port_t SMBusHardwareResources)
{
	location = 0;
	smbus = new SMBus(SMBusHardwareResources, DeviceAddr);
	present = smbus->devicePresent();
}

Battery::Battery(SMBus::port_t SMBusHardwareResources,uint8_t _location)
{
	location = _location;
	smbus = new SMBus(SMBusHardwareResources, DeviceAddr);
	present = smbus->devicePresent();
}

Battery::~Battery()
{
	if(smbus){
		delete smbus;
	}
}



// SMBus Commands
// -------------------------------------------------------------------------------------------
// Reading and writing from/to the battery.
//
/* This file contains the following functions:
    SMBus_R(); Low level function used to read on the SMBus
    SMBus_w(); Low level function used to write to the SMBus
    getCurrent(); Returns the status of the bus and saves the current in a buffer
    getCycleCount(); Returns the status of the bus and saves the number of Cycles in a buffer
    getRunTimeToEmpty(); Returns the status of the bus and saves how much battery time there is left in a buffer
    getTemperature(); Returns the status of the bus and saves the current Temperature in a buffer
    getVoltage(); Returns the status of the bus and saves the Voltage in a buffer
    SetChargeCurrentLimit(); Changes the Charge Current Limit
    SetInputCurrentLimit(); Changes the Input Current limit

Look under the specific function for more information
*/
// -------------------------------------------------------------------------------------------

float Battery::getCurrent(void){
	if(smbus->devicePresent()){
		// Returns the current being supplied (or accepted) through the battery's terminals.
		uint8_t const N_bytes = 2;
		uint8_t responbuf_tmp[N_bytes];

		smbus->Read(CmdAddr::read::Current,responbuf_tmp,N_bytes);

		int16_t number = responbuf_tmp[0] | responbuf_tmp[1] << 8;
		float current = ((float)number) / 1000;

		return current;
	}
	else{
		return 0;
	}
}

float Battery::getTemperature(void){
	if(smbus->devicePresent()){
		// Returns the pack's internal temperature.
		uint8_t N_bytes = 2;
		uint8_t responbuf_tmp[2];

		smbus->Read(CmdAddr::read::Temperature,responbuf_tmp,N_bytes);

		int number = responbuf_tmp[0] | responbuf_tmp[1] << 8;
		float temperature = ((float)number) / 10 - 273.15;

		return temperature;
	}
	else{
		return 0;
	}
}

float Battery::getVoltage(void){
	if(smbus->devicePresent()){
		// Returns the battery's voltage (measured at the cell stack)
		uint8_t N_bytes = 2;
		uint8_t responbuf_tmp[2];

		smbus->Read(CmdAddr::read::Voltage,responbuf_tmp,N_bytes);

		int16_t number = responbuf_tmp[0] | responbuf_tmp[1] << 8;
		float voltage = ((float)number) / 1000;

		return voltage;
	}
	else{
		return 0;
	}
}

float Battery::getCharge(void)           // Current charge in Ah  (If unmeasured NaN)
{
	if(smbus->devicePresent()){
		// Returns the battery's charge
		uint8_t N_bytes = 2;
		uint8_t responbuf_tmp[2];

		smbus->Read(CmdAddr::read::RemainingCapacity,responbuf_tmp,N_bytes);

		uint16_t number = responbuf_tmp[0] | responbuf_tmp[1] << 8;
		float charge = ((float)number) / 1000;

		return charge;
	}
	else{
		return 0;
	}
}


float Battery::getCapacity(void)         // Capacity in Ah (last full capacity)  (If unmeasured NaN)
{
	if(smbus->devicePresent()){
		// Returns the battery's capacity
		uint8_t N_bytes = 2;
		uint8_t responbuf_tmp[2];

		smbus->Read(CmdAddr::read::FullChargeCapacity,responbuf_tmp,N_bytes);

		uint16_t number = responbuf_tmp[0] | responbuf_tmp[1] << 8;
		float Capacity = ((float)number) / 1000;

		return Capacity;
	}
	else{
		return 0;
	}
}

float Battery::getDesign_capacity(void)  // Capacity in Ah (design capacity)  (If unmeasured NaN)
{
	if(smbus->devicePresent()){
		// Returns the battery's Design Capacity
		uint8_t N_bytes = 2;
		uint8_t responbuf_tmp[2];

		smbus->Read(CmdAddr::read::DesignCapacity,responbuf_tmp,N_bytes);

		uint16_t number = responbuf_tmp[0] | responbuf_tmp[1] << 8;
		float DesignCapacity = ((float)number) / 1000;

		return DesignCapacity;
	}
	else{
		return 0;
	}
}

float Battery::getPercentage(void)       // Charge percentage on 0 to 1 range  (If unmeasured NaN)
{
	if(smbus->devicePresent()){
		// Returns the battery's charge percentage
		uint8_t N_bytes = 2;
		uint8_t responbuf_tmp[2];

		smbus->Read(CmdAddr::read::RelativeStateOfCharge,responbuf_tmp,N_bytes);

		uint16_t number = responbuf_tmp[0] | responbuf_tmp[1] << 8;
		float chargePercentage = ((float)number) / 100;

		return chargePercentage;
	}
	else{
		return 0;
	}
}

uint8_t   Battery::getPowerSupplyStatus(void)     // The charging status as reported. Values defined above
{
	if(smbus->devicePresent()){
		// Returns the battery's status
		uint8_t N_bytes = 2;
		uint8_t responbuf_tmp[2];

		smbus->Read(CmdAddr::read::BatteryStatus,responbuf_tmp,N_bytes);

		uint16_t number = responbuf_tmp[0] | responbuf_tmp[1] << 8;
		std::bitset<16> tmp(number);

		if (tmp.test(6)) return POWER_SUPPLY_STATUS_DISCHARGING; // Discharging??
		else if (tmp.test(5)) return POWER_SUPPLY_STATUS_FULL; // fully charged?
		else return POWER_SUPPLY_STATUS_UNKNOWN;
	}
	else{
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}
}

uint8_t   Battery::getPowerSupplyHealth(void)     // The battery health metric. Values defined above
{
	if(smbus->devicePresent()){
		// Returns the battery's health
		uint8_t N_bytes = 2;
		uint8_t responbuf_tmp[2];

		smbus->Read(CmdAddr::read::BatteryStatus,responbuf_tmp,N_bytes);

		uint16_t number = responbuf_tmp[0] | responbuf_tmp[1] << 8;
		std::bitset<16> tmp(number);

		if (tmp.test(12)) return POWER_SUPPLY_HEALTH_OVERHEAT;
		else if (tmp.test(15)) return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		else if (tmp.test(14)) return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		else if (tmp.test(11)) return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		else if (tmp.test(9)) return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		else if (tmp.test(8)) return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		else return POWER_SUPPLY_HEALTH_GOOD;
	}
	else{
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}
}

uint8_t   Battery::getPowerSupplyTechnology(void) // The battery chemistry. Values defined above
{
	if(smbus->devicePresent()){
		uint8_t N_bytes = 10;
		uint8_t responbuf_tmp[N_bytes];

		uint8_t bytesRead = smbus->blockRead(CmdAddr::read::DeviceChemistry,responbuf_tmp,N_bytes);

		char responbuf_tmp2[bytesRead+1];

		for (int i = 0; i<bytesRead;i++){
			responbuf_tmp2[i] = toupper((char)responbuf_tmp[i]);
		}
		responbuf_tmp2[bytesRead] = {'\0'};

		if (strcmp(responbuf_tmp2, "LION") == 0){return POWER_SUPPLY_TECHNOLOGY_LION;}
		else if (strcmp(responbuf_tmp2, "NIMH") == 0){return POWER_SUPPLY_TECHNOLOGY_NIMH;}
		else if (strcmp(responbuf_tmp2, "NICD") == 0){return POWER_SUPPLY_TECHNOLOGY_NICD;}
		else if (strcmp(responbuf_tmp2, "LIP") == 0){return POWER_SUPPLY_TECHNOLOGY_LIPO;}
		else{return POWER_SUPPLY_TECHNOLOGY_UNKNOWN;}
	}
	else{
		return 0;
	}
}

bool    Battery::checkPresent(void)        // True if the battery is present
{
	return present;
}

uint8_t Battery::getLocation(void)        // The location into which the battery is inserted. (slot number or plug)
{
	return location;
}

uint32_t Battery::getSerialNumber(void)
{
	if(smbus->devicePresent()){
		// Returns the battery's serial number

		/* Explanation
		 * SerialNumber(): This function is used to return a serial number. This number when combined with the ManufacturerName(),
		 * the DeviceName(), and the ManufactureDate() will uniquely identify the battery (unsigned int).
		 *
		 * Since we already know the manufacture and device name (just look at it) we return a combination of ManufactureDate() and SerialNumber()
		 * to uniqly identify the battery!
		*/


		uint8_t N_bytes = 2;
		uint8_t responbuf_tmp[2];

		smbus->Read(CmdAddr::read::ManufactureDate,responbuf_tmp,N_bytes);

		uint16_t ManufactureDate = responbuf_tmp[0] | responbuf_tmp[1] << 8;

		smbus->Read(CmdAddr::read::SerialNumber,responbuf_tmp,N_bytes);

		uint16_t serialNumber = responbuf_tmp[0] | responbuf_tmp[1] << 8;

		uint32_t uniqueIdentifier = ManufactureDate << 16 | serialNumber;
		return uniqueIdentifier;
	}
	else{
		return 0;
	}
}

uint16_t Battery::getCycleCount(void){
	if(smbus->devicePresent()){
		// Returns the number of charge/discharge cycles the battery has experienced. A charge/discharge cycle is defined as: an amount of discharge approximately equal to the value of DesignCapacity.
		uint8_t N_bytes = 2;
		uint8_t responbuf_tmp[2];

		smbus->Read(CmdAddr::read::CycleCount,responbuf_tmp,N_bytes);

		uint16_t nCycle = responbuf_tmp[0] | responbuf_tmp[1] << 8;

		return nCycle;
	}
	else{
		return 0;
	}
}

bool Battery::isBatteryGettingOld(void){
	if (getCycleCount() > gettingOldCycleCount){
		return true;
	}
	else{return false;}
}

