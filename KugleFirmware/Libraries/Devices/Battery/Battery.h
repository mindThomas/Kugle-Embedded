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
 
#ifndef DEVICES_BATTERY_H
#define DEVICES_BATTERY_H

#include "SMBus.h"

class Battery
{

public:
	Battery(SMBus::port_t SMBusHardwareResources,uint8_t _location);
	Battery(SMBus::port_t SMBusHardwareResources);
	~Battery();

	float getCurrent(void);
	float getTemperature(void);
	float getVoltage(void);


	float getCharge(void);           // Current charge in Ah  (If unmeasured NaN)
	float getCapacity(void);         // Capacity in Ah (last full capacity)  (If unmeasured NaN)
	float getDesign_capacity(void);  // Capacity in Ah (design capacity)  (If unmeasured NaN)
	float getPercentage(void);       // Charge percentage on 0 to 1 range  (If unmeasured NaN)
	uint8_t   getPowerSupplyStatus(void);     // The charging status as reported. Values defined above
	uint8_t   getPowerSupplyHealth(void);     // The battery health metric. Values defined above
	uint8_t   getPowerSupplyTechnology(void); // The battery chemistry. Values defined above
	bool    checkPresent(void);        // True if the battery is present
	uint16_t getCycleCount(void);

	//float cell_voltage[10];   // An array of individual cell voltages for each cell in the pack
						   // If individual voltages unknown but number of cells known set each to NaN
	uint8_t getLocation(void);        // The location into which the battery is inserted. (slot number or plug)
	uint32_t getSerialNumber(void);   // The best approximation of the battery serial number
	bool isBatteryGettingOld(void);   // The best approximation of the battery serial number

	enum {
	    POWER_SUPPLY_STATUS_UNKNOWN = 0u,
	    POWER_SUPPLY_STATUS_CHARGING = 1u,
	    POWER_SUPPLY_STATUS_DISCHARGING = 2u,
	    POWER_SUPPLY_STATUS_NOT_CHARGING = 3u,
	    POWER_SUPPLY_STATUS_FULL = 4u,
	    POWER_SUPPLY_HEALTH_UNKNOWN = 0u,
	    POWER_SUPPLY_HEALTH_GOOD = 1u,
	    POWER_SUPPLY_HEALTH_OVERHEAT = 2u,
	    POWER_SUPPLY_HEALTH_DEAD = 3u,
	    POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4u,
	    POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5u,
	    POWER_SUPPLY_HEALTH_COLD = 6u,
	    POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7u,
	    POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8u,
	    POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0u,
	    POWER_SUPPLY_TECHNOLOGY_NIMH = 1u,
	    POWER_SUPPLY_TECHNOLOGY_LION = 2u,
	    POWER_SUPPLY_TECHNOLOGY_LIPO = 3u,
	    POWER_SUPPLY_TECHNOLOGY_LIFE = 4u,
	    POWER_SUPPLY_TECHNOLOGY_NICD = 5u,
	    POWER_SUPPLY_TECHNOLOGY_LIMN = 6u,
	};

private:
	SMBus *smbus;
	uint8_t location; // the idx of the SMBus that the battery is attached to 0 for unknown
	uint8_t present;
	uint8_t DeviceAddr = 0x0B; //0x16
	uint16_t gettingOldCycleCount = 300;

	class CmdAddr
	{
	public:
		class read{
		public:
			enum : uint8_t
			{ // according to smbus protocol v 1.1
				ManufacturerAccess = 0x00,
				RemainingCapacityAlarm,
				RemainingTimeAlarm,
				BatteryMode,
				AtRate,
				AtRateTimeToFull,
				AtRateTimeToEmpty,
				AtRateOK,
				Temperature,
				Voltage,
				Current,
				AverageCurrent,
				MaxError,
				RelativeStateOfCharge,
				AbsoluteStateOfCharge,
				RemainingCapacity,
				FullChargeCapacity,
				RunTimeToEmpty,
				AverageTimeToEmpty,
				AverageTimeToFull,
				ChargingCurrent,
				ChargingVoltage,
				BatteryStatus,
				CycleCount,
				DesignCapacity,
				DesignVoltage,
				SpecificationInfo,
				ManufactureDate,
				SerialNumber,
				// 0x1d - 0x1f reserved
				ManufacturerName = 0x20,
				DeviceName,
				DeviceChemistry,
				ManufacturerData
				// 0x25 - 0x2e reserved
				// 0x2f OptionalMfgFunction5
				// 0x30 - 0x3b reserved
				// 0x3c - 0x3f OptionalMfgFunction4-1
			};
		};

		class write{
		public:
			enum : uint8_t
			{ // according to smbus protocol v 1.1
				ManufacturerAccess = 0x00,
				RemainingCapacityAlarm,
				RemainingTimeAlarm,
				BatteryMode
				// 0x25 - 0x2e reserved
				// 0x2f OptionalMfgFunction5
				// 0x30 - 0x3b reserved
				// 0x3c - 0x3f OptionalMfgFunction4-1
			};
		};
	};

};
	
	
#endif
