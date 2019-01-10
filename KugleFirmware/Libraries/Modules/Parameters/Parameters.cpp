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
 
#include "Parameters.h"
#include "EEPROM.h"

// Create global parameter variable in project scope
static Parameters * paramsGlobal = 0;

Parameters::Parameters() : eeprom_(0)
{

}

Parameters::~Parameters()
{

}

void Parameters::LoadParametersFromEEPROM(EEPROM * eeprom)
{
	if (!eeprom && !eeprom_) return; // EEPROM not configured

	if (eeprom)
		eeprom->ReadData(eeprom->sections.parameters, (uint8_t *)this, sizeof(Parameters)-1);
	else
		eeprom_->ReadData(eeprom_->sections.parameters, (uint8_t *)this, sizeof(Parameters)-1);
}

void Parameters::StoreParameters(void)
{
	if (!eeprom_) return; // EEPROM not configured
	eeprom_->WriteData(eeprom_->sections.parameters, (uint8_t *)this, sizeof(Parameters)-1);
}

Parameters& Parameters::Get()
{
	if (!paramsGlobal) // first time initializes the global parameters objects
		paramsGlobal = new Parameters;

	return *paramsGlobal;
}

Parameters& Parameters::Get(EEPROM * eeprom)
{
	if (!eeprom) return Get();

	if (!paramsGlobal) { // first time initializes the global parameters objects
		paramsGlobal = new Parameters;
		eeprom->EnableSection(eeprom->sections.parameters, sizeof(Parameters)-1);
		paramsGlobal->LoadParametersFromEEPROM(eeprom);
		if (paramsGlobal->ParametersSize != sizeof(Parameters)) { // ensure that parameter size has not changed/been reorganized, as we will then have to reinitialize the EEPROM with default values
			delete(paramsGlobal);
			paramsGlobal = new Parameters; // this loads the default parameters

			paramsGlobal->eeprom_ = eeprom;
			paramsGlobal->ParametersSize = sizeof(Parameters);
			paramsGlobal->StoreParameters(); // Initialize EEPROM with default values
		}
	}

	return *paramsGlobal;
}
