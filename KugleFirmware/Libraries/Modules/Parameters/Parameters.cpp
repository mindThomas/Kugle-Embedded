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
#include "Debug.h"

// Create global parameter variable in project scope
static Parameters * paramsGlobal = 0;

Parameters::Parameters(EEPROM * eeprom) : eeprom_(0), readSemaphore_(0), writeSemaphore_(0), changeCounter_(0)
{
	if (!paramsGlobal) { // first parameter object being created
		// Create global object to hold all parameters
		paramsGlobal = (Parameters *)1; // needs to set this to a value, since "new Parameters" will call the constructor again
		paramsGlobal = new Parameters;
		if (eeprom) {
			AttachEEPROM(eeprom);
		}

		paramsGlobal->readSemaphore_ = xSemaphoreCreateBinary();
		if (paramsGlobal->readSemaphore_ == NULL) {
			ERROR("Could not create Parameters read semaphore");
			return;
		}
		vQueueAddToRegistry(paramsGlobal->readSemaphore_, "Parameters read");
		xSemaphoreGive( paramsGlobal->readSemaphore_ ); // give the resource the first time

		paramsGlobal->writeSemaphore_ = xSemaphoreCreateBinary();
		if (paramsGlobal->writeSemaphore_ == NULL) {
			ERROR("Could not create Parameters write semaphore");
			return;
		}
		vQueueAddToRegistry(paramsGlobal->writeSemaphore_, "Parameters write");
		xSemaphoreGive( paramsGlobal->writeSemaphore_ ); // give the resource the first time
	}

	ParametersSize = PARAMETERS_LENGTH;

	if ((uint32_t)paramsGlobal > 1) { // global object exist - load parameters from this
		Refresh(); // get parameters from global object into this
	}
}

Parameters::~Parameters()
{

}

/* This function attaches an EEPROM to the global object and loads the content */
void Parameters::AttachEEPROM(EEPROM * eeprom)
{
	if (!eeprom) return;
	paramsGlobal->eeprom_ = eeprom;
	paramsGlobal->eeprom_->EnableSection(paramsGlobal->eeprom_->sections.parameters, PARAMETERS_LENGTH);

	if (paramsGlobal->ForceDefaultParameters) {  // store default parameters into EEPROM, since forced default is enabled
		paramsGlobal->StoreParameters(); // Initialize EEPROM with default values
	}
	else { // load parameters from EEPROM and ensure they fit the current firmware (parameter size etc.)
		LoadParametersFromEEPROM(eeprom); // we load the EEPROM parameters into the non-global object initially, to verify that the parameters are valid
		if (ParametersSize == PARAMETERS_LENGTH) {
			// If the parameters are valid, we load them into the global
			paramsGlobal->LoadParametersFromEEPROM(eeprom);
		} else { // if the parameter size has changed or been reorganized we will have to reinitialize the EEPROM with default values
			paramsGlobal->StoreParameters(); // Initialize EEPROM with default values
		}
	}
}

/* Get the latest parameters from the global/master object */
void Parameters::Refresh(void)
{
	if (!paramsGlobal) return;

	ParametersSize = PARAMETERS_LENGTH;

	if (xSemaphoreTake( paramsGlobal->readSemaphore_, ( TickType_t ) 0) == pdTRUE) { // ensure we are allowed to copy the parameters without anybody changing them (corrupting it) meanwhile
		if (changeCounter_ != paramsGlobal->changeCounter_) { // only reload parameters if they have been changed
			changeCounter_ = paramsGlobal->changeCounter_;
			memcpy((uint8_t *)this, (uint8_t *)paramsGlobal, PARAMETERS_LENGTH); // copy global parameters into this object
		}
		xSemaphoreGive( paramsGlobal->readSemaphore_ ); // give back the protection semaphore
	}
}

void Parameters::LockForChange(void)
{
	if (!paramsGlobal) return;
	xSemaphoreTake( paramsGlobal->writeSemaphore_, ( TickType_t ) portMAX_DELAY);
	xSemaphoreTake( paramsGlobal->readSemaphore_, ( TickType_t ) portMAX_DELAY);
	memcpy((uint8_t *)&ForceDefaultParameters, (uint8_t *)&paramsGlobal->ForceDefaultParameters, PARAMETERS_LENGTH); // load latest parameters into current object
	changeCounter_++;
}

void Parameters::UnlockAfterChange(void)
{
	if (!paramsGlobal) return;
	xSemaphoreGive( paramsGlobal->readSemaphore_ ); // give back the protection semaphore since we are now finished with changes

	memcpy((uint8_t *)&paramsGlobal->ForceDefaultParameters, (uint8_t *)&ForceDefaultParameters, PARAMETERS_LENGTH); // copy changed parameters (from current object) into global parameters object
 	paramsGlobal->StoreParameters(); // store the newly update global parameters in EEPROM (if it exists)

	xSemaphoreGive( paramsGlobal->writeSemaphore_ ); // give back the EEPROM storing protection semaphore
}

void Parameters::LoadParametersFromEEPROM(EEPROM * eeprom)
{
	if (!eeprom) return; // EEPROM not configured

	eeprom->ReadData(eeprom->sections.parameters, (uint8_t *)&ForceDefaultParameters, PARAMETERS_LENGTH);
}

void Parameters::StoreParameters(void)
{
	if (!eeprom_) return; // EEPROM not configured
	eeprom_->WriteData(eeprom_->sections.parameters, (uint8_t *)&ForceDefaultParameters, PARAMETERS_LENGTH);
}


/*
void Parameters::StoreThread(void)
{
	// Make a thread in the global pararameter object which checks for changes and writes to the EEPROM (eg. checking every 10 seconds)
}
*/

#if 0
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
		if (paramsGlobal->ForceDefaultParameters)
			return *paramsGlobal; // do not proceed in loading EEPROM parameters, since forced default is enabled

		eeprom->EnableSection(eeprom->sections.parameters, PARAMETERS_LENGTH);
		paramsGlobal->LoadParametersFromEEPROM(eeprom);
		if (paramsGlobal->ParametersSize != PARAMETERS_LENGTH) { // ensure that parameter size has not changed/been reorganized, as we will then have to reinitialize the EEPROM with default values
			delete(paramsGlobal);
			paramsGlobal = new Parameters; // this loads the default parameters

			paramsGlobal->eeprom_ = eeprom;
			paramsGlobal->ParametersSize = PARAMETERS_LENGTH;
			paramsGlobal->StoreParameters(); // Initialize EEPROM with default values
		}
	}

	return *paramsGlobal;
}
#endif
