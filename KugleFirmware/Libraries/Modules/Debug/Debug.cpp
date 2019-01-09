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
 
#include "Debug.h"
#include "cmsis_os.h"
#include "LSPC.hpp"
 
Debug * Debug::debugHandle = 0;

// Necessary to export for compiler such that the Error_Handler function can be called by C code
extern "C" __EXPORT void Error_Handler(void);

Debug::Debug(void * com) : com_(com)
{
	if (debugHandle) {
		ERROR("Debug object already created");
		return;
	}

	if (!com) {
		ERROR("LSPC object does not exist");
		return;
	}

	mutex_ = xSemaphoreCreateBinary();
	if (mutex_ == NULL) {
		ERROR("Could not create Debug mutex");
		return;
	}
	vQueueAddToRegistry(mutex_, "Debug mutex");
	xSemaphoreGive( mutex_ ); // give the semaphore the first time

	currentBufferLocation_ = 0;
	memset(messageBuffer_, 0, MAX_DEBUG_TEXT_LENGTH);
	xTaskCreate( Debug::PackageGeneratorThread, (char *)"Attitude Controller", THREAD_STACK_SIZE, (void*) this, THREAD_PRIORITY, &_TaskHandle);

	debugHandle = this;
}

Debug::~Debug()
{
	debugHandle = 0;
}


void Debug::PackageGeneratorThread(void * pvParameters)
{
	Debug * debug = (Debug *)pvParameters;

	while (1)
	{
		osDelay(1);
		xSemaphoreTake( debug->mutex_, ( TickType_t ) portMAX_DELAY ); // take debug mutex
		if (debug->currentBufferLocation_ > 0) {
			((LSPC*)debug->com_)->TransmitAsync(lspc::MessageTypesOut::Debug, (const uint8_t *)debug->messageBuffer_, debug->currentBufferLocation_);
			debug->currentBufferLocation_ = 0;
		}
		xSemaphoreGive( debug->mutex_ ); // give hardware resource back
	}
}

void Debug::Message(const char * msg)
{
	if (!debugHandle) return;
	if (!debugHandle->com_) return;
	if (!((LSPC*)debugHandle->com_)->Connected()) return;

	xSemaphoreTake( debugHandle->mutex_, ( TickType_t ) portMAX_DELAY ); // take debug mutex

	uint16_t stringLength = strlen(msg);
	if (stringLength > MAX_DEBUG_TEXT_LENGTH) return; // message is too long
	if (stringLength > (MAX_DEBUG_TEXT_LENGTH-debugHandle->currentBufferLocation_)) {// stringLength = (MAX_DEBUG_TEXT_LENGTH-debugHandle->currentBufferLocation_); // "cut away" any parts above the maximum string length
		// Send package now and clear buffer
		((LSPC*)debugHandle->com_)->TransmitAsync(lspc::MessageTypesOut::Debug, (const uint8_t *)debugHandle->messageBuffer_, debugHandle->currentBufferLocation_);
		debugHandle->currentBufferLocation_ = 0;
	}

	memcpy(&debugHandle->messageBuffer_[debugHandle->currentBufferLocation_], msg, stringLength);
	debugHandle->currentBufferLocation_ += stringLength;
	xSemaphoreGive( debugHandle->mutex_ ); // give hardware resource back
}

void Debug::Message(std::string msg)
{
	Message(msg.c_str());
	Message("\n");
}

void Debug::Message(const char * functionName, const char * msg)
{
	Message("[");
	Message(functionName);
	Message("] ");
	Message(msg);
	Message("\n");
}

void Debug::Message(const char * functionName, std::string msg)
{
	Message("[");
	Message(functionName);
	Message("] ");
	Message(msg.c_str());
	Message("\n");
}

void Debug::Message(const char * type, const char * functionName, const char * msg)
{
	Message(type);
	Message("[");
	Message(functionName);
	Message("] ");
	Message(msg);
	Message("\n");
}

void Debug::Message(std::string type, const char * functionName, std::string msg)
{
	Message("[");
	Message(functionName);
	Message("] ");
	Message(msg.c_str());
	Message("\n");
}

void Debug::print(const char * msg)
{
	Message(msg);
}

void Debug::printf( const char *msgFmt, ... )
{
	va_list args;

	if (!debugHandle) return;
	if (!debugHandle->com_) return;
	if (!((LSPC*)debugHandle->com_)->Connected()) return;

	va_start( args,  msgFmt );

	char * strBuf = (char *) pvPortMalloc(MAX_DEBUG_TEXT_LENGTH);
	if (!strBuf) return;

	vsnprintf( strBuf, MAX_DEBUG_TEXT_LENGTH, msgFmt, args );

	Message(strBuf);

	vPortFree(strBuf);

	va_end( args );
}

void Debug::Error(const char * type, const char * functionName, const char * msg)
{
	// At errors do not continue current task/thread but print instead the error message repeatedly
	while (1)
	{
		Debug::Message(type, functionName, msg);
		osDelay(500);
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	Debug::Error("ERROR: ", "Error_Handler", "Global ");
}
