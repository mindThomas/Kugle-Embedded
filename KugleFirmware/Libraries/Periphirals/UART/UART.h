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
 
#ifndef PERIPHIRALS_UART_H
#define PERIPHIRALS_UART_H

#include "stm32h7xx_hal.h"
#include "cmsis_os.h" // for memory allocation (for the buffer) and callback

#define UART_CALLBACK_PARAMS (uint8_t * buffer, uint32_t bufLen)
#define UART_INTERRUPT_PRIORITY		5

class UART
{

public:
	typedef enum port_t {
		PORT_UNDEFINED = 0,
		PORT_UART3,
		PORT_UART4,
		PORT_UART7
	} port_t;

public:
	UART(port_t port, uint32_t baud); // unbuffered constructor = polling only
	UART(port_t port, uint32_t baud, uint32_t bufferLength); // ring-buffered constructor
	~UART();
	void InitPeripheral();
	void DeInitPeripheral();
	void ConfigurePeripheral();
	void RegisterRXcallback(void (*callback)UART_CALLBACK_PARAMS); // each byte callback
	void RegisterRXcallback(void (*callback)UART_CALLBACK_PARAMS, uint32_t chunkLength); // each byte callback
	void TransmitBlocking(uint8_t * buffer, uint32_t bufLen);
	void TransmitBlockingHard(uint8_t * buffer, uint32_t bufLen);
	void (*RXcallback)UART_CALLBACK_PARAMS;
	void BufferPush(uint8_t byte);
	uint8_t BufferPop();
	uint32_t BufferContentSize();
	uint8_t * BufferPopN(uint32_t numberOfBytesToPush);
	void Write(uint8_t byte);
	uint32_t Write(uint8_t * buffer, uint32_t length);
	int16_t Read();
	bool Available();
	uint32_t WaitForNewData(uint32_t xTicksToWait);

public:
	SemaphoreHandle_t TransmitByteFinished;
	SemaphoreHandle_t RXdataAvailable;

private:
	port_t _port;
	uint32_t _baud;
	UART_HandleTypeDef _handle;
	uint8_t rxByte;
	uint8_t * _buffer;
	uint32_t _bufferLength;
	uint32_t _bufferWriteIdx;
	uint32_t _bufferReadIdx;
	uint32_t _callbackChunkLength;
	TaskHandle_t _callbackTaskHandle;
	SemaphoreHandle_t _resourceSemaphore;

public:
	static void UART_Interrupt(port_t port);
	static void UART_IncomingDataInterrupt(UART * uart);
	static void CallbackThread(void * pvParameters);

private:
	static UART * objUART3;
	static UART * objUART4;
	static UART * objUART7;
};
	
#endif
