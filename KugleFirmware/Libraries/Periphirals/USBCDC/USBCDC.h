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
 
#ifndef PERIPHIRALS_USBCDC_H
#define PERIPHIRALS_USBCDC_H

#include "stm32h7xx_hal.h"
#include "cmsis_os.h" // for USB processing task

#include "usbd_conf.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#define USBCDC_TX_PROCESSING_THREAD_STACK_SIZE		128
#define USBCDC_TX_QUEUE_LENGTH	10
#define USBCDC_RX_QUEUE_LENGTH	10

class USBCDC
{

public:
	USBCDC(uint32_t processingTaskPriority);
	~USBCDC();
	bool GetPackage(USB_CDC_Package_t * packageBuffer);
	void Write(uint8_t byte);
	uint32_t Write(uint8_t * buffer, uint32_t length);
	int16_t Read();
	bool Available();
	uint32_t WaitForNewData(uint32_t xTicksToWait = portMAX_DELAY);

private:
	TaskHandle_t _processingTaskHandle;
	USB_CDC_Package_t _tmpPackageForRead;
	uint8_t _readIndex;
	SemaphoreHandle_t _TXfinishedSemaphore;
	SemaphoreHandle_t _RXdataAvailable;
	QueueHandle_t _TXqueue;
	QueueHandle_t _RXqueue;
	SemaphoreHandle_t _resourceSemaphore;

public:


public:
	static void ProcessingThread(void * pvParameters);
	static void SysInit();

public:
	static USBCDC * usbHandle;
	static USBD_HandleTypeDef hUsbDeviceFS;
	static PCD_HandleTypeDef hpcd_USB_OTG_FS;
	
};
	
	
#endif
