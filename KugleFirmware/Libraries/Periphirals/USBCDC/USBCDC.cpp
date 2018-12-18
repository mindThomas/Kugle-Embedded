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
 
#include "USBCDC.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h" // for USB processing task
#include "Debug.h"

#include "usbd_conf.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

USBCDC * USBCDC::usbHandle = 0;
USBD_HandleTypeDef USBCDC::hUsbDeviceFS;
PCD_HandleTypeDef USBCDC::hpcd_USB_OTG_FS;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" __EXPORT void OTG_FS_IRQHandler(void);

USBCDC::USBCDC(UBaseType_t processingTaskPriority) : _processingTaskHandle(0), TXfinishedSemaphore(0), RXqueue(0)
{
	if (usbHandle) {
		ERROR("USB object already created");
		return;
	}

	tmpPackageForRead.length = 0;
	readIndex = 0;

	TXqueue = xQueueCreate( USBCDC_TX_QUEUE_LENGTH, sizeof(USB_CDC_Package_t) );
	vQueueAddToRegistry(TXqueue, "USB TX");
	CDC_RegisterReceiveQueue(TXqueue);

	RXqueue = xQueueCreate( USBCDC_RX_QUEUE_LENGTH, sizeof(USB_CDC_Package_t) );
	vQueueAddToRegistry(RXqueue, "USB RX");
	CDC_RegisterReceiveQueue(RXqueue);

	TXfinishedSemaphore = xSemaphoreCreateBinary();
	vQueueAddToRegistry(TXfinishedSemaphore, "USB TX Finished");
	USBD_CDC_SetTXfinishedSemaphore(TXfinishedSemaphore);

	/* Init USB Device Library, add supported class and start the library. */
	USBD_LL_SetPCD(&USBCDC::hpcd_USB_OTG_FS);
	USBD_Init(&USBCDC::hUsbDeviceFS, &FS_Desc, DEVICE_FS);
	USBD_RegisterClass(&USBCDC::hUsbDeviceFS, &USBD_CDC);
	CDC_RegisterUsbDeviceObject(&USBCDC::hUsbDeviceFS);
	USBD_CDC_RegisterInterface(&USBCDC::hUsbDeviceFS, &USBD_Interface_fops_FS);
	USBD_Start(&USBCDC::hUsbDeviceFS);

	HAL_PWREx_EnableUSBVoltageDetector();

	xTaskCreate(USBCDC::ProcessingThread, (char *)"USB handler", USBCDC_PROCESSING_THREAD_STACK_SIZE, (void*) this, processingTaskPriority, &_processingTaskHandle);
}

USBCDC::~USBCDC()
{
	
}

bool USBCDC::GetPackage(USB_CDC_Package_t * packageBuffer)
{
	if (!packageBuffer) return false;

	if ( xQueueReceive( RXqueue, packageBuffer, ( TickType_t ) 0 ) == pdPASS )
		return true;
	else
		return false;
}

void USBCDC::Write(uint8_t byte)
{
	USB_CDC_Package_t package;
	package.data[0] = byte;
	package.length = 1;
	xQueueSend(TXqueue, (void *)&package, (TickType_t) 1);
}

void USBCDC::Write(uint8_t * buffer, uint32_t length)
{
	USB_CDC_Package_t package;

	uint32_t txLength = length;
	uint8_t packageLength;

	// Split buffer data into packages
	while (txLength > 0) {
		if (txLength > USB_PACKAGE_MAX_SIZE)
			packageLength = USB_PACKAGE_MAX_SIZE;
		else
			packageLength = txLength;

		memcpy(package.data, buffer, packageLength);
		package.length = packageLength;

		xQueueSend(TXqueue, (void *)&package, (TickType_t) 1);

		buffer += packageLength;
		txLength -= packageLength;
	}
}

uint8_t USBCDC::Read()
{
	uint8_t returnValue;

	if (readIndex == tmpPackageForRead.length) { // load in new package for reading (if possible)
		if ( xQueueReceive( RXqueue, &tmpPackageForRead, ( TickType_t ) 1 ) != pdPASS ) {
			return 0; // no new package
		}
		readIndex = 0;
	}

	returnValue = tmpPackageForRead.data[readIndex];
	readIndex++;

	return returnValue;
}

bool USBCDC::Available()
{
	if (readIndex != tmpPackageForRead.length || uxQueueMessagesWaiting(RXqueue) > 0)
		return true;
	else
		return false;
}

void USBCDC::ProcessingThread(void * pvParameters)
{
	USB_CDC_Package_t package;
	USBCDC * usb = (USBCDC *)pvParameters;

	// Send initial zero package - wait for communication channel to be opened
	memset(package.data, 0, USB_PACKAGE_MAX_SIZE);
	while (CDC_Transmit_FS(package.data, USB_PACKAGE_MAX_SIZE) != USBD_OK) {
		osDelay(1);
	}

	// Transmit processing loop
	while (1) {
		if ( xQueueReceive( usb->TXqueue, &package, ( TickType_t ) portMAX_DELAY ) == pdPASS ) {
			CDC_Transmit_FS_ThreadBlocking(package.data, package.length);
		}
	}
}

void OTG_FS_IRQHandler(void)
{
	HAL_PCD_IRQHandler(&USBCDC::hpcd_USB_OTG_FS);
}
