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
 
#include "I2C.h"
#include "stm32h7xx_hal.h"
#include "Debug.h"
#include <math.h>
#include <string.h> // for memset

I2C::hardware_resource_t * I2C::resI2C1 = 0;
I2C::hardware_resource_t * I2C::resI2C3 = 0;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" __EXPORT void I2C1_EV_IRQHandler(void);
extern "C" __EXPORT void I2C1_ER_IRQHandler(void);
extern "C" __EXPORT void I2C3_EV_IRQHandler(void);
extern "C" __EXPORT void I2C3_ER_IRQHandler(void);
extern "C" __EXPORT void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle);
extern "C" __EXPORT void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle);
extern "C" __EXPORT void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle);

I2C::I2C(port_t port, uint8_t devAddr, uint32_t frequency)
{
	_devAddr = devAddr << 1;
	InitPeripheral(port, frequency);
	ConfigurePeripheral();
}

I2C::I2C(port_t port, uint8_t devAddr) : I2C(port, devAddr, I2C_DEFAULT_FREQUENCY)
{
}

I2C::~I2C()
{
	if (!_hRes) return;

	_hRes->instances--;
	if (_hRes->instances == 0) { // deinitialize port and periphiral
		DeInitPeripheral();

		// Delete hardware resource
		port_t tmpPort = _hRes->port;
		delete(_hRes);

		switch (tmpPort)
		{
			case PORT_I2C1:
				resI2C1 = 0;
				break;
			case PORT_I2C3:
				resI2C3 = 0;
				break;
			default:
				ERROR("Undefined I2C port");
				return;
		}
	}
}

void I2C::DeInitPeripheral()
{
	if (!_hRes) return;
	if (_hRes->port == PORT_I2C1) {
	    /* Peripheral clock disable */
	    __HAL_RCC_I2C1_CLK_DISABLE();

	    /**I2C1 GPIO Configuration
	    PB8     ------> I2C1_SCL
	    PB9     ------> I2C1_SDA
	    */
	    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

	    /* I2C1 interrupt DeInit */
	    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
	    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
	}
	else if (_hRes->port == PORT_I2C3)
	{
	    /* Peripheral clock disable */
	    __HAL_RCC_I2C3_CLK_DISABLE();

	    /**I2C3 GPIO Configuration
	    PC9     ------> I2C3_SDA
	    PA8     ------> I2C3_SCL
	    */
	    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);
	    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

	    /* I2C3 interrupt DeInit */
	    HAL_NVIC_DisableIRQ(I2C3_ER_IRQn);
	    HAL_NVIC_DisableIRQ(I2C3_EV_IRQn);
	}
}

void I2C::InitPeripheral(port_t port, uint32_t frequency)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	bool firstTime = false;

	switch (port)
	{
		case PORT_I2C1:
			if (!resI2C1) {
				resI2C1 = new I2C::hardware_resource_t;
				memset(resI2C1, 0, sizeof(I2C::hardware_resource_t));
				firstTime = true;
			}
			_hRes = resI2C1;
			break;
		case PORT_I2C3:
			if (!resI2C3) {
				resI2C3 = new I2C::hardware_resource_t;
				memset(resI2C3, 0, sizeof(I2C::hardware_resource_t));
				firstTime = true;
			}
			_hRes = resI2C3;
			break;
		default:
			ERROR("Undefined I2C port");
			_hRes = 0;
			return;
	}

	if (firstTime) { // first time configuring peripheral
		_hRes->port = port;
		_hRes->frequency = frequency;
		_hRes->configured = false;
		_hRes->instances = 0;
		_hRes->resourceSemaphore = xSemaphoreCreateBinary();
		if (_hRes->resourceSemaphore == NULL) {
			_hRes = 0;
			ERROR("Could not create I2C resource semaphore");
			return;
		}
		vQueueAddToRegistry(_hRes->resourceSemaphore, "I2C Resource");
		xSemaphoreGive( _hRes->resourceSemaphore ); // give the semaphore the first time

		_hRes->transmissionFinished = xSemaphoreCreateBinary();
		if (_hRes->transmissionFinished == NULL) {
			_hRes = 0;
			ERROR("Could not create I2C transmission semaphore");
			return;
		}
		vQueueAddToRegistry(_hRes->transmissionFinished, "I2C Finish");
		xSemaphoreGive( _hRes->transmissionFinished ); // ensure that the semaphore is not taken from the beginning
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY ); // ensure that the semaphore is not taken from the beginning

		// Configure pins for I2C and I2C peripheral accordingly
		if (port == PORT_I2C1) {
		    __HAL_RCC_GPIOB_CLK_ENABLE();
		    /**I2C1 GPIO Configuration
		    PB8     ------> I2C1_SCL
		    PB9     ------> I2C1_SDA
		    */
		    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
		    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		    GPIO_InitStruct.Pull = GPIO_NOPULL;
		    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
		    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		    /* Peripheral clock enable */
		    __HAL_RCC_I2C1_CLK_ENABLE();

		    /* NVIC for I2C1 */
		    HAL_NVIC_SetPriority(I2C1_ER_IRQn, I2C_INTERRUPT_PRIORITY, 0);
		    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
		    HAL_NVIC_SetPriority(I2C1_EV_IRQn, I2C_INTERRUPT_PRIORITY, 0);
		    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
		}
		else if (port == PORT_I2C3)
		{
		    __HAL_RCC_GPIOC_CLK_ENABLE();
		    __HAL_RCC_GPIOA_CLK_ENABLE();
		    /**I2C3 GPIO Configuration
		    PC9     ------> I2C3_SDA
		    PA8     ------> I2C3_SCL
		    */
		    GPIO_InitStruct.Pin = GPIO_PIN_9;
		    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		    GPIO_InitStruct.Pull = GPIO_NOPULL;
		    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
		    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		    GPIO_InitStruct.Pin = GPIO_PIN_8;
		    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		    GPIO_InitStruct.Pull = GPIO_NOPULL;
		    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
		    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		    /* Peripheral clock enable */
		    __HAL_RCC_I2C3_CLK_ENABLE();

		    /* NVIC for I2C3 */
		    HAL_NVIC_SetPriority(I2C3_ER_IRQn, I2C_INTERRUPT_PRIORITY, 0);
		    HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);
		    HAL_NVIC_SetPriority(I2C3_EV_IRQn, I2C_INTERRUPT_PRIORITY, 0);
		    HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
		}
	}

	_hRes->instances++;
}

void I2C::ConfigurePeripheral()
{
	if (!_hRes) return;
	if (!_hRes->configured) { // only configure periphiral once
		switch (_hRes->port) {
			case PORT_I2C1:
				_hRes->handle.Instance = I2C1;
				break;
			case PORT_I2C3:
				_hRes->handle.Instance = I2C3;
				break;
			default:
				_hRes = 0;
				ERROR("Undefined I2C port");
				return;
		}

		_hRes->handle.Init.OwnAddress1 = 0;
		_hRes->handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		_hRes->handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		_hRes->handle.Init.OwnAddress2 = 0;
		_hRes->handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
		_hRes->handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		_hRes->handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

		//_hRes->handle.Init.Timing = 0x10C0ECFF;
		/* Determine I2C timing configuration  (see reference manual page 1015)
		    tI2CCLK = 1/100Mhz = 10 ns

			PRESC = 0001 = 1
			tPRESC = (PRESC+1)*tI2CCLK = 20 ns

			SCLDEL = 1100 = 12
			tSCLDEL = (SCLDEL+1)*tPRESC = 260 ns     (Data setup time)

			SDADEL = 0000 = 0
			tSDADEL = (SDADEL+1)*tPRESC = 20 ns      (Data hold time)

			SCLH = 11101100 = 236
			tSCLH = (SCLH+1)*tPRESC = 4740 ns        (SCL high period)

			SCLL = 11111111 = 255
			tSCLL = (SCLL+1)*tPRESC = 5120 ns        (SCL low period)

			Period = 4740 + 5120 = 9860 ns
			I2C frequency = 101419,88 Hz
		*/

		uint32_t I2C_Clock = HAL_RCC_GetPCLK1Freq(); // assuming all I2C periphirals to be configured with the same clock frequency running at the same frequency as APB1
		uint32_t timingParam = 0x10C00000; // set prescaler, data setup and data hold time as listed above
		uint8_t PRESC =  (timingParam & 0xF0000000) >> 28;
		uint8_t SCLDEL = (timingParam & 0x00F00000) >> 20;
		uint8_t SDADEL = (timingParam & 0x000F0000) >> 16;
		uint16_t period = ((I2C_Clock/(PRESC+1)) / _hRes->frequency) / 2;
		uint16_t periodHigh = period - (SCLDEL+1) - (SDADEL+1);
		uint16_t periodLow = period;
		_hRes->handle.Init.Timing = timingParam | (periodHigh << 8) | periodLow;

		if (HAL_I2C_Init(&_hRes->handle) != HAL_OK)
		{
			_hRes = 0;
			ERROR("Could not initialize I2C port");
			return;
		}

		if (HAL_I2CEx_ConfigAnalogFilter(&_hRes->handle, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
		{
			_hRes = 0;
			ERROR("Could not initialize analog filter for I2C port");
			return;
		}

		if (HAL_I2CEx_ConfigAnalogFilter(&_hRes->handle, 0) != HAL_OK)
		{
			_hRes = 0;
			ERROR("Could not initialize digital filters for I2C port");
			return;
		}
	}
}

void I2C::Write(uint8_t reg, uint8_t value)
{
	Write(reg, &value, 1);
}

void I2C::Write(uint8_t reg, uint8_t * buffer, uint8_t writeLength)
{
	if (!_hRes) return;
	xSemaphoreTake( _hRes->resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource

	// Consider to use task notifications instead: https://www.freertos.org/RTOS-task-notifications.html
	// However using notifications can possibly lead to other problems if multiple objects are going to notify the same task simultaneously
	if (uxSemaphoreGetCount(_hRes->transmissionFinished)) // semaphore is available to be taken - which it should not be at this state before starting the transmission, since we use the semaphore for flagging the finish transmission event
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY ); // something incorrect happened, as the transmissionFinished semaphore should always be taken before a transmission starts

	/*uint8_t * txBuffer = (uint8_t *)pvPortMalloc(writeLength+1);

	if (!txBuffer) return;

	txBuffer[0] = reg;
	memcpy(&txBuffer[1], buffer, writeLength);*/

	if (HAL_I2C_Mem_Write_IT(&_hRes->handle, (uint16_t)_devAddr, reg, I2C_MEMADD_SIZE_8BIT, buffer, writeLength) == HAL_OK)
	{
		// Wait for the transmission to finish
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY );
	} else {
		DEBUG("Failed I2C transmission");
	}

	uint32_t errCode = HAL_I2C_GetError(&_hRes->handle);

	//vPortFree(txBuffer);

	xSemaphoreGive( _hRes->resourceSemaphore ); // give hardware resource back
}

uint8_t I2C::Read(uint8_t reg)
{
	uint8_t rx;
	Read(reg, &rx, 1);
	return rx;
}

void I2C::Read(uint8_t reg, uint8_t * buffer, uint8_t readLength)
{
	if (!_hRes) return;
	xSemaphoreTake( _hRes->resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource

	if (uxSemaphoreGetCount(_hRes->transmissionFinished)) // semaphore is available to be taken - which it should not be at this state before starting the transmission, since we use the semaphore for flagging the finish transmission event
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY ); // something incorrect happened, as the transmissionFinished semaphore should always be taken before a transmission starts

	/*uint8_t * txBuffer = (uint8_t *)pvPortMalloc(readLength+1);
	uint8_t * rxBuffer = (uint8_t *)pvPortMalloc(readLength+1);

	if (!txBuffer || !rxBuffer) return;

	memset(txBuffer, 0, readLength+1);
	txBuffer[0] = reg;*/

	if (HAL_I2C_Mem_Read_IT(&_hRes->handle, (uint16_t)_devAddr, reg, I2C_MEMADD_SIZE_8BIT, buffer, readLength) == HAL_OK)
	{
		// Wait for the transmission to finish
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY );
	} else {
		DEBUG("Failed I2C transmission");
	}

	uint32_t errCode = HAL_I2C_GetError(&_hRes->handle);

	/*memcpy(buffer, &rxBuffer[1], readLength);

	vPortFree(txBuffer);
	vPortFree(rxBuffer);*/

	xSemaphoreGive( _hRes->resourceSemaphore ); // give hardware resource back
}

void I2C1_EV_IRQHandler(void)
{
	if (I2C::resI2C1)
		HAL_I2C_EV_IRQHandler(&I2C::resI2C1->handle);
}
void I2C1_ER_IRQHandler(void)
{
	if (I2C::resI2C1)
		HAL_I2C_ER_IRQHandler(&I2C::resI2C1->handle);
}

void I2C3_EV_IRQHandler(void)
{
	if (I2C::resI2C3)
		HAL_I2C_EV_IRQHandler(&I2C::resI2C3->handle);
}
void I2C3_ER_IRQHandler(void)
{
	if (I2C::resI2C3)
		HAL_I2C_ER_IRQHandler(&I2C::resI2C3->handle);
}

void I2C::TransmissionCompleteCallback(I2C_HandleTypeDef *I2cHandle)
{
	// Tx Transfer completed
	I2C::hardware_resource_t * i2c;
	if (I2cHandle->Instance == I2C1)
		i2c = I2C::resI2C1;
	else if (I2cHandle->Instance == I2C3)
		i2c = I2C::resI2C3;
	else
		return;

	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( i2c->transmissionFinished, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	// Tx Transfer completed
	I2C::TransmissionCompleteCallback(I2cHandle);
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	// Rx Transfer completed
	I2C::TransmissionCompleteCallback(I2cHandle);
}
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	// Tx Transfer completed
	I2C::TransmissionCompleteCallback(I2cHandle);
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	// Rx Transfer completed
	I2C::TransmissionCompleteCallback(I2cHandle);
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
	I2C::TransmissionCompleteCallback(I2cHandle);
}
