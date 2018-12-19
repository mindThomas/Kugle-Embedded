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
 
#include "UART.h"
#include "stm32h7xx_hal.h"
#include "Debug.h"
 
UART * UART::objUART3 = 0;
UART * UART::objUART4 = 0;
UART * UART::objUART7 = 0;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" __EXPORT void USART3_IRQHandler(void);
extern "C" __EXPORT void UART4_IRQHandler(void);
extern "C" __EXPORT void UART7_IRQHandler(void);

UART::UART(port_t port, uint32_t baud, uint32_t bufferLength) : _port(port), _baud(baud), _bufferLength(bufferLength), _bufferWriteIdx(0), _bufferReadIdx(0), RXcallback(0)
{
	if (_bufferLength > 0)
		_buffer = (uint8_t *)pvPortMalloc(_bufferLength);
	else
		_buffer = 0;
	InitPeripheral();
	ConfigurePeripheral();
}

UART::UART(port_t port, uint32_t baud) : UART(port, baud, 0)
{
}

UART::~UART()
{
	DeInitPeripheral();
	if (_buffer)
		vPortFree(_buffer);
}

void UART::ConfigurePeripheral()
{
	switch (_port) {
		case PORT_UART3:
			if (objUART3) {
				ERROR("UART3 already in used");
				return;
			}
			_handle.Instance = USART3;
			break;
		case PORT_UART4:
			if (objUART4) {
				ERROR("UART4 already in used");
				return;
			}
			_handle.Instance = UART4;
			break;
		case PORT_UART7:
			if (objUART7) {
				ERROR("UART7 already in used");
				return;
			}
			_handle.Instance = UART7;
			break;
		default:
			ERROR("Undefined UART port");
			return;
	}

	_handle.Init.BaudRate = _baud;
	_handle.Init.WordLength = UART_WORDLENGTH_8B;
	_handle.Init.StopBits = UART_STOPBITS_1;
	_handle.Init.Parity = UART_PARITY_NONE;
	_handle.Init.Mode = UART_MODE_TX_RX;
	_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	_handle.Init.OverSampling = UART_OVERSAMPLING_16;
	_handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	_handle.Init.Prescaler = UART_PRESCALER_DIV1;
	_handle.Init.FIFOMode = UART_FIFOMODE_DISABLE;
	_handle.Init.TXFIFOThreshold = UART_TXFIFO_THRESHOLD_1_8;
	_handle.Init.RXFIFOThreshold = UART_RXFIFO_THRESHOLD_1_8;
	_handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_UART_Init(&_handle) != HAL_OK)
	{
		ERROR("Could not initialize UART port");
		return;
	}

	switch (_port) {
		case PORT_UART3:
			objUART3 = this;
			break;
		case PORT_UART4:
			objUART4 = this;
			break;
		case PORT_UART7:
			objUART7 = this;
			break;
		default:
			break;
	}

	_resourceSemaphore = xSemaphoreCreateBinary();
	if (_resourceSemaphore == NULL) {
		ERROR("Could not create UART resource semaphore");
		return;
	}
	vQueueAddToRegistry(_resourceSemaphore, "UART Resource");

	// Create binary semaphore for indicating when a single byte has finished transmitting (for flagging to the transmit thread)
	TransmitByteFinished = xSemaphoreCreateBinary();
	if (TransmitByteFinished == NULL) {
		ERROR("Could not create UART transmit finished semaphore");
		return;
	}
	vQueueAddToRegistry(TransmitByteFinished, "UART Finished");
	xSemaphoreGive( TransmitByteFinished ); // give the semaphore the first time

	RXdataAvailable = xSemaphoreCreateBinary();
	if (RXdataAvailable == NULL) {
		ERROR("Could not create UART RX available semaphore");
		return;
	}
	vQueueAddToRegistry(RXdataAvailable, "UART RX Available");

    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    SET_BIT(_handle.Instance->CR3, USART_CR3_EIE);

    /* Enable the UART Parity Error interupt and RX FIFO Threshold interrupt
       (if FIFO mode is enabled) or Data Register Not Empty interrupt
       (if FIFO mode is disabled).
    */
    if (READ_BIT(_handle.Instance->CR1, USART_CR1_FIFOEN) != RESET)
    {
      SET_BIT(_handle.Instance->CR1, USART_CR1_PEIE);
      SET_BIT(_handle.Instance->CR3, USART_CR3_RXFTIE);
    }
    else
    {
      SET_BIT(_handle.Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);
    }

    /* Disable the UART Transmit Complete Interrupt */
    CLEAR_BIT(_handle.Instance->CR1, USART_CR1_TCIE);

    __HAL_UART_CLEAR_OREFLAG(&_handle);
}

void UART::InitPeripheral()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if(_port == PORT_UART4)
	{
		/* Peripheral clock enable */
		__HAL_RCC_UART4_CLK_ENABLE();

		__HAL_RCC_GPIOD_CLK_ENABLE();
		/**UART4 GPIO Configuration
		PD0     ------> UART4_RX
		PD1     ------> UART4_TX
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		/* UART4 interrupt Init */
		HAL_NVIC_SetPriority(UART4_IRQn, UART_INTERRUPT_PRIORITY, 0);
		HAL_NVIC_EnableIRQ(UART4_IRQn);
	}
	else if(_port == PORT_UART7)
	{
		/* Peripheral clock enable */
		__HAL_RCC_UART7_CLK_ENABLE();

		__HAL_RCC_GPIOE_CLK_ENABLE();
		/**UART7 GPIO Configuration
		PE7     ------> UART7_RX
		PE8     ------> UART7_TX
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF7_UART7;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

		/* UART7 interrupt Init */
		HAL_NVIC_SetPriority(UART7_IRQn, UART_INTERRUPT_PRIORITY, 0);
		HAL_NVIC_EnableIRQ(UART7_IRQn);
	}
	else if(_port == PORT_UART3)
	{
		/* Peripheral clock enable */
		__HAL_RCC_USART3_CLK_ENABLE();

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**USART3 GPIO Configuration
		PB10     ------> USART3_TX
		PB11     ------> USART3_RX
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* USART3 interrupt Init */
		HAL_NVIC_SetPriority(USART3_IRQn, UART_INTERRUPT_PRIORITY, 0);
		HAL_NVIC_EnableIRQ(USART3_IRQn);
	}
}

void UART::DeInitPeripheral()
{
  if(_port == PORT_UART3)
  {
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PD0     ------> UART4_RX
    PD1     ------> UART4_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* UART4 interrupt DeInit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  }
  else if(_port == PORT_UART7)
  {
    /* Peripheral clock disable */
    __HAL_RCC_UART7_CLK_DISABLE();

    /**UART7 GPIO Configuration
    PE7     ------> UART7_RX
    PE8     ------> UART7_TX
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_7|GPIO_PIN_8);

    /* UART7 interrupt DeInit */
    HAL_NVIC_DisableIRQ(UART7_IRQn);
  }
  else if(_port == PORT_UART3)
  {
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  }

}

void UART::RegisterRXcallback(void (*callback)UART_CALLBACK_PARAMS)
{
	RXcallback = callback;
	_callbackChunkLength = 1;

	switch (_port) {
		case PORT_UART3:
			xTaskCreate(UART::CallbackThread, (char *)"UART3 callback", 128, (void*) this, 3, &_callbackTaskHandle);
			break;
		case PORT_UART4:
			xTaskCreate(UART::CallbackThread, (char *)"UART4 callback", 128, (void*) this, 3, &_callbackTaskHandle);
			break;
		case PORT_UART7:
			xTaskCreate(UART::CallbackThread, (char *)"UART7 callback", 128, (void*) this, 3, &_callbackTaskHandle);
			break;
		default:
			break;
	}
}

void UART::RegisterRXcallback(void (*callback)UART_CALLBACK_PARAMS, uint32_t chunkLength)
{
	RXcallback = callback;
	_callbackChunkLength = chunkLength;

	switch (_port) {
		case PORT_UART3:
			xTaskCreate(UART::CallbackThread, (char *)"UART3 callback", 128, (void*) this, 3, &_callbackTaskHandle);
			break;
		case PORT_UART4:
			xTaskCreate(UART::CallbackThread, (char *)"UART4 callback", 128, (void*) this, 3, &_callbackTaskHandle);
			break;
		case PORT_UART7:
			xTaskCreate(UART::CallbackThread, (char *)"UART7 callback", 128, (void*) this, 3, &_callbackTaskHandle);
			break;
		default:
			break;
	}
}

void UART::TransmitBlocking(uint8_t * buffer, uint32_t bufLen)
{
    /* Enable the TX FIFO threshold interrupt (if FIFO mode is enabled) or
       Transmit Data Register Empty interrupt (if FIFO mode is Disabled).
    */
    if (READ_BIT(_handle.Instance->CR1, USART_CR1_FIFOEN) != RESET)
    {
      SET_BIT(_handle.Instance->CR3, USART_CR3_TXFTIE);
    }
    else
    {
      SET_BIT(_handle.Instance->CR1, USART_CR1_TXEIE);
    }

	do {
		xSemaphoreTake( TransmitByteFinished, ( TickType_t ) portMAX_DELAY ); // block until it has finished sending the byte
		_handle.Instance->TDR = *buffer++;
	    /* Enable the TX FIFO threshold interrupt (if FIFO mode is enabled) or
	       Transmit Data Register Empty interrupt (if FIFO mode is Disabled).
	    */
	    if (READ_BIT(_handle.Instance->CR1, USART_CR1_FIFOEN) != RESET)
	    {
	      SET_BIT(_handle.Instance->CR3, USART_CR3_TXFTIE);
	    }
	    else
	    {
	      SET_BIT(_handle.Instance->CR1, USART_CR1_TXEIE);
	    }
	} while (--bufLen > 0);

    /* Disable the TX FIFO threshold interrupt (if FIFO mode is enabled) or
       Transmit Data Register Empty interrupt (if FIFO mode is Disabled).
    */
    if (READ_BIT(_handle.Instance->CR1, USART_CR1_FIFOEN) != RESET)
    {
      CLEAR_BIT(_handle.Instance->CR3, USART_CR3_TXFTIE);
    }
    else
    {
      CLEAR_BIT(_handle.Instance->CR1, USART_CR1_TXEIE);
    }
}

void UART::TransmitBlockingHard(uint8_t * buffer, uint32_t bufLen)
{
	do {
		//if(UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TXE, RESET, tickstart, Timeout) != HAL_OK)
		while (__HAL_UART_GET_FLAG(&_handle, UART_FLAG_TXE) == RESET);
		_handle.Instance->TDR = *buffer++;
	} while (--bufLen > 0);
}

void UART::Write(uint8_t byte)
{
	xSemaphoreTake( _resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource

	// OBS! This should be replaced with a non-blocking call by making a TX queue and a processing thread in the UART object
	TransmitBlocking(&byte, 1);

	xSemaphoreGive( _resourceSemaphore ); // give hardware resource back
}

uint32_t UART::Write(uint8_t * buffer, uint32_t length)
{
	xSemaphoreTake( _resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource

	TransmitBlocking(buffer, length);

	xSemaphoreGive( _resourceSemaphore ); // give hardware resource back
	return length;
}

int16_t UART::Read()
{
	if (!_bufferLength) return -1; // error, buffer not enabled
	if (BufferContentSize() == 0) return -1; // error, buffer is empty
	return BufferPop();
}

bool UART::Available()
{
	if (!_bufferLength) return false; // error, buffer not enabled
	return (_bufferWriteIdx != _bufferReadIdx);
}

void UART::CallbackThread(void * pvParameters)
{
	UART * uart = (UART *)pvParameters;

	while (1) {
		vTaskSuspend(NULL); // suspend current thread - this could also be replaced by semaphore-based waiting (flagging)

		if (uart->RXcallback) {
			if (uart->_callbackChunkLength == 1) {
				uart->RXcallback(&uart->rxByte, 1);
			}
			else if (uart->BufferContentSize() >= uart->_callbackChunkLength)
			{
				uint8_t * chunkBuffer = uart->BufferPopN(uart->_callbackChunkLength);
				if (chunkBuffer) {
					uart->RXcallback(chunkBuffer, uart->_callbackChunkLength);
					vPortFree(chunkBuffer);
				}
			}
		}
	}
}

void UART::UART_IncomingDataInterrupt(UART * uart)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	// Only call this function is RXNE interrupt flag is set
	if (!uart) {
		ERROR("UART interrupt for unconfigured port");
		return;
	}

	uint16_t uhdata = (uint16_t) READ_REG(uart->_handle.Instance->RDR);
	uart->rxByte = uhdata & 0x00FF;
    /* Clear RXNE interrupt flag */
    //__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST); // should already have been cleared by reading

	uart->BufferPush(uart->rxByte); // push into local buffer
	if (uart->RXcallback)
		xTaskResumeFromISR(uart->_callbackTaskHandle);

  if (uart->RXdataAvailable)
	  xSemaphoreGiveFromISR( uart->RXdataAvailable, &xHigherPriorityTaskWoken );

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

uint32_t UART::WaitForNewData(uint32_t xTicksToWait) // blocking call
{
	return xSemaphoreTake( RXdataAvailable, ( TickType_t ) xTicksToWait );
}


void USART3_IRQHandler(void)
{
	UART::UART_Interrupt(UART::PORT_UART3);
}

void UART4_IRQHandler(void)
{
	UART::UART_Interrupt(UART::PORT_UART4);
}

void UART7_IRQHandler(void)
{
	UART::UART_Interrupt(UART::PORT_UART7);
}

void UART::UART_Interrupt(port_t port)
{
	UART * uart = 0;
	switch (port) {
		case PORT_UART3:
			uart = objUART3;
			break;
		case PORT_UART4:
			uart = objUART4;
			break;
		case PORT_UART7:
			uart = objUART7;
			break;
		default:
			break;
	}

	if (!uart) {
		ERROR("UART interrupt for unconfigured port");
		return;
	}

	uint32_t isrflags   = READ_REG(uart->_handle.Instance->ISR);
	uint32_t cr1its     = READ_REG(uart->_handle.Instance->CR1);
	uint32_t cr3its     = READ_REG(uart->_handle.Instance->CR3);
	uint32_t errorflags;

	/* If no error occurs */
	errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
	if (errorflags == RESET)
	{
		/* UART in mode Receiver ---------------------------------------------------*/
		if(((isrflags & USART_ISR_RXNE) != RESET)
			&& (   ((cr1its & USART_CR1_RXNEIE) != RESET)
				|| ((cr3its & USART_CR3_RXFTIE) != RESET)) )
		{
			//UART_Receive_IT(huart);
			UART_IncomingDataInterrupt(uart);
			return;
		}
	}

	/* If some errors occur */
	if(   (errorflags != RESET)
		&& (   ((cr3its & (USART_CR3_RXFTIE | USART_CR3_EIE)) != RESET)
			|| ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE)) != RESET)))
	{
		/* UART parity error interrupt occurred -------------------------------------*/
		if(((isrflags & USART_ISR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET))
		{
			__HAL_UART_CLEAR_IT(&uart->_handle, UART_CLEAR_PEF);

			uart->_handle.ErrorCode |= HAL_UART_ERROR_PE;
		}

		/* UART frame error interrupt occurred --------------------------------------*/
		if(((isrflags & USART_ISR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
		{
			__HAL_UART_CLEAR_IT(&uart->_handle, UART_CLEAR_FEF);

			uart->_handle.ErrorCode |= HAL_UART_ERROR_FE;
		}

		/* UART noise error interrupt occurred --------------------------------------*/
		if(((isrflags & USART_ISR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
		{
			__HAL_UART_CLEAR_IT(&uart->_handle, UART_CLEAR_NEF);

			uart->_handle.ErrorCode |= HAL_UART_ERROR_NE;
		}

		/* UART Over-Run interrupt occurred -----------------------------------------*/
		if(   ((isrflags & USART_ISR_ORE) != RESET)
		   &&(  ((cr1its & USART_CR1_RXNEIE) != RESET) ||
				((cr3its & USART_CR3_RXFTIE) != RESET) ||
				((cr3its & USART_CR3_EIE) != RESET)) )
		{
			__HAL_UART_CLEAR_IT(&uart->_handle, UART_CLEAR_OREF);

			uart->_handle.ErrorCode |= HAL_UART_ERROR_ORE;
		}

		/* Call UART Error Call back function if need be --------------------------*/
		if(uart->_handle.ErrorCode != HAL_UART_ERROR_NONE)
		{
			/* UART in mode Receiver ---------------------------------------------------*/
			if(((isrflags & USART_ISR_RXNE) != RESET)
			&& (   ((cr1its & USART_CR1_RXNEIE) != RESET)
			|| ((cr3its & USART_CR3_RXFTIE) != RESET)) )
			{
				//UART_Receive_IT(huart);
				UART_IncomingDataInterrupt(uart);
			}

			/* If Overrun error occurs, or if any error occurs in DMA mode reception,
			consider error as blocking */
			if (((uart->_handle.ErrorCode & HAL_UART_ERROR_ORE) != RESET) ||
			(HAL_IS_BIT_SET(uart->_handle.Instance->CR3, USART_CR3_DMAR)))
			{
				/* Blocking error : transfer is aborted
				Set the UART state ready to be able to start again the process,
				Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
				//UART_EndRxTransfer(huart);

				/* Disable the UART DMA Rx request if enabled */
				if (HAL_IS_BIT_SET(uart->_handle.Instance->CR3, USART_CR3_DMAR))
				{
					CLEAR_BIT(uart->_handle.Instance->CR3, USART_CR3_DMAR);

					/* Abort the UART DMA Rx channel */
					if(uart->_handle.hdmarx != NULL)
					{
						/* Set the UART DMA Abort callback :
						will lead to call HAL_UART_ErrorCallback() at end of DMA abort procedure */
						//huart->hdmarx->XferAbortCallback = UART_DMAAbortOnError;

						/* Abort DMA RX */
						if(HAL_DMA_Abort_IT(uart->_handle.hdmarx) != HAL_OK)
						{
							/* Call Directly huart->hdmarx->XferAbortCallback function in case of error */
							uart->_handle.hdmarx->XferAbortCallback(uart->_handle.hdmarx);
						}
					}
					else
					{
						/* Call user error callback */
						//HAL_UART_ErrorCallback(huart);
					}
				}
				else
				{
					/* Call user error callback */
					//HAL_UART_ErrorCallback(huart);
				}
			}
			else
			{
				/* Non Blocking error : transfer could go on.
				Error is notified to user through user error callback */
				//HAL_UART_ErrorCallback(huart);
				uart->_handle.ErrorCode = HAL_UART_ERROR_NONE;
			}
		}
		return;

	} /* End if some error occurs */

	/* UART wakeup from Stop mode interrupt occurred ---------------------------*/
	if(((isrflags & USART_ISR_WUF) != RESET) && ((cr3its & USART_CR3_WUFIE) != RESET))
	{
		__HAL_UART_CLEAR_IT(&uart->_handle, UART_CLEAR_WUF);
		/* Set the UART state ready to be able to start again the process */
		uart->_handle.gState  = HAL_UART_STATE_READY;
		uart->_handle.RxState = HAL_UART_STATE_READY;
		//HAL_UARTEx_WakeupCallback(huart);
		return;
	}

	/* UART in mode Transmitter ------------------------------------------------*/
	if(((isrflags & USART_ISR_TXE) != RESET)
	&& (   ((cr1its & USART_CR1_TXEIE) != RESET)
	|| ((cr3its & USART_CR3_TXFTIE) != RESET)) )
	{
		//UART_Transmit_IT(huart);
	    /* Disable the TX FIFO threshold interrupt (if FIFO mode is enabled) or
	       Transmit Data Register Empty interrupt (if FIFO mode is Disabled).
	    */
	    if (READ_BIT(uart->_handle.Instance->CR1, USART_CR1_FIFOEN) != RESET)
	    {
	      CLEAR_BIT(uart->_handle.Instance->CR3, USART_CR3_TXFTIE);
	    }
	    else
	    {
	      CLEAR_BIT(uart->_handle.Instance->CR1, USART_CR1_TXEIE);
	    }

		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR( uart->TransmitByteFinished, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		return;
	}

	/* UART in mode Transmitter (transmission end) -----------------------------*/
	if(((isrflags & USART_ISR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
	{
		//UART_EndTransmit_IT(huart);
		__HAL_UART_CLEAR_IT(&uart->_handle, UART_CLEAR_TCF);

		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR( uart->TransmitByteFinished, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		return;
	}

	/* UART TX FIFO Empty  -----------------------------------------------------*/
	if(((isrflags & USART_ISR_TXFE) != RESET) && ((cr1its & USART_CR1_TXFEIE) != RESET))
	{
		CLEAR_BIT(uart->_handle.Instance->CR1, USART_CR1_TXFEIE);
	}
}

void UART::BufferPush(uint8_t byte)
{
	if (!_bufferLength) return; // error, buffer not enabled
	if (BufferContentSize() >= _bufferLength) return; // error, buffer is full

	_buffer[_bufferWriteIdx] = byte;
	_bufferWriteIdx++;
	if (_bufferWriteIdx == _bufferLength) _bufferWriteIdx = 0;
}

uint8_t UART::BufferPop()
{
	if (!_bufferLength) return 0; // error, buffer not enabled
	if (BufferContentSize() == 0) return 0; // error, buffer is empty

	uint8_t byte = _buffer[_bufferReadIdx];
	_bufferReadIdx++;
	if (_bufferReadIdx == _bufferLength) _bufferReadIdx = 0;

	return byte;
}

uint32_t UART::BufferContentSize()
{
	if (!_bufferLength) return 0; // error, buffer not enabled
	uint32_t length = (_bufferWriteIdx - _bufferReadIdx) % _bufferLength;

	return length;
}

uint8_t * UART::BufferPopN(uint32_t numberOfBytesToPop)
{
	uint8_t * popBuffer = 0;
	if (numberOfBytesToPop > BufferContentSize()) return 0; // error, not enough content in buffer
	popBuffer = (uint8_t *)pvPortMalloc(numberOfBytesToPop);

	if (popBuffer) {
		for (uint32_t i = 0; i < numberOfBytesToPop; i++) {
			popBuffer[i] = BufferPop();
		}
	}

	return popBuffer;
}
