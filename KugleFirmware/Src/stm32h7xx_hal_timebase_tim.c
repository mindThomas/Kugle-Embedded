/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_hal_timebase_TIM.c 
  * @brief   HAL time base based on the hardware TIM.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"
 
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef        htim16; 
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static __IO uint32_t uwTickHighRes;
static uint32_t frequency;

/**
  * @brief  This function configures the TIM16 as a time base source. 
  *         The time source is configured  to have 1ms time base with a dedicated 
  *         Tick interrupt priority. 
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig(). 
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock = 0;
  uint32_t              uwPrescalerValue = 0;
  uint32_t              pFLatency;
  
  /*Configure the TIM16 IRQ priority */
  HAL_NVIC_SetPriority(TIM16_IRQn, TickPriority ,0); 
  
  /* Enable the TIM16 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM16_IRQn); 
  
  /* Enable TIM16 clock */
  __HAL_RCC_TIM16_CLK_ENABLE();
  
  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
  
  /* Compute TIM16 clock */
  uwTimclock = 2*HAL_RCC_GetPCLK2Freq();
   
  frequency = 10000; // 10 kHz

  /* Compute the prescaler value to have TIM16 counter clock equal to 10 kHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / frequency) - 1);
  
  /* Initialize TIM16 */
  htim16.Instance = TIM16;
  
  /* Initialize TIMx peripheral as follow:
  + Period = [(TIM16CLK/1000) - 1]. to have a (1/1000) s time base.
  + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
  + ClockDivision = 0
  + Counter direction = Up
  */
  // Modified by Thomas: Configure timer to overflow every second, thus firing an interrupt @ 1 Hz rate
  htim16.Init.Period = (10000 / 1) - 1;
  htim16.Init.Prescaler = uwPrescalerValue;
  htim16.Init.ClockDivision = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&htim16) == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    return HAL_TIM_Base_Start_IT(&htim16);
  }
  
  /* Return function status */
  return HAL_ERROR;
}

uint32_t HAL_GetTickTimerValue(void)
{
	return (uint32_t)__HAL_TIM_GET_COUNTER(&htim16);
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM16 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Disable TIM16 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim16, TIM_IT_UPDATE);                                                  
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TIM16 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
  /* Enable TIM16 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim16, TIM_IT_UPDATE);
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim16);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM16) {
    HAL_IncTick();
  }
}

/**
  * @brief This function is called to increment  a global variable "uwTick"
  *        used as application time base.
  * @note In the default implementation, this variable is incremented each 1ms
  *       in Systick ISR.
 * @note This function is declared as __weak to be overwritten in case of other
  *      implementations in user file.
  * @retval None
  */
void HAL_IncTick(void)
{
	uwTickHighRes += 10000; // timer update rate configured 1 Hz but with timer count frequency running at 10 kHz
}

uint32_t HAL_GetHighResTick(void)
{
	return (uwTickHighRes + HAL_GetTickTimerValue());
}

uint32_t HAL_tic()
{
	return HAL_GetHighResTick();
}

float HAL_toc(uint32_t timerPrev)
{
	uint32_t timerDelta;
	uint32_t timerNow = HAL_GetHighResTick();
	if (timerNow > timerPrev)
		timerDelta = timerNow - timerPrev;
	else
		timerDelta = ((uint32_t)0xFFFFFFFF - timerPrev) + timerNow;

	float microsTime = (float)timerDelta / frequency;
	return microsTime;
}

void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /*
  // Add a freq to guarantee minimum wait
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }
  */

  while ((HAL_GetTick() - tickstart) < wait)
  {
  }
}

void HAL_DelayHighRes(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetHighResTick();
  uint32_t wait = Delay;

  /*
  // Add a freq to guarantee minimum wait
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }
  */

  while ((HAL_GetHighResTick() - tickstart) < wait)
  {
  }
}

/**
  * @brief Provides a tick value in millisecond.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @retval tick value
  */
uint32_t HAL_GetTick(void)
{
  return (uwTickHighRes / 10); // divide by 10 since we have configured HAL timer to run at 10 kHz in stm32h7xx_hal_timebase_tim.c
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
