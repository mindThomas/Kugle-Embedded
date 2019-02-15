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

#include "ProcessorInit.h"
#include "main.h"
#include "cmsis_os.h"
#include "Debug.h" // for Error_Handler
#include "usbd_core.h"

/* Configure clocks to:
	- 400 MHz CPU clock
	- 200 MHz timer periphiral clock
	- 80 MHz SPI periphiral clock
	- 100 MHZ UART periphiral clock
	- 100 MHZ I2C periphiral clock
	- 100 MHz ADC periphiral clock
*/

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Supply configuration update enable 
  */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);
  /**Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY) 
  {
    
  }
  /**Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
      //Error_Handler();
	  HAL_Delay(200);
	  NVIC_SystemReset(); // reset and try again
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
      //Error_Handler();
	  HAL_Delay(200);
	  NVIC_SystemReset(); // reset and try again
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_UART7|RCC_PERIPHCLK_SPI3
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_I2C3
                              |RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_SPI6|RCC_PERIPHCLK_I2C4
                              |RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 50;
  PeriphClkInitStruct.PLL2.PLL2P = 5;
  PeriphClkInitStruct.PLL2.PLL2Q = 5;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.PLL3.PLL3M = 1;
  PeriphClkInitStruct.PLL3.PLL3N = 42;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 7;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
      //Error_Handler();
	  HAL_Delay(200);
	  NVIC_SystemReset(); // reset and try again
  }
}

void Enter_DFU_Bootloader(void)
{
	//call this at any time to initiate a reboot into bootloader
	//__disable_irq();

	// Important to stop current USB periphiral before entering DFU bootloader (see "EnterBootloader_Callback" in MainTask.cpp)

	//HAL_SuspendTick();
	/* Disable SysTick Interrupt */
    //SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    //taskENTER_CRITICAL();
    //portDISABLE_INTERRUPTS();
    *BOOTLOADER_MAGIC_ADDR = BOOTLOADER_MAGIC_TOKEN;
    SCB_InvalidateDCache_by_Addr((uint32_t *) BOOTLOADER_MAGIC_ADDR, 4);

    SCB_CleanDCache();
	SCB_DisableDCache();

	//(*(__IO uint32_t *) (BKPSRAM_BASE + 0)) = A_VALUE;
    NVIC_SystemReset();
}
