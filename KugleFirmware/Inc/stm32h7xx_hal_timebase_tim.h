#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

uint32_t HAL_GetTickTimerValue(void);
uint32_t HAL_GetHighResTick(void);
void HAL_DelayHighRes(uint32_t Delay);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
