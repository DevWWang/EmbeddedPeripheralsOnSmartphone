/**
  ******************************************************************************
  * @file    TIM.h
  * @author  ECSE-426 Group 08
  * @version V1.0.0
  * @date    01-April-2016
  * @brief   Header file of TIM.c 
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H
#define __TIM_H

/* Include data Extension module */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

/* Extern Variables */
extern TIM_HandleTypeDef TIM3_Handle;
extern int msecond;	// start count
extern int second; 

/* Exported functions --------------------------------------------------------*/

/* Configuration ***********************************/
void TIM3_IRQHandler(void);
void TimInit(TIM_HandleTypeDef *TIM_Handle);

/* TIM operation functions ******************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /*__TIM_H */
