/**
  ******************************************************************************
  * @file    Thread_LED.h
  * @author  ECSE-426 Group 08
  * @version V1.0.0
  * @date    11-April-2016
  * @brief   Header file containing LED configuration and its basic functions
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef THREAD_LED_H
#define THREAD_LED_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"	

/* Exported functions --------------------------------------------------------*/

/* Thread ***********************************/
int start_Thread_LED (void);
void Thread_LED (void const *argument);

/* Configuration ***********************************/
void ledInit(void);
void initializeLED_IO (void);

/* LED operation functions ******************************************************/
void ledOFF(void);
void ledON(void);
void Scenario_Signal(int ledtag);
void BrightnessControl(void);

#endif /*__THREAD_LED_H */
