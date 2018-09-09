/**
  ******************************************************************************
  * @file    THREAD_ADCTemp.h
  * @author  ECSE-426 Group 08
  * @version V1.0.0
  * @date    02-April-2016
  * @brief   Header file containing ADC configuration and its basic functions
  ******************************************************************************
  */
	
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef THREAD_ADCTemp_H
#define THREAD_ADCTemp_H

/* Include data Extension module */
#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_adc.h"

/* Extern variables */
extern ADC_HandleTypeDef ADC1_Handle;
extern ADC_InitTypeDef ADC_InitStructure;
extern ADC_ChannelConfTypeDef ADC_Channel;

extern uint16_t tempSent;

/* Exported functions --------------------------------------------------------*/

/* Thread ***********************************/
int start_Thread_ADCTemp(void);
void Thread_ADCTemp (void const *argument);

/* Configuration ***********************************/
void ConfigADC(void);
void initializeADCTemp_IO(void);

/* ADC operation functions ******************************************************/
extern void ADC_Init(ADC_HandleTypeDef* hadc);
float ConvertTemp(int);

#endif /*__THREAD_ADCTemp_H */
