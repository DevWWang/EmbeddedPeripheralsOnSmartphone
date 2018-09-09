/**
  ******************************************************************************
  * @file    THREAD_SPI.h
  * @author  ECSE-426 Group 08
  * @version V1.0.0
  * @date    07-April-2016
  * @brief   Header file containing spi communication between Nucleo and Discovery boards
  *          and its basic functions
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef THREAD_SPI_H
#define THREAD_SPI_H

/* Include data Extension module */
#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_spi.h"

/* Exported functions --------------------------------------------------------*/
extern SPI_HandleTypeDef Spi3Handle;

/* Thread ***********************************/
int start_Thread_SPI(void);
void Thread_SPI(void const *argument);

/* Configuration ***********************************/
void initializeSPI_IO(void);

/* SPI operation functions ******************************************************/
uint16_t SPI3_ReceiveData(void);
void SPI3_SendData(uint16_t data);
uint16_t SPI3_Slave(uint16_t byte);

#endif /*__THREAD_SPI_H */
