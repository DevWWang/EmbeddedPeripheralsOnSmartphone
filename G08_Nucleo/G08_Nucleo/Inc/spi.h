/**
  ******************************************************************************
  * @file    spi.h
  * @author  ECSE-426 Group 08
  * @version V1.0.0
  * @date    07-April-2016
  * @brief   Header file containing spi communication between Nucleo and Discovery boards
  *          and its basic functions
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef SPI_H
#define SPI_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_spi.h"

/* Exported functions --------------------------------------------------------*/

/* Configuration ***********************************/
void SPI2_init(void);

/* SPI operation functions ******************************************************/
void SPI_SendData(SPI_HandleTypeDef *hspi, uint16_t Data);
uint16_t SPI_ReceiveData(SPI_HandleTypeDef *hspi);
uint16_t SPI_Master(uint16_t byte);

#endif /*__SPI_H */
