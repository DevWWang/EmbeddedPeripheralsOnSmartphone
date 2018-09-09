/**
  ******************************************************************************
  * @file    spi.c 
  * @author  Group 08
  * @version V1.0.0
  * @date    01-April-2016
  * @brief   This application contains an example which shows how implementing
  *          a communication between Discovery board and Nucleo board.
  *          The communication is done using SPI perihperal.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "spi.h"

/* Private Variables ---------------------------------------------------------*/
SPI_HandleTypeDef    Spi2Handle;
int Timeout = 500;

/**
  * @brief  Configures Nucleo board (Master) for SPI communication with Discovery board (Slave)
  * @param  None
  * @retval None
  */
void SPI2_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

	/* configure pins used by SPI2
   * PC13 = SCK
   * PC14 = MISO
   * PC15 = MOSI
   */
  
  /* Enable GPIO Pins' clocks */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull  = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
  GPIO_InitStructure.Alternate = GPIO_AF5_SPI2;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.Pin = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.Pin =  GPIO_PIN_14;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* SPI nCS pin configuration */	
  GPIO_InitStructure.Pin   = GPIO_PIN_12;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  
	/* SPI Master Data Ready pin configuration */		
  GPIO_InitStructure.Pin   = GPIO_PIN_1;
  GPIO_InitStructure.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStructure.Mode  = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* SPI Slave Data Ready pin configuration */		
  GPIO_InitStructure.Pin   = GPIO_PIN_2;
  GPIO_InitStructure.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStructure.Mode  = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Enable peripheral clock */
  __HAL_RCC_SPI2_CLK_ENABLE();

  HAL_SPI_DeInit(&Spi2Handle);
  Spi2Handle.Instance 					= SPI2;
  Spi2Handle.Init.BaudRatePrescaler 	= SPI_BAUDRATEPRESCALER_128;
  Spi2Handle.Init.Direction 			= SPI_DIRECTION_2LINES;
  Spi2Handle.Init.CLKPhase 				= SPI_PHASE_1EDGE;
  Spi2Handle.Init.CLKPolarity 			= SPI_POLARITY_LOW;
  Spi2Handle.Init.CRCCalculation		= SPI_CRCCALCULATION_DISABLED;
  Spi2Handle.Init.CRCPolynomial 		= 7;
  Spi2Handle.Init.DataSize 				= SPI_DATASIZE_16BIT;
  Spi2Handle.Init.FirstBit 				= SPI_FIRSTBIT_MSB;
  Spi2Handle.Init.NSS 					= SPI_NSS_SOFT;
  Spi2Handle.Init.TIMode 				= SPI_TIMODE_DISABLED;
  Spi2Handle.Init.Mode 					= SPI_MODE_MASTER;
	
  if (HAL_SPI_Init(&Spi2Handle) != HAL_OK) {printf ("ERROR: Error in initialising SPI2 \n");};
  
  __HAL_SPI_ENABLE(&Spi2Handle);
}

/**
  * @brief  Returns the most recent received data by the SPIx/I2Sx peripheral. 
  * @param  *hspi: Pointer to the SPI handle. Its member Instance can point to either SPI1, SPI2 or SPI3 
  * @retval The value of the received data.
  */
uint16_t SPI_ReceiveData(SPI_HandleTypeDef *spi){
	return spi->Instance->DR;
}

/**
  * @brief  Transmits a Data through the SPIx/I2Sx peripheral.
  * @param  *hspi: Pointer to the SPI handle. Its member Instance can point to either SPI1, SPI2 or SPI3 
  * @param  Data: Data to be transmitted.
  * @retval None
  */
void SPI_SendData(SPI_HandleTypeDef *spi, uint16_t Data){
	spi->Instance->DR = Data;
}

/**
  * @brief  Receive 2 bytes through the SPI interface and return the Bytes received
  *         from the SPI bus.
  * @retval The received value in hexadecimal
  */
uint16_t SPI_Master(uint16_t byte){
	uint16_t data;

	/* Verify the Data Ready Signal */
	while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2) != GPIO_PIN_SET){
		if((Timeout--) == 0){
			Timeout = 500;
			return 0xdead;
		}
	}
	
	/* Select the Slave */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	/* Wait for Transmit buffer Empty */
	while (__HAL_SPI_GET_FLAG(&Spi2Handle, SPI_FLAG_TXE) == 0x00000000)
	{}
	SPI_SendData(&Spi2Handle, byte);
	/* Wait for Receive buffer Not Empty */
	while (__HAL_SPI_GET_FLAG(&Spi2Handle, SPI_FLAG_TXE) == 0x00000000)
	{}
	while (__HAL_SPI_GET_FLAG(&Spi2Handle, SPI_FLAG_RXNE) == 0x00000000)
	{}
	data = SPI_ReceiveData(&Spi2Handle);
	/* Verify SPI Busy flag */
	while(__HAL_SPI_GET_FLAG(&Spi2Handle, SPI_FLAG_BSY) != 0x00000000)
	{}
	/* Wait for Transmission to complete */
	while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2) == GPIO_PIN_SET);
	
	/* Unselect the Slave */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	
	return data;
}
