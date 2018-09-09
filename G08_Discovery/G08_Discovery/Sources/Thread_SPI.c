/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX

#include "stm32f4xx_hal.h"
#include "main.h"
#include "TIM.h"
#include "Thread_ADCTemp.h"
#include "Thread_ACCEL.h"
#include "Thread_SPI.h"

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef Spi3Handle;

// Threading
void Thread_SPI (void const *argument);                 // thread function
osThreadId tid_Thread_SPI;                              // thread id
osThreadDef(Thread_SPI, osPriorityNormal, 1, 0);	// thread name, priority, instances, stack size
extern int brightness;
extern int ledspinspeed;
uint16_t ledattr;

//local variables
int Timeout = 500;
int temp;
extern int DOUBLETAP;

/**
  * @brief  Configures Discovery board for SPI communication with Nucleo board
  * @param  None
  * @retval None
  */
void initializeSPI_IO(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	//SPI_HandleTypeDef    SpiHandle;

// enable peripheral clock
    __HAL_RCC_SPI3_CLK_ENABLE();

    // enable clock for used IO pins
    __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOE_CLK_ENABLE();
    /* configure pins used by SPI3
     * PC10 = SCK
     * PC11 = MISO
     * PC12 = MOSI
     */
  GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull  = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
  GPIO_InitStructure.Alternate = GPIO_AF6_SPI3;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.Pin = GPIO_PIN_10;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.Pin =  GPIO_PIN_11;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.Pin = GPIO_PIN_12;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

 /* SPI nCS pin configuration */
		
	GPIO_InitStructure.Pin   = GPIO_PIN_15;
  GPIO_InitStructure.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
 /* SPI Data Ready pin configuration */
		
  GPIO_InitStructure.Pin   = GPIO_PIN_1;
  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
	
	GPIO_InitStructure.Pin   = GPIO_PIN_2;
  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	__SPI3_CLK_ENABLE();
	
	HAL_SPI_DeInit(&Spi3Handle);
  Spi3Handle.Instance 							  = SPI3;
  Spi3Handle.Init.BaudRatePrescaler 	= SPI_BAUDRATEPRESCALER_4;
  Spi3Handle.Init.Direction 					= SPI_DIRECTION_2LINES;
  Spi3Handle.Init.CLKPhase 					= SPI_PHASE_1EDGE;
  Spi3Handle.Init.CLKPolarity 				= SPI_POLARITY_LOW;
  Spi3Handle.Init.CRCCalculation			= SPI_CRCCALCULATION_DISABLED;
  Spi3Handle.Init.CRCPolynomial 			= 7;
  Spi3Handle.Init.DataSize 					= SPI_DATASIZE_16BIT;
  Spi3Handle.Init.FirstBit 					= SPI_FIRSTBIT_MSB;
  Spi3Handle.Init.NSS 								= SPI_NSS_SOFT;
  Spi3Handle.Init.TIMode 						= SPI_TIMODE_DISABLED;
  Spi3Handle.Init.Mode 							= SPI_MODE_SLAVE;
	if (HAL_SPI_Init(&Spi3Handle) != HAL_OK) {printf ("ERROR: Error in initialising SPI3 \n");};
  
	__HAL_SPI_ENABLE(&Spi3Handle);
}

/**
  * @brief  Returns the most recent received data by the SPIx/I2Sx peripheral. 
  * @param  *hspi: Pointer to the SPI handle. Its member Instance can point to either SPI1, SPI2 or SPI3 
  * @retval The value of the received data.
  */
uint16_t SPI3_ReceiveData(){
  /* Return the data in the DR register */
  return Spi3Handle.Instance->DR;
}

/**
  * @brief  Transmits a Data through the SPIx/I2Sx peripheral.
  * @param  *hspi: Pointer to the SPI handle. Its member Instance can point to either SPI1, SPI2 or SPI3 
  * @param  Data: Data to be transmitted.
  * @retval None
  */
void SPI3_SendData( uint16_t data){ 
  /* Write in the DR register the data to be sent */
	Spi3Handle.Instance->DR = data;
}

/**
  * @brief  Send 2 Byte through the SPI interface and return the Byte received
  *         from the SPI bus.
  * @param  Byte : Byte send.
  * @retval The received byte value
  */
uint16_t SPI3_Slave(uint16_t byte){
	uint16_t data;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
	while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15) == GPIO_PIN_SET){
		if((Timeout--) == 0) 
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
			Timeout = 500;
			return 0xffff;
		}
	}
	while (__HAL_SPI_GET_FLAG(&Spi3Handle, SPI_FLAG_TXE) == 0x00000000)
	{
	}
	SPI3_SendData(byte);
	while (__HAL_SPI_GET_FLAG(&Spi3Handle,SPI_FLAG_RXNE) == RESET);
	data = SPI3_ReceiveData();
		
	while (__HAL_SPI_GET_FLAG(&Spi3Handle, SPI_FLAG_BSY) == SPI_FLAG_BSY)
	{
	}
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
	while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15) == GPIO_PIN_RESET);
	return data;
}

/**
   * @brief  Start the Thread
   * @param  None
   * @retval Exit code
   */
int start_Thread_SPI (void) 
{

  tid_Thread_SPI = osThreadCreate(osThread(Thread_SPI), NULL); // Start Thread
	if (!tid_Thread_SPI) return(-1); 
  return(0);
}

/**
   * @brief  A function used send data to and receive data from Nucleo Board using SPI (main.c)
   * @param  Void const (no change in argument passed in)
   * @retval None
   */
void Thread_SPI (void const *argument) 
{
	while(1){
		osSignalWait(0x0003, osWaitForever);
		
		if(temp ==0){
			ledattr = SPI3_Slave(0x0000);
			while(ledattr == 0xffff){
				ledattr = SPI3_Slave(0x0000);
			}

			ledattr = SPI3_Slave(tempSent);
			while(ledattr == 0xffff){
				ledattr = SPI3_Slave(tempSent);
			}
			brightness = ledattr/256;
			ledspinspeed = (int)(ledattr%256-10);
			temp++;
		}
		if(temp ==1){
			ledattr = SPI3_Slave(0x0001);
			while(ledattr == 0xffff){
				ledattr = SPI3_Slave(0x0001);
			}
			ledattr = SPI3_Slave(anglePitch);
			while(ledattr == 0xffff){
				ledattr = SPI3_Slave(anglePitch);
			}
			brightness = ledattr/256;
			ledspinspeed = (int)(ledattr%256-10);
			temp++;
		}
		if(temp ==2){
			ledattr = SPI3_Slave(0x0002);
			while(ledattr == 0xffff){
				ledattr = SPI3_Slave(0x0002);
			}
			ledattr = SPI3_Slave(angleRoll);
			while(ledattr == 0xffff){
				ledattr = SPI3_Slave(angleRoll);
			}
			brightness = ledattr/256;
			ledspinspeed = (int)(ledattr%256-10);
			temp++;
		}
		if(temp ==3){
			ledattr = SPI3_Slave(0x0003);
			while(ledattr == 0xffff){
			ledattr = SPI3_Slave(0x0003);
			}
		//if taptemp is not equal to doubletap then it means user has doubletapped the board
			if(DOUBLETAP >=1){
				while(SPI3_Slave(0xbbbb) == 0xffff){
					}
				printf("DoubleTap!\n");
				DOUBLETAP = 0;
				}
			else{
				while(SPI3_Slave(0xaaaa) == 0xffff){
				}
			}
			brightness = ledattr/256;
			ledspinspeed = (int)(ledattr%256-10);
			temp = 0;
		}
		
	}
}
