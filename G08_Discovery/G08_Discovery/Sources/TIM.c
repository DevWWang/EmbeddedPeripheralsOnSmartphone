 /* ******************************************************************************
  * File Name          : TIM.c
  * Description        : ST's TIM hardware timer's configuration and initializatin
	* Author						 : Yan Qi
	* Version            : 1.0.0
	* Date							 : Mar 4th, 2016
  ******************************************************************************
  */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "TIM.h"
#include "main.h"
#include "Thread_ACCEL.h"
#include "Thread_LED.h"

/* Private Variables ---------------------------------------------------------*/
int ten;
int hundred;
extern int brightness;

// FOR FINAL PROJECT
int msecond;	// start count
int second; 
int timcounter =0;

TIM_HandleTypeDef TIM3_Handle;

/**
  * @brief  Initialize the TIM
  * @param  TIM_Handle(TIM_HandleTypeDef)
  * @retval None
  */	
void TimInit(TIM_HandleTypeDef *TIM_Handle)
{
	__TIM3_CLK_ENABLE();
	//16 bit for prescaler and Period -> max value for both: 65555
	// PRESCALAR x PERIOD / 84 x10^6 = ms/cycle
	TIM_Handle -> Init.Prescaler = 10;		// 100,840 -> 1 ms/cycle		
	TIM_Handle -> Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM_Handle -> Init.Period = 84;  			
	TIM_Handle -> Instance = TIM3; //use TIM 3, GP
	
	HAL_TIM_Base_Init(TIM_Handle); // initiate TIM
	HAL_TIM_Base_Start_IT(TIM_Handle); // start interrupts
		
	HAL_NVIC_SetPriority(TIM3_IRQn,0,1); // TIM3's priority is highest
	HAL_NVIC_EnableIRQ(TIM3_IRQn);	// 
}

/**
  * @brief  Period Elapased Call Back
  * @param  TIM_Handle(TIM_HandleTypeDef)
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	timcounter++;
	BrightnessControl();
	if (ten == 1000)
	{
		// 100hz for ADC
		osSignalSet(tid_Thread_ADCTemp, 0x0002);
		ten = 0;
	}
	if (hundred == 2500)
	{
		// 10hz for SPI
		osSignalSet(tid_Thread_SPI, 0x0003);
		hundred=0;
	}
	ten++;
	hundred++;
	
	// double tap of 1.5s max
	if (TAP == 1)
	{
		msecond++;
	}		
	// could have a signal instead of a flag in here to notify Double Tap towards somewhere
	if (msecond == 1500000)
	{	
		TAP = 0;
		msecond = 0;
	}
}

/**
  * @brief  Setup TIM3 Handler
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&TIM3_Handle);
}
