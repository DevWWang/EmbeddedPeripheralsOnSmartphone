/*******************************************************************************
  * @file    Thread_LED.c
  * @author  ECSE-426 Group 08
	* @version V1.0.0
  * @date    12-April-2016
  * @brief   This file initializes one LED as an output, implements the LED thread 
  *					 which toggles and LED, and function which creates and starts the thread	
  ******************************************************************************
  */
	
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "stm32f4xx_hal.h"
#include "Thread_LED.h"
#include "Thread_ACCEL.h"
#include "math.h"

//void Thread_LED (void const *argument);                 // thread function
osThreadId tid_Thread_LED;                              // thread id
osThreadDef(Thread_LED, osPriorityNormal, 1, 0);		// not configured
GPIO_InitTypeDef 				LED_configuration;
GPIO_InitTypeDef 				GPIO_InitDef; // for LED LAB3

int ledspinspeed;
int brightness;
int ledflag;
int count;

extern int timcounter;

/*----------------------------------------------------------------------------
 *      Create the thread within RTOS context
 *---------------------------------------------------------------------------*/
int start_Thread_LED (void) {

  tid_Thread_LED = osThreadCreate(osThread(Thread_LED ), NULL); // Start LED_Thread
  if (!tid_Thread_LED) return(-1); 
  return(0);
}

/*----------------------------------------------------------------------------
 *      Initialize the GPIO associated with the LED
 *---------------------------------------------------------------------------*/
	void initializeLED_IO (void){
	
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
	LED_configuration.Pin		= GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	LED_configuration.Mode 	= GPIO_MODE_OUTPUT_PP;
	LED_configuration.Speed	= GPIO_SPEED_FREQ_VERY_HIGH;
	LED_configuration.Pull	= GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &LED_configuration);	
}

/*----------------------------------------------------------------------------
 *      Functions needed for PWM/LEDSPIN
 *---------------------------------------------------------------------------*/

// looking when connector is down
#define GREEN		GPIO_PIN_12	// right
#define ORANGE	GPIO_PIN_13	// down
#define RED			GPIO_PIN_14	// left
#define	BLUE		GPIO_PIN_15	// up

/**
	* @brief initalize the led's clock and GPIO 
	* @param Port D, GPIO_InitDef
	* @retval None
*/
void ledInit(void)
{
	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitDef.Pin = GREEN | ORANGE | RED | BLUE;
	GPIO_InitDef.Mode 	= GPIO_MODE_OUTPUT_PP; 
	GPIO_InitDef.Pull 	= GPIO_NOPULL;
	GPIO_InitDef.Speed 	= GPIO_SPEED_MEDIUM; 
	HAL_GPIO_Init(GPIOD, &GPIO_InitDef);
}

/**
	* @brief Turn Off 4 Led lights
	* @retval None
*/
void ledOFF()
{ 
	HAL_GPIO_WritePin(GPIOD, GREEN ,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, ORANGE ,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, RED ,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, BLUE ,GPIO_PIN_RESET);
}

/**
	* @brief Turn On 4 Led lights
	* @retval None
*/
void ledON()
{ 
	HAL_GPIO_WritePin(GPIOD, GREEN ,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, ORANGE ,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, RED ,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, BLUE ,GPIO_PIN_SET);
}


/**
	* @brief Different LED light up scenarioes
  * @param	ledtag: choice of scenario
	* @retval None
*/
void Scenario_Signal(int ledtag){
	if(ledtag == 0){
		HAL_GPIO_WritePin(GPIOD, GREEN ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, ORANGE ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, RED ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, BLUE ,GPIO_PIN_RESET);
	}
	else if(ledtag == 1){
		HAL_GPIO_WritePin(GPIOD, GREEN ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, ORANGE ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, RED ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, BLUE ,GPIO_PIN_SET);
	}
	else if(ledtag == 2){
		HAL_GPIO_WritePin(GPIOD, GREEN ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, ORANGE ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, RED ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, BLUE ,GPIO_PIN_RESET);	
	}
	else if(ledtag == 3){
		HAL_GPIO_WritePin(GPIOD, GREEN ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, ORANGE ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, RED ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, BLUE ,GPIO_PIN_RESET);
	}
	else if(ledtag == 6){
		ledON();
	}
}

/**
	* @brief Contol LED light's brightness
	* @param None
	* @retval None
*/
void BrightnessControl(void){
	count = timcounter%100;
		if(count<brightness){
			Scenario_Signal(ledflag);
		}
		else{
		ledOFF();
		}
}
/*----------------------------------------------------------------------------
 *       Thread  'LED_Thread': Toggles LED
 *---------------------------------------------------------------------------*/
	void Thread_LED (void const *argument) {
		while(1){
				if(ledspinspeed > 0){
					if(timcounter >= ((11 - ledspinspeed) * 5000)){
						timcounter = 0;
						ledflag--;
						if(ledflag <= -1){
							ledflag = 3;
						}
					}
				}
				else if(ledspinspeed < 0){
					if(timcounter>= ((11 + ledspinspeed) * 5000)){
						timcounter = 0;
						ledflag++;
						if(ledflag >= 4){
							ledflag = 0;
						}
					}
				}
				else{
					ledflag = 6;
						if(timcounter >= 1000){
							timcounter =0;
						}
				}
			}
}
