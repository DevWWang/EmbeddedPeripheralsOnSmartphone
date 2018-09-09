/**
  ******************************************************************************
  * @file    Thread_ACCEL.h
  * @author  ECSE-426 Group 08
  * @version V1.0.0
  * @date    08-April-2016
  * @brief   Header file containing accelerometer configuration and its basic functions
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef THREAD_ACCEL_H
#define THREAD_ACCEL_H

/* Include data Extension module */
#include "main.h"
#include "cmsis_os.h"  
#include "stm32f4xx.h"
#include "lis3dsh.h"

/* Extern variables */
extern uint16_t anglePitch;
extern uint16_t angleRoll;
extern int TAP;
extern int DOUBLETAP;

/* Exported functions --------------------------------------------------------*/

/* Thread ***********************************/
int start_Thread_ACCEL (void);
void Thread_ACCEL (void const *argument);

/* Configuration ***********************************/
void initializeACCEL_IO(void);

/* Accelerometer operation functions ******************************************************/
void CalibrateXYZ(float* value, float* calibrated);
void BetterCaliXYZ(float* value, float* calibrated);
float calcPitch(float* xyz);
float calcRoll(float* xyz);
void tapFound(float* value, float* previous);


#endif /*__THREAD_ACCEL_H */
