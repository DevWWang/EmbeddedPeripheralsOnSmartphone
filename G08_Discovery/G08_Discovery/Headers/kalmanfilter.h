/**
  ******************************************************************************
  * @file    kalmanfilter.h
  * @author  ECSE-426 Group 08
  * @version V1.0.0
  * @date    01-April-2016
  * @brief   Header file of TIM.c 
  ******************************************************************************
*/

	
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef kalmanfilter_H
#define kalmanfilter_H

/** 
 * @brief Initial kalman filter parameters' values
 */
#define DEF_q						0.01
#define DEF_r						0.1
#define DEF_x						0
#define DEF_p						0.1  
#define DEF_k						0.0

/** 
 * @brief Structure containing kalman filter parameter
 */
struct Kalmanfilter_c 
{
	float q; //process noise covariance
	float r; //measurement noise covariance
	float x; //estimated value
	float p; //estimation error covariance
	float k; //adaptive Kalman filter gain	
};

/* Exported functions --------------------------------------------------------*/

/* Configuration ***********************************/
void init_KF(float q, float r, float x, float p, float k, struct Kalmanfilter_c* KFState);

/* Kalman Filter operation functions ******************************************************/
float update_KF(float measurement, struct Kalmanfilter_c* KFState);


#endif /*__kalmanfilter_H */
