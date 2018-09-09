#include "kalmanfilter.h"

/**
   * @brief A function used to initialize the KalmanFilter
	 * @param Takes in all the struct parameters and a Filter State to initialize
   * @retval None
   */
void init_KF(float q, float r, float x, float p, float k, struct Kalmanfilter_c* KFState) 
{	
	KFState->q = q;
	KFState->r = r; 
	KFState->x = x;
	KFState->p = p; 
	KFState->k = k;
};
/**
   * @brief A function used to update the filter like in python, returns the filtered x value
	 * @param Takes in a measurement (x) and the Filter State 
   * @retval The filtered value
   */
float update_KF(float measurement, struct Kalmanfilter_c* KFState)
{
	KFState->p = KFState->p + KFState->q;
	KFState->k = KFState->p / (KFState->p + KFState->r);
	KFState->x = KFState->x + KFState->k * (measurement - KFState->x);
	KFState->p = (1 - KFState->k) * KFState->p;
	
	return KFState->x;
};
