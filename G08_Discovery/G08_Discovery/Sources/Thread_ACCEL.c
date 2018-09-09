/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"  
#include "supporting_functions.h"
#include "lis3dsh.h"

#include "Thread_ACCEL.h"
#include "kalmanfilter.h"
#include "TIM.h"

// For calculating pitch/roll
#include "math.h"
#define PI		3.14159265

// Thread declaration
osThreadId tid_Thread_ACCEL;                              // thread id
osThreadDef(Thread_ACCEL, osPriorityAboveNormal, 1, 800);	// thread name, priority, instances, stack size

// Private Variables
float AccelOutput[3];
float CalibratedAO[3];
float FilteredAO[3];
// interrupt freq 25hz
int CALIBRATEDXYZ;
// Private Variables
float PitchOutput;
float RollOutput;

// NON PRIVATE VARS for display
uint16_t angleRoll;
uint16_t anglePitch;

// FOR DOUBLE TAP
int TAP;
int DOUBLETAP;

float previous[3];

// Private struct init type for accel
LIS3DSH_InitTypeDef AccelInit;
LIS3DSH_DRYInterruptConfigTypeDef AccelInterrupt;
GPIO_InitTypeDef AccelGPIO; 
/**
   * @brief start the Accelerometer thread
	 * @param None
   * @retval None
   */
int start_Thread_ACCEL (void) 
{
  tid_Thread_ACCEL = osThreadCreate(osThread(Thread_ACCEL ), NULL); // Start LED_Thread
	//printf("Accelerometer: 0x%08x \n",tid_Thread_ACCEL);
  if (!tid_Thread_ACCEL) return(-1); 
  return(0);
}
/**
   * @brief Accelerometer Thread
	 * @param None
   * @retval None
   */
void Thread_ACCEL (void const *argument) 
{
		// Init State Vars
		float q = DEF_q;
		float r = DEF_r;
		float x = DEF_x;
		float p = DEF_p;
		float k = DEF_k;	
		// Init Filter Struct for all 3 axis
		struct Kalmanfilter_c KFStateX;
		struct Kalmanfilter_c KFStateY;
		struct Kalmanfilter_c KFStateZ;
		
		// Init the KFilter State for all 3 axis
		init_KF(q, r, x, p, k, &KFStateX);
		init_KF(q, r, x, p, k, &KFStateY);
		init_KF(q, r, x, p, k, &KFStateZ);
	
		// FOR FINAL PROJECT
		previous[0] = 0;
		previous[1] = 0;
		previous[2] = 1000;	
		
		while(1){
			// wait for self at 25hz
			osSignalWait(0x0001, osWaitForever);
			
			LIS3DSH_ReadACC(AccelOutput);
			BetterCaliXYZ(AccelOutput, CalibratedAO);
					
			// Update KFilters for more stable values
			FilteredAO[0] = update_KF(CalibratedAO[0], &KFStateX);
			FilteredAO[1] = update_KF(CalibratedAO[1], &KFStateY);
			FilteredAO[2] = update_KF(CalibratedAO[2], &KFStateZ);
				
			// See if a tap has been found
			tapFound(FilteredAO, previous);			
				
			// the previous update has to be after the tap found	
			previous[0] = FilteredAO[0];
			previous[1] = FilteredAO[1];
			previous[2] = FilteredAO[2];						
				
			// give pitch and roll angles
			PitchOutput = calcPitch(FilteredAO);	
			RollOutput = calcRoll(FilteredAO);
					
			anglePitch = (uint16_t)((PitchOutput + 90) * 100); // for display
			angleRoll = (uint16_t)((RollOutput + 90) * 100); // for display
		}
	}
/**
   * @brief A function used to configurate the accelerometer
	 * @param None
   * @retval None
   */
void initializeACCEL_IO(void)
{
	// Enable the clock for Port E
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	// Init LIS3DSH (accel) struct
	AccelInit.Power_Mode_Output_DataRate = LIS3DSH_DATARATE_25; 	// 25Hz
	AccelInit.Axes_Enable = LIS3DSH_XYZ_ENABLE; 									// all 3 directions enable
	AccelInit.Continous_Update = LIS3DSH_ContinousUpdate_Disabled;// disable this for MSB/LSB
	AccelInit.AA_Filter_BW = LIS3DSH_AA_BW_50; 										// TA told us 50 is fine (low pass)
	AccelInit.Full_Scale = LIS3DSH_FULLSCALE_2; 									// +- 2g total / more precision
	
	// Use accel init funtion
	LIS3DSH_Init(&AccelInit);
	
	// Init LIS3DSH (accel interrupt) struct
	AccelInterrupt.Dataready_Interrupt = LIS3DSH_DATA_READY_INTERRUPT_ENABLED;	// Enable interrupt 
	AccelInterrupt.Interrupt_signal = LIS3DSH_ACTIVE_HIGH_INTERRUPT_SIGNAL;			// Active high interrupt
	AccelInterrupt.Interrupt_type = LIS3DSH_INTERRUPT_REQUEST_PULSED;						// Pulsed (non-latch) type
	
	// User accel interrupt function
	LIS3DSH_DataReadyInterruptConfig(&AccelInterrupt);
	
	// GPIO struct for interrupt signal - EXTI_Line0 (PE0)
	AccelGPIO.Pin = GPIO_PIN_0;							// Pin 0 for PE0
	AccelGPIO.Mode = GPIO_MODE_IT_FALLING; 	// Falling edge interrupt
	AccelGPIO.Pull = GPIO_NOPULL;						// No pull because I don't think it does anything to the outside
	AccelGPIO.Speed = GPIO_SPEED_MEDIUM; 		// Medium speed for 25Hz
	
	// Init the GPIO
	HAL_GPIO_Init(GPIOE, &AccelGPIO);
	
	// Enable interrupts on Line0	
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 1);
}

/**
   * @brief A function used to calibrate the acceleration using average offsets from P1
	 * @param Takes in an XYZ acceleration vector and a Calibrated vector (to return)
   * @retval None
   */
void CalibrateXYZ(float* value, float* calibrated)
{
	float OFFSETX = 1.426888548057262;
	float OFFSETY = 7.889645194274019;
	float OFFSETZ = 1019.964970070552 - 1000;
	
	calibrated[0] = value[0] - OFFSETX;
	calibrated[1] = value[1] - OFFSETY;
	calibrated[2] = value[2] - OFFSETZ;
}
/**
   * @brief A function used to calibrate the acceleration using All Px positions
	 * @param Takes in an XYZ acceleration vector and a Calibrated vector (to return)
   * @retval None
   */
void BetterCaliXYZ(float* value, float* calibrated)
{
	// After getting all X with least square method we get these values
	// 0.985722303638867;			-0.0156810488029380;		0.0122582349342607
  // -0.0283712429004606;		0.992028798330257;			-0.0123347118561540
	// -0.00483318731562110;	-0.0157062347955642;		0.986790400008687
	// 0.00167972764024671;		0.00259456495217067;		-0.00638219455126809
	
	calibrated[0] = (float) (0.985722303638867*value[0] + -0.0283712429004606*value[1] + -0.00483318731562110*value[2] + 0.00167972764024671*1000);
	calibrated[1] = (float) (-0.0156810488029380*value[0] + 0.992028798330257*value[1] + -0.0157062347955642*value[2] + 0.00259456495217067*1000);
	calibrated[2] = (float) (0.0122582349342607*value[0] + -0.0123347118561540*value[1] + 0.986790400008687*value[2] + -0.00638219455126809*1000);
}
/**
   * @brief A function used to calculate the pitch
	 * @param Takes in an XYZ acceleration vector
   * @retval Returns the pitch in degrees
   */
float calcPitch(float* xyz)
{
	float pitch;
	// x = 0 , y = 1, z = 2
	pitch = atan( xyz[0] / sqrt(pow(xyz[1], 2) + pow(xyz[2], 2)) ) * 180/PI;
	
	return pitch;
}
/**
   * @brief A function used to calculate the roll
	 * @param Takes in an XYZ acceleration vector
   * @retval Returns the roll in degrees
   */
float calcRoll(float* xyz)
{
	float roll;
	// x = 0 , y = 1, z = 2
	roll = atan( xyz[1] / sqrt(pow(xyz[0], 2) + pow(xyz[2], 2)) ) * 180/PI;
	
	return roll;
}
/**
   * @brief A function used to configurate the IRQ Handler for PE0 / EXTI0
	 * @param None
   * @retval None
   */
void EXTI0_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);	
}
/**
   * @brief A function used to configurate the callback from EXTI 
   * This function calls a calibration function. 
	 * @param Takes in a pin to check if it's the one we need
   * @retval None
   */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0) // check if pins are correct
	{		
		osSignalSet(tid_Thread_ACCEL, 0x0001);
	}
}

// ======================================= DOUBLE TAP STARTS FROM HERE

/**
   * @brief A function used to see a tap 
   * The difference is needed and a threshold of the difference of 2 values
   * Recall that it's at 25samples/sec so don't forget to X25 for threshold and difference
	 * @param Takes in values XYZ, sets a flag if "tap" is found
   * @retval None
   */
void tapFound(float* value, float* previous)
{
	float Xprev = previous[0];
	float Yprev = previous[1];
	float Zprev = previous[2];
	
	float Xthis = value[0];
	float Ythis = value[1];
	float Zthis = value[2];
	
	float Xdiff = (Xthis - Xprev);
	float Ydiff = (Ythis - Yprev);
	float Zdiff = (Zthis - Zprev);
	
	
	if (Xdiff >= (fabs)(40.0) || Ydiff >= (fabs)(40.0) || Zdiff >= (fabs)(30.0))
	{
		TAP++;
		osDelay(80); // dont really want to set a signal of 80ms in TIM
	}
	// could have a signal here to notify double tap instead of a flag/integer value
	if (TAP == 2 && msecond <= 1500000)
	{
		TAP = 0;
		DOUBLETAP++ ;
		msecond = 0;
	}
	
}

