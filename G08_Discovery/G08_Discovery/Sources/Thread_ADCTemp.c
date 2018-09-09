/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX

#include "stm32f4xx_hal.h"
#include "Thread_ADCTemp.h"
#include "kalmanfilter.h"
#include "main.h"
#include "TIM.h"

// KalmanFilter_C 
#define DEF_qt					0.001		
#define DEF_rt					0.1
#define DEF_xt					25
#define DEF_pt					0.1                                      
#define DEF_kt					0.0
// overheat temp
//#define OVERHEAT				35.0f

// Initialize the struct of the ADC Handler
ADC_HandleTypeDef ADC1_Handle;
// Initialize the struct of the ADC Init
ADC_InitTypeDef ADC_InitStructure;
// Initialize ADC channel config
ADC_ChannelConfTypeDef ADC_Channel;
// Init ADC Get Values
uint32_t getADCValue;

// Threading
void Thread_ADCTemp (void const *argument);                 // thread function
osThreadId tid_Thread_ADCTemp;                              // thread id
osThreadDef(Thread_ADCTemp, osPriorityAboveNormal, 1, 400);	// thread name, priority, instances, stack size

//local variables
float temperature;
float filteredTemp;
uint16_t tempSent;
/**
   * @brief A function used to setup the ADC to sample Channel 16
	 * @param  None
   * @retval None
   */
void initializeADCTemp_IO()
{
	__HAL_RCC_ADC1_CLK_ENABLE();
	// Populate the struct ADC Handle
	// Instance
	ADC1_Handle.Instance = ADC1;
	// Init struct 
	ADC1_Handle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV6; // Clock Prescaler at 168/4Mhz
	ADC1_Handle.Init.Resolution = ADC_RESOLUTION_12B; 							// Biggest resolution for best accuracy
	ADC1_Handle.Init.ScanConvMode = DISABLE;												// One channel only so no scan 
	ADC1_Handle.Init.ContinuousConvMode = DISABLE;									// Continuous conversion for stream of data
	ADC1_Handle.Init.DiscontinuousConvMode = DISABLE;								// Want to be continuous conversion
	ADC1_Handle.Init.NbrOfDiscConversion = 0;												// No discontinuous conversions (even though it should be 1-8, I don't have the discontConvMode ON)
	ADC1_Handle.Init.ExternalTrigConvEdge = ADC_SOFTWARE_START;			// Disable external events
	ADC1_Handle.Init.ExternalTrigConv= ADC_SOFTWARE_START;					// Disable external edges
	ADC1_Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;								// Align the data to the right
	ADC1_Handle.Init.NbrOfConversion = 1;														// Just 1 conversion for continuous is ok
	ADC1_Handle.Init.DMAContinuousRequests = DISABLE;								// We don't have DMA, but it should be ENABLE if we use continuous stream
	ADC1_Handle.Init.EOCSelection = DISABLE;												// Don't use the EOC flag
	// Set Init
	HAL_ADC_Init(&ADC1_Handle);
	// Channel config
	ADC_Channel.Channel = ADC_CHANNEL_16;														// Set to CH16
	ADC_Channel.SamplingTime = ADC_SAMPLETIME_480CYCLES; 						// Must be bigger than 15Cycles to work correctly (for us)
	ADC_Channel.Rank = 1;																						// Rank 1 (in group sequencer) & no offset
	ADC_Channel.Offset = 0;
	// Set Channel
	HAL_ADC_ConfigChannel(&ADC1_Handle, &ADC_Channel);
	
}

/**
   * @brief A function used to convert the ADC (V) to Temperature (C)
	 * @param  ADC Value in Voltage
   * @retval A converted Temperature in C
   */
float ConvertTemp(int getADCValue)
{	
	// T in C = (Vsense - V25)/avgSlope + 25
	// V25 = 0.76C; avgSlope = 2.5mV/C
	
	float tValueC;
	float normalize = 0;
	
	tValueC = getADCValue;
	tValueC *= 3000.0f;	// 3V over 4095
	tValueC /= 0xfff;
	tValueC /= 1000.0f;	// 1k to try and keep more precision
	tValueC -= 0.760f;	// V25
	tValueC /= .0025f;	// avgSlope
	tValueC += 25.0f;		// Room Temp
	tValueC -= normalize; // If we want to normalize the temperature at some value
		
	return tValueC;
}

/**
   * @brief  Start the Thread
   * @param  None
   * @retval Exit code
   */
int start_Thread_ADCTemp (void) 
{

  tid_Thread_ADCTemp = osThreadCreate(osThread(Thread_ADCTemp), NULL); // Start Thread
  //printf("ADCTemp: 0x%08x \n",tid_Thread_ADCTemp);
	if (!tid_Thread_ADCTemp) return(-1); 
  return(0);
}

/**
   * @brief  A function used get the filtered temperature then pass it to the global var filteredTemp (main.c)
   * @param  Void const (no change in argument passed in)
   * @retval None
   */
void Thread_ADCTemp (void const *argument) 
{
	
	// Init Struct
	struct Kalmanfilter_c KFStateT;
	// Init State Vars
	float q = DEF_qt;
	float r = DEF_rt;
	float x = DEF_xt;
	float p = DEF_pt;
	float k = DEF_kt;
	init_KF(q, r, x, p, k, &KFStateT);
	
	while(1){		
			// wait for 100hz
			osSignalWait(0x0002, osWaitForever);
		
			//start ADC
			HAL_ADC_Start(&ADC1_Handle);
		
			//get the latest temperature data
			if (HAL_ADC_PollForConversion(&ADC1_Handle, 100) == HAL_OK){
				//CLEAR EOC FLAG
				__HAL_ADC_CLEAR_FLAG(&ADC1_Handle, ADC_FLAG_EOC);			
			
				getADCValue = HAL_ADC_GetValue(&ADC1_Handle);		
				temperature = ConvertTemp(getADCValue);
				filteredTemp = update_KF(temperature, &KFStateT);
				tempSent = (uint16_t)(filteredTemp * 100);
			}
	}
}
