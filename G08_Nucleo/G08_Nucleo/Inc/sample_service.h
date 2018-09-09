/**
  ******************************************************************************
  * @file    sample_service.h
  * @author  ECSE-426 Group 08
  * @version V1.0.0
  * @date    07-April-2016
  * @brief   
  ******************************************************************************
  */
	
/* Define to prevent recursive inclusion -------------------------------------*/  
#ifndef __SAMPLE_SERVICE
#define __SAMPLE_SERVICE

/* Includes ------------------------------------------------------------------*/
#include "cube_hal.h"
#include "hal_types.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap.h"
#include "string.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "hci_const.h"
#include "gp_timer.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_aci_const.h"   
#include "hci.h"
#include "hal.h"
#include "sm.h"
#include "debug.h"

#include <stdlib.h>

#define IDB04A1 0
#define IDB05A1 1


/** @addtogroup SAMPLE_SERVICE_Exported_Functions
 *  @{
 */
tBleStatus Add_Double_Tap_Service(void);
tBleStatus Free_Fall_Notify(void);
tBleStatus Add_LED_Service(void);
tBleStatus Add_Environmental_Sensor_Service(void);
void       setConnectable(void);
void       enableNotification(void);
void       GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
void       GAP_DisconnectionComplete_CB(void);
void       HCI_Event_CB(void *pckt);
void 			 Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data);

#endif /* __SAMPLE_SERVICE_H */
