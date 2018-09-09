/**
  ******************************************************************************
  * @file    sample_service.c
  * @author  Group 08
  * @version V1.0.0
  * @date    12-April-2016
  * @brief   Add three customized services using a vendor specific profile.
	*					 - Double Tap Service: double tap notify handle
	* 				 - LED Service : LED handle, including PWM and spin speed
	* 				 - Environmental Sensor Service : temperature handle, pitch handle, roll handle
  ******************************************************************************
  */
#include "sample_service.h"

/** @addtogroup X-CUBE-BLE1_Applications
 *  @{
 */

/** @addtogroup SensorDemo
 *  @{
 */
 
/** @defgroup SENSOR_SERVICE
 * @{
 */

/** @defgroup SENSOR_SERVICE_Private_Variables
 * @{
 */
/* Private variables ---------------------------------------------------------*/
volatile int connected = FALSE;
volatile uint8_t set_connectable = 1;
volatile uint16_t connection_handle = 0;
volatile uint8_t notification_enabled = FALSE;
volatile uint16_t ledAttr;

extern uint16_t buf[3];
extern uint16_t temp, pitch, roll;
extern uint8_t bnrg_expansion_board;

uint16_t ledServHandle, ledCharHandle;
uint16_t dtServHandle, doubleTapCharHandle;
uint16_t envSensServHandle, tempCharHandle, pitchCharHandle, rollCharHandle;


/**
 * @}
 */

/** @defgroup SENSOR_SERVICE_Private_Macros
 * @{
 */
/* Private macros ------------------------------------------------------------*/
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

  #define COPY_DT_SERVICE_UUID(uuid_struct)  			 COPY_UUID_128(uuid_struct,0x02,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
  #define COPY_DOUBLE_TAP_UUID(uuid_struct)    		 COPY_UUID_128(uuid_struct,0xe2,0x3e,0x78,0xa0, 0xcf,0x4a, 0x11,0xe1, 0x8f,0xfc, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)

	#define COPY_SAMPLE_SERVICE_UUID(uuid_struct)  	 COPY_UUID_128(uuid_struct,0x82,0x63,0xe6,0x08, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
  #define COPY_SAMPLE_CHAR_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x34,0x0a,0x1b,0x80, 0xcf,0x4b, 0x11,0xe1, 0xac,0x36, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)

  #define COPY_ENV_SENS_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x42,0x82,0x1a,0x40, 0xe4,0x77, 0x11,0xe2, 0x82,0xd0, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
  #define COPY_TEMP_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0xa3,0x2e,0x55,0x20, 0xe4,0x77, 0x11,0xe2, 0xa9,0xe3, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
  #define COPY_PITCH_CHAR_UUID(uuid_struct)        COPY_UUID_128(uuid_struct,0xcd,0x20,0xc4,0x80, 0xe4,0x8b, 0x11,0xe2, 0x84,0x0b, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
  #define COPY_ROLL_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x01,0xc5,0x0b,0x60, 0xe4,0x8c, 0x11,0xe2, 0xa0,0x73, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* Store Value into a buffer in Little Endian Format */
#define STORE_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )
/**
 * @}
 */

/** @defgroup SAMPLE_SERVICE_Exported_Functions 
 * @{
 */ 
/**
 * @brief  Add a double tap service using a vendor specific profile.
 *
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_Double_Tap_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];
  
  COPY_DT_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 7,
                          &dtServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;    
  
  COPY_DOUBLE_TAP_UUID(uuid);
  ret =  aci_gatt_add_char(dtServHandle, UUID_TYPE_128, uuid, 1,
                           CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0,
                           16, 0, &doubleTapCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  PRINTF("Service Double Tap added. Handle 0x%04X, DOUBLE TAP Charac handle: 0x%04X\n", dtServHandle, doubleTapCharHandle);	
  return BLE_STATUS_SUCCESS; 
  
fail:
  PRINTF("Error while adding Double Tap service.\n");
  return BLE_STATUS_ERROR ;
    
}

/**
 * @brief  Send a notification for a Free Fall detection.
 *
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Free_Fall_Notify(void)
{  
  uint8_t val;
  tBleStatus ret;
	
  val = 0x01;	
  ret = aci_gatt_update_char_value(dtServHandle, doubleTapCharHandle, 0, 1,
                                   &val);
	
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating DOUBLE TAP characteristic.\n") ;
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;	
}

/**
* @brief  Add LED service using a vendor specific profile.
 *
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_LED_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];
  
  COPY_SAMPLE_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 7,
                          &ledServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;    
  
  COPY_SAMPLE_CHAR_UUID(uuid);  
  ret =  aci_gatt_add_char(ledServHandle, UUID_TYPE_128, uuid, 2,
                           CHAR_PROP_WRITE|CHAR_PROP_WRITE_WITHOUT_RESP,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE,
                           16, 0, &ledCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
 
  PRINTF("Service LED added. Handle 0x%04X, LED Charac handle: 0x%04X\n", ledServHandle, ledCharHandle);	
  return BLE_STATUS_SUCCESS; 
  
fail:
  PRINTF("Error while adding LED service.\n");
  return BLE_STATUS_ERROR ;
    
}

/**
 * @brief  Update led characteristic value.
 *
 * @param  Structure containing led attribute data 
 * @retval Status
 */
void LED_Update(uint8_t* sample)
{  
  ledAttr = ((sample[0] << 8) | sample[1]);
	PRINTF("Update LED Char Value: %X\n", ledAttr);
}

/**
 * @brief  Add the Environmental service including ADC and accelerometer.
 *
 * @param  None
 * @retval Status
 */
tBleStatus Add_Environmental_Sensor_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];
  uint16_t uuid16;
  charactFormat charFormat;
  uint16_t descHandle;
  
  COPY_ENV_SENS_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 10,
                          &envSensServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  /* Temperature Characteristic */
  COPY_TEMP_CHAR_UUID(uuid);  
  ret =  aci_gatt_add_char(envSensServHandle, UUID_TYPE_128, uuid, 2,
                           CHAR_PROP_READ, ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &tempCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  charFormat.format = FORMAT_UINT16;
  charFormat.exp = -1;
  charFormat.unit = UNIT_TEMP_CELSIUS;
  charFormat.name_space = 0;
  charFormat.desc = 0;
  
  uuid16 = CHAR_FORMAT_DESC_UUID;
  
  ret = aci_gatt_add_char_desc(envSensServHandle,
                               tempCharHandle,
                               UUID_TYPE_16,
                               (uint8_t *)&uuid16, 
                               7,
                               7,
                               (void *)&charFormat, 
                               ATTR_PERMISSION_NONE,
                               ATTR_ACCESS_READ_ONLY,
                               0,
                               16,
                               FALSE,
                               &descHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  /* Pitch Angle Characteristic */
  if(1){
    COPY_PITCH_CHAR_UUID(uuid);  
    ret =  aci_gatt_add_char(envSensServHandle, UUID_TYPE_128, uuid, 2,
                           CHAR_PROP_READ, ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &pitchCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
    
	charFormat.format = FORMAT_UINT16;
    charFormat.exp = -2;
    charFormat.unit = UNIT_UNITLESS;
    charFormat.name_space = 0;
    charFormat.desc = 0;
    
    uuid16 = CHAR_FORMAT_DESC_UUID;
    
    ret = aci_gatt_add_char_desc(envSensServHandle,
                                 pitchCharHandle,
                                 UUID_TYPE_16,
                                 (uint8_t *)&uuid16, 
                                 7,
                                 7,
                                 (void *)&charFormat, 
                                 ATTR_PERMISSION_NONE,
                                 ATTR_ACCESS_READ_ONLY,
                                 0,
                                 16,
                                 FALSE,
                                 &descHandle);
    if (ret != BLE_STATUS_SUCCESS) goto fail;
  }    
  /* Roll Angle Characteristic */
  if(1){
    COPY_ROLL_CHAR_UUID(uuid);  
    ret =  aci_gatt_add_char(envSensServHandle, UUID_TYPE_128, uuid, 2,
                             CHAR_PROP_READ, ATTR_PERMISSION_NONE,
                             GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                             16, 0, &rollCharHandle);
    if (ret != BLE_STATUS_SUCCESS) goto fail;
    
    charFormat.format = FORMAT_UINT16;
    charFormat.exp = -2;
    charFormat.unit = UNIT_UNITLESS;
    charFormat.name_space = 0;
    charFormat.desc = 0;
    
    uuid16 = CHAR_FORMAT_DESC_UUID;
    
    ret = aci_gatt_add_char_desc(envSensServHandle,
                                 rollCharHandle,
                                 UUID_TYPE_16,
                                 (uint8_t *)&uuid16, 
                                 7,
                                 7,
                                 (void *)&charFormat, 
                                 ATTR_PERMISSION_NONE,
                                 ATTR_ACCESS_READ_ONLY,
                                 0,
                                 16,
                                 FALSE,
                                 &descHandle);
    if (ret != BLE_STATUS_SUCCESS) goto fail;
  } 
  PRINTF("Service ENV_SENS added. Handle 0x%04X, TEMP Charac handle: 0x%04X, PITCH Charac handle: 0x%04X, ROLL Charac handle: 0x%04X\n",envSensServHandle, tempCharHandle, pitchCharHandle, rollCharHandle);	
  return BLE_STATUS_SUCCESS; 
  
fail:
  PRINTF("Error while adding ENV_SENS service.\n");
  return BLE_STATUS_ERROR ;
  
}

/**
 * @brief  Update temperature characteristic value.
* @param  uint16_t temp: temperature in tenths of degree Celsius
 * @retval Status
 */
tBleStatus Temp_Update(uint16_t temp)
{  
  tBleStatus ret;
  uint8_t buff[2];
  STORE_LE_16(buff, temp);
  
  ret = aci_gatt_update_char_value(envSensServHandle, tempCharHandle, 0, 2,
                                   buff);
  
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating TEMP characteristic.\n") ;
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;
	
}

/**
 * @brief  Update pitch characteristic value.
* @param  uint16_t pitch: pitch angle of discovery board in degree
 * @retval tBleStatus Status
 */
tBleStatus Pitch_Update(uint16_t pitch)
{  
  tBleStatus ret;
  
  ret = aci_gatt_update_char_value(envSensServHandle, pitchCharHandle, 0, 2,
                                   (uint8_t*)&pitch);
  
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating PITCH characteristic.\n") ;
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;
	
}

/**
 * @brief  Update roll characteristic value.
* @param  uint16_t roll: roll angle of discovery board in degree
 * @retval tBleStatus      Status
 */
tBleStatus Roll_Update(uint16_t roll)
{  
  tBleStatus ret;
  
  ret = aci_gatt_update_char_value(envSensServHandle, rollCharHandle, 0, 2,
                                   (uint8_t*)&roll);
  
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating ROLL characteristic.\n") ;
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;
  
}

/**
 * @brief  Puts the device in connectable mode.
 *         If you want to specify a UUID list in the advertising data, those data can
 *         be specified as a parameter in aci_gap_set_discoverable().
 *         For manufacture data, aci_gap_update_adv_data must be called.
 * @param  None 
 * @retval None
 */
/* Ex.:
 *
 *  tBleStatus ret;    
 *  const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','L','E','-','G','0','8'};  
 *  const uint8_t serviceUUIDList[] = {AD_TYPE_16_BIT_SERV_UUID,0x34,0x12};    
 *  const uint8_t manuf_data[] = {4, AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 0x05, 0x02, 0x01};
 *  
 *  ret = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE,
 *                                 8, local_name, 3, serviceUUIDList, 0, 0);    
 *  ret = aci_gap_update_adv_data(5, manuf_data);
 *
 */
void setConnectable(void)
{  
  tBleStatus ret;
  
  const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','L','E','-','G','0','8'};
  
  /* disable scan response */
  hci_le_set_scan_resp_data(0,NULL);
  PRINTF("General Discoverable Mode.\n");
  
  ret = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                 sizeof(local_name), local_name, 0, NULL, 0, 0);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error while setting discoverable mode (%d)\n", ret);    
  }  
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t Address of peer device
 * @param  uint16_t Connection handle
 * @retval None
 */
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{  
  connected = TRUE;
  connection_handle = handle;
  
  PRINTF("Connected to device:");
  for(int i = 5; i > 0; i--){
    PRINTF("%02X-", addr[i]);
  }
  PRINTF("%02X\n", addr[0]);
}

/**
 * @brief  This function is called when the peer device gets disconnected.
 * @param  None 
 * @retval None
 */
void GAP_DisconnectionComplete_CB(void)
{
  connected = FALSE;
  PRINTF("Disconnected\n");
  /* Make the device connectable again. */
  set_connectable = TRUE;
  notification_enabled = FALSE;
}

/**
 * @brief  Read request callback.
 * @param  uint16_t Handle of the attribute
 * @retval None
 */
void Read_Request_CB(uint16_t handle)
{  
  if(handle == tempCharHandle + 1){
    uint16_t data; 
		data = buf[0];
		Temp_Update(data);
		PRINTF("Reading TEMP Characteristic\n");
  }
  else if(handle == pitchCharHandle + 1){
    //struct timer t;  
    //Timer_Set(&t, CLOCK_SECOND/10);
    uint16_t data;
		data = buf[1];
		Pitch_Update(data);
		PRINTF("Reading PITCH Characteristic\n");
  }
  else if(handle == rollCharHandle + 1){
    uint16_t data;
		data = buf[2];
		Roll_Update(data);
		PRINTF("Reading ROLL Characteristic\n");
  }
	
  //EXIT:
  if(connection_handle != 0)
    aci_gatt_allow_read(connection_handle);
}

/**
 * @brief  Callback processing the ACI events.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = pckt;
  /* obtain event packet */
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
  
  if(hci_pckt->type != HCI_EVENT_PKT)
    return;
  
  switch(event_pckt->evt){
    
  case EVT_DISCONN_COMPLETE:
    {
      GAP_DisconnectionComplete_CB();
    }
    break;
    
  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;
      
      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
          evt_le_connection_complete *cc = (void *)evt->data;
          GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;
      }
    }
    break;
    
  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
      switch(blue_evt->ecode){

      case EVT_BLUE_GATT_READ_PERMIT_REQ:
        {
          evt_gatt_read_permit_req *pr = (void*)blue_evt->data;                    
          Read_Request_CB(pr->attr_handle);                    
        }
        break;
			case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:         
        {
          /* this callback is invoked when a GATT attribute is modified
          extract callback data and pass to suitable handler function */
          evt_gatt_attr_modified_IDB04A1 *evt = (evt_gatt_attr_modified_IDB04A1*)blue_evt->data;
          Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data);                 
        }
        break; 
      }
    }
    break;
  }    
}

/**
 * @brief  This function is called attribute value corresponding to 
 *         ledCharHandle characteristic gets modified.
 * @param  Handle of the attribute
 * @param  Size of the modified attribute data
 * @param  Pointer to the modified attribute data
 * @retval None
 */
void Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data)
{	
  if(handle == ledCharHandle + 1){
		/*
		int i;
		PRINTF("Attribute Modified CB: handle = %04x\n", handle - 1);
		PRINTF("LED Char Modified Data: ");
		for(i = 0; i < data_length; i++)
			PRINTF("%X, ", att_data[i]);
		PRINTF("\n");
		*/
		LED_Update(att_data);
  }
}

/**
 * @}
 */
 
/**
 * @}
 */

/**
 * @}
 */

 /**
 * @}
 */
 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
