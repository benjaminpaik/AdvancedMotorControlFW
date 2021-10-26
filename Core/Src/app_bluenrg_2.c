/**
  ******************************************************************************
  * File Name          : app_bluenrg_2.c
  * Description        : Implementation file
  *
  ******************************************************************************
  *
  * COPYRIGHT 2021 STMicroelectronics
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_bluenrg_2.h"

#include <stdlib.h>

#include "sensor.h"
#include "bluenrg1_aci.h"
#include "bluenrg1_hci_le.h"
#include "bluenrg1_events.h"
#include "hci_tl.h"
#include "gatt_db.h"
#include "bluenrg_utils.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private defines -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern volatile uint16_t connection_handle;
extern volatile uint8_t set_connectable;
extern volatile int     connected;
uint8_t bdaddr[BDADDR_SIZE];
extern volatile uint8_t send_env;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static uint8_t Sensor_DeviceInit(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

void MX_BlueNRG_2_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN BlueNRG_2_Init_PreTreatment */

  /* USER CODE END BlueNRG_2_Init_PreTreatment */

  /* Initialize the peripherals and the BLE Stack */
  uint8_t ret;

  hci_init(APP_UserEvtRx, NULL);

  PRINT_DBG("BlueNRG-2 SensorDemo_BLESensor-App Application\r\n");

  /* Init Sensor Device */
  ret = Sensor_DeviceInit();
  if (ret != BLE_STATUS_SUCCESS)
  {
//    BSP_LED_On(LED2);
    PRINT_DBG("SensorDeviceInit()--> Failed 0x%02x\r\n", ret);
    while(1);
  }

  PRINT_DBG("BLE Stack Initialized & Device Configured\r\n");

  /* USER CODE BEGIN BlueNRG_2_Init_PostTreatment */

  /* USER CODE END BlueNRG_2_Init_PostTreatment */
}

/*
 * BlueNRG-2 background task
 */
void MX_BlueNRG_2_Process(uint16_t current_time)
{
  /* USER CODE BEGIN BlueNRG_2_Process_PreTreatment */
  static uint16_t previous_time = 0;
  uint16_t delta_time = 0;
  /* USER CODE END BlueNRG_2_Process_PreTreatment */

  hci_user_evt_proc();

  /* USER CODE BEGIN BlueNRG_2_Process_PostTreatment */

  // make the device discoverable
  if(set_connectable) {
    Set_DeviceConnectable();
    set_connectable = FALSE;
  }

  // without this variable, the expression evaluates to false on rollover
  delta_time = current_time - previous_time;
  if(delta_time >= 100) {
    previous_time = current_time;
    ble_send_notifications();
  }
  /* USER CODE END BlueNRG_2_Process_PostTreatment */
}

/**
 * @brief  Initialize the device sensors
 *
 * @param  None
 * @retval None
 */
uint8_t Sensor_DeviceInit(void)
{
  uint8_t ret;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  uint8_t device_name[] = {BLE_DEVICE_NAME};
  uint8_t  hwVersion;
  uint16_t fwVersion;
  uint8_t  bdaddr_len_out;
  uint8_t  config_data_stored_static_random_address = 0x80; /* Offset of the static random address stored in NVM */

  /* Sw reset of the device */
  hci_reset();
  /**
   *  To support both the BlueNRG-2 and the BlueNRG-2N a minimum delay of 2000ms is required at device boot
   */
  HAL_Delay(2000);

  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  PRINT_DBG("HWver %d\nFWver %d\r\n", hwVersion, fwVersion);

  ret = aci_hal_read_config_data(config_data_stored_static_random_address,
                                 &bdaddr_len_out, bdaddr);

  if (ret) {
    PRINT_DBG("Read Static Random address failed.\r\n");
  }

  if ((bdaddr[5] & 0xC0) != 0xC0) {
    PRINT_DBG("Static Random address not well formed.\r\n");
    while(1);
  }

  /* Set the TX power -2 dBm */
  aci_hal_set_tx_power_level(1, 4);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("Error in aci_hal_set_tx_power_level() 0x%04x\r\n", ret);
    return ret;
  }
  else
  {
    PRINT_DBG("aci_hal_set_tx_power_level() --> SUCCESS\r\n");
  }

  /* GATT Init */
  ret = aci_gatt_init();
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_gatt_init() failed: 0x%02x\r\n", ret);
    return ret;
  }
  else
  {
    PRINT_DBG("aci_gatt_init() --> SUCCESS\r\n");
  }

  /* GAP Init */
  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x07, &service_handle, &dev_name_char_handle,
                     &appearance_char_handle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_gap_init() failed: 0x%02x\r\n", ret);
    return ret;
  }
  else
  {
    PRINT_DBG("aci_gap_init() --> SUCCESS\r\n");
  }

  /* Update device name */
  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, sizeof(device_name),
                                   device_name);
  if(ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_gatt_update_char_value() failed: 0x%02x\r\n", ret);
    return ret;
  }
  else
  {
    PRINT_DBG("aci_gatt_update_char_value() --> SUCCESS\r\n");
  }

  /* BLE Security v4.2 is supported: BLE stack FW version >= 2.x (new API prototype) */
  ret = aci_gap_set_authentication_requirement(BONDING,
                                               MITM_PROTECTION_REQUIRED,
                                               SC_IS_SUPPORTED,
                                               KEYPRESS_IS_NOT_SUPPORTED,
                                               7,
                                               16,
                                               USE_FIXED_PIN_FOR_PAIRING,
                                               123456,
                                               0x00);
  if(ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_gap_set_authentication_requirement()failed: 0x%02x\r\n", ret);
    return ret;
  }
  else
  {
    PRINT_DBG("aci_gap_set_authentication_requirement() --> SUCCESS\r\n");
  }

  PRINT_DBG("BLE Stack Initialized with SUCCESS\r\n");

  ret = add_custom_telemetry_service();
  if(ret == BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("BlueNRG2 HW service added successfully.\r\n");
  }
  else
  {
    PRINT_DBG("Error while adding BlueNRG2 HW service: 0x%02x\r\n", ret);
    while(1);
  }

  return BLE_STATUS_SUCCESS;
}

void ble_send_notifications(void)
{
  // notification updates
  if (connected && send_env) {
    load_read_data();
  }
}

/**
 * @brief  Get hardware and firmware version
 *
 * @param  Hardware version
 * @param  Firmware version
 * @retval Status
 */
uint8_t getBlueNRGVersion(uint8_t *hwVersion, uint16_t *fwVersion)
{
  uint8_t status;
  uint8_t hci_version, lmp_pal_version;
  uint16_t hci_revision, manufacturer_name, lmp_pal_subversion;

  status = hci_read_local_version_information(&hci_version, &hci_revision, &lmp_pal_version,
				                              &manufacturer_name, &lmp_pal_subversion);

  if (status == BLE_STATUS_SUCCESS) {
    *hwVersion = hci_revision >> 8;
    *fwVersion = (hci_revision & 0xFF) << 8;              // Major Version Number
    *fwVersion |= ((lmp_pal_subversion >> 4) & 0xF) << 4; // Minor Version Number
    *fwVersion |= lmp_pal_subversion & 0xF;               // Patch Version Number
  }
  return status;
}

/**
 * @brief  BSP Push Button callback
 *
 * @param  Button Specifies the pin connected EXTI line
 * @retval None
 */
void ble_set_connectable(void)
{
  // make connectable when the button is pressed
  set_connectable = TRUE;
}

/* ***************** BlueNRG-1 Stack Callbacks ********************************/
/**
 * @brief  This event indicates that a new connection has been created
 *
 * @param  See file bluenrg1_events.h
 * @retval See file bluenrg1_events.h
 */
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)
{
  connected = TRUE;
  connection_handle = Connection_Handle;

//  BSP_LED_Off(LED2); //activity led
}

/**
 * @brief  This event occurs when a connection is terminated
 *
 * @param  See file bluenrg1_events.h
 * @retval See file bluenrg1_events.h
 */
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  connected = FALSE;
  connection_handle = 0;
  PRINT_DBG("Disconnected\r\n");

//  BSP_LED_On(LED2); //activity led
}

/**
 * @brief  This event is given when a read request is received
 *         by the server from the client
 * @param  See file bluenrg1_events.h
 * @retval See file bluenrg1_events.h
 */
void aci_gatt_read_permit_req_event(uint16_t Connection_Handle,
                                    uint16_t Attribute_Handle,
                                    uint16_t Offset)
{
  Read_Request_CB(Attribute_Handle);
}

/**
 * @brief  This event is given when an attribute changes his value
 * @param  See file bluenrg1_events.h
 * @retval See file bluenrg1_events.h
 */
void aci_gatt_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t Attribute_Handle,
                                       uint16_t Offset,
                                       uint16_t Attr_Data_Length,
                                       uint8_t Attr_Data[])
{
  Attribute_Modified_Request_CB(Connection_Handle, Attribute_Handle, Offset, Attr_Data_Length, Attr_Data);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
