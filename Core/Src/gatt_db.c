/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : gatt_db.c
* Author             : SRA
* Version            : V1.0.0
* Date               : Oct-2019
* Description        : Functions to build GATT DB and handle GATT events.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include "gatt_db.h"
#include "bluenrg1_aci.h"
#include "bluenrg1_hci_le.h"
#include "bluenrg1_gatt_aci.h"
#include "app_bluenrg_2.h"
#include "dsp.h"

/* Private macros ------------------------------------------------------------*/
#define CUSTOM_SERVICE_CHARACTERISTICS    2
// 2-5 attributes: declaration, value, notify/indicate, broadcast, extended property
#define CHARACTERISTIC_ATTRIBUTES         3
// custom service base UUID (bytes are indexed in reverse)
#define CUSTOM_SERVICE_UUID       {0xE1, 0x71, 0x7A, 0xEF, 0xBB, 0x4E, 0xDD, 0xA8, 0x08, 0x42, 0xCD, 0x0D, 0x00, 0x00, 0xB6, 0xEA}
#define UUID16_INDEX_0    13
#define UUID16_INDEX_1    12

#define WRITE_BUFFER_LENGTH   3
#define READ_BUFFER_LENGTH    8

/* Private variables ---------------------------------------------------------*/
uint16_t CustomServiceHandle, ReadCharHandle, WriteCharHandle;

uint8_t write_buffer[WRITE_BUFFER_LENGTH];
uint8_t read_buffer[READ_BUFFER_LENGTH];

/* UUIDS */
Service_UUID_t service_uuid = {.Service_UUID_128 = CUSTOM_SERVICE_UUID};
Char_UUID_t char_read_uuid =     {.Char_UUID_128 = CUSTOM_SERVICE_UUID};
Char_UUID_t char_write_uuid =    {.Char_UUID_128 = CUSTOM_SERVICE_UUID};

volatile uint8_t send_env;
extern volatile uint16_t connection_handle;

tBleStatus add_custom_telemetry_service(void)
{
  tBleStatus ret;
  /* number of attribute records that can be added to this service */
  uint8_t max_attribute_records = (CUSTOM_SERVICE_CHARACTERISTICS * CHARACTERISTIC_ATTRIBUTES) + 1;

  // add custom telemetry service
  service_uuid.Service_UUID_128[UUID16_INDEX_0] = 0xC7;
  service_uuid.Service_UUID_128[UUID16_INDEX_1] = 0x00;
  ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE,
                             max_attribute_records, &CustomServiceHandle);
  if (ret != BLE_STATUS_SUCCESS) return BLE_STATUS_ERROR;

  // add custom read characteristic
  char_read_uuid.Char_UUID_128[UUID16_INDEX_0] = 0xC7;
  char_read_uuid.Char_UUID_128[UUID16_INDEX_1] = 0x01;
  ret =  aci_gatt_add_char(CustomServiceHandle, UUID_TYPE_128, &char_read_uuid,
                           READ_BUFFER_LENGTH,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &ReadCharHandle);
  if (ret != BLE_STATUS_SUCCESS) return BLE_STATUS_ERROR;

  // add custom write characteristic
  char_write_uuid.Char_UUID_128[UUID16_INDEX_0] = 0xC7;
  char_write_uuid.Char_UUID_128[UUID16_INDEX_1] = 0x02;
  ret =  aci_gatt_add_char(CustomServiceHandle, UUID_TYPE_128, &char_write_uuid,
                           WRITE_BUFFER_LENGTH,
                           CHAR_PROP_WRITE|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE,
                           16, 0, &WriteCharHandle);
  if (ret != BLE_STATUS_SUCCESS) return BLE_STATUS_ERROR;

  return BLE_STATUS_SUCCESS;
}

uint8_t get_ble_data8(uint8_t offset)
{
  return write_buffer[offset];
}

int16_t get_ble_data16(uint8_t offset)
{
  return BYTE_TO_INT(write_buffer + offset);
}

int32_t get_ble_data32(uint8_t offset)
{
  return BYTES_TO_LONG(write_buffer + offset);
}

void set_ble_data(int32_t telemetry[], uint16_t length)
{
  uint16_t i, offset;

  for(i = 0; i < length; i++) {
    offset = BYTES_PER_LONG * i;
    LONG_TO_BYTES((read_buffer + offset), telemetry[i]);
  }
}

tBleStatus load_read_data(void)
{
  tBleStatus ret;
  ret = aci_gatt_update_char_value(CustomServiceHandle, ReadCharHandle, 0, READ_BUFFER_LENGTH, read_buffer);

  if (ret != BLE_STATUS_SUCCESS){
    PRINT_DBG("Error while updating TEMP characteristic: 0x%04X\r\n",ret) ;
    return BLE_STATUS_ERROR ;
  }

  return BLE_STATUS_SUCCESS;
}

void Read_Request_CB(uint16_t handle)
{
  tBleStatus ret;

  if (handle == ReadCharHandle + 1) {
    load_read_data();
  }

  if(connection_handle != 0)
  {
    ret = aci_gatt_allow_read(connection_handle);
    if (ret != BLE_STATUS_SUCCESS)
    {
      PRINT_DBG("aci_gatt_allow_read() failed: 0x%02x\r\n", ret);
    }
  }
}

/**
 * @brief  This function is called when there is a change on the gatt attribute.
 *         With this function it's possible to understand if one application
 *         is subscribed to the one service or not.
 *
 * @param  uint16_t att_handle Handle of the attribute
 * @param  uint8_t  *att_data attribute data
 * @param  uint8_t  data_length length of the data
 * @retval None
 */
void Attribute_Modified_Request_CB(uint16_t Connection_Handle, uint16_t attr_handle, uint16_t Offset, uint8_t data_length, uint8_t *att_data)
{
  uint8_t i;
  // notify enable
  if(attr_handle == ReadCharHandle + 2) {
    send_env = att_data[0] ? TRUE : FALSE;
  }
  else if(attr_handle == WriteCharHandle + 1) {
    for(i = 0; i < WRITE_BUFFER_LENGTH; i++) {
      write_buffer[i] = att_data[i];
    }
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
