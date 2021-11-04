/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
 * File Name          : gatt_db.h
 * Author             : SRA
 * Version            : V1.0.0
 * Date               : Oct-2019
 * Description        : Header file for gatt_db.c
 *******************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 ******************************************************************************/

#ifndef GATT_DB_H
#define GATT_DB_H

/* Includes ------------------------------------------------------------------*/
#include "hci.h"

/* Exported function prototypes ----------------------------------------------*/
tBleStatus add_custom_telemetry_service(void);

void Read_Request_CB(uint16_t handle);
void Attribute_Modified_Request_CB(uint16_t Connection_Handle, uint16_t attr_handle,
                                   uint16_t Offset, uint8_t data_length, uint8_t *att_data);
uint8_t get_ble_data8(uint8_t offset);
int16_t get_ble_data16(uint8_t offset);
int32_t get_ble_data32(uint8_t offset);
void set_ble_data(int32_t telemetry[], uint16_t length);
tBleStatus load_read_data(void);

#endif /* GATT_DB_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
