/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
 * File Name          : sensor.h
 * Author             : SRA
 * Version            : V1.0.0
 * Date               : Oct-2019
 * Description        : Header file for sensor.c
 *******************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 ******************************************************************************/

#ifndef SENSOR_H
#define SENSOR_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/
#define BLE_DEVICE_NAME   'B','e','n',' ','B','L','E'
#define BDADDR_SIZE        6

/* Exported function prototypes ----------------------------------------------*/
void Set_DeviceConnectable(void);
void APP_UserEvtRx(void *pData);

#endif /* SENSOR_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
