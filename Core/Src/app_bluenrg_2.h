/**
  ******************************************************************************
  * File Name          : app_bluenrg_2.h
  * Description        : Header file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_BLUENRG_2_H
#define APP_BLUENRG_2_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Exported Defines ----------------------------------------------------------*/
/*************** Don't Change the following defines *************/

/* Package Version only numbers 0->9 */
#define PACK_VERSION_MAJOR '3'
#define PACK_VERSION_MINOR '1'
#define PACK_VERSION_PATCH '0'

/* Define the application Name (MUST be 7 char long) */
#define APP_NAME 'S','D','E','M',PACK_VERSION_MAJOR,PACK_VERSION_MINOR,PACK_VERSION_PATCH

/* Package Name */
#define BLUENRG_PACKAGENAME "X-CUBE-BLE2"

/* USER CODE BEGIN ED */

/* USER CODE END ED */

/* Exported Variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported Functions Prototypes ---------------------------------------------*/
void MX_BlueNRG_2_Init(void);
void MX_BlueNRG_2_Process(uint16_t current_time);

/* USER CODE BEGIN EFP */
void ble_set_connectable(void);
void ble_send_notifications(void);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif
#endif /* APP_BLUENRG_2_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
