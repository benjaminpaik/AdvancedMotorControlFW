/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
 * File Name          : sensor.c
 * Author             : AMS - VMA RF Application team
 * Version            : V1.0.0
 * Date               : 23-November-2015
 * Description        : Sensor init and sensor state machines
 *******************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sensor.h"
#include "gatt_db.h"
#include "bluenrg1_gap.h"
#include "bluenrg1_gap_aci.h"
#include "bluenrg1_hci_le.h"
#include "hci_const.h"
#include "bluenrg1_gatt_aci.h"

/* Private typedef -----------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define SOFTWARE_VERSION  0x01, 0x00, 0x00, 0x00
// MAC address (MSB first)
#define BLE_MAC   bdaddr[5], bdaddr[4], bdaddr[3], bdaddr[2], bdaddr[1], bdaddr[0]

#define POWER_ADV     AD_TYPE_TX_POWER_LEVEL, 0
#define NAME_ADV      AD_TYPE_COMPLETE_LOCAL_NAME, BLE_DEVICE_NAME
#define MANUF_ADV     AD_TYPE_MANUFACTURER_SPECIFIC_DATA, SOFTWARE_VERSION, BLE_MAC

/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern uint8_t bdaddr[BDADDR_SIZE];
extern uint8_t bnrg_expansion_board;

volatile uint8_t set_connectable = FALSE;
volatile uint16_t connection_handle = FALSE;
volatile uint8_t  notification_enabled = FALSE;
volatile uint32_t connected = FALSE;

/* Private function prototypes -----------------------------------------------*/
void GAP_DisconnectionComplete_CB(void);
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Set_DeviceConnectable
 * @note   Puts the device in connectable mode
 * @param  None
 * @retval None
 */
void Set_DeviceConnectable(void)
{
  uint8_t ret;
  uint8_t power_adv[] = {POWER_ADV};
  uint8_t name_adv[] = {NAME_ADV};
  uint8_t manuf_adv[] = {MANUF_ADV};

  uint8_t adv_data[] = {
    sizeof(power_adv), POWER_ADV,
    sizeof(name_adv), NAME_ADV,
    sizeof(manuf_adv), MANUF_ADV
  };

  hci_le_set_scan_response_data(0, NULL);

  PRINT_DBG("Set General Discoverable Mode.\r\n");

  ret = aci_gap_set_discoverable(ADV_DATA_TYPE,
                                 ADV_INTERV_MIN, ADV_INTERV_MAX,
                                 STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE,
                                 sizeof(name_adv), name_adv, 0, NULL, 0, 0);

  aci_gap_update_adv_data(sizeof(adv_data), adv_data);

  if(ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_gap_set_discoverable() failed: 0x%02x\r\n", ret);
  }
  else
    PRINT_DBG("aci_gap_set_discoverable() --> SUCCESS\r\n");
}

/**
 * @brief  Callback processing the ACI events
 * @note   Inside this function each event must be identified and correctly
 *         parsed
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
void APP_UserEvtRx(void *pData)
{
  uint32_t i;

  hci_spi_pckt *hci_pckt = (hci_spi_pckt *)pData;

  if(hci_pckt->type == HCI_EVENT_PKT)
  {
    hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

    if(event_pckt->evt == EVT_LE_META_EVENT)
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;

      for (i = 0; i < (sizeof(hci_le_meta_events_table)/sizeof(hci_le_meta_events_table_type)); i++)
      {
        if (evt->subevent == hci_le_meta_events_table[i].evt_code)
        {
          hci_le_meta_events_table[i].process((void *)evt->data);
        }
      }
    }
    else if(event_pckt->evt == EVT_VENDOR)
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;

      for (i = 0; i < (sizeof(hci_vendor_specific_events_table)/sizeof(hci_vendor_specific_events_table_type)); i++)
      {
        if (blue_evt->ecode == hci_vendor_specific_events_table[i].evt_code)
        {
          hci_vendor_specific_events_table[i].process((void *)blue_evt->data);
        }
      }
    }
    else
    {
      for (i = 0; i < (sizeof(hci_events_table)/sizeof(hci_events_table_type)); i++)
      {
        if (event_pckt->evt == hci_events_table[i].evt_code)
        {
          hci_events_table[i].process((void *)event_pckt->data);
        }
      }
    }
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
