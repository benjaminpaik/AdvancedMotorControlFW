/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "definitions.h"
#include "parameters.h"
#include "parameter_functions.h"
#include "math.h"
#include "state_space.h"
#include "system.h"
#include "usb_hid_api.h"
#include "app_bluenrg_2.h"
#include "gatt_db.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "eeprom_emul.h"
#include "usb_device.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint32_t g_adc_buffer[3];
extern uint32_t g_adc2_buffer[3];
SYSTEM S;
PARAMETER P;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for commTask */
osThreadId_t commTaskHandle;
const osThreadAttr_t commTask_attributes = {
  .name = "commTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void host_processor(void);
void implement_parameters(void);
void load_telemetry(void);
void load_bootloader(void);
/* USER CODE END FunctionPrototypes */

void DefaultTask(void *argument);
void LedTask(void *argument);
void ControlTask(void *argument);
void CommTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  init_motor_drive(&S.motor, &htim1);
  init_encoder(&S.motor.encoder, &htim8);
  crc32_table_generator(CRC32_SEED);
  S.status.bit.rom_fault = rom_check(&S.rom_crc32);
  S.int_flash_flag = TRUE;

  load_parameters((int32_t *)(&P), NUM_PARAMETERS);
  implement_parameters();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(DefaultTask, NULL, &defaultTask_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(LedTask, NULL, &ledTask_attributes);

  /* creation of controlTask */
  controlTaskHandle = osThreadNew(ControlTask, NULL, &controlTask_attributes);

  /* creation of commTask */
  commTaskHandle = osThreadNew(CommTask, NULL, &commTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_DefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_DefaultTask */
void DefaultTask(void *argument)
{
  /* init code for USB_Device */
  MX_USB_Device_Init();
  /* USER CODE BEGIN DefaultTask */
  /* Infinite loop */
  for(;;)
  {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  /* USER CODE END DefaultTask */
}

/* USER CODE BEGIN Header_LedTask */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LedTask */
void LedTask(void *argument)
{
  /* USER CODE BEGIN LedTask */
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));

    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
  }
  /* USER CODE END LedTask */
}

/* USER CODE BEGIN Header_ControlTask */
/**
* @brief Function implementing the controlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ControlTask */
void ControlTask(void *argument)
{
  /* USER CODE BEGIN ControlTask */
  // positive current limit
  HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095);
  // negative current limit
  HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 1);
  // start timer to trigger ADC conversions
  HAL_TIM_Base_Start_IT(&htim3);

  /* Infinite loop */
  for(;;)
  {
    vTaskDelay(pdMS_TO_TICKS(CTRL_TASK_PERIOD));

    switch(S.mode) {

    case(IDLE_MODE):
      enable_drive(&S.motor, FALSE);
      set_usb_tx_mode(IDLE_MODE);
      break;

    case(RUN_MODE):
      S.motor.torque_cmd = S.cmd;
      enable_drive(&S.motor, TRUE);
      set_usb_tx_mode(RUN_MODE);
      break;

    case(CAL_MODE):
      cal_angle(&S.motor);
      break;

    default:
  	  enable_drive(&S.motor, FALSE);
  	  set_usb_tx_mode(S.mode);
      break;
    }

    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&g_adc2_buffer, ARRAY_SIZE(g_adc2_buffer));
  }
  /* USER CODE END ControlTask */
}

/* USER CODE BEGIN Header_CommTask */
/**
* @brief Function implementing the commTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CommTask */
void CommTask(void *argument)
{
  /* USER CODE BEGIN CommTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();

#ifdef USB_COMM
  init_usb_data();
#else
    int32_t telemetry[2];
    MX_BlueNRG_2_Init();
    ble_set_connectable();
#endif

  /* Infinite loop */
  for(;;)
  {

#ifdef USB_COMM
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    host_processor();
#else
    vTaskDelay(pdMS_TO_TICKS(1));
    MX_BlueNRG_2_Process(xTaskGetTickCount());
    vTaskDelay(pdMS_TO_TICKS(5));

    S.mode = get_ble_data8(0);
    S.cmd.raw = get_ble_data16(1);

    // set telemetry data
    telemetry[0] = S.cmd.raw;
    INT_TO_FLOAT_BITS(telemetry[1]) = S.motor.encoder.out;
    set_ble_data(telemetry, 2);
#endif
  }
  /* USER CODE END CommTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void host_processor(void)
{
  S.mode_previous = S.mode;
  S.mode = get_usb_rx_mode();

  switch(S.mode) {

  case(IDLE_MODE):
    S.cmd = get_usb_data32(0);
    load_telemetry();
    set_usb_tx_mode(IDLE_MODE);
    break;

  case(RUN_MODE):
    S.cmd = get_usb_data32(0);
    load_telemetry();
    set_usb_tx_mode(RUN_MODE);
    break;

  case(CAL_MODE):
    S.cmd = get_usb_data32(0);
    load_telemetry();
    set_usb_tx_mode(CAL_MODE);
    break;

  case(WRITE_PARAMETER_MODE):
    if(write_parameters((int32_t *)(&P), NUM_TELEMETRY_STATES - 1, get_usb_tx_data(), get_usb_rx_data())) {
      implement_parameters();
    }
    set_usb_tx_data();
    set_usb_tx_mode(WRITE_PARAMETER_MODE);
    break;

  case(READ_PARAMETER_MODE):
    read_parameters((int32_t *)(&P), NUM_TELEMETRY_STATES - 1, get_usb_tx_data(), get_usb_rx_data());
    set_usb_tx_data();
    set_usb_tx_mode(READ_PARAMETER_MODE);
    break;

  // save parameters in RAM to flash memory
  case(FLASH_PARAMETER_MODE):
    if(S.int_flash_flag) {
      S.int_flash_flag = FALSE;
      flash_parameters((int32_t *)(&P), NUM_PARAMETERS);
      set_usb_tx_mode(FLASH_PARAMETER_MODE);
    }
    break;

  case(BOOT_MODE):
    if(S.mode_previous != BOOT_MODE) {
      S.boot_delay_timer = 0;
    }
    else if(++S.boot_delay_timer > BOOT_DELAY_TIME) {
      load_bootloader();
    }
    set_usb_tx_mode(BOOT_MODE);
    break;

  case(NULL_MODE):
    S.int_flash_flag = TRUE;
//    framework.status.bit.parameters_updated = FALSE;
    load_telemetry();
    set_usb_tx_mode(NULL_MODE);
    break;

  default:
    S.cmd = get_usb_data32(0);
    load_telemetry();
    break;
  }
  update_usb_timestamp();
  load_usb_tx_bytes();
}

void implement_parameters(void)
{
  P.InfoNumParameters = NUM_PARAMETERS;
  P.InfoSwChecksum = *(int32_t *)(&S.rom_crc32);
  P.InfoSwVersion = SW_VERSION;

  init_pid(&S.motor.id_pid, TRUE, P.PidFocPropGain, P.PidFocIntGain, 0, P.PidFocLimit, -P.PidFocLimit, CONTROL_FREQUENCY);
  init_pid(&S.motor.iq_pid, TRUE, P.PidFocPropGain, P.PidFocIntGain, 0, P.PidFocLimit, -P.PidFocLimit, CONTROL_FREQUENCY);
}

void load_telemetry(void)
{
  set_usb_data32(0, S.cmd);
//  set_usb_data32(1, FLOAT_TO_INT_BITS(S.controller.u));
//  set_usb_data32(2, FLOAT_TO_INT_BITS(S.controller.r));
//  set_usb_data32(3, FLOAT_TO_INT_BITS(S.pwm_cmd));
//  set_usb_data32(4, FLOAT_TO_INT_BITS(S.motor.encoder.out));
//  set_usb_data32(5, FLOAT_TO_INT_BITS(S.controller.obs.ss.x[0]));
//  set_usb_data32(6, FLOAT_TO_INT_BITS(S.controller.obs.ss.x[1]));
//  set_usb_data32(7, FLOAT_TO_INT_BITS(S.controller.obs.ss.x[2]));
//  set_usb_data32(8, FLOAT_TO_INT_BITS(S.controller.obs.error));
//  set_usb_data32(9, g_adc_buffer[0]);
//  set_usb_data32(10, g_adc_buffer[1]);
}

void load_bootloader(void)
{
  *((uint32_t *)&__RAM_END) = 0xDEADBEEF;
  NVIC_SystemReset();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  drive_control(&S.motor, CURRENT_SCALE(g_adc_buffer[0]), CURRENT_SCALE(g_adc_buffer[1]));
}

/* USER CODE END Application */

