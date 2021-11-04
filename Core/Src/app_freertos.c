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
#include "math.h"
#include "state_space.h"
#include "system.h"
#include "usb_hid_api.h"
#include "app_bluenrg_2.h"
#include "gatt_db.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
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
uint16_t g_kill_switch = FALSE;
SYSTEM S;
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
  init_trap_drive(&S.motor, &htim1, &htim2, 1);
  init_encoder(&S.motor.encoder, &htim8);
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
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));

    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
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
  init_rate_limiter(&S.cmd.rate, (4.0F * CTRL_TASK_PERIOD));
  // positive current limit
  HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4094);
  // negative current limit
  HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 1);
  // start timer to trigger ADC conversions
  HAL_TIM_Base_Start_IT(&htim3);
  S.motor.encoder.gain = (2*PI)/(S.motor.encoder.tim->Init.Period + 1);
  S.motor.encoder.offset = -7290;
  init_observer(&S.observer);
  S.cmd.scale = (PI / 6) * 0.001F;
  S.observer.K[0] = 5.3908;
  S.observer.K[1] = 0.3758;
  S.observer.K[2] = 0.0103;

  /* Infinite loop */
  for(;;)
  {
    vTaskDelay(pdMS_TO_TICKS(CTRL_TASK_PERIOD));
    update_encoder_position(&S.motor.encoder);
    update_hall_velocity(&S.motor.hall, VELOCITY_GAIN);
    S.cmd.out = S.cmd.scale * S.cmd.raw;

    switch(S.mode) {

    case(IDLE_MODE):
      S.observer.ss.x[0] = S.motor.encoder.out;
      S.observer.ss.x[1] = 0;
      S.observer.ss.x[2] = 0;
      g_kill_switch = FALSE;

      update_pwm_cmd(&S.motor, (0.001F * S.cmd.rate.out));
      enable_trap_drive(&S.motor, FALSE);
      set_usb_tx_mode(IDLE_MODE);
      break;

    case(RUN_MODE):
      if(fabs(S.motor.encoder.out) > S.observer.ss.x_limit[0]) {
        g_kill_switch = TRUE;
      }

      if(g_kill_switch) {
        enable_trap_drive(&S.motor, FALSE);
        set_usb_tx_mode(IDLE_MODE);
      }
      else {
        update_control_effort(&S.observer, S.cmd.out);
        update_observer(&S.observer, S.motor.encoder.out);
        S.pwm_cmd = (S.observer.u / INPUT_VOLTAGE) * 1.5F;
        S.pwm_cmd = LIMIT(S.pwm_cmd, 0.25, -0.25);

        update_pwm_cmd(&S.motor, S.pwm_cmd);
        enable_trap_drive(&S.motor, TRUE);
        set_usb_tx_mode(RUN_MODE);
      }

//      update_pwm_cmd(&S.motor, (0.001F * S.cmd.rate.out));
//      enable_trap_drive(&S.motor, TRUE);
//      set_usb_tx_mode(RUN_MODE);
      break;

    case(CAL_MODE):
      update_pwm_cmd(&S.motor, (0.001F * S.cmd.rate.out));
      if(update_trap_cal(&S.motor)) {
        set_usb_tx_mode(NULL_MODE);
      }
      else {
        set_usb_tx_mode(CAL_MODE);
      }
      break;

    default:
  	  enable_trap_drive(&S.motor, FALSE);
  	  set_usb_tx_mode(S.mode);
      break;
    }

//    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&g_adc2_buffer, ARRAY_SIZE(g_adc2_buffer));
//    HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, g_dac);
//    HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, (4095 - g_dac));
//    g_dac++;
//    g_dac &= 0xFFF;
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
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  init_usb_data();

//  int32_t telemetry[2];
//  MX_BlueNRG_2_Init();
//  ble_set_connectable();

  /* Infinite loop */
  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    // load telemetry feedback
    S.cmd.raw = get_usb_data32(0);
    S.mode = get_usb_rx_mode();

    set_usb_data32(0, FLOAT_TO_INT_BITS(S.cmd.out));
    set_usb_data32(1, FLOAT_TO_INT_BITS(S.observer.u));
    set_usb_data32(2, g_kill_switch);
    set_usb_data32(3, FLOAT_TO_INT_BITS(S.pwm_cmd));
    set_usb_data32(4, FLOAT_TO_INT_BITS(S.motor.encoder.out));
    set_usb_data32(5, FLOAT_TO_INT_BITS(S.observer.ss.x[0]));
    set_usb_data32(6, FLOAT_TO_INT_BITS(S.observer.ss.x[1]));
    set_usb_data32(7, FLOAT_TO_INT_BITS(S.observer.ss.x[2]));
    set_usb_data32(8, FLOAT_TO_INT_BITS(S.observer.error));
    set_usb_data32(9, g_adc_buffer[0]);
    set_usb_data32(10, g_adc_buffer[1]);
    set_usb_data32(11, g_adc_buffer[2]);

    update_usb_timestamp();
    load_usb_tx_data();


//    vTaskDelay(pdMS_TO_TICKS(1));
//    MX_BlueNRG_2_Process(xTaskGetTickCount());
//    vTaskDelay(pdMS_TO_TICKS(5));
//
//    S.mode = get_ble_data8(0);
//    S.cmd.raw = get_ble_data16(1);
//
//    // set telemetry data
//    telemetry[0] = S.cmd.raw;
//    INT_TO_FLOAT_BITS(telemetry[1]) = S.motor.encoder.out;
//    set_ble_data(telemetry, 2);
  }
  /* USER CODE END CommTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
volatile uint32_t g_call_count = 0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  g_call_count++;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  update_hall_state(&S.motor.hall);
  update_state_cmd(&S.motor);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
