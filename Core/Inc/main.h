/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "hci_tl_interface.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HALL_A_Pin GPIO_PIN_13
#define HALL_A_GPIO_Port GPIOC
#define HALL_A_EXTI_IRQn EXTI15_10_IRQn
#define HALL_B_Pin GPIO_PIN_14
#define HALL_B_GPIO_Port GPIOC
#define HALL_B_EXTI_IRQn EXTI15_10_IRQn
#define HALL_C_Pin GPIO_PIN_15
#define HALL_C_GPIO_Port GPIOC
#define HALL_C_EXTI_IRQn EXTI15_10_IRQn
#define PHASE_A_UPPER_Pin GPIO_PIN_0
#define PHASE_A_UPPER_GPIO_Port GPIOC
#define PHASE_B_UPPER_Pin GPIO_PIN_1
#define PHASE_B_UPPER_GPIO_Port GPIOC
#define PHASE_C_UPPER_Pin GPIO_PIN_2
#define PHASE_C_UPPER_GPIO_Port GPIOC
#define PHASE_A_CURRENT_Pin GPIO_PIN_0
#define PHASE_A_CURRENT_GPIO_Port GPIOA
#define PHASE_B_CURRENT_Pin GPIO_PIN_1
#define PHASE_B_CURRENT_GPIO_Port GPIOA
#define MOTOR_VOLTAGE_Pin GPIO_PIN_2
#define MOTOR_VOLTAGE_GPIO_Port GPIOA
#define ANALOG_IN_1_Pin GPIO_PIN_6
#define ANALOG_IN_1_GPIO_Port GPIOA
#define ANALOG_IN_2_Pin GPIO_PIN_7
#define ANALOG_IN_2_GPIO_Port GPIOA
#define MOTOR_TEMP_Pin GPIO_PIN_4
#define MOTOR_TEMP_GPIO_Port GPIOC
#define LED_GREEN_Pin GPIO_PIN_1
#define LED_GREEN_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_2
#define LED_RED_GPIO_Port GPIOB
#define PHASE_A_LOWER_Pin GPIO_PIN_13
#define PHASE_A_LOWER_GPIO_Port GPIOB
#define PHASE_B_LOWER_Pin GPIO_PIN_14
#define PHASE_B_LOWER_GPIO_Port GPIOB
#define PHASE_C_LOWER_Pin GPIO_PIN_15
#define PHASE_C_LOWER_GPIO_Port GPIOB
#define ENCODER_A_Pin GPIO_PIN_6
#define ENCODER_A_GPIO_Port GPIOC
#define ENCODER_B_Pin GPIO_PIN_7
#define ENCODER_B_GPIO_Port GPIOC
#define SCI_DIR_Pin GPIO_PIN_8
#define SCI_DIR_GPIO_Port GPIOA
#define SCI_TX_Pin GPIO_PIN_9
#define SCI_TX_GPIO_Port GPIOA
#define SCI_RX_Pin GPIO_PIN_10
#define SCI_RX_GPIO_Port GPIOA
#define BLE_SPI_SCK_Pin GPIO_PIN_10
#define BLE_SPI_SCK_GPIO_Port GPIOC
#define BLE_SPI_MISO_Pin GPIO_PIN_11
#define BLE_SPI_MISO_GPIO_Port GPIOC
#define BLE_SPI_MOSI_Pin GPIO_PIN_12
#define BLE_SPI_MOSI_GPIO_Port GPIOC
#define BLE_SPI_CS_Pin GPIO_PIN_2
#define BLE_SPI_CS_GPIO_Port GPIOD
#define BLE_SPI_IRQ_Pin GPIO_PIN_4
#define BLE_SPI_IRQ_GPIO_Port GPIOB
#define BLE_SPI_IRQ_EXTI_IRQn EXTI4_IRQn
#define BLE_SPI_RESET_Pin GPIO_PIN_5
#define BLE_SPI_RESET_GPIO_Port GPIOB
#define ENCODER_Z_Pin GPIO_PIN_6
#define ENCODER_Z_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
