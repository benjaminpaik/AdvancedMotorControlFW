/*
 * motor.c
 *
 *  Created on: Nov 20, 2019
 *      Author: bpaik
 */

#include "motor.h"
#include "tim.h"
#include "stdlib.h"
#include "limits.h"

//const uint8_t g_hall_table[] = {0, 4, 6, 5, 2, 3, 1, 0};
const uint8_t g_hall_table[] = {0, 4, 2, 3, 6, 5, 1, 0};

void init_trap_drive(TRAP_DRIVE *trap_drive, TIM_HandleTypeDef *pwm_tim, TIM_HandleTypeDef *hall_tim, int32_t direction)
{
  trap_drive->pwm_tim = pwm_tim;
  trap_drive->hall_tim = hall_tim;
  // set default state data
  trap_drive->cmd_state = 1;
  trap_drive->hall_index = (HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin) << 2) |
                           (HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin) << 1) |
                           (HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin));

  trap_drive->hall_state = g_hall_table[trap_drive->hall_index];
  trap_drive->hall_polarity = 1;
  trap_drive->period = 0;

  // initialize duty cycle variables
  trap_drive->pwm_command = 0;
  trap_drive->compare = 0;
  trap_drive->enable = 0;

  if(direction > 0) {
    trap_drive->polarity = 1;
  }
  else {
    trap_drive->polarity = -1;
  }

  disable_trap_drive(trap_drive);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void update_hall_state(TRAP_DRIVE *trap_drive)
{
  trap_drive->period = __HAL_TIM_GET_COUNTER(trap_drive->hall_tim);
  __HAL_TIM_SET_COUNTER(trap_drive->hall_tim, 0);
  HAL_TIM_Base_Start(trap_drive->hall_tim);

  trap_drive->hall_index = (HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin) << 2) |
                           (HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin) << 1) |
                           (HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin));

  trap_drive->hall_state = g_hall_table[trap_drive->hall_index];
  trap_drive->hall_state_delta = trap_drive->hall_state - trap_drive->hall_state_previous;

  if(trap_drive->hall_state_delta == HALL_ROLLOVER || trap_drive->hall_state_delta == 1) {
    trap_drive->position++;
  }
  else {
    trap_drive->position--;
  }
  trap_drive->hall_state_previous = trap_drive->hall_state;
}

void update_hall_velocity(TRAP_DRIVE *trap_drive, float_t gain)
{
  uint32_t counter = __HAL_TIM_GET_COUNTER(trap_drive->hall_tim);
  trap_drive->position_delta = trap_drive->position - trap_drive->position_previous;

  // valid hall state transition and new velocity measurement
  if(trap_drive->position_delta != 0) {
    trap_drive->hall_polarity = (trap_drive->position_delta > 0) ? 1 : -1;
    if(trap_drive->period > 0) {
      trap_drive->velocity = (gain / trap_drive->period) * trap_drive->hall_polarity;
    }
  }
  // no hall state transition and more time has passed since the previous sample
  else if(counter > trap_drive->period) {
    trap_drive->velocity = (gain / counter) * trap_drive->hall_polarity;
  }
  // timeout
  else if(counter == 0) {
    trap_drive->velocity = 0;
  }
  // store the previous hall state value
  trap_drive->position_previous = trap_drive->position;
}
