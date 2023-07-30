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
  init_hall_sensors(&trap_drive->hall, hall_tim);

  trap_drive->pwm_tim = pwm_tim;
  // set default state data
  trap_drive->cmd_state = 1;
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

void init_hall_sensors(HALL_SENSORS *hall, TIM_HandleTypeDef *hall_tim)
{
  hall->tim = hall_tim;
  hall->index = (HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin) << 2) |
                (HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin) << 1) |
                (HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin));

  hall->state = g_hall_table[hall->index];
  hall->polarity = 1;
  hall->period = 0;
}

void init_encoder(ENCODER *encoder, TIM_HandleTypeDef *encoder_tim)
{
  encoder->tim = encoder_tim;
  HAL_TIM_Encoder_Start(encoder_tim, TIM_CHANNEL_ALL);
}

void enable_trap_drive(TRAP_DRIVE *trap_drive, uint8_t enable)
{
  if(enable != trap_drive->enable) {
    trap_drive->enable = enable;
    update_state_cmd(trap_drive);
  }
}

void update_pwm_cmd(TRAP_DRIVE *trap_drive, float_t command)
{
 trap_drive->pwm_command = command * trap_drive->polarity;
 trap_drive->compare = abs(trap_drive->pwm_tim->Init.Period * trap_drive->pwm_command);

 __HAL_TIM_SET_COMPARE(trap_drive->pwm_tim, TIM_CHANNEL_1, trap_drive->compare);
 __HAL_TIM_SET_COMPARE(trap_drive->pwm_tim, TIM_CHANNEL_2, trap_drive->compare);
 __HAL_TIM_SET_COMPARE(trap_drive->pwm_tim, TIM_CHANNEL_3, trap_drive->compare);
}

void update_state_cmd(TRAP_DRIVE *trap_drive)
{
  if(!trap_drive->enable) {
    disable_trap_drive(trap_drive);
    return;
  }

  if(trap_drive->pwm_command >= 0) {
    trap_drive->cmd_state = trap_drive->hall.state + 1;
    if(trap_drive->cmd_state > 6) {
      trap_drive->cmd_state -= 6;
    }
  }
  else {
    trap_drive->cmd_state = trap_drive->hall.state - 2;
    if(trap_drive->cmd_state < 1) {
      trap_drive->cmd_state += 6;
    }
  }

  // configure PWM registers based on the commanded state
  switch(trap_drive->cmd_state) {

  case(1):
    // disable phase
    SET_PWM3_OFF(trap_drive->pwm_tim->Instance);
    // phase lowers
    SET_PWM2_LOW(trap_drive->pwm_tim->Instance);
    // PWM phase
    SET_PWM1_ON(trap_drive->pwm_tim->Instance);
    break;

  case(2):
    // disable phase
    SET_PWM2_OFF(trap_drive->pwm_tim->Instance);
    // phase lowers
    SET_PWM3_LOW(trap_drive->pwm_tim->Instance);
    // PWM phase
    SET_PWM1_ON(trap_drive->pwm_tim->Instance);
    break;

  case(3):
    // disable phase
    SET_PWM1_OFF(trap_drive->pwm_tim->Instance);
    // phase lowers
    SET_PWM3_LOW(trap_drive->pwm_tim->Instance);
    // PWM phase
    SET_PWM2_ON(trap_drive->pwm_tim->Instance);
    break;

  case(4):
    // disable phase
    SET_PWM3_OFF(trap_drive->pwm_tim->Instance);
    // phase lowers
    SET_PWM1_LOW(trap_drive->pwm_tim->Instance);
    // PWM phase
    SET_PWM2_ON(trap_drive->pwm_tim->Instance);
    break;

  case(5):
    // disable phase
    SET_PWM2_OFF(trap_drive->pwm_tim->Instance);
    // phase lowers
    SET_PWM1_LOW(trap_drive->pwm_tim->Instance);
    // PWM phase
    SET_PWM3_ON(trap_drive->pwm_tim->Instance);
    break;

  case(6):
    // disable phase
    SET_PWM1_OFF(trap_drive->pwm_tim->Instance);
    // phase lowers
    SET_PWM2_LOW(trap_drive->pwm_tim->Instance);
    // PWM phase
    SET_PWM3_ON(trap_drive->pwm_tim->Instance);
    break;

  default:
    break;
  }
}

void disable_trap_drive(TRAP_DRIVE *trap_drive)
{
  SET_PWM1_OFF(trap_drive->pwm_tim->Instance);
  SET_PWM2_OFF(trap_drive->pwm_tim->Instance);
  SET_PWM3_OFF(trap_drive->pwm_tim->Instance);
}

void update_hall_state(HALL_SENSORS *hall)
{
  hall->period = __HAL_TIM_GET_COUNTER(hall->tim);
  __HAL_TIM_SET_COUNTER(hall->tim, 0);
  HAL_TIM_Base_Start(hall->tim);

  hall->index = (HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin) << 2) |
                (HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin) << 1) |
                (HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin));

  hall->state = g_hall_table[hall->index];
  hall->state_delta = hall->state - hall->state_previous;

  if(hall->state_delta == HALL_ROLLOVER || hall->state_delta == 1) {
    hall->position++;
  }
  else {
    hall->position--;
  }
  hall->state_previous = hall->state;
}

void update_hall_velocity(HALL_SENSORS *hall, float_t gain)
{
  uint32_t counter = __HAL_TIM_GET_COUNTER(hall->tim);
  hall->position_delta = hall->position - hall->position_previous;

  // valid hall state transition and new velocity measurement
  if(hall->position_delta != 0) {
    hall->polarity = (hall->position_delta > 0) ? 1 : -1;
    if(hall->period > 0) {
      hall->velocity = (gain / hall->period) * hall->polarity;
    }
  }
  // no hall state transition and more time has passed since the previous sample
  else if(counter > hall->period) {
    hall->velocity = (gain / counter) * hall->polarity;
  }
  // timeout
  else if(counter == 0) {
    hall->velocity = 0;
  }
  // store the previous hall state value
  hall->position_previous = hall->position;
}

void update_encoder_position(ENCODER *encoder)
{
  encoder->position = encoder->tim->Instance->CNT;
}
