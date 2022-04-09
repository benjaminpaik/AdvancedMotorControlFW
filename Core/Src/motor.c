/*
 * motor.c
 *
 *  Created on: Nov 20, 2019
 *      Author: bpaik
 */

#include "definitions.h"
#include "motor.h"
#include "dsp.h"
#include "tim.h"
#include "stdlib.h"
#include "limits.h"

void disable_trap_drive(TRAP_DRIVE *trap_drive);
void init_hall_sensors(HALL_SENSORS *hall, TIM_HandleTypeDef *hall_tim);

void init_trap_drive(TRAP_DRIVE *trap_drive, TIM_HandleTypeDef *pwm_tim, TIM_HandleTypeDef *hall_tim, int32_t direction)
{
  init_hall_sensors(&trap_drive->hall, hall_tim);

  trap_drive->pwm_tim = pwm_tim;
  // set default state data
  trap_drive->cmd_state = 1;
  // initialize duty cycle variables
  trap_drive->pwm_command = 0;
  trap_drive->compare = 0;
  trap_drive->enable = FALSE;

  // scaling factor for normalized 12-bit duty cycle to compare value
  trap_drive->scale = pwm_tim->Init.Period;

  if(direction > 0) {
    trap_drive->polarity = 1;
  }
  else {
    trap_drive->polarity = -1;
  }

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  disable_trap_drive(trap_drive);
}

void init_hall_sensors(HALL_SENSORS *hall, TIM_HandleTypeDef *hall_tim)
{
  hall->tim = hall_tim;
  hall->index = (HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin) << 2) |
                (HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin) << 1) |
                (HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin));

  hall->map[0] = 0;
  hall->map[1] = 3;
  hall->map[2] = 5;
  hall->map[3] = 4;
  hall->map[4] = 1;
  hall->map[5] = 2;
  hall->map[6] = 6;

  hall->state = hall->map[hall->index];
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

    if(enable) {
      update_state_cmd(trap_drive);
    }
    else {
      disable_trap_drive(trap_drive);
    }
  }
}

void update_pwm_cmd(TRAP_DRIVE *trap_drive, float command)
{
 trap_drive->pwm_command = LIMIT(command, 1.0F, -1.0F) * trap_drive->polarity;
 trap_drive->compare = abs(trap_drive->scale * trap_drive->pwm_command);

 __HAL_TIM_SET_COMPARE(trap_drive->pwm_tim, TIM_CHANNEL_1, trap_drive->compare);
 __HAL_TIM_SET_COMPARE(trap_drive->pwm_tim, TIM_CHANNEL_2, trap_drive->compare);
 __HAL_TIM_SET_COMPARE(trap_drive->pwm_tim, TIM_CHANNEL_3, trap_drive->compare);
}

void update_state_cmd(TRAP_DRIVE *trap_drive)
{
  if(trap_drive->enable) {

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
      SET_PWM3_OFF(trap_drive->pwm_tim->Instance);
      SET_PWM2_LOW(trap_drive->pwm_tim->Instance);
      SET_PWM1_ON(trap_drive->pwm_tim->Instance);
      break;

    case(2):
      SET_PWM2_OFF(trap_drive->pwm_tim->Instance);
      SET_PWM3_LOW(trap_drive->pwm_tim->Instance);
      SET_PWM1_ON(trap_drive->pwm_tim->Instance);
      break;

    case(3):
      SET_PWM1_OFF(trap_drive->pwm_tim->Instance);
      SET_PWM3_LOW(trap_drive->pwm_tim->Instance);
      SET_PWM2_ON(trap_drive->pwm_tim->Instance);
      break;

    case(4):
      SET_PWM3_OFF(trap_drive->pwm_tim->Instance);
      SET_PWM1_LOW(trap_drive->pwm_tim->Instance);
      SET_PWM2_ON(trap_drive->pwm_tim->Instance);
      break;

    case(5):
      SET_PWM2_OFF(trap_drive->pwm_tim->Instance);
      SET_PWM1_LOW(trap_drive->pwm_tim->Instance);
      SET_PWM3_ON(trap_drive->pwm_tim->Instance);
      break;

    case(6):
      SET_PWM1_OFF(trap_drive->pwm_tim->Instance);
      SET_PWM2_LOW(trap_drive->pwm_tim->Instance);
      SET_PWM3_ON(trap_drive->pwm_tim->Instance);
      break;

    default:
      break;
    }
  }
}

void update_current(TRAP_DRIVE *trap_drive, float phase_a, float phase_b)
{
  switch(trap_drive->cmd_state) {

  case(1):
    trap_drive->current = CURRENT_SCALE(phase_a);
    break;

  case(2):
    trap_drive->current = CURRENT_SCALE(phase_a);
    break;

  case(3):
    trap_drive->current = CURRENT_SCALE(phase_b);
    break;

  case(4):
    trap_drive->current = CURRENT_SCALE(phase_b);
    break;

  case(5):
    trap_drive->current = -CURRENT_SCALE(phase_a);
    break;

  case(6):
    trap_drive->current = -CURRENT_SCALE(phase_b);
    break;

  default:
    trap_drive->current =  CURRENT_SCALE(phase_a);
    break;
  }
}

int32_t update_trap_cal(TRAP_DRIVE *trap_drive)
{
  static PERSISTENCE persistence = {.threshold = 500, .latch = FALSE};
  trap_drive->enable = FALSE;

  // check for valid state command
  if(trap_drive->cal_state >= 1 && trap_drive->cal_state <= 6) {
    // if the state has not changed for one second
    if(persistence_counter(&persistence, (trap_drive->hall.index_previous == trap_drive->hall.index))) {
      // reset the persistence counter
      persistence.timer = 0;
      // the hall index is in bounds
      if(trap_drive->hall.index <= HALL_MAX) {
        // the hall index corresponds to the commanded state
        trap_drive->hall.map[trap_drive->hall.index] = trap_drive->cal_state;
      }
      // command the next hall state
      trap_drive->cal_state++;
    }
    // store the previous hall states
    trap_drive->hall.index_previous = trap_drive->hall.index;
  }

  // configure PWM registers based on the commanded state
  switch(trap_drive->cal_state) {

  case(1):
    // load duty cycle
    SET_PWM1_ON(trap_drive->pwm_tim->Instance);
    SET_PWM3_ON(trap_drive->pwm_tim->Instance);
      // phase lowers
    SET_PWM2_LOW(trap_drive->pwm_tim->Instance);
    break;

  case(2):
    // load duty cycle
    SET_PWM1_ON(trap_drive->pwm_tim->Instance);
    // phase lowers
    SET_PWM2_LOW(trap_drive->pwm_tim->Instance);
    SET_PWM3_LOW(trap_drive->pwm_tim->Instance);
    break;

  case(3):
    // load duty cycle
    SET_PWM1_ON(trap_drive->pwm_tim->Instance);
    SET_PWM2_ON(trap_drive->pwm_tim->Instance);
    // phase lowers
    SET_PWM3_LOW(trap_drive->pwm_tim->Instance);
    break;

  case(4):
    // load duty cycle
    SET_PWM2_ON(trap_drive->pwm_tim->Instance);
    // phase lowers
    SET_PWM1_LOW(trap_drive->pwm_tim->Instance);
    SET_PWM3_LOW(trap_drive->pwm_tim->Instance);
    break;

  case(5):
    // load duty cycle
    SET_PWM2_ON(trap_drive->pwm_tim->Instance);
    SET_PWM3_ON(trap_drive->pwm_tim->Instance);
    // phase lowers
    SET_PWM1_LOW(trap_drive->pwm_tim->Instance);
    break;

  case(6):
    // load duty cycle
    SET_PWM3_ON(trap_drive->pwm_tim->Instance);
    // phase lowers
    SET_PWM1_LOW(trap_drive->pwm_tim->Instance);
    SET_PWM2_LOW(trap_drive->pwm_tim->Instance);
    break;

  default:
    disable_trap_drive(trap_drive);
    // reset the state command
    trap_drive->cal_state = 0;

    // check for valid hall mapping
    if(trap_drive->hall.map[1] + trap_drive->hall.map[2] + trap_drive->hall.map[3] +
       trap_drive->hall.map[4] + trap_drive->hall.map[5] + trap_drive->hall.map[6] == HALL_MAP_SUM) {
      // generate the hall map value
      return ((int32_t)trap_drive->hall.map[6] << 20) |
             ((int32_t)trap_drive->hall.map[5] << 16) |
             ((int32_t)trap_drive->hall.map[4] << 12) |
             ((int32_t)trap_drive->hall.map[3] << 8)  |
             ((int32_t)trap_drive->hall.map[2] << 4)  |
             ((int32_t)trap_drive->hall.map[1]);

    }
    // invalid hall mapping
    else {
      // restart calibration sequence
      trap_drive->cal_state = 1;
    }
  }
  // hall map not yet determined
  return 0;
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

  hall->state = hall->map[hall->index];
  hall->state_delta = hall->state - hall->state_previous;

  if(hall->state_delta == HALL_ROLLOVER || hall->state_delta == 1) {
    hall->position++;
  }
  else {
    hall->position--;
  }
  hall->state_previous = hall->state;
}

void update_hall_velocity(HALL_SENSORS *hall, float gain)
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
  encoder->position += encoder->offset;

  if(encoder->position > (int32_t)encoder->tim->Init.Period) {
    encoder->position -= (encoder->tim->Init.Period + 1);
  }
  else if(encoder->position < 0) {
    encoder->position += (encoder->tim->Init.Period + 1);
  }
  encoder->position -= ((encoder->tim->Init.Period + 1) >> 1);
  encoder->out = encoder->gain * encoder->position;
}

float scale_voltage_command(float u)
{
  float pwm;
  pwm = (u / INPUT_VOLTAGE) * 1.2F;
  pwm = LIMIT(pwm, 0.25, -0.25);
  return pwm;
}
