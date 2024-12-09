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

void disable_drive(MOTOR_DRIVE *drive);

void init_motor_drive(MOTOR_DRIVE *drive, TIM_HandleTypeDef *pwm_tim)
{
  drive->pwm_tim = pwm_tim;
  drive->enable = FALSE;

  drive->half_period = drive->pwm_tim->Init.Period / 2;

  foc_init(&drive->foc, 0.2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  disable_drive(drive);
}

void init_encoder(ENCODER *encoder, TIM_HandleTypeDef *encoder_tim)
{
  encoder->tim = encoder_tim;
  HAL_TIM_Encoder_Start(encoder_tim, TIM_CHANNEL_ALL);
}

void enable_drive(MOTOR_DRIVE *drive, uint8_t enable)
{
  drive->enable = enable;
  if(!enable) {
    disable_drive(drive);
  }
}

void drive_control(MOTOR_DRIVE *drive, float_t phase_a, float_t phase_b)
{
  update_encoder_position(&drive->encoder);

  foc_input(&drive->foc, phase_a, phase_b, drive->encoder.out);
  foc_clarke(&drive->foc);
  foc_park(&drive->foc);

  if(drive->enable) {
    pi_control(&drive->id_pid, 0, drive->foc.park.id, PID_IN_RANGE);
    pi_control(&drive->iq_pid, drive->torque_cmd, drive->foc.park.iq, PID_IN_RANGE);

    foc_ipark(&drive->foc, drive->id_pid.out, drive->iq_pid.out);
    foc_svpwm(&drive->foc);
    update_pwm_outputs(drive);
  }
}

void update_pwm_outputs(MOTOR_DRIVE* drive)
{
  drive->compare_1 = (drive->foc.sv.t1 * drive->half_period) + drive->half_period;
  drive->compare_2 = (drive->foc.sv.t2 * drive->half_period) + drive->half_period;
  drive->compare_3 = (drive->foc.sv.t3 * drive->half_period) + drive->half_period;

  if(drive->compare_1 < 0) drive->compare_1 = 0;
  if(drive->compare_2 < 0) drive->compare_2 = 0;
  if(drive->compare_3 < 0) drive->compare_3 = 0;

  __HAL_TIM_SET_COMPARE(drive->pwm_tim, TIM_CHANNEL_1, drive->compare_1);
  __HAL_TIM_SET_COMPARE(drive->pwm_tim, TIM_CHANNEL_2, drive->compare_2);
  __HAL_TIM_SET_COMPARE(drive->pwm_tim, TIM_CHANNEL_3, drive->compare_3);
}

void cal_angle(MOTOR_DRIVE *drive)
{
  SET_PWM1_LOW(drive->pwm_tim->Instance);
  SET_PWM2_ON(drive->pwm_tim->Instance);
  SET_PWM3_ON(drive->pwm_tim->Instance);
}

void disable_drive(MOTOR_DRIVE *drive)
{
  SET_PWM1_OFF(drive->pwm_tim->Instance);
  SET_PWM2_OFF(drive->pwm_tim->Instance);
  SET_PWM3_OFF(drive->pwm_tim->Instance);
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
