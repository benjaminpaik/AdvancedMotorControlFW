/*
 * motor.h
 *
 *  Created on: Nov 20, 2019
 *      Author: bpaik
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32g4xx_hal.h"
#include "math.h"
#include "main.h"
#include "foc.h"
#include "dsp.h"

#define SET_PWM1_ON(I)     (I->CCER |= (TIM_CCER_CC1E_Msk | TIM_CCER_CC1NE_Msk)); \
                           (I->CCMR1 = (I->CCMR1 & ~TIM_CCMR1_OC1M_Msk) | TIM_OCMODE_PWM1)
#define SET_PWM2_ON(I)     (I->CCER |= (TIM_CCER_CC2E_Msk | TIM_CCER_CC2NE_Msk)); \
                           (I->CCMR1 = (I->CCMR1 & ~TIM_CCMR1_OC2M_Msk) | (TIM_OCMODE_PWM1 << 8))
#define SET_PWM3_ON(I)     (I->CCER |= (TIM_CCER_CC3E_Msk | TIM_CCER_CC3NE_Msk)); \
                           (I->CCMR2 = (I->CCMR2 & ~TIM_CCMR2_OC3M_Msk) | TIM_OCMODE_PWM1)

#define SET_PWM1_LOW(I)     (I->CCER |= (TIM_CCER_CC1E_Msk | TIM_CCER_CC1NE_Msk)); \
                            (I->CCMR1 = ((I->CCMR1 & ~TIM_CCMR1_OC1M_Msk) | TIM_OCMODE_FORCED_INACTIVE))
#define SET_PWM2_LOW(I)     (I->CCER |= (TIM_CCER_CC2E_Msk | TIM_CCER_CC2NE_Msk)); \
                            (I->CCMR1 = ((I->CCMR1 & ~TIM_CCMR1_OC2M_Msk) | (TIM_OCMODE_FORCED_INACTIVE << 8)))
#define SET_PWM3_LOW(I)     (I->CCER |= (TIM_CCER_CC3E_Msk | TIM_CCER_CC3NE_Msk)); \
                            (I->CCMR2 = ((I->CCMR2 & ~TIM_CCMR2_OC3M_Msk) | TIM_OCMODE_FORCED_INACTIVE))

#define SET_PWM1_OFF(I)     (I->CCER = (I->CCER & ~TIM_CCER_CC1E_Msk) | TIM_CCER_CC1NE); \
                            (I->CCMR1 = (I->CCMR1 & ~TIM_CCMR1_OC1M_Msk) | TIM_OCMODE_FORCED_INACTIVE)
#define SET_PWM2_OFF(I)     (I->CCER = (I->CCER & ~TIM_CCER_CC2E_Msk) | TIM_CCER_CC2NE); \
                            (I->CCMR1 = (I->CCMR1 & ~TIM_CCMR1_OC2M_Msk) | (TIM_OCMODE_FORCED_INACTIVE << 8))
#define SET_PWM3_OFF(I)     (I->CCER = (I->CCER & ~TIM_CCER_CC3E_Msk) | TIM_CCER_CC3NE); \
                            (I->CCMR2 = (I->CCMR2 & ~TIM_CCMR2_OC3M_Msk) | TIM_OCMODE_FORCED_INACTIVE)

typedef struct {
  TIM_HandleTypeDef *tim;
  volatile int32_t position;
  volatile float_t out;
  float_t gain;
  int16_t offset;
} ENCODER;

typedef struct {
  ENCODER encoder;
  FOC foc;
  PID iq_pid;
  PID id_pid;

  volatile float_t torque_cmd;

  volatile int16_t compare_1;
  volatile int16_t compare_2;
  volatile int16_t compare_3;

  volatile uint8_t enable;
  TIM_HandleTypeDef *pwm_tim;
  float_t half_period;
} MOTOR_DRIVE;

void init_motor_drive(MOTOR_DRIVE* drive, TIM_HandleTypeDef* pwm_tim);
void init_encoder(ENCODER* encoder, TIM_HandleTypeDef* encoder_tim);
void enable_drive(MOTOR_DRIVE* drive, uint8_t enable);
__attribute__ ((long_call, section (".RamFunc"))) void drive_control(MOTOR_DRIVE* drive, float_t phase_a, float_t phase_b, float_t vdc);
void update_pwm_outputs(MOTOR_DRIVE* drive);
void cal_angle(MOTOR_DRIVE* drive);
void update_encoder_position(ENCODER* encoder);

#endif /* INC_MOTOR_H_ */
