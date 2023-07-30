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

#define HALL_ROLLOVER         -5

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
  int8_t polarity;
  int8_t index;
  int8_t state_delta;

  volatile int8_t state;
  volatile int8_t state_previous;
  volatile int32_t position;
  volatile float_t period;

  int32_t position_previous;
  int32_t position_delta;
  float_t velocity;
} HALL_SENSORS;

typedef struct {
  TIM_HandleTypeDef *tim;
  volatile uint32_t position;
} ENCODER;

typedef struct {
  HALL_SENSORS hall;
  ENCODER encoder;

  volatile uint8_t enable;
  TIM_HandleTypeDef *pwm_tim;
  int8_t polarity;
  int32_t pwm_command;
  uint16_t compare;
  int16_t cmd_state;
} TRAP_DRIVE;

void init_trap_drive(TRAP_DRIVE *trap_drive, TIM_HandleTypeDef *pwm_tim, TIM_HandleTypeDef *hall_tim, int32_t direction);
void init_hall_sensors(HALL_SENSORS *hall, TIM_HandleTypeDef *hall_tim);
void init_encoder(ENCODER *encoder, TIM_HandleTypeDef *encoder_tim);
void enable_trap_drive(TRAP_DRIVE *trap_drive, uint8_t enable);
void update_pwm_cmd(TRAP_DRIVE *trap_drive, float_t command);
void update_state_cmd(TRAP_DRIVE *trap_drive);
void disable_trap_drive(TRAP_DRIVE *trap_drive);
void update_hall_state(HALL_SENSORS *hall);
void update_hall_velocity(HALL_SENSORS *hall, float_t gain);
void update_encoder_position(ENCODER *encoder);

#endif /* INC_MOTOR_H_ */
