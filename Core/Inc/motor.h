/*
 * motor.h
 *
 *  Created on: Nov 20, 2019
 *      Author: bpaik
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32g4xx_hal.h"
#include "main.h"

// hall index limits
#define HALL_MAX    6
#define HALL_MIN    1
#define HALL_ROLLOVER         -5
// expected sum of the valid hall indices
#define HALL_MAP_SUM  21

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
  int8_t index_previous;
  int8_t index;
  int8_t state_delta;

  uint16_t map[7];
  volatile int8_t state;
  volatile int8_t state_previous;
  volatile int32_t position;
  volatile float period;

  int32_t position_previous;
  int32_t position_delta;
  float velocity;
} HALL_SENSORS;

typedef struct {
  TIM_HandleTypeDef *tim;
  volatile int32_t position;
  volatile float out;
  float gain;
  int16_t offset;
} ENCODER;

typedef struct {
  HALL_SENSORS hall;
  ENCODER encoder;

  int32_t scale;
  volatile uint8_t enable;
  TIM_HandleTypeDef *pwm_tim;
  int8_t polarity;
  float pwm_command;
  uint16_t compare;
  int16_t cmd_state;
  int16_t cal_state;

  float current;
} TRAP_DRIVE;

void init_trap_drive(TRAP_DRIVE *trap_drive, TIM_HandleTypeDef *pwm_tim, TIM_HandleTypeDef *hall_tim, int32_t direction);
void init_encoder(ENCODER *encoder, TIM_HandleTypeDef *encoder_tim);
void enable_trap_drive(TRAP_DRIVE *trap_drive, uint8_t enable);
void update_state_cmd(TRAP_DRIVE *trap_drive);
void update_pwm_cmd(TRAP_DRIVE *trap_drive, float command);
__attribute__ ((long_call, section (".RamFunc"))) void update_current(TRAP_DRIVE *trap_drive, float phase_a, float phase_b);
int32_t update_trap_cal(TRAP_DRIVE *trap_drive);
__attribute__ ((long_call, section (".RamFunc"))) void update_hall_state(HALL_SENSORS *hall);
void update_hall_velocity(HALL_SENSORS *hall, float gain);
void update_encoder_position(ENCODER *encoder);
float scale_voltage_command(float u);

#endif /* INC_MOTOR_H_ */
