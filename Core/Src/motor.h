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
    volatile uint8_t enable;
    TIM_HandleTypeDef *pwm_tim;
    TIM_HandleTypeDef *hall_tim;
    int8_t polarity;
    int32_t pwm_command;
    uint16_t compare;
    int16_t cmd_state;

    volatile float_t period;
    int8_t hall_polarity;
    int8_t hall_index;
    int8_t hall_state_delta;

    volatile uint32_t encoder_position;

    volatile int8_t hall_state;
    volatile int8_t hall_state_previous;
    volatile int32_t position;
    int32_t position_previous;
    int32_t position_delta;
    float_t velocity;
} TRAP_DRIVE;

void init_trap_drive(TRAP_DRIVE *trap_drive, TIM_HandleTypeDef *pwm_tim, TIM_HandleTypeDef *hall_tim, int32_t direction);
void update_hall_state(TRAP_DRIVE *trap_drive);
void update_hall_velocity(TRAP_DRIVE *trap_drive, float_t gain);

#endif /* INC_MOTOR_H_ */
