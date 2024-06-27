/*
 * foc.h
 *
 *  Created on: Jun 23, 2024
 *      Author: benja
 */

#ifndef INC_FOC_H_
#define INC_FOC_H_

#include "stm32g4xx_hal.h"
#include "math.h"

#define INV_SQRT3   0.57735F
#define COS_30      0.866F

typedef struct {
  float_t alpha;
  float_t beta;
} FOC_CLARKE;

typedef struct {
  float_t sin;
  float_t cos;
  float_t ds;
  float_t dq;
} FOC_PARK;

typedef struct {
  float_t alpha;
  float_t beta;
} FOC_IPARK;

typedef struct {
  float_t ta, tb, tc;
  float_t temp1, temp2, temp3;
  uint16_t sector;
} SV_PWM;

typedef struct {
  FOC_CLARKE clarke;
  FOC_PARK park;
  FOC_IPARK ipark;
  SV_PWM sv_pwm;
} FOC;

inline void clarke_transform(FOC* foc, float_t phase_a, float_t phase_b);
inline void park_transform(FOC* foc, float_t angle);
inline void ipark_transform(FOC* restrict foc, float_t vd, float_t vq);
inline void sv_pwm_update(SV_PWM* restrict sv, float_t alpha, float_t beta);

#endif /* INC_FOC_H_ */
