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

#define SQRT3       1.732051F
#define COS_30      0.866F

typedef struct {
  volatile float_t phase_a;
  volatile float_t phase_b;
  volatile float_t sin;
  volatile float_t cos;
} FOC_INPUTS;

typedef struct {
  volatile float_t alpha;
  volatile float_t beta;
} FOC_CLARKE;

typedef struct {
  volatile float_t ds;
  volatile float_t dq;
} FOC_PARK;

typedef struct {
  volatile float_t alpha;
  volatile float_t beta;
  volatile float_t vd_limited;
  volatile float_t vq_limited;
  float_t vd_limit;
  float_t vq_limit;
} FOC_IPARK;

typedef struct {
  float_t a, b, c;
  uint16_t sector;
  volatile float_t t1;
  volatile float_t t2;
  volatile float_t t3;
} SV_PWM;

typedef struct {
  FOC_INPUTS input;
  FOC_CLARKE clarke;
  FOC_PARK park;
  FOC_IPARK ipark;
  SV_PWM sv;
} FOC;


void foc_input(FOC* restrict foc, float_t phase_a, float_t phase_b, float_t angle);
void foc_clarke(FOC* restrict foc);
void foc_park(FOC* restrict foc);
void foc_ipark(FOC* restrict foc, float_t vd, float_t vq);
void foc_svpwm(FOC* restrict foc);

#endif /* INC_FOC_H_ */
