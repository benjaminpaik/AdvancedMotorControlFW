/*
 * foc.c
 *
 *  Created on: Jun 23, 2024
 *      Author: benja
 */

#include "foc.h"

inline void clarke_transform(FOC* restrict foc, float_t phase_a, float_t phase_b)
{
  foc->clarke.alpha = phase_a;
  foc->clarke.beta = ((phase_a + (phase_b * 2)) * INV_SQRT3);
}

inline void park_transform(FOC* restrict foc, float_t angle)
{
  foc->park.sin = sinf(angle);
  foc->park.cos = cosf(angle);

  foc->park.ds = (foc->clarke.alpha * foc->park.cos) +
                 (foc->clarke.beta * foc->park.sin);

  foc->park.dq = (foc->clarke.beta * foc->park.cos) -
                 (foc->clarke.alpha * foc->park.sin);
}

inline void ipark_transform(FOC* restrict foc, float_t vd, float_t vq)
{
  foc->ipark.alpha = (vd * foc->park.cos) - (vq * foc->park.sin);
  foc->ipark.beta = (vq * foc->park.cos) + (vd * foc->park.sin);
}

inline void sv_pwm_update(SV_PWM* restrict sv, float_t alpha, float_t beta)
{
  sv->temp1 = beta;
  sv->temp2 = (beta / 2) + (alpha * COS_30);
  sv->temp3 = sv->temp2 - sv->temp1;

  sv->sector = 3;
  sv->sector = (sv->temp2 > 0) ? (sv->sector - 1) : sv->sector;
  sv->sector = (sv->temp3 > 0) ? (sv->sector - 1) : sv->sector;
  sv->sector = (sv->temp1 < 0) ? (7 - sv->sector) : sv->sector;

  switch(sv->sector) {

  case(1):
  case(4):
    sv->ta = sv->temp2;
    sv->tb = sv->temp1 - sv->temp3;
    sv->tc = -sv->temp2;
    break;

  case(2):
  case(5):
    sv->ta = sv->temp3 + sv->temp2;
    sv->tb = sv->temp1;
    sv->tc = -sv->temp1;
    break;

  default:
    sv->ta = sv->temp3;
    sv->tb = -sv->temp3;
    sv->tc = -(sv->temp1 + sv->temp2);
    break;
  }

}
