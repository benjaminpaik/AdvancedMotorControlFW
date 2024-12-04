/*
 * foc.c
 *
 *  Created on: Feb 23, 2023
 *      Author: benja
 */

#include "foc.h"
#include "dsp.h"

void foc_init(FOC* restrict foc, float_t vd_limit, float_t vq_limit)
{
  float_t vq_limit_max;
  foc->ipark.vd_limit = limit_f(vd_limit, 1.0F, 0.0F);
  vq_limit_max = sqrtf(1.0F - (foc->ipark.vd_limit * foc->ipark.vd_limit));
  foc->ipark.vq_limit = limit_f(vq_limit, vq_limit_max, 0.0F);
}

void foc_input(FOC* restrict foc, float_t phase_a, float_t phase_b, float_t angle)
{
  foc->input.phase_a = phase_a;
  foc->input.phase_b = phase_b;

  foc->input.sin = sinf(angle);
  foc->input.cos = cosf(angle);
}

void foc_clarke(FOC* restrict foc)
{
  foc->clarke.alpha = foc->input.phase_a;
  foc->clarke.beta = ((foc->input.phase_a + (foc->input.phase_b * 2)) * (1.0F / SQRT3));
}

void foc_park(FOC* restrict foc)
{
  foc->park.id = (foc->clarke.alpha * foc->input.cos) +
                 (foc->clarke.beta * foc->input.sin);

  foc->park.iq = (foc->clarke.beta * foc->input.cos) -
                 (foc->clarke.alpha * foc->input.sin);
}

void foc_ipark(FOC* restrict foc, float_t vd, float_t vq)
{
  foc->ipark.vd_limited = limit_f(vd, foc->ipark.vd_limit, -foc->ipark.vd_limit);
  foc->ipark.vq_limited = limit_f(vq, foc->ipark.vq_limit, -foc->ipark.vq_limit);

  foc->ipark.alpha = (foc->ipark.vd_limited * foc->input.cos) -
                     (foc->ipark.vq_limited * foc->input.sin);
  foc->ipark.beta = (foc->ipark.vq_limited * foc->input.cos) +
                    (foc->ipark.vd_limited * foc->input.sin);
}

void foc_svpwm(FOC* restrict foc)
{
  // calculate 3-phase sinusoidal outputs (60 degrees apart)
  foc->sv.p90 = foc->ipark.beta;
  foc->sv.p30 = (foc->ipark.beta * COS_60) + (foc->ipark.alpha * COS_30);
  foc->sv.p330 = foc->sv.p30 - foc->sv.p90;

  // determine the active sector
  foc->sv.sector_key = ((foc->sv.p330 > 0) << 2) |
                       ((foc->sv.p30 > 0) << 1) |
                        (foc->sv.p90 > 0);

  switch(foc->sv.sector_key) {

  case(SECTOR_1):
  case(SECTOR_4):
    foc->sv.t1 = foc->sv.p30;
    foc->sv.t2 = foc->sv.p90 - foc->sv.p330;
    foc->sv.t3 = -foc->sv.p30;
    break;

  case(SECTOR_2):
  case(SECTOR_5):
    foc->sv.t1 = foc->sv.p330 + foc->sv.p30;
    foc->sv.t2 = foc->sv.p90;
    foc->sv.t3 = -foc->sv.p90;
    break;

  case(SECTOR_3):
  case(SECTOR_6):
    foc->sv.t1 = foc->sv.p330;
    foc->sv.t2 = -foc->sv.p330;
    foc->sv.t3 = -(foc->sv.p90 + foc->sv.p30);
    break;

  default:
    break;
  }

}
