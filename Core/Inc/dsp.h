/*
 * dsp.h
 *
 *  Created on: Apr 17, 2021
 *      Author: benja
 */

#ifndef SRC_DSP_H_
#define SRC_DSP_H_

#include "stm32g4xx_hal.h"
#include "math.h"

#define BYTES_PER_INT       2
#define BYTES_PER_LONG      4
#define PI                  (3.14159F)

// type conversions
#define LONG_TO_BYTES(B, L) (B[0] = ((L >> 24) & 0xFF)); \
                            (B[1] = ((L >> 16) & 0xFF)); \
                            (B[2] = ((L >> 8) & 0xFF)); \
                            (B[3] = ((L) & 0xFF))
#define BYTES_TO_LONG(B)  (((int32_t)(B)[0] << 24) | ((int32_t)(B)[1] << 16) | ((int32_t)(B)[2] << 8) | ((int32_t)(B)[3]))
#define BYTE_TO_INT(B)    (((int16_t)(B)[0] << 8) | (int16_t)(B)[1])
#define INT_TO_FLOAT_BITS(X)    (*(float_t *)(&X))
#define FLOAT_TO_INT_BITS(X)    (*(int32_t *)(&X))

#define CRC_TABLE_SIZE  256

typedef struct {
  uint16_t flag;
  uint16_t latch;
  uint16_t timer;
  uint16_t threshold;
} PERSISTENCE;

typedef struct {
  uint16_t threshold;
  uint16_t counter;
  uint16_t input_prev;
  uint16_t out;
} DEBOUNCE;

typedef enum {
  PID_IN_RANGE,
  PID_UPPER_LIMIT,
  PID_LOWER_LIMIT
} PID_STATE;

typedef struct {
	uint16_t enable;
	volatile PID_STATE state;
	PID_STATE upper_limit_state;
	PID_STATE lower_limit_state;

	float_t Kp;
	float_t Ki;
	float_t Kd;

	volatile float_t error;
	volatile float_t i_delta;
	volatile float_t i_error_previous;
	volatile float_t d_error;

	volatile float_t p_term;
	volatile float_t i_term;
	volatile float_t d_term;

	float_t out_limit_upper;
	float_t out_limit_lower;
	volatile float_t out;
} PID;

typedef struct {
  float_t alpha;
  volatile float_t previous;
  volatile float_t out;
} LOWPASS;

static inline float_t limit_f(float_t input, float_t upper_limit, float_t lower_limit)
{
  if(input > upper_limit) {
    return upper_limit;
  }
  else if(input < lower_limit) {
    return lower_limit;
  }
  else {
    return input;
  }
}

void init_pid(PID *pid, uint16_t enable, float_t Kp, float_t Ki, float_t Kd, float_t out_limit_upper, float_t out_limit_lower, float_t frequency);
void pi_control(PID *pid, float_t setpoint, float_t feedback, PID_STATE inner_loop_state);
void pid_reset(PID *pid);

void init_persistence(PERSISTENCE *persistence, uint16_t threshold, uint16_t latch);
uint16_t persistence_check(PERSISTENCE *persistence, uint16_t condition);
void persistence_reset(PERSISTENCE *persistence);

#endif /* SRC_DSP_H_ */
