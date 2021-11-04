/*
 * dsp.h
 *
 *  Created on: Apr 17, 2021
 *      Author: benja
 */

#ifndef SRC_DSP_H_
#define SRC_DSP_H_

#include "stm32g4xx_hal.h"

#define MAX_LONG            (2147483647)
#define BYTES_PER_INT       2
#define BYTES_PER_LONG      4
#define PI                  (3.14159F)

// convert a 32-bit integer into a byte array
#define LONG_TO_BYTES(B, L) (B[0] = ((L >> 24) & 0xFF)); \
                            (B[1] = ((L >> 16) & 0xFF)); \
                            (B[2] = ((L >> 8) & 0xFF)); \
                            (B[3] = ((L) & 0xFF))
// convert 4 bytes into a 32-bit integer
#define BYTES_TO_LONG(B)  (((int32_t)(B)[0] << 24) | ((int32_t)(B)[1] << 16) | ((int32_t)(B)[2] << 8) | ((int32_t)(B)[3]))
#define BYTE_TO_INT(B)    (((int16_t)(B)[0] << 8) | (int16_t)(B)[1])
// convert a 32-bit integer to a 32-bit float
#define INT_TO_FLOAT_BITS(X)    (*(float *)(&X))
#define FLOAT_TO_INT_BITS(X)    (*(int32_t *)(&X))
// limit macro
#define LIMIT(xx, ulim, llim) (((xx)>(ulim))?(ulim):(((xx)<(llim))?(llim):(xx)))

#define DERIVATIVE_DELTA    10
#define DERIVATIVE_BUFFER   (1U << 4)
#define DERIVATIVE_MASK     (DERIVATIVE_BUFFER - 1)

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

typedef struct {
  float gain;
  int32_t accumulator;
  int32_t out;
} RATE_LIMIT;

typedef enum {
  PID_IN_RANGE,
  PID_UPPER_LIMIT,
  PID_LOWER_LIMIT
} PID_STATE;

typedef struct {
	//latch to enable/disable the PID block
	uint16_t enable;
	//integrator saturated
	volatile PID_STATE state;
	PID_STATE upper_limit_state;
	PID_STATE lower_limit_state;

	//scaled proportional gain term
	float Kp;
	//scaled integral gain term
	float Ki;
	//scaled derivative gain term
	float Kd;

	//proportional error
	volatile float error;
	volatile float i_delta;
	//previous error term
	volatile float prev_error[DERIVATIVE_BUFFER];
	//previous integral error
	volatile float i_error_previous;
	//derivative error term
	volatile float d_error;

	//proportional term
	volatile float p_term;
	//integral term
	volatile float i_term;
	//derivative term
	volatile float d_term;

	//frame counter
	volatile uint16_t frame_count;

	//output limit;
	float out_limit_upper;
	float out_limit_lower;

	//PID output
	volatile float out;
} PID;

typedef struct {
  // alpha
  float alpha;
  // the current scaled filtered output
  volatile float previous_out;
  // the current filtered output
  volatile float out;
} LOWPASS;

void init_rate_limiter(RATE_LIMIT *rl, float gain);
void rate_limiter(RATE_LIMIT *rl, float input);
void init_pid(PID *pid, uint16_t enable, float Kp, float Ki, float Kd, float out_limit_upper, float out_limit_lower, float frequency);
void pi_control(PID *pid, float setpoint, float feedback, PID_STATE inner_loop_state);
void pid_reset(PID *pid);

void init_persistence_counter(PERSISTENCE *persistence, uint16_t threshold, uint16_t latch);
uint16_t persistence_counter(PERSISTENCE *persistence, uint16_t condition);
void persistence_reset(PERSISTENCE *persistence);
void init_debounce(DEBOUNCE *debounce, uint16_t threshold);
uint16_t debounce(DEBOUNCE *debounce, uint16_t input);

#endif /* SRC_DSP_H_ */
