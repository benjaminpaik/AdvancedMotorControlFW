/*
 * dsp.c
 *
 *  Created on: Apr 17, 2021
 *      Author: benja
 */
#include "definitions.h"
#include "dsp.h"

void init_persistence_counter(PERSISTENCE *persistence, uint16_t threshold, uint16_t latch)
{
  persistence->threshold = threshold;
  persistence->latch = latch;
  // initialize the timer
  persistence->timer = 0;
  persistence->flag = FALSE;
}

uint16_t persistence_counter(PERSISTENCE *persistence, uint16_t condition)
{
  if (condition) {
    // if the timer has not reached the threshold
    if (persistence->timer < persistence->threshold) {
      // increment the persistence timer
      persistence->timer++;
    }
  }
  // don't clear persistence if the latch and flag are set
  else if (persistence->latch == FALSE || persistence->flag == FALSE) {
    // reset the timer
    persistence->timer = 0;
  }
  // set or clear the persistence flag
  persistence->flag = (persistence->timer >= persistence->threshold);
  // return the persistence flag
  return persistence->flag;
}

void init_rate_limiter(RATE_LIMIT *rl, float gain)
{
  rl->gain = gain;
  rl->accumulator = 0;
  rl->out = 0;
}

void rate_limiter(RATE_LIMIT *rl, float input)
{
  float temp;
  if (input > rl->out && rl->accumulator < MAX_LONG) {
    rl->accumulator++;
    temp = rl->accumulator * rl->gain;

    if (temp > input) {
      // set the output to match the input and undo accumulation
      rl->out = input;
      rl->accumulator--;
    }
    else {
      rl->out = temp;
    }
  }
  else if (input < rl->out && rl->accumulator > -MAX_LONG) {
    rl->accumulator--;
    temp = rl->accumulator * rl->gain;

    if (temp < input) {
      // set the output to match the input and undo accumulation
      rl->out = input;
      rl->accumulator++;
    }
    else {
      rl->out = temp;
    }
  }
}

void init_pid(PID *pid, uint16_t enable, float Kp, float Ki, float Kd, float out_limit_upper, float out_limit_lower, float frequency)
{
  // set gains
  pid->Kp = Kp;
  pid->Ki = Ki / frequency;
  pid->Kd = Kd;

  // initialize terms
  pid->error = 0;
  pid->d_error = 0;
  pid->p_term = 0;
  pid->i_term = 0;
  pid->i_error_previous = 0;
  pid->d_term = 0;
  pid->out = 0;

  //enable/disable PID block
  pid->enable = enable;
  //initialize limit flag
  pid->state = PID_IN_RANGE;

  pid->upper_limit_state = (pid->Ki >= 0) ? PID_UPPER_LIMIT : PID_LOWER_LIMIT;
  pid->lower_limit_state = (pid->Ki >= 0) ? PID_LOWER_LIMIT : PID_UPPER_LIMIT;

  //output limit
  pid->out_limit_upper = out_limit_upper;
  pid->out_limit_lower = (out_limit_upper > out_limit_lower) ? out_limit_lower : (out_limit_upper - 1.0F);
}

// 2.87us worst case
void pi_control(PID *pid, float setpoint, float feedback, PID_STATE inner_loop_state)
{
  if (pid->enable) {
    // calculate PID error
    pid->error = setpoint - feedback;

    // proportional term
    pid->p_term = pid->Kp * pid->error;

    // integral term
    pid->i_delta = pid->Ki * pid->error;
    if((inner_loop_state == PID_IN_RANGE) ||
       (inner_loop_state == PID_UPPER_LIMIT && pid->i_delta < 0) ||
       (inner_loop_state == PID_LOWER_LIMIT && pid->i_delta > 0)) {
      pid->i_term += pid->i_delta;
      pid->i_term = LIMIT(pid->i_term, pid->out_limit_upper, pid->out_limit_lower);
    }

    // calculate and clamp the PID output
    pid->out = pid->p_term + pid->i_term;

    // output range check
    if (pid->out >= pid->out_limit_upper) {
      pid->out = pid->out_limit_upper;
      pid->state = pid->upper_limit_state;
    }
    else if (pid->out <= pid->out_limit_lower) {
      pid->out = pid->out_limit_lower;
      pid->state = pid->lower_limit_state;
    }
    else if(inner_loop_state == PID_UPPER_LIMIT) {
      pid->state = pid->upper_limit_state;
    }
    else if(inner_loop_state == PID_LOWER_LIMIT) {
      pid->state = pid->lower_limit_state;
    }
  }
  else {
    pid->out = setpoint;
    pid->state = inner_loop_state;
  }
}
