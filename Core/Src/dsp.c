/*
 * dsp.c
 *
 *  Created on: Apr 17, 2021
 *      Author: benja
 */
#include "definitions.h"
#include "dsp.h"

uint32_t g_crc32_table[CRC_TABLE_SIZE];

void init_persistence(PERSISTENCE *persistence, uint16_t threshold, uint16_t latch)
{
  persistence->threshold = threshold;
  persistence->latch = latch;
  persistence->timer = 0;
  persistence->flag = FALSE;
}

uint16_t persistence_check(PERSISTENCE *persistence, uint16_t condition)
{
  if(condition) {
    if (persistence->timer < persistence->threshold) {
      persistence->timer++;
    }
  }
  else {
    persistence->timer = 0;
  }

  if(persistence->latch) {
    persistence->flag |= (persistence->timer >= persistence->threshold);
  }
  else {
    persistence->flag = (persistence->timer >= persistence->threshold);
  }
  return persistence->flag;
}

void init_pid(PID *pid, uint16_t enable, float_t Kp, float_t Ki, float_t Kd, float_t out_limit_upper, float_t out_limit_lower, float_t frequency)
{
  pid->Kp = Kp;
  pid->Ki = Ki / frequency;
  pid->Kd = Kd;

  pid->error = 0;
  pid->d_error = 0;
  pid->p_term = 0;
  pid->i_term = 0;
  pid->i_error_previous = 0;
  pid->d_term = 0;
  pid->out = 0;

  pid->enable = enable;
  pid->state = PID_IN_RANGE;

  pid->upper_limit_state = (pid->Ki >= 0) ? PID_UPPER_LIMIT : PID_LOWER_LIMIT;
  pid->lower_limit_state = (pid->Ki >= 0) ? PID_LOWER_LIMIT : PID_UPPER_LIMIT;

  pid->out_limit_upper = out_limit_upper;
  pid->out_limit_lower = out_limit_lower;
}

// 2.87us worst case
void pi_control(PID *pid, float_t setpoint, float_t feedback, PID_STATE inner_loop_state)
{
  if (pid->enable) {
    pid->error = setpoint - feedback;

    pid->p_term = pid->Kp * pid->error;

    pid->i_delta = pid->Ki * pid->error;
    if((inner_loop_state == PID_IN_RANGE) ||
       (inner_loop_state == PID_UPPER_LIMIT && pid->i_delta < 0) ||
       (inner_loop_state == PID_LOWER_LIMIT && pid->i_delta > 0)) {
      pid->i_term += pid->i_delta;
      pid->i_term = limit_f(pid->i_term, pid->out_limit_upper, pid->out_limit_lower);
    }

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

uint16_t rom_check(uint32_t *rom_crc32)
{
  uint16_t *rom_address;

  for(rom_address = (uint16_t *)(TEXT_START_ADDRESS); rom_address < (uint16_t *)(ISO_START_ADDRESS); rom_address++) {
    *rom_crc32 = crc32_iteration(*rom_crc32, (*rom_address) >> 8);
    *rom_crc32 = crc32_iteration(*rom_crc32, (*rom_address));
  }
  return (*rom_crc32 != ROM_CRC32);
}

void crc32_table_generator(uint32_t crc32_seed)
{
  uint32_t dividend = 0;
  uint32_t current_byte = 0;
  uint16_t bit = 0;

  for(dividend = 0; dividend < CRC_TABLE_SIZE; dividend++) {
    current_byte = dividend << 24;
    for(bit = 0; bit < 8; bit++) {
      if((current_byte & 0x80000000) != 0) {
        current_byte <<= 1;
        current_byte ^= crc32_seed;
      }
      else {
        current_byte <<= 1;
      }
    }
    g_crc32_table[dividend] = current_byte;
  }
}

uint32_t crc32_iteration(uint32_t crc, uint8_t byte)
{
  uint16_t index = ((crc ^ ((uint32_t)byte << 24)) >> 24) & 0xFF;
  return ((crc << 8) ^ g_crc32_table[index]);
}
