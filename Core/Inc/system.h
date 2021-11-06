/*
 * system.h
 *
 *  Created on: Oct 27, 2021
 *      Author: BPaik
 */

#ifndef INC_SYSTEM_H_
#define INC_SYSTEM_H_

#include "motor.h"
#include "state_space.h"
#include "dsp.h"

typedef struct {
  float scale;
  int32_t raw;
  RATE_LIMIT rate;
  float out;
} COMMAND;

typedef struct {
  uint8_t mode;
  COMMAND cmd;
  float pwm_cmd;
  TRAP_DRIVE motor;
  STATE_CONTROLLER controller;
} SYSTEM;

#endif /* INC_SYSTEM_H_ */
