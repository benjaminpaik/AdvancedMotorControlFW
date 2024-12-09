/*
 * system.h
 *
 *  Created on: Oct 27, 2021
 *      Author: BPaik
 */

#ifndef INC_SYSTEM_H_
#define INC_SYSTEM_H_

#include "motor.h"
#include "dsp.h"

typedef union {
  struct {
    uint32_t rom_fault:1;
  } bit;
  uint32_t all;
} STATUS;

typedef struct {
  STATUS status;
  uint32_t rom_crc32;
  uint16_t boot_delay_timer;
  uint8_t int_flash_flag;

  uint8_t mode;
  uint8_t mode_previous;

  int32_t cmd;
  MOTOR_DRIVE motor;
} SYSTEM;

#endif /* INC_SYSTEM_H_ */
