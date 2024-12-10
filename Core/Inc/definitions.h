/*
 * definitions.h
 *
 *  Created on: Oct 19, 2021
 *      Author: benja
 */

#ifndef INC_DEFINITIONS_H_
#define INC_DEFINITIONS_H_

#include "stm32g4xx_hal.h"

#ifndef TRUE
#define TRUE  (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif

#define SW_VERSION            1.0F
#define ROM_CRC32             0x2e22b13f

#define USB_COMM              TRUE

extern uint16_t __RAM_END, __CODE_START, __CODE_END, __EEPROM_START;
#define CRC32_SEED            0x04C11DB7

// encoder definitions
#define ENCODER_GAIN        ((2.0F * PI) / (S.motor.encoder.tim->Init.Period + 1))
#define ENCODER_OFFSET      -7290
#define CONTROL_FREQUENCY   20000

#define BOOT_DELAY_TIME     1000
#define CTRL_TASK_PERIOD    10
#define COMM_TASK_PERIOD    1

typedef enum {
  NULL_MODE = 0,
  IDLE_MODE = 1,
  RUN_MODE = 2,
  CAL_MODE = 3,
  READ_PARAMETER_MODE = 252,
  WRITE_PARAMETER_MODE = 253,
  FLASH_PARAMETER_MODE = 254,
  BOOT_MODE = 255
} HOST_MODES;

#define ARRAY_SIZE(X)     (sizeof(X) / sizeof(X[0]))
#define TIMER_CLOCK_FREQ  (144000000.0F)
#define STATES_PER_REV    (24)
#define VELOCITY_GAIN     (TIMER_CLOCK_FREQ * (60.0F / STATES_PER_REV))
#define CURRENT_SCALE(X)  (((12.5F / 2047.5F) * (X)) - 12.5F)
#define VOLTAGE_SCALE(X)  (0.02442F * (X))

#endif /* INC_DEFINITIONS_H_ */
