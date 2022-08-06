/*
 * definitions.h
 *
 *  Created on: Oct 19, 2021
 *      Author: benja
 */

#ifndef INC_DEFINITIONS_H_
#define INC_DEFINITIONS_H_

#ifndef TRUE
#define TRUE  (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif

#define RAM_END_ADDRESS     (0x20008000 - 0x10)

#define INPUT_VOLTAGE       26.0F

#define BOOT_DELAY_TIME     1000
#define CTRL_TASK_PERIOD    10
#define COMM_TASK_PERIOD    1

#define NULL_MODE   0xFF
#define IDLE_MODE	  1
#define RUN_MODE	  2
#define CAL_MODE	  3
#define READ_MODE   17
#define WRITE_MODE  18
#define FLASH_MODE  19
#define BOOT_MODE   30

#define ARRAY_SIZE(X)     (sizeof(X) / sizeof(X[0]))
#define TIMER_CLOCK_FREQ  (144000000.0F)
#define STATES_PER_REV    (24)
#define VELOCITY_GAIN     (TIMER_CLOCK_FREQ * (60.0F / STATES_PER_REV))
#define CURRENT_SCALE(X)  (((12.5F / 2047.5F) * (X)) - 12.5F)

#endif /* INC_DEFINITIONS_H_ */
