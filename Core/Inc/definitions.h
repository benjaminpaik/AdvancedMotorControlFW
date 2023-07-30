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

#define SW_VERSION            1.0F
#define ROM_CRC32             0x2e22b13f
#define TEXT_START_ADDRESS    0x08003000
#define ISO_START_ADDRESS     0x0801F800
#define RAM_END_ADDRESS       (0x20008000 - 0x10)
#define CRC32_SEED            0x04C11DB7

#define INPUT_VOLTAGE       26.0F

#define BOOT_DELAY_TIME     1000
#define CTRL_TASK_PERIOD    10
#define COMM_TASK_PERIOD    1

#define NULL_MODE       0
#define IDLE_MODE       1
#define RUN_MODE        2
#define CAL_MODE        3
#define BOOT_MODE       252
#define READ_MODE       253
#define WRITE_MODE      254
#define FLASH_MODE      255

#define ARRAY_SIZE(X)     (sizeof(X) / sizeof(X[0]))
#define TIMER_CLOCK_FREQ  (144000000.0F)
#define STATES_PER_REV    (24)
#define VELOCITY_GAIN     (TIMER_CLOCK_FREQ * (60.0F / STATES_PER_REV))
#define CURRENT_SCALE(X)  (((12.5F / 2047.5F) * (X)) - 12.5F)

#endif /* INC_DEFINITIONS_H_ */
