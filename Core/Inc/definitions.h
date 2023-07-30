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

#define ARRAY_SIZE(X)     (sizeof(X) / sizeof(X[0]))
#define TIMER_CLOCK_FREQ  (144000000.0F)
#define STATES_PER_REV    (24)
#define VELOCITY_GAIN     (TIMER_CLOCK_FREQ * (60.0F / STATES_PER_REV))

#endif /* INC_DEFINITIONS_H_ */
