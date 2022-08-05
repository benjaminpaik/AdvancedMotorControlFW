/*
 * parameter_functions.h
 *
 *  Created on: Aug 3, 2022
 *      Author: benja
 */

#ifndef INC_PARAMETER_FUNCTIONS_H_
#define INC_PARAMETER_FUNCTIONS_H_

void read_parameters(int32_t parameters[], uint16_t parameters_per_tx, int32_t tx[], int32_t rx[]);
uint16_t write_parameters(int32_t parameters[], uint16_t parameters_per_rx, int32_t tx[], int32_t rx[]);
uint16_t flash_parameters(uint32_t parameters[], uint16_t length);
void load_parameters(uint32_t parameters[], uint16_t length);


#endif /* INC_PARAMETER_FUNCTIONS_H_ */
