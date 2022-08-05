/*
 * parameter_functions.c
 *
 *  Created on: Aug 3, 2022
 *      Author: benja
 */
#include "cmsis_os.h"
#include "parameters.h"
#include "definitions.h"
#include "eeprom_emul.h"

// parameter indices
#define TRANSFER_INDEX      0
#define PARAMETER_INDEX     1

void read_parameters(int32_t parameters[], uint16_t parameters_per_tx, int32_t tx[], int32_t rx[])
{
  uint16_t i = 0;
  uint16_t param_start = 0;

  param_start = rx[TRANSFER_INDEX] * parameters_per_tx;
  tx[TRANSFER_INDEX] = rx[TRANSFER_INDEX];

  for(i = 0; i < parameters_per_tx; i++) {
    tx[i + PARAMETER_INDEX] = parameters[i + param_start];
    if((i + param_start) >= (NUM_PARAMETERS - 1)) {
      break;
    }
  }
}

uint16_t write_parameters(int32_t parameters[], uint16_t parameters_per_rx, int32_t tx[], int32_t rx[])
{
  uint16_t i = 0;
  uint16_t param_start = 0;

  param_start = rx[TRANSFER_INDEX] * parameters_per_rx;
  tx[TRANSFER_INDEX] = rx[TRANSFER_INDEX];

  for(i = 0; i < parameters_per_rx; i++) {
    parameters[i + param_start] = rx[i + PARAMETER_INDEX];
    if((i + param_start) >= (NUM_PARAMETERS - 1)) {
      return TRUE;
    }
  }
  return FALSE;
}

uint16_t flash_parameters(int32_t parameters[], uint16_t length)
{
  uint16_t i;
  for(i = 0; i < length; i++) {
    if(EE_WriteVariable32bits(i + 1, *(uint32_t *)(parameters + i)) != EE_OK) {
      return FALSE;
    }
  }
  return TRUE;
}


void load_parameters(int32_t parameters[], uint16_t length)
{
  uint16_t i;
  for(i = 0; i < length; i++) {
    EE_ReadVariable32bits(i + 1, (uint32_t *)(parameters + i));
  }
}

