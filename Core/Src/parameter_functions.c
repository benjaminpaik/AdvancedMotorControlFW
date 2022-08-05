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

  // find the starting point for the parameters are loading
  param_start = rx[TRANSFER_INDEX] * parameters_per_tx;
  // return the parameter transfer index
  tx[TRANSFER_INDEX] = rx[TRANSFER_INDEX];

  // store long integer parameter values to our SCI send buffer
  for(i = 0; i < parameters_per_tx; i++) {
    // load parameter values to be transmitted
    tx[i + PARAMETER_INDEX] = parameters[i + param_start];
    // stop once we have filled the entire parameter array
    if((i + param_start) >= (PARAMETER_ARRAY_SIZE - 1)) {
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
    // convert bytes in the packet to int32 for the parameters in a transfer
    parameters[i + param_start] = rx[i + PARAMETER_INDEX];
    // stop once we have filled the entire parameter array
    if((i + param_start) >= (PARAMETER_ARRAY_SIZE - 1)) {
      // all parameters written
      return TRUE;
    }
  }
  // more parameters to write
  return FALSE;
}

uint16_t flash_parameters(uint32_t parameters[], uint16_t length)
{
  uint16_t i;
  for(i = 0; i < length; i++) {
    if(EE_WriteVariable32bits(i + 1, parameters[i]) != EE_OK) {
      return FALSE;
    }
  }
  return TRUE;
}


void load_parameters(uint32_t parameters[], uint16_t length)
{
  uint16_t i;
  for(i = 0; i < length; i++) {
    EE_ReadVariable32bits(i + 1, &parameters[i]);
  }
}

//void load_parameters(FRAMEWORK *framework, int32 parameters[], Uint16 length, Uint32 sector)
//{
//  Uint16 memory_length = length * 2;
//  flash_read((Uint16 *)parameters, memory_length, (Uint16 *)sector);
//  flash_read((Uint16 *)&framework->expected_parameter_checksum,
//             sizeof(framework->expected_parameter_checksum),
//             (Uint16 *)(sector + memory_length));
//}

