/*
 * usb_hid_protocol.h
 *
 *  Created on: Oct 18, 2019
 *      Author: bpaik
 */

#ifndef USB_HID_API_H_
#define USB_HID_API_H_

#include "stm32g4xx_hal.h"
#include "stm32g431xx.h"

#define NUM_BYTES             64
#define NUM_STATES            15

#define COMMAND_MODE_INDEX    0
#define TIMESTAMP_INDEX			  1
#define DATA_START_INDEX		  4

#define BYTES_PER_LONG        4
// convert a 32-bit integer into a byte array
#define LONG_TO_BYTES(B, L) (B[0] = ((L >> 24) & 0xFF)); \
                            (B[1] = ((L >> 16) & 0xFF)); \
                            (B[2] = ((L >> 8) & 0xFF)); \
                            (B[3] = ((L) & 0xFF))

// convert 4 bytes into a 32-bit integer
#define BYTES_TO_LONG(B)  (((int32_t)(B)[0] << 24) | ((int32_t)(B)[1] << 16) | ((int32_t)(B)[2] << 8) | ((int32_t)(B)[3]))
// convert a 32-bit integer to a 32-bit float
#define INT_TO_FLOAT_BITS(X)    (*(float_t *)(&X))
#define FLOAT_TO_INT_BITS(X)    (*(int32_t *)(&X))

// function prototypes
void init_usb_data(void);
uint8_t get_usb_mode(void);
void set_usb_mode(uint8_t mode);
void update_usb_timestamp(void);
int32_t get_usb_data32(uint8_t index);
void set_usb_data32(uint8_t index, int32_t value);
void load_usb_rx_data(uint8_t *data);
void load_usb_tx_data(void);
uint8_t* get_usb_tx_buffer(void);

#endif /* USB_HID_API_H_ */
