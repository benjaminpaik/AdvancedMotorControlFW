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

#define NUM_USB_BYTES         64
#define NUM_TELEMETRY_STATES  15

#define COMMAND_MODE_INDEX    0
#define TIMESTAMP_INDEX			  1
#define DATA_START_INDEX		  4

// function prototypes
void init_usb_data(void);
uint8_t get_usb_rx_mode(void);
uint8_t get_usb_tx_mode(void);
void set_usb_tx_mode(uint8_t mode);
void update_usb_timestamp(void);
int32_t get_usb_data32(uint8_t index);
void set_usb_data32(uint8_t index, int32_t value);
int32_t* get_usb_rx_data();
int32_t* get_usb_tx_data();
void set_usb_tx_data();
void load_usb_rx_bytes(uint8_t *data);
void load_usb_tx_bytes(void);
uint8_t* get_usb_tx_buffer(void);

#endif /* USB_HID_API_H_ */
