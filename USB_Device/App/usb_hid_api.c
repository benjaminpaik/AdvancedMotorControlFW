/*
 * usb_hid_protocol.c
 *
 *  Created on: Oct 18, 2019
 *      Author: bpaik
 */

#include "usb_hid_api.h"
#include "../Inc/dsp.h"
#include <string.h>

volatile uint8_t g_ping_pong_flag = 0;
uint8_t g_timestamp = 0;
uint8_t g_usb_rx_bytes[NUM_USB_BYTES];
uint8_t g_usb_tx_bytes_0[NUM_USB_BYTES];
uint8_t g_usb_tx_bytes_1[NUM_USB_BYTES];

int32_t g_usb_rx_data[NUM_TELEMETRY_STATES];
int32_t g_usb_tx_data[NUM_TELEMETRY_STATES];

void init_usb_data(void)
{
  uint8_t i = 0;
  for(i = 0; i < NUM_USB_BYTES; i++) {
    g_usb_rx_bytes[i] = 0;
    g_usb_tx_bytes_0[i] = 0;
    g_usb_tx_bytes_1[i] = 0;
  }
}

uint8_t get_usb_rx_mode(void)
{
  return g_usb_rx_bytes[COMMAND_MODE_INDEX];
}

uint8_t get_usb_tx_mode(void)
{
  return g_usb_tx_bytes_0[COMMAND_MODE_INDEX];
}

void set_usb_tx_mode(uint8_t mode)
{
  g_usb_tx_bytes_0[COMMAND_MODE_INDEX] = mode;
  g_usb_tx_bytes_1[COMMAND_MODE_INDEX] = mode;
}

void update_usb_timestamp(void)
{
  g_timestamp++;
  g_usb_tx_bytes_0[TIMESTAMP_INDEX] = g_timestamp;
  g_usb_tx_bytes_1[TIMESTAMP_INDEX] = g_timestamp;
}

inline int32_t get_usb_data32(uint8_t index)
{
  uint8_t offset = (BYTES_PER_LONG * index) + DATA_START_INDEX;
  g_usb_rx_data[index] = BYTES_TO_LONG(g_usb_rx_bytes + offset);
  return g_usb_rx_data[index];
}

void set_usb_data32(uint8_t index, int32_t value)
{
  g_usb_tx_data[index] = value;
  uint8_t offset = ((BYTES_PER_LONG * index) + DATA_START_INDEX);

  if(g_ping_pong_flag) {
    LONG_TO_BYTES((g_usb_tx_bytes_0 + offset), value);
  }
  else {
    LONG_TO_BYTES((g_usb_tx_bytes_1 + offset), value);
  }
}

int32_t* get_usb_rx_data()
{
  uint16_t i;
  for(i = 0; i < NUM_TELEMETRY_STATES; i++) {
    get_usb_data32(i);
  }
  return g_usb_rx_data;
}

int32_t* get_usb_tx_data()
{
  return g_usb_tx_data;
}

void set_usb_tx_data()
{
  uint16_t i;
  for(i = 0; i < NUM_TELEMETRY_STATES; i++) {
    set_usb_data32(i, g_usb_tx_data[i]);
  }
}

void load_usb_rx_bytes(uint8_t *data)
{
  memcpy(g_usb_rx_bytes, data, NUM_USB_BYTES);
}

void load_usb_tx_bytes(void)
{
	g_ping_pong_flag = !g_ping_pong_flag;
}

uint8_t* get_usb_tx_buffer(void)
{
	if(g_ping_pong_flag) {
		return g_usb_tx_bytes_1;
	}
	else {
		return g_usb_tx_bytes_0;
	}
}
