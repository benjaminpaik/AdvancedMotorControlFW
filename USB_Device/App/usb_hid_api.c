/*
 * usb_hid_protocol.c
 *
 *  Created on: Oct 18, 2019
 *      Author: bpaik
 */

#include "usb_hid_api.h"

#include <string.h>

volatile uint8_t g_ping_pong_flag = 0;
uint8_t g_timestamp = 0;
uint8_t g_usb_rx_bytes[NUM_BYTES];
uint8_t g_usb_tx_bytes_0[NUM_BYTES];
uint8_t g_usb_tx_bytes_1[NUM_BYTES];

void init_usb_data(void)
{
  uint8_t i = 0;
  for(i = 0; i < NUM_BYTES; i++) {
    g_usb_rx_bytes[i] = 0;
    g_usb_tx_bytes_0[i] = 0;
    g_usb_tx_bytes_1[i] = 0;
  }
}

uint8_t get_usb_mode(void)
{
  return g_usb_rx_bytes[COMMAND_MODE_INDEX];
}

void set_usb_mode(uint8_t mode)
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

int32_t get_usb_data32(uint8_t index)
{
  uint8_t offset = (BYTES_PER_LONG * index) + DATA_START_INDEX;
  return BYTES_TO_LONG(g_usb_rx_bytes + offset);
}

void set_usb_data32(uint8_t index, int32_t value)
{
  uint8_t offset = ((BYTES_PER_LONG * index) + DATA_START_INDEX);

  if(g_ping_pong_flag) {
    LONG_TO_BYTES((g_usb_tx_bytes_0 + offset), value);
  }
  else {
    LONG_TO_BYTES((g_usb_tx_bytes_1 + offset), value);
  }
}

void load_usb_rx_data(uint8_t *data)
{
  memcpy(g_usb_rx_bytes, data, NUM_BYTES);
}

void load_usb_tx_data(void)
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
