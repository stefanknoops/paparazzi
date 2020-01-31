/*
 * Copyright (C) nilay
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/percevite_wifi/percevite_wifi.c"
 * @author nilay
 * wifi ssid broadcast
 */

#include "modules/percevite_wifi/percevite_wifi.h"
#include "subsystems/datalink/telemetry.h"
#include <stdio.h>

uint8_t str1_len = 50;
uint8_t str2_len = 50;
char str1[50] = {};
char str2[50] = {};

#define ESP_MAX_LEN 40 // lat,long,alt,bearing = 40 bytes max

// generic JEVOIS message structure
struct esp_msg_t {
  uint8_t type;
  uint8_t id;
  char str[ESP_MAX_LEN];
};

// decoder state
enum esp_state {
  ESP_SYNC = 0,
  ESP_ID,
  ESP_RX_MSG
};

// jevois struct
struct esp_t {
  enum esp_state state; // decoder state
  char buf[ESP_MAX_LEN]; // temp buffer
  uint8_t idx; // temp buffer index
  struct esp_msg_t msg; // last decoded message
  bool data_available; // new data to report
};

struct esp_t esp;

// state machine: raw message parsing function /* struct esp_t *esp, */
static void esp_parse(char c) {
  printf("char rxed: %c\n", c);

  if (c == '(') {
    esp.state = ESP_SYNC;
  }

  printf("esp.state: %d, esp.msg.str: %s\n", esp.state, esp.msg.str);
  switch (esp.state) {
    case ESP_SYNC:
                  /* first char, sync string */
                  esp.state = ESP_ID; break;
    case ESP_ID:  /* take note of drone ID */
                  if (c!='\0' || c!='\n' || c!='\r') {
                    // only first byte 00-09 (10) drones for now
                    char tmp_str[3] = {'0','0', '\0'};
                    tmp_str[1] = c;
                    esp.msg.id = (uint8_t) atoi(tmp_str);
                    esp.state = ESP_RX_MSG;
                  }
                  break;
    case ESP_RX_MSG: {
                      /* record ssid until full lat,long,alt are stored in esp.msg.str */
                      /* don't change state machine until str terminate char is received */
                      static uint8_t byte_ctr = 0;
                      if (c!='\0' || c!='\n' || c!='\r') {
                        esp.msg.str[byte_ctr] = c;
                        byte_ctr = byte_ctr + 1;
                      }
                      /* after receiving the msg, terminate ssid string also
                      and restart the state machine */ 
                      else {
                        esp.msg.str[byte_ctr] = '\0';
                        byte_ctr = 0;
                        esp.state = ESP_SYNC; 
                      }
                    }
                    break;
    default: 
                    esp.state = ESP_SYNC;
                    break;
  }
}

// event based UART polling function 
void esp_event_uart_rx(void)
{
  // Look for data on serial link and send to parser
  while (uart_char_available(&(ESP_UART_PORT))) {
    uint8_t ch = uart_getch(&(ESP_UART_PORT));
    esp_parse(ch);
  }
}


static void msg_cb(struct transport_tx *trans, struct link_device *dev) {
  pprz_msg_send_PERCEVITE_WIFI(trans, dev, AC_ID, str1_len, str1, str2_len, str2);
  printf("drone1: %s, drone2: %s\n", str1, str2);
}

void uart_esp_init() {
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PERCEVITE_WIFI, msg_cb);
}

void uart_esp_loop() {

}



