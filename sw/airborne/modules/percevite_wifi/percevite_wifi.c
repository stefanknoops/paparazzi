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

/**************************************************************************************
 * CURRENTLY INCONSISTENT
 * | Startbyte(0) | DroneID (1-2) | North (3-8)     | East (9-14)     | Down (15-20)    | Heading (20-25) | Endbytes (26-27) |
 * |--------------|---------------|-----------------|-----------------|-----------------|-----------------|------------------|
 * | '$' (char)   | 1 bytes (char)| 4 bytes (float) | 4 bytes (float) | 4 bytes (float) | 4 bytes (float) | '*'0 (char)      |
 * |                                                                                                                         |
 * |--------------ESP-------------|--------------PPRZ----------PPRZ-----------------PPRZ----------------------PPRZ-----------|
 *  This firmware only sends the middle 4*6 bytes to ESP, header of the packet is encoded by ESP32
 * ************************************************************************************/

#include <stdio.h>
#include <string.h>

#include "modules/percevite_wifi/percevite_wifi.h"
#include "subsystems/datalink/telemetry.h"
#include "state.h"

drone_data_t dr_data[MAX_DRONES];
uint8_t esp_state = ESP_SYNC;

// Send a hex array to esp
static uint8_t esp_send_string(uint8_t *s, uint8_t len)
{
	// start byte
	uart_put_byte(&(ESP_UART_PORT), 0, '$');
	uart_put_byte(&(ESP_UART_PORT), 0, 178);

	// maximum of 255 bytes
 	uint8_t i = 0;
	for (i=0; i<len; i++) {
		uart_put_byte(&(ESP_UART_PORT), 0, (uint8_t)(s[i]));
	}

	// end byte
	// uart_put_byte(&(ESP_UART_PORT), 0, '*');

	return (i+2);
}


// Send an ack to esp, that other drone's data was successfully received
static uint8_t esp_send_ack(void)
{
	// start bytes
	uart_put_byte(&(ESP_UART_PORT), 0, '$');
	uart_put_byte(&(ESP_UART_PORT), 0, 178);

	uint8_t s[2] = {0x04, ACK_FRAME};
	uint8_t i = 0; // maximum of 255 bytes
	for (i=0; i<2; i++) {
		uart_put_byte(&(ESP_UART_PORT), 0, (uint8_t)(s[i]));
	}

	// end byte
	// uart_put_byte(&(ESP_UART_PORT), 0, '*');

	return (i+2);
}

uint8_t localbuf[ESP_MAX_LEN] = {0};

// state machine: raw message parsing function, parse other drone id's reported by esp32
static void esp_parse(uint8_t c) {

	static uint8_t byte_ctr = 0;
	static uint8_t drone_id = 0;
	static uint8_t packet_length = 0;
	static uint8_t packet_type = 0;

  printf("esp_state: %d, char rxed: 0x%02x\n", esp_state, c);
  switch (esp_state) {
    case ESP_SYNC: {

			/* first char, sync string */
			if (c == '$') {
				byte_ctr = byte_ctr + 1;
    	}

			/* second char: are you really the start of the packet? */
			if ((byte_ctr == 1) && (c == 178)) {
				byte_ctr = byte_ctr + 1;
				esp_state = ESP_DRONE_INFO;
			}
		} break;

    case ESP_DRONE_INFO: {
			if (byte_ctr == 2) {
				/* take note of packet length */
				drone_id = c;
				byte_ctr = byte_ctr + 1;
			} else if (byte_ctr == 3) {
				/* take note of packet type */
				packet_type = c;
				byte_ctr = byte_ctr + 1;
			} else if (byte_ctr == 4) {
				/* take note of drone ID */
				packet_length = c;
				byte_ctr = byte_ctr + 1;

				// info frame populated!! 
				printf("packet_length: %d, packet_type: %d, drone_id: %d\n", packet_length, packet_type, drone_id);
				if (packet_type == ACK_FRAME && packet_length == 4) {
					// TODO: esp received ssid change signal and sent you ack, 
					// indicate that on bool pprz esp ping? 
					esp_state = ESP_RX_OK;
					byte_ctr = 0;
				}

				/* packet length will always be shorter than padded struct, create some leeway */
				else if ((packet_type == DATA_FRAME) && (packet_length >= (sizeof(drone_data_t)-5))) {
					esp_state = ESP_DRONE_DATA;
				}	else {
					// do nothing?!
				} 
			} else {
				// do nothing?!
			}
			
		} break;

		case ESP_DRONE_DATA: {
			uint8_t st_byte_pos = byte_ctr;
			
			if (byte_ctr < packet_length) {
				localbuf[byte_ctr - st_byte_pos] = c;
				byte_ctr = byte_ctr + 1;
				printf("ctr: %d\n", byte_ctr);
			}			
			/* after receiving the msg, terminate ssid string */
			if (byte_ctr == packet_length) {
				byte_ctr = 0;
				esp_state = ESP_RX_OK; 
			}
			// else {
			// 	byte_ctr = 0;
			// 	esp_state = ESP_RX_ERR; 
			// }
		} break;
    case ESP_RX_OK: {
			memcpy(&dr_data[drone_id], &localbuf, sizeof(drone_data_t));

			/* string is okay, print it out and reset the state machine */
			printf("statemc: %d, droneid: %d, msgstr: %s\n", esp_state, drone_id, localbuf);

			/* reset state machine */
			esp_state = ESP_SYNC;

		} break;
    case ESP_RX_ERR: {
						printf("ESP_RX_ERR: string terminated before drone info\n");
						byte_ctr = 0;
						/* reset state machine, string terminated earlier than expected */
						esp_state = ESP_SYNC;
    } break;
    default: {
						byte_ctr = 0;
						esp_state = ESP_SYNC;
		} break;
  }
}

// event based UART polling function 
void esp_event_uart_rx(void)
{
	// // Look for data on serial link and send to parser
	// while (uart_char_available(&(ESP_UART_PORT))) {
	// 	uint8_t ch = uart_getch(&(ESP_UART_PORT));
	// 	esp_parse(ch);
	// }

}


// static void msg_cb(struct transport_tx *trans, struct link_device *dev) {

// 	// char buf1[10] = {0};
// 	// char buf2[10] = {0};
  
// 	// // TODO: debug, index out of bounds, send messages for drone1 and drone2
// 	// pprz_msg_send_PERCEVITE_WIFI(trans, dev, AC_ID, 
// 	// 								strlen(drone_status[0].), idrone_status[0].east_str, 
// 	// 								strlen(dr_status[1].pos.x), gcvt(dr_status[1].pos.x, 6, buf2));

// 	// DEBUG: 
// 	// printf("east_len: %d, east_str: %s\n", strlen(drone_status[0].east_str), drone_status[1].east_str);
// }


static void clear_drone_status(void) {
	for (uint8_t id = 0; id < MAX_DRONES; id++) {
		// initialize at tropical waters of eastern Altanic ocean, facing the artic
		dr_data[id].pos.x = 0;
		dr_data[id].pos.y = 0;
		dr_data[id].pos.z = 0;
		dr_data[id].heading = 0;
		dr_data[id].vel.x = 0;
		dr_data[id].vel.y = 0;
		dr_data[id].vel.z = 0;
	}
}

void uart_esp_init() {
	
	/* reset receive buffer state machine */
	esp_state = ESP_SYNC;

	/* check pprz message for drone1 and drone2 (sort of like esp heartbeat) */
	// register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PERCEVITE_WIFI, msg_cb);

	clear_drone_status();







	
}


// frequency: 2 Hz
void uart_esp_loop() {

	// TODO: send these over instead of hardcoded
	struct NedCoor_f *optipos = stateGetPositionNed_f();
	struct NedCoor_f *optivel = stateGetSpeedNed_f();
	struct FloatEulers *att = stateGetNedToBodyEulers_f();

	// size of string = sizeof(uart_packet_t)




		#define TX_STRING_LEN 31
	uint8_t tx_string[TX_STRING_LEN] = {0};
	uart_packet_t uart_packet = {
		.info = {
			.drone_id = SELF_ID,
			.packet_type = DATA_FRAME,
			.packet_length = 2 + sizeof(uart_packet_t),
		},
		.data = {
			.pos = {
				.x = -1.53,
				.y = -346.234,
				.z = 23455.234,
			},
			.heading = -452.12,
			.vel = {
				.x = -1.53,
				.y = -346.234,
				.z = 23455.234,
			},
		},
	};

	printf("START: sizeof(uart_packet_t) = %d\n", sizeof(uart_packet_t));
	// tx_string[0] = '$';
	// tx_string[1] = 178;
	memcpy(&tx_string[0], &uart_packet, sizeof(uart_packet_t));
	
	// DEBUG: 
	printf("ssid should be: ");
	for (int i = 0; i < sizeof(uart_packet_t); i++) {
		printf("0x%02x,", tx_string[i]);
	}
	printf("*******\n");

	esp_send_string(tx_string, sizeof(uart_packet_t));

	// mutex, don't tx to esp when ack is being sent
  //if (esp.state!= ESP_RX_OK) {
		
	//}
	uint8_t test_str[34] = {0};
	test_str[0] = '$';
	test_str[1] = 178;
	strncpy(&test_str[2], tx_string, sizeof(uart_packet_t));

	// need to send one additional byte at the end for state machine to switch. Send 0x00; 
	for (int i = 0; i <= 34; i ++) {
		esp_parse(test_str[i]);
	}






}



