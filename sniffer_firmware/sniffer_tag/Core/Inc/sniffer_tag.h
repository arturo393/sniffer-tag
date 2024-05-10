/*
 * sniffer_tag.h
 *
 *  Created on: May 2, 2024
 *      Author: uqommdev
 */

#ifndef INC_SNIFFER_TAG_H_
#define INC_SNIFFER_TAG_H_

#include "main.h"
#include "stdlib.h"
#include "uwb3000Fxx.h"

extern UART_HandleTypeDef huart1;

/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define INITIAL_COMUNICATION_DATA_SIZE 5

typedef struct {
	double tof;
	double value;
	double readings[DISTANCE_READINGS];
	double new[DISTANCE_READINGS];
	double sum;
	double last;
	uint8_t error_times;
	uint8_t counter;
} Distance_t;

typedef struct {
	uint32_t id;
	int detection_times;
	Distance_t distance;
} TAG_t;

typedef struct buffer {
    uint8_t *buffer;
    uint8_t buffer_size;
    int delay;
    int rx_timeout;
    int preamble_timeout;
} TX_BUFFER_t;


typedef enum{
	TAG_OK,
	TAG_NO_RESPONSE,
	TAG_NO_RXCG_DETECTED,
	TAG_RX_FRAME_TIMEOUT,
	TAG_RX_PREAMBLE_DETECTION_TIMEOUT,
	TAG_RX_CRC_VALID,
	TAG_RX_ERROR,
	TAG_RX_DATA_ZERO,
	TAG_RX_NO_COMMAND,
	TAG_TX_ERROR,
	TAG_HUMAN_DISTANCE_OK,
	TAG_RESET

}TAG_STATUS_t;

#define TAG_TIMESTAMP_QUERY 0x11
#define TAG_SET_SLEEP_MODE 0x12

TAG_STATUS_t handle_sniffer_tag(TAG_t *tag);
TAG_STATUS_t handle_human_tag(TAG_t *tag);
double calculate_distance_human_tag(uint8_t *rx_buffer, Distance_t *distance);
double calculate_tag_distance(uint8_t *rx_buffer, Distance_t *distance);
TAG_STATUS_t send_message_with_timestamps();
uint32_t send_response_with_timestamps(uint8_t *tx_resp_msg, uint8_t size,
		uint32_t frame_seq_nb);
uint8_t read_human_tag_first_message(uint8_t *rx_buffer);
double distance_moving_average(Distance_t *distance);
void uart_transmit_hexa_to_text( uint8_t *message,
		uint8_t size);
void uart_transmit_float_to_text( double distanceValue);
int uart_transmit_string(char *message);
void uart_transmit_int_to_text(int distanceValue);
int start_transmission_inmediate_with_response_expected(TX_BUFFER_t tx);
TAG_STATUS_t wait_rx_data();
#endif /* INC_SNIFFER_TAG_H_ */

