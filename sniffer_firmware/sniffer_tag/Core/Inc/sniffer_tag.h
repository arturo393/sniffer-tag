/*
 * sniffer_tag.h
 *
 *  Created on: May 2, 2024
 *      Author: uqommdev
 */

#ifndef INC_SNIFFER_TAG_H_
#define INC_SNIFFER_TAG_H_

#include "main.h"
#include "uwb3000Fxx.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart1;
extern SPI_HW_t *hw_a;
extern SPI_HW_t *hw_b;
extern SPI_HW_t *hw;

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

typedef struct{
	uint8_t calibrated;
	uint8_t raw;
	float real;
}Mesurement_data_t;

typedef struct {
	uint32_t id;
	int readings;
	uint8_t command;
	Distance_t *distance;
	Distance_t distance_a;
	Distance_t distance_b;
	Mesurement_data_t temperature;
	Mesurement_data_t battery_voltage;
	uint32_t resp_tx_timestamp;
	uint32_t poll_rx_timestamp;
} TAG_t;


typedef struct buffer {
    uint8_t *buffer;
    uint8_t buffer_size;
    int delay;
    int rx_timeout;
    int preamble_timeout;
	uint32_t resp_tx_timestamp;
	uint32_t poll_rx_timestamp;
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
	TAG_END_READINGS

}TAG_STATUS_t;

#define TAG_TIMESTAMP_QUERY 0x11
#define TAG_SET_SLEEP_MODE 0x12
TAG_t *create_TAG();
void reset_TAG_values(TAG_t *tag);
TAG_STATUS_t handle_sniffer_tag(TAG_t *tag);
TAG_STATUS_t handle_human_tag(TAG_t *tag);
double calculate_distance_human_tag(uint8_t *rx_buffer, Distance_t *distance);
double calculate_tag_distance(uint8_t *rx_buffer, Distance_t *distance);
TAG_STATUS_t send_message_with_timestamps();
uint32_t send_response_with_timestamps(uint8_t *tx_resp_msg, uint8_t size,
		uint32_t frame_seq_nb);
uint32_t allocate_and_read_received_frame(uint8_t **rx_buffer);
double distance_moving_average(Distance_t *distance);
int start_transmission_inmediate_with_response_expected(TX_BUFFER_t tx);
TAG_STATUS_t wait_rx_data();
void debug(TAG_t *tag);
double distance_smooth(Distance_t *distance);
void set_battery_voltage(Mesurement_data_t *battery_voltage);
void set_temperature(Mesurement_data_t * temperature);
#endif /* INC_SNIFFER_TAG_H_ */

