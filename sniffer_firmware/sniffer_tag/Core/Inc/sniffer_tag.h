/*
 * sniffer_tag.h
 *
 *  Created on: May 2, 2024
 *      Author: uqommdev
 */

#ifndef INC_SNIFFER_TAG_H_
#define INC_SNIFFER_TAG_H_

#include "main.h"
#include <uwb3000Fxx.h>

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
	uint64_t detection_times;
	Distance_t distance;
} TAG_t;


typedef enum{
	TAG_OK,
	TAG_NO_RESPONSE,

}TAG_STATUS_t;
TAG_STATUS_t handle_sniffer_tag(TAG_t *tag);
TAG_STATUS_t handle_human_tag(TAG_t *tag);
double calculate_distance_human_tag(uint8_t *rx_buffer, Distance_t *distance);
double calculate_tag_distance(uint8_t *rx_buffer, Distance_t *distance);
TAG_STATUS_t send_message_with_timestamps(uint8_t *tx_final_msg, uint8_t size);
uint32_t send_response_with_timestamps(uint8_t *tx_resp_msg, uint8_t size,
		uint32_t frame_seq_nb);
uint8_t read_human_tag_first_message(uint8_t *rx_buffer);
double distance_moving_average(Distance_t *distance);
#endif /* INC_SNIFFER_TAG_H_ */

