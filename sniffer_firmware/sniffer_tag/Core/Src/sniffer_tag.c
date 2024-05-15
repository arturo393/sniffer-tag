/*
 * sniffer_tag.c
 *
 *  Created on: May 2, 2024
 *      Author: uqommdev
 */

#include "sniffer_tag.h"

TAG_t* create_TAG() {
	TAG_t *tag = (TAG_t*) malloc(sizeof(TAG_t));
	if (tag == NULL) {
		// Handle memory allocation failure
		return NULL;
	}

	reset_TAG_values(tag);

	return (tag);
}

void reset_TAG_values(TAG_t *tag) {
	tag->id = 0;
	tag->readings = 0;
	tag->command = 0;
	tag->distance = NULL;
	tag->distance_a.tof = 0.0;
	tag->distance_a.value = 0.0;
	for (int i = 0; i < DISTANCE_READINGS; i++) {
		tag->distance_a.readings[i] = 0.0;
		tag->distance_a.new[i] = 0.0;
	}
	tag->distance_a.sum = 0.0;
	tag->distance_a.last = 0.0;
	tag->distance_a.error_times = 0;
	tag->distance_a.counter = 0;

	tag->distance_b.tof = 0.0;
	tag->distance_b.value = 0.0;
	for (int i = 0; i < DISTANCE_READINGS; i++) {
		tag->distance_b.readings[i] = 0.0;
		tag->distance_b.new[i] = 0.0;
	}
	tag->distance_b.sum = 0.0;
	tag->distance_b.last = 0.0;
	tag->distance_b.error_times = 0;
	tag->distance_b.counter = 0;

	tag->resp_tx_timestamp = 0;
	tag->poll_rx_timestamp = 0;
}

TAG_STATUS_t handle_sniffer_tag(TAG_t *tag) {

	TX_BUFFER_t tx;
	tx.buffer = NULL;
	tx.buffer_size = 1;
	tx.delay = POLL_TX_TO_RESP_RX_DLY_UUS_6M8;
	tx.rx_timeout = RESP_RX_TIMEOUT_UUS_6M8;
	tx.preamble_timeout = PRE_TIMEOUT_6M8;
	tx.buffer = (uint8_t*) malloc(tx.buffer_size);
	if (tx.buffer == NULL)
		Error_Handler();

	if (tag->readings < DISTANCE_READINGS) {
		tx.buffer[0] = TAG_TIMESTAMP_QUERY;
		if (tag->readings < (DISTANCE_READINGS / 2) - 1) {
			hw = hw_a;
			tag->distance = &(tag->distance_a);
		} else {
			tx.buffer[0] = TAG_TIMESTAMP_QUERY;
			hw = hw_b;
			tag->distance = &(tag->distance_b);
		}

		if (tag->readings == DISTANCE_READINGS - 1)
			tx.buffer[0] = TAG_SET_SLEEP_MODE;
	} else {
		tag->readings = 0;
		return (TAG_TX_ERROR);
	}

	if (start_transmission_inmediate_with_response_expected(tx) == DWT_ERROR) {
		free(tx.buffer);
		return (TAG_TX_ERROR);
	}
	free(tx.buffer);
	TAG_STATUS_t status_reg = wait_rx_data();
	if (status_reg != TAG_RX_CRC_VALID)
		return (status_reg);
	uint8_t *rx_buffer;
	uint32_t rx_buffer_size = 0;
	rx_buffer_size = allocate_and_read_received_frame(&rx_buffer);

	tag->command = rx_buffer[0];
	switch (tag->command) {
	case TAG_TIMESTAMP_QUERY:
		tag->id = *(uint32_t*) (rx_buffer + 1);
		tag->distance->value = calculate_tag_distance(rx_buffer, tag->distance);
		tag->poll_rx_timestamp = *(uint32_t*) (rx_buffer + 1 + 4);
		tag->resp_tx_timestamp = *(uint32_t*) (rx_buffer + 1 + 2*4);
		tag->battery_voltage.raw = *(uint8_t*) (rx_buffer + 1 + 3*4 );
		tag->battery_voltage.calibrated = *(uint8_t*) (rx_buffer + 1 + 3*4 + 1);
		tag->temperature.raw = *(uint8_t*) (rx_buffer + 1 + 3*4 + 2*1);
		tag->temperature.calibrated = *(uint8_t*) (rx_buffer + 1 + 3*4 + 3*1);
		free(rx_buffer);
		return (TAG_HUMAN_DISTANCE_OK);
		break;
	case TAG_SET_SLEEP_MODE:
		tag->id = *(uint32_t*) (rx_buffer + 1);
		tag->distance->value = calculate_tag_distance(rx_buffer, tag->distance);
		free(rx_buffer);
		return (TAG_END_READINGS);
	default:
		return (TAG_RX_NO_COMMAND);
		free(rx_buffer);
		break;
	}
	free(rx_buffer);
	return (TAG_OK);
}

TAG_STATUS_t handle_human_tag(TAG_t *tag) {
	uint32_t status_reg = 0;
	/* Loop forever responding to ranging requests. */
	dwt_setpreambledetecttimeout(0);
	/* Clear reception timeout to start next ranging process. */
	dwt_setrxtimeout(0);
	/* Activate reception immediately. */
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
	/* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID))
			& (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO
					| SYS_STATUS_ALL_RX_ERR))) {
	};

	if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
		/* Clear RX error/timeout events in the DW IC status register. */
		dwt_write32bitreg(SYS_STATUS_ID,
				SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
		return (1);
	}

	uint32_t frame_len;
	/* Clear good RX frame event in the DW IC status register. */
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

	uint8_t rx_buffer[RX_BUF_LEN] = { 0 };
	uint8_t rx_buffer_size = read_human_tag_first_message(rx_buffer);
	if (rx_buffer_size == 0)
		return (2);

//uart_transmit_string("Human Tag send: ");
//uart_transmit_hexa_to_text(&huart1,rx_buffer,rx_buffer_size);
	int ret = send_message_with_human_tag_timestamp();

	/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
	if (ret == DWT_ERROR)
		return (3);

	/* Poll for reception of expected "final" frame or error/timeout. See NOTE 8 below. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID))
			& (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO
					| SYS_STATUS_ALL_RX_ERR))) {

	};
	/* Increment frame sequence number after transmission of the response message (modulo 256). */
	if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
		/* Clear RX error/timeout events in the DW IC status register. */
		dwt_write32bitreg(SYS_STATUS_ID,
				SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
		return (3);
	}

	/* Clear good RX frame event and TX frame sent in the DW IC status register. */
	dwt_write32bitreg(SYS_STATUS_ID,
			SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

	/* A frame has been received, read it into the local buffer. */
	frame_len = dwt_read32bitreg(
			RX_FINFO_ID) & FRAME_LEN_MAX_EX;
	if (frame_len <= RX_BUF_LEN) {
		dwt_readrxdata(rx_buffer, (uint16_t) frame_len, 0);
	}

	/* Check that the frame is a final message sent by "DS TWR initiator" example.
	 * As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
	rx_buffer[ALL_MSG_SN_IDX] = 0;
	uint8_t rx_final_msg[] = { 0x11, 0x00, 0x00, 0x00, 0x00 };
	if (memcmp(rx_buffer, rx_final_msg,
	INITIAL_COMUNICATION_DATA_SIZE) != 0) {
		return (4);
	}
	calculate_distance_human_tag(rx_buffer, &(tag->distance));

	return (TAG_OK);
}

double calculate_distance_human_tag(uint8_t *rx_buffer, Distance_t *distance) {
	uint32_t poll_tx_ts;
	uint32_t resp_rx_ts;
	uint32_t final_tx_ts;
	uint32_t poll_rx_ts_32;
	uint32_t resp_tx_ts_32;
	uint32_t final_rx_ts_32;
	double Ra;
	double Rb;
	double Da;
	double Db;
	int64_t tof_dtu;
	uint64_t final_rx_timestamp;
	uint64_t resp_tx_timestamp;
	uint64_t poll_rx_timestamp;

	/* Retrieve response transmission and final reception timestamps. */
	resp_tx_timestamp = get_tx_timestamp_u64();
	final_rx_timestamp = get_rx_timestamp_u64();

	/* Get timestamps embedded in the final message. */
	final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
	final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
	final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

	/* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */

	poll_rx_ts_32 = (uint32_t) poll_rx_timestamp;
	resp_tx_ts_32 = (uint32_t) resp_tx_timestamp;
	final_rx_ts_32 = (uint32_t) final_rx_timestamp;
	Ra = (double) (resp_rx_ts - poll_tx_ts);
	Rb = (double) (final_rx_ts_32 - resp_tx_ts_32);
	Da = (double) (final_tx_ts - resp_rx_ts);
	Db = (double) (resp_tx_ts_32 - poll_rx_ts_32);
	tof_dtu = (int64_t) ((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

	double tof;
	tof = (double) tof_dtu * DWT_TIME_UNITS;
	distance->value = tof * SPEED_OF_LIGHT;
	distance_moving_average(distance);
	return (distance->value);
}

double calculate_tag_distance(uint8_t *rx_buffer, Distance_t *distance) {
	uint64_t human_tag_rx_timestamp = 0;
	uint64_t human_tag_tx_timestamp = 0;
	uint64_t sniffer_tag_tx_timestamp = 0;
	uint64_t sniffer_tag_rx_timestamp = 0;
	uint64_t rtd_init = 0;
	uint64_t rtd_resp = 0;
	float clockOffsetRatio = 0.0;
	uint64_t final_tx_time = 0;

	/* Retrieve poll transmission and response reception timestamp. */
	sniffer_tag_tx_timestamp = get_tx_timestamp_u64();
	sniffer_tag_rx_timestamp = get_rx_timestamp_u64();

	/* Get timestamps embedded in response message. */
	human_tag_rx_timestamp = *(uint32_t*) (rx_buffer + 1 + 4);
	human_tag_tx_timestamp = *(uint32_t*) (rx_buffer + 1 + 4 + 4);
//uart_transmit_string("poll_rx_ts: ");
//uart_transmit_hexa_to_text((uint8_t*)&human_tag_rx_timestamp, 4);
//uart_transmit_int_to_text(human_tag_rx_timestamp);
//uart_transmit_string("resp_tx_ts: ");
//uart_transmit_hexa_to_text((uint8_t*)&human_tag_tx_timestamp, 4);
//uart_transmit_int_to_text(human_tag_tx_timestamp);

//resp_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_rx_ts);
//resp_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_tx_ts);

	/* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
	rtd_init = sniffer_tag_rx_timestamp - sniffer_tag_tx_timestamp;
	rtd_resp = human_tag_tx_timestamp - human_tag_rx_timestamp;

	/*  Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
	clockOffsetRatio = ((float) dwt_readclockoffset()) / (uint32_t) (1 << 26);
	/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
	double tof;
	tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0)
			* DWT_TIME_UNITS;

	distance->value = tof * SPEED_OF_LIGHT;

//The data were smoothed and filtered
	distance_smooth(distance);
	return (distance->value);
}

TAG_STATUS_t send_message_with_timestamps() {

	uint64_t poll_tx_timestamp;
	uint64_t resp_rx_timestamp;
	uint64_t final_tx_timestamp;
	uint32_t final_tx_time;
	uint8_t tx_final_msg[13];

	uint8_t timestamp_data[20] = { 0 };
	int ret;
	tx_final_msg[0] = 0x11;
	/* Retrieve poll transmission and response reception timestamp. */
	poll_tx_timestamp = get_tx_timestamp_u64();
	resp_rx_timestamp = get_rx_timestamp_u64();
	final_tx_time = (resp_rx_timestamp
			+ (RESP_RX_TO_FINAL_TX_DLY_UUS_6M8 * UUS_TO_DWT_TIME)) >> 8;
	dwt_setdelayedtrxtime(final_tx_time);

	/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
	final_tx_timestamp = (((uint64_t) (final_tx_time & 0xFFFFFFFEUL)) << 8)
			+ TX_ANT_DLY_HP;

	uint32_t timestamps[3] = { poll_tx_timestamp, resp_rx_timestamp,
			final_tx_timestamp };
// Copy timestamps to final message using memcpy
	memcpy(tx_final_msg + 1, timestamps, sizeof(timestamps));

	/* Write and send final message. See NOTE 9 below. */

	dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(sizeof(tx_final_msg) + FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging bit set. */

	ret = dwt_starttx(DWT_START_TX_DELAYED);
	/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 13 below. */

	if (ret == DWT_SUCCESS) {
		/* Poll DW IC until TX frame sent event set. See NOTE 10 below. */
		while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
		};

		/* Clear TXFRS event. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

		/* Increment frame sequence number after transmission of the final message (modulo 256). */
		return (0);
	}
	return (1);
}

uint32_t send_response_with_timestamps(uint8_t *tx_resp_msg, uint8_t size,
		uint32_t frame_seq_nb) {
	uint32_t resp_tx_time;
	uint64_t resp_tx_timestamp;
	uint64_t poll_rx_timestamp;
	int ret;
	uint32_t status_reg = 0;

	/* Retrieve poll reception timestamp. */
	poll_rx_timestamp = get_rx_timestamp_u64();

	/* Set send time for response. See NOTE 9 below. */
	resp_tx_time = (uint32_t) ((poll_rx_timestamp
			+ ((POLL_RX_TO_RESP_TX_DLY_UUS_6M8) * UUS_TO_DWT_TIME)) >> 8);

	/* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
	resp_tx_timestamp = (((uint64_t) (resp_tx_time & 0xFFFFFFFEUL)) << 8)
			+ TX_ANT_DLY_HP;

	/* Write all timestamps in the final message. See NOTE 8 below. */
	resp_msg_set_ts(&tx_resp_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_rx_timestamp);
	resp_msg_set_ts(&tx_resp_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_tx_timestamp);

	tx_resp_msg[ALL_MSG_SN_IDX] = (uint8_t) frame_seq_nb;
	dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
	/*DWT_START_TX_DELAYED DWT_START_TX_IMMEDIATE*/
	ret = dwt_starttx(
	DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

	/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
	if (ret == DWT_SUCCESS) {

		/* Poll for reception of expected "final" frame or error/timeout. See NOTE 8 below. */
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID))
				& (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO
						| SYS_STATUS_ALL_RX_ERR))) {

		};
		dwt_setdelayedtrxtime(resp_tx_time);
		/* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
		dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS_6M8);
		/* FINAL_RX_TIMEOUT_UUS. */
		dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS_6M8);
		/* Set preamble timeout for expected frames. See NOTE 6 below. */
		dwt_setpreambledetecttimeout(PRE_TIMEOUT_6M8);

		/* Increment frame sequence number after transmission of the response message (modulo 256). */
		frame_seq_nb++;
	}
	return (status_reg);
}

uint32_t allocate_and_read_received_frame(uint8_t **rx_buffer) {
	uint32_t rx_buffer_size = 0;

	/* Read the frame information and extract the frame length. */
	rx_buffer_size = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;

	/* Check if the frame length is non-zero. */
	if (rx_buffer_size > 0) {
		/* Allocate memory dynamically for the buffer. */
		*rx_buffer = (uint8_t*) malloc(rx_buffer_size * sizeof(uint8_t));

		if (*rx_buffer != NULL) {
			/* Read the received data into the dynamically allocated buffer. */
			dwt_readrxdata(*rx_buffer, (uint16_t) rx_buffer_size, 0);

			/* Add any additional processing here if needed. */

			return rx_buffer_size;
		} else {
			/* Memory allocation failed. */
			return 0;
		}
	}

	/* No data received, return 0. */
	return 0;
}

double distance_moving_average(Distance_t *distance) {
	/*The data were smoothed and filtered */
	if (distance->counter < 10) {
		distance->readings[distance->counter] = distance->value;
		distance->counter++;
		distance->sum = 0;
		for (int i = 0; i < distance->counter; i++) {
			distance->sum += distance->readings[i];
		}
		distance->value = distance->sum / (double) distance->counter;
	} else {
		if (fabs(distance->value - distance->last) < MAX_DISTANCE_ERROR) {
			distance->error_times = 0;
			for (int i = 0; i < distance->counter - 1; i++) {
				distance->new[i] = distance->readings[i + 1];
			}
			distance->new[distance->counter - 1] = distance->value;
			distance->sum = 0;
			for (int i = 0; i < distance->counter; i++) {
				distance->sum += distance->new[i];
				distance->readings[i] = distance->new[i];
			}
			distance->last = distance->value;
			distance->value = distance->sum / (double) distance->counter;
		} else {
			distance->value = distance->last;
			distance->error_times++;
			if (distance->error_times > 20) {
				distance->error_times = 0;
				distance->counter = 0;
			}
		}
	}

	return (distance->value);
}
double distance_smooth(Distance_t *distance) {
////Perform average on sensor readings
// subtract the last reading:
	distance->sum = distance->sum - distance->readings[distance->counter];
// read the sensor:
	if (fabs(distance->value - distance->last) > MAX_DISTANCE_ERROR)
		distance->value = distance->last;
	distance->readings[distance->counter] = distance->value;
// add value to total:
	distance->sum = distance->sum + distance->readings[distance->counter];
// handle index
	distance->counter = distance->counter + 1;
	if (distance->counter >= DISTANCE_READINGS) {
		distance->counter = 0;
	}
// calculate the average:
	distance->last = distance->sum / DISTANCE_READINGS;
	distance->last = distance->value;
	return (distance->value);
}

int send_message_with_human_tag_timestamp() {
	uint32_t resp_tx_time;
	uint64_t resp_tx_timestamp;
	uint64_t poll_rx_timestamp;
	int ret;

	/* Retrieve poll reception timestamp. */
	poll_rx_timestamp = get_rx_timestamp_u64();

	/* Set send time for response. See NOTE 9 below. */
	resp_tx_time = (uint32_t) ((poll_rx_timestamp
			+ ((POLL_RX_TO_RESP_TX_DLY_UUS_6M8) * UUS_TO_DWT_TIME)) >> 8);

	dwt_setdelayedtrxtime(resp_tx_time);

	/* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
	resp_tx_timestamp = (((uint64_t) (resp_tx_time & 0xFFFFFFFEUL)) << 8)
			+ TX_ANT_DLY_HP;
	uint8_t tx_resp_msg[20] = { 0 };
	tx_resp_msg[0] = 0x11;
	uint32_t timestamps[2] = { (uint32_t) poll_rx_timestamp,
			(uint32_t) resp_tx_timestamp };
	memcpy(tx_resp_msg + 1, timestamps, sizeof(timestamps));
	/* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
	dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS_6M8);
	/* FINAL_RX_TIMEOUT_UUS. */
	dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS_6M8);
	/* Set preamble timeout for expected frames. See NOTE 6 below. */
	dwt_setpreambledetecttimeout(PRE_TIMEOUT_6M8);
	/* Write and send the response message. See NOTE 10 below.*/
	dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
	/*DWT_START_TX_DELAYED DWT_START_TX_IMMEDIATE*/
	ret = dwt_starttx(
	DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

	return (ret);
}

int start_transmission_inmediate_with_response_expected(TX_BUFFER_t tx) {
	dwt_setrxaftertxdelay(tx.delay);
	dwt_setrxtimeout(tx.rx_timeout);
	dwt_setpreambledetecttimeout(tx.preamble_timeout);
	/* Write frame data to DW IC and prepare transmission. See NOTE 9 below. */

//tx_poll_msg[ALL_MSG_SN_IDX] = (uint8_t) tag->detection_times;
	dwt_writetxdata(tx.buffer_size, tx.buffer, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(tx.buffer_size + FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging. */

	/* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
	 * set by dwt_setrxaftertxdelay() has elapsed. */
	int rets = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

	if (rets == DWT_ERROR) {
		/* Clear RX error/timeout events in the DW IC status register. */
		dwt_write32bitreg(SYS_STATUS_ID,
				SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK);

	}

	return (rets);
}
#define RX_TIMEOUT_MS 1000

TAG_STATUS_t wait_rx_data() {
	uint32_t status_reg;
	uint32_t start_time = HAL_GetTick(); // Get the current time in milliseconds

	while ((HAL_GetTick() - start_time) < RX_TIMEOUT_MS) {
		status_reg = dwt_read32bitreg(SYS_STATUS_ID);

		/* Check for receive errors */
		if (status_reg & SYS_STATUS_ALL_RX_ERR) {
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
			return (TAG_RX_ERROR);
		}

		/* Check for receive frame timeouts */
		if (status_reg & SYS_STATUS_RXFTO_BIT_MASK) {
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFTO_BIT_MASK);
			return (TAG_RX_FRAME_TIMEOUT);
		}

		/* Check for receive preamble detection timeouts */
		if (status_reg & SYS_STATUS_RXPTO_BIT_MASK) {
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXPTO_BIT_MASK);
			return (TAG_RX_PREAMBLE_DETECTION_TIMEOUT);
		}

		/* Check for good RX frame event */
		if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
			/* Clear good RX frame event in the DW IC status register */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
			return (TAG_RX_CRC_VALID);
		}
	}
	dwt_write32bitreg(SYS_STATUS_ID,
			SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK);

	return (TAG_NO_RXCG_DETECTED);
}

void uart_transmit_int_to_text(int distanceValue) {
	/* Calculate the size needed for the formatted string */
	int size = snprintf(NULL, 0, "%u\n\r", distanceValue);
	size++;

	/* Dynamically allocate memory for dist_str */
	char *dist_str = (char*) malloc(size * sizeof(char));
	if (dist_str == NULL) {

		return;
	}

	/* Format the string into dist_str */
	sprintf(dist_str, "%u\n\r", distanceValue);

	/* Transmit the formatted string */
	HAL_UART_Transmit(&huart1, (uint8_t*) dist_str, size, HAL_MAX_DELAY);

	/* Free the dynamically allocated memory */
	free(dist_str);
}

void debug(TAG_t *tag) {
	/* Format the string directly into the dynamically allocated buffer */
	char *dist_str = NULL;
	int size = 0;

	size =
			asprintf(&dist_str,
					"{ID: %lu},{command: %d},{times: %lu},{poll_rx_ts: %lu},{resp_tx_ts: %lu},{distance_a: %.2f},{battery_voltage: %.2f},{temperature: %.2f}\n\r",
					(unsigned long) tag->id, (int) tag->command,
					(unsigned long) tag->readings,
					(unsigned long) tag->poll_rx_timestamp,
					(unsigned long) tag->resp_tx_timestamp,
					tag->distance_a.value, (float) tag->battery_voltage.real,
					(float) tag->temperature.real);

	if (size != -1 && dist_str != NULL) {
		/* Transmit the formatted string */
		HAL_UART_Transmit(&huart1, (uint8_t*) dist_str, size, HAL_MAX_DELAY);

		/* Free the allocated memory */
		free(dist_str);
	}
}

void set_battery_voltage(Mesurement_data_t *measurement) {
	/* Bench measurements gives approximately:
	 * VDDBAT = sar_read * Vref / max_code * 16x_atten   - assume Vref @ 3.0V
	 */
	measurement->real = (float) ((float) (measurement->raw
			- measurement->calibrated) * 0.4f * 16 / 255) + 3.0f;
}

void set_temperature(Mesurement_data_t *measurement) {
	/* the User Manual formula is:
	 * Temperature (�C) = ( (SAR_LTEMP ?OTP_READ(Vtemp @ 20�C) ) x 1.05)        // Vtemp @ 20�C
	 */
	measurement->real = (float) ((measurement->raw - measurement->calibrated)
			* 1.05f) + 20.0f;
}
