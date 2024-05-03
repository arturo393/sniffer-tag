/*
 * sniffer_tag.c
 *
 *  Created on: May 2, 2024
 *      Author: uqommdev
 */

#include "sniffer_tag.h"

TAG_STATUS_t handle_sniffer_tag(TAG_t *tag) {
	int rets;
	uint8_t tx_poll_msg[] = { 0x11, 0x00, 0x00, 0x00, 0x00 };

	uint8_t tx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E',
			0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint32_t status_reg = 0;
	uint8_t dist_str[30];

	/* Write frame data to DW IC and prepare transmission. See NOTE 9 below. */
	//tx_poll_msg[ALL_MSG_SN_IDX] = (uint8_t) tag->detection_times;
	dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(sizeof(tx_poll_msg) + FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging. */

	/* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
	 * set by dwt_setrxaftertxdelay() has elapsed. */
	rets = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

	if (rets == DWT_ERROR) { /* Clear RX error/timeout events in the DW IC status register. */
		dwt_write32bitreg(SYS_STATUS_ID,
				SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK);

		return (TAG_NO_RESPONSE);
	}

	/* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 10 below. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID))
			& (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO
					| SYS_STATUS_ALL_RX_ERR))) {
	};

	if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
		uint16_t size = sprintf((char*) dist_str, "No TAG response\n\r");
		HAL_UART_Transmit(&huart1, dist_str, (uint16_t) size,
		HAL_MAX_DELAY);
		return (2);
	}
	/* Clear good RX frame event and TX frame sent in the DW IC status register. */
	dwt_write32bitreg(SYS_STATUS_ID,
			SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

	uint8_t rx_buffer[RX_BUF_LEN] = { 0 };
	read_human_tag_first_message(rx_buffer);
	uint8_t command = rx_buffer[0];
	if (command != 0x11)
		return (3);

	tag->distance.value = calculate_tag_distance(rx_buffer, &(tag->distance));
	tx_final_msg[ALL_MSG_SN_IDX] = tag->detection_times;
	if (send_message_with_timestamps(tx_final_msg, sizeof(tx_final_msg))
			== TAG_OK) {
		tag->detection_times++;
	}
	/*
	 for (int i = 0; i < (int) sizeof(tx_final_msg); i++) {
	 sprintf((char*) dist_str, "%02X ", tx_final_msg[i]); // Print with leading zeros for 2-digit format
	 HAL_UART_Transmit(&huart1, dist_str, (uint16_t) 2,
	 HAL_MAX_DELAY);
	 }
	 sprintf((char*) dist_str, "\n\r");
	 HAL_UART_Transmit(&huart1, dist_str, (uint16_t) 2,
	 HAL_MAX_DELAY);
	 */
	/* Display computed distance on HMI display. */
	uint16_t size = sprintf((char*) dist_str, "times: %d distance:%.2f[m]\n\r",
			tag->detection_times, tag->distance.value);
	HAL_UART_Transmit(&huart1, dist_str, (uint16_t) size,
	HAL_MAX_DELAY);
	memset(rx_buffer, 0, RX_BUF_LEN);

	return (TAG_OK);
}

TAG_STATUS_t handle_human_tag(TAG_t *tag) {
	uint32_t status_reg = 0;

	uint8_t dist_str[30];
	uint8_t frame_seq_nb;
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
	if (read_human_tag_first_message(rx_buffer))
		return (2);

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

	/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
	if (ret == DWT_ERROR)
		return (3);

	/* Poll for reception of expected "final" frame or error/timeout. See NOTE 8 below. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID))
			& (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO
					| SYS_STATUS_ALL_RX_ERR))) {

	};
	/* Increment frame sequence number after transmission of the response message (modulo 256). */
	frame_seq_nb++;
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
	for (int i = 0; i < (int) frame_len; i++) {
		sprintf((char*) dist_str, "%02X ", rx_buffer[i]); // Print with leading zeros for 2-digit format
		HAL_UART_Transmit(&huart1, dist_str, (uint16_t) 2,
		HAL_MAX_DELAY);
	}
	sprintf((char*) dist_str, "\n\r");
	HAL_UART_Transmit(&huart1, dist_str, (uint16_t) 2,
	HAL_MAX_DELAY);

	/* Check that the frame is a final message sent by "DS TWR initiator" example.
	 * As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
	rx_buffer[ALL_MSG_SN_IDX] = 0;
	uint8_t rx_final_msg[] = { 0x11, 0x00, 0x00, 0x00, 0x00 };
	if (memcmp(rx_buffer, rx_final_msg,
	INITIAL_COMUNICATION_DATA_SIZE) != 0) {
		return (4);
	}
	calculate_distance_human_tag(rx_buffer, &(tag->distance));

	/* Display computed distance on HMI display. */
	int size = sprintf((char*) dist_str, "distance:%.2f\n\r",
			tag->distance.value);
	HAL_UART_Transmit(&huart1, dist_str, (uint16_t) size,
	HAL_MAX_DELAY);
	flags.ds_twr_timeout = 1;

	/* Changing the value of target_allow_run_time adjusts the interval (in ms) between runs of the example again */
//flags.target_allow_run_time = DSTWR_RESP_RERUN_INTERVAL;
	flags.time_to_allow_run = 1;

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
	uint32_t poll_rx_ts;
	uint32_t resp_tx_ts;
	uint64_t poll_tx_timestamp;
	uint64_t resp_rx_timestamp;
	uint64_t final_tx_timestamp;
	int32_t rtd_init;
	uint32_t rtd_resp;
	float clockOffsetRatio;
	uint32_t final_tx_time;

	/* Retrieve poll transmission and response reception timestamp. */
	poll_tx_timestamp = get_tx_timestamp_u64();
	resp_rx_timestamp = get_rx_timestamp_u64();
	final_tx_time = (resp_rx_timestamp
			+ (RESP_RX_TO_FINAL_TX_DLY_UUS_6M8 * UUS_TO_DWT_TIME)) >> 8;

	dwt_setdelayedtrxtime(final_tx_time);

	/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
	final_tx_timestamp = (((uint64_t) (final_tx_time & 0xFFFFFFFEUL)) << 8)
			+ TX_ANT_DLY_HP;

	/*  Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
	clockOffsetRatio = ((float) dwt_readclockoffset()) / (uint32_t) (1 << 26);

	/* Get timestamps embedded in response message. */
	poll_rx_ts =*(uint32_t*) (rx_buffer + 1);
	resp_tx_ts =*(uint32_t*) (rx_buffer + 1+4);

	//resp_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_rx_ts);
	//resp_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_tx_ts);

	/* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
	rtd_init = resp_rx_timestamp - poll_tx_timestamp;
	rtd_resp = resp_tx_ts - poll_rx_ts;

	/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
	double tof;
	tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0)
			* DWT_TIME_UNITS;
	distance->value = tof * SPEED_OF_LIGHT;
	//The data were smoothed and filtered
	distance_moving_average(distance);
	return (distance->value);
}

TAG_STATUS_t send_message_with_timestamps(uint8_t *tx_final_msg, uint8_t size) {

	uint64_t poll_tx_timestamp;
	uint64_t resp_rx_timestamp;
	uint64_t final_tx_timestamp;
	uint32_t final_tx_time;
	uint8_t timestamp_data[20] = { 0 };
	int ret;

	/* Retrieve poll transmission and response reception timestamp. */
	poll_tx_timestamp = get_tx_timestamp_u64();
	resp_rx_timestamp = get_rx_timestamp_u64();
	final_tx_time = (resp_rx_timestamp
			+ (RESP_RX_TO_FINAL_TX_DLY_UUS_6M8 * UUS_TO_DWT_TIME)) >> 8;
	dwt_setdelayedtrxtime(final_tx_time);

	/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
	final_tx_timestamp = (((uint64_t) (final_tx_time & 0xFFFFFFFEUL)) << 8)
			+ TX_ANT_DLY_HP;

	/* Write all timestamps in the final message. See NOTE 12 below. */
	final_msg_set_ts(tx_final_msg + FINAL_MSG_POLL_TX_TS_IDX,
			poll_tx_timestamp);
	final_msg_set_ts(tx_final_msg + FINAL_MSG_RESP_RX_TS_IDX,
			resp_rx_timestamp);
	final_msg_set_ts(tx_final_msg + FINAL_MSG_FINAL_TX_TS_IDX,
			final_tx_timestamp);

	uint64_t timestamps[3] = { poll_tx_timestamp, resp_rx_timestamp,
			final_tx_timestamp };
	// Copy timestamps to final message using memcpy
	memcpy(timestamp_data + FINAL_MSG_POLL_TX_TS_IDX, timestamps,
			sizeof(timestamps));

	/* Write and send final message. See NOTE 9 below. */

	dwt_writetxdata(size, tx_final_msg, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(size + FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging bit set. */

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

uint8_t read_human_tag_first_message(uint8_t *rx_buffer) {

	uint8_t rx_resp_msg[] = { 0x11 };
	uint32_t frame_len;

	/* A frame has been received, read it into the local buffer. */
	frame_len =
	dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
	if (frame_len <= RX_BUF_LEN) {
		dwt_readrxdata(rx_buffer, (uint16_t) frame_len, 0);
		return (TAG_OK);
	}
	return (TAG_NO_RESPONSE);

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

