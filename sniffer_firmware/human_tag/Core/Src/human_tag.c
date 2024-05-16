/*
 * sniffer_tag.c
 *
 *  Created on: May 2, 2024
 *      Author: uqommdev
 */

#include <human_tag.h>

#define TX_BUFFER_SIZE (sizeof(uint8_t) + 3 * sizeof(uint32_t) + 4 * sizeof(uint8_t))

TAG_STATUS_t handle_human_tag(TAG_t *tag) {
	TAG_STATUS_t status_reg = 0;
	uint8_t tx_buffer_size = TX_BUFFER_SIZE;
	uint8_t tx_buffer[TX_BUFFER_SIZE] = { 0 };
	int index = 0;
	tx_buffer[index++] = tag->command;
	*(uint32_t*) (tx_buffer + index) = tag->id;
	index += sizeof(uint32_t);
	*(uint32_t*) (tx_buffer + index) = 0;
	index += sizeof(uint32_t);
	*(uint32_t*) (tx_buffer + index) = 0;
	index += sizeof(uint32_t);
	tx_buffer[index++] = tag->raw_battery_voltage;
	tx_buffer[index++] = tag->calibrated_battery_voltage;
	tx_buffer[index++] = tag->raw_temperature;
	tx_buffer[index++] = tag->calibrateds_temperature;

	start_tag_reception_inmediate(0, 0);
	status_reg = wait_rx_data();
	if (status_reg != TAG_RX_CRC_VALID)
		return (status_reg);

	uint32_t rx_buffer_size = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
	uint8_t rx_buffer[3];
	if (rx_buffer_size > 0) {
		dwt_readrxdata(rx_buffer, (uint16_t) rx_buffer_size, 0);
	}

	if (rx_buffer_size == 0)
		return (TAG_RX_DATA_ZERO);
	tag->command = rx_buffer[0];
	if (tag->command == TAG_TIMESTAMP_QUERY
			|| tag->command == TAG_SET_SLEEP_MODE) {
		uint64_t poll_rx_timestamp = get_rx_timestamp_u64();

		/** Set send time for response */
		uint32_t resp_tx_time = (uint32_t) ((poll_rx_timestamp
				+ ((POLL_RX_TO_RESP_TX_DLY_UUS_6M8) * UUS_TO_DWT_TIME))
				>> RESPONSE_TX_TIME_SHIFT_AMOUNT);
		dwt_setdelayedtrxtime(resp_tx_time);

		/** Calculate the response TX timestamp */
		uint64_t resp_tx_timestamp =
				(((uint64_t) (resp_tx_time & RESPONSE_TX_TIME_MASK_VALUE))
						<< RESPONSE_TX_TIME_SHIFT_AMOUNT) + TX_ANT_DLY_LP;
		/** Calculate the size needed for the response message buffer */
		int index = 0;
		index++;
		index += sizeof(uint32_t);
		*(uint32_t*) (tx_buffer + index) = poll_rx_timestamp;
		index += sizeof(uint32_t);
		*(uint32_t*) (tx_buffer + index) = resp_tx_timestamp;

		tag->poll_rx_timestamp = poll_rx_timestamp;
		tag->resp_tx_timestamp = resp_tx_timestamp;

		if (dwt_writetxdata(tx_buffer_size, tx_buffer, 0) == DWT_ERROR) /* Zero offset in TX buffer. */
			return (TAG_TX_ERROR);
		dwt_writetxfctrl(tx_buffer_size + 2, 0, 1);
		/*DWT_START_TX_DELAYED DWT_START_TX_IMMEDIATE*/
		if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_ERROR)
			return (TAG_TX_ERROR);

		if (tag->command == TAG_TIMESTAMP_QUERY)
			return (TAG_OK);
		else if (tag->command == TAG_SET_SLEEP_MODE)
			return (TAG_SLEEP);
	}
	return (TAG_RX_COMMAND_ERROR);
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

uint8_t read_human_tag_first_message(uint8_t *rx_buffer) {

	uint32_t frame_len = 0;

	/* A frame has been received, read it into the local buffer. */
	frame_len =
	dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
	if (frame_len <= RX_BUF_LEN)
		dwt_readrxdata(rx_buffer, (uint16_t) frame_len, 0);

	return (frame_len);

}

void uart_transmit_hexa_to_text(uint8_t *message, uint8_t size) {
	uint8_t *hexa_text = (uint8_t*) malloc(sizeof(uint8_t) * size);
	for (int i = 0; i < size; i++) {
		sprintf((char*) hexa_text, "%02X ", message[i]); // Print with leading zeros for 2-digit format
		HAL_UART_Transmit(&huart1, hexa_text, (uint16_t) 2,
		HAL_MAX_DELAY);
	}
	sprintf((char*) hexa_text, "\n\r");
	HAL_UART_Transmit(&huart1, hexa_text, (uint16_t) 2,
	HAL_MAX_DELAY);
	free(hexa_text);
}

void uart_transmit_float_to_text(float distanceValue) {
	/* Calculate the size needed for the formatted string */
	int size = snprintf(NULL, 0, "%.2f\n\r", distanceValue);
	size++; // Include space for the null terminator

	/* Dynamically allocate memory for dist_str */
	char *dist_str = (char*) malloc(size * sizeof(char));
	if (dist_str == NULL) {
		// Handle allocation failure
		// For example: return an error code or exit the function
		return;
	}

	/* Format the string into dist_str */
	sprintf(dist_str, "%.2f\n\r", distanceValue);

	/* Transmit the formatted string */
	HAL_UART_Transmit(&huart1, (uint8_t*) dist_str, size, HAL_MAX_DELAY);

	/* Free the dynamically allocated memory */
	free(dist_str);
}

void uart_transmit_int_to_text(int distanceValue) {
	/* Calculate the size needed for the formatted string */
	int size = snprintf(NULL, 0, "%u\n\r", distanceValue);
	size++; // Include space for the null terminator

	/* Dynamically allocate memory for dist_str */
	char *dist_str = (char*) malloc(size * sizeof(char));
	if (dist_str == NULL) {
		// Handle allocation failure
		// For example: return an error code or exit the function
		return;
	}

	/* Format the string into dist_str */
	sprintf(dist_str, "%u\n\r", distanceValue);

	/* Transmit the formatted string */
	HAL_UART_Transmit(&huart1, (uint8_t*) dist_str, size, HAL_MAX_DELAY);

	/* Free the dynamically allocated memory */
	free(dist_str);
}

int uart_transmit_string(char *message) {
	uint16_t message_size = strlen(message) + 1; // Include space for the null terminator

	// Dynamically allocate memory for the message
	char *dynamic_message = (char*) malloc(message_size * sizeof(char));
	if (dynamic_message == NULL) {
		// Handle allocation failure
		return -1; // Return an error code
	}

	// Copy the message to the dynamically allocated memory
	strcpy(dynamic_message, message);

	// Transmit the dynamic message
	HAL_UART_Transmit(&huart1, (uint8_t*) dynamic_message, message_size,
	HAL_MAX_DELAY);

	// Free the dynamically allocated memory
	free(dynamic_message);

	return (2);
}

uint32_t create_message_and_alloc_buffer(TX_BUFFER_t *tx, TAG_t *tag) {
	uint32_t resp_tx_time = 0;
	uint64_t resp_tx_timestamp = 0;
	uint64_t poll_rx_timestamp = 0;

	/** Retrieve poll reception timestamp */
	poll_rx_timestamp = get_rx_timestamp_u64();

	/** Set send time for response */
	resp_tx_time = (uint32_t) ((poll_rx_timestamp
			+ ((POLL_RX_TO_RESP_TX_DLY_UUS_6M8) * UUS_TO_DWT_TIME))
			>> RESPONSE_TX_TIME_SHIFT_AMOUNT);
	dwt_setdelayedtrxtime(resp_tx_time);

	/** Calculate the response TX timestamp */
	resp_tx_timestamp =
			(((uint64_t) (resp_tx_time & RESPONSE_TX_TIME_MASK_VALUE))
					<< RESPONSE_TX_TIME_SHIFT_AMOUNT) + TX_ANT_DLY_LP;
	/** Calculate the size needed for the response message buffer */
	tx->buffer_size = sizeof(uint8_t) + 3 * sizeof(uint32_t)
			+ 4 * sizeof(uint8_t);

	/** Allocate memory for the response message buffer */
	tx->buffer = (uint8_t*) malloc(tx->buffer_size);
	if (tx->buffer == NULL) {
		/** Handle memory allocation failure */
		return (0);
	}

	// Set the first byte of the buffer to TAG_TIMESTAMP_QUERY
	tx->buffer[0] = tag->command;

	// Write tag_id to the buffer starting from the second byte
	*(uint32_t*) (tx->buffer + 1) = _dwt_otpread(PARTID_ADDRESS);

	// Write poll_rx_timestamp to the buffer
	*(uint32_t*) (tx->buffer + 1 + sizeof(uint32_t)) = poll_rx_timestamp;

	// Write resp_tx_timestamp to the buffer
	*(uint32_t*) (tx->buffer + 1 + 2 * sizeof(uint32_t)) = resp_tx_timestamp;
	// Write resp_tx_timestamp to the buffer
	*(uint8_t*) (tx->buffer + 1 + 3 * sizeof(uint32_t)) =
			tag->raw_battery_voltage;
	// Write resp_tx_timestamp to the buffer
	*(uint8_t*) (tx->buffer + 1 + 3 * sizeof(uint32_t) + sizeof(uint8_t)) =
			tag->calibrated_battery_voltage;
	// Write resp_tx_timestamp to the buffer
	*(uint8_t*) (tx->buffer + 1 + 3 * sizeof(uint32_t) + 2 * sizeof(uint8_t)) =
			tag->raw_temperature;
	// Write resp_tx_timestamp to the buffer
	*(uint8_t*) (tx->buffer + 1 + 3 * sizeof(uint32_t) + 3 * sizeof(uint8_t)) =
			tag->calibrateds_temperature;

	tx->poll_rx_timestamp = poll_rx_timestamp;
	tx->resp_tx_timestamp = resp_tx_timestamp;

	return (tx->buffer_size);
}

void start_tag_reception_inmediate(uint8_t preamble_timeout, uint8_t rx_timeout) {

	/* Loop forever responding to ranging requests. */
	dwt_setpreambledetecttimeout(preamble_timeout);
	/* Clear reception timeout to start next ranging process. */
	dwt_setrxtimeout(rx_timeout);
	/* Activate reception immediately. */
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
	/* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
}

TAG_STATUS_t wait_rx_data() {
	uint32_t status_reg;
	/* wait for crc check and (frame wait timeout and preamble detect timeout) mask. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID))
			& (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO
					| SYS_STATUS_ALL_RX_ERR))) {
	};
	/* check for any receive error*/
	if ((status_reg & SYS_STATUS_ALL_RX_ERR)) {
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
		return (TAG_RX_ERROR);
	}
	/* check for receive timeouts */
	if ((status_reg & SYS_STATUS_ALL_RX_TO)) {
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO);
		return (TAG_RX_TIMEOUT);
	}
	if ((status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
		/* Clear RX error/timeout events in the DW IC status register. */
		dwt_write32bitreg(SYS_STATUS_ID,
				SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
		return (TAG_RX_CRC_VALID);
	}

	/* Clear good RX frame event in the DW IC status register. */
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

	return (TAG_NO_RXCG_DETECTED);
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
			return 0; // Return 0 to indicate failure
		}
	}

	/* No data received, return 0. */
	return 0;
}

int start_transmission_delayed_with_response_expected(TX_BUFFER_t tx) {
	/* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
	dwt_setrxaftertxdelay(tx.delay);
	/* FINAL_RX_TIMEOUT_UUS. */
	dwt_setrxtimeout(tx.rx_timeout);
	/* Set preamble timeout for expected frames. See NOTE 6 below. */
	dwt_setpreambledetecttimeout(tx.preamble_timeout);
	/* Write and send the response message. See NOTE 10 below.*/
	if (dwt_writetxdata(tx.buffer_size, tx.buffer, 0) == DWT_ERROR) /* Zero offset in TX buffer. */
		return (DWT_ERROR);
	dwt_writetxfctrl(tx.buffer_size + 2, 0, 1);
	/*DWT_START_TX_DELAYED DWT_START_TX_IMMEDIATE*/
	if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_ERROR)
		return (DWT_ERROR);
	/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
	return (DWT_SUCCESS);

}

void debug(TAG_t *tag) {
	/* Calculate the size needed for the formatted string */
	uint8_t dist_str[150] = { 0 };
	int size =
			sprintf(dist_str,
					"{ID: %lu} , {command: %d} , {times: %lu} , {poll_rx_timestamp: %lu} , {resp_tx_timestamp: %lu} ,\n\r",
					(unsigned long) tag->id, (int) tag->command,
					(unsigned long) tag->readings,
					(unsigned long) tag->poll_rx_timestamp,
					(unsigned long) tag->resp_tx_timestamp);
	/* Transmit the formatted string */
	HAL_UART_Transmit(&huart1, (uint8_t*) dist_str, (uint16_t) size,
	HAL_MAX_DELAY);
}

void sleep_config(uint16_t sleep_mode, uint16_t mode, uint8_t wake) {
	// Add predefined sleep settings before writing the mode
	sleep_mode |= mode;
	dwt_write16bitoffsetreg(AON_DIG_CFG_ID, 0, pdw3000local->sleep_mode);

	dwt_write8bitoffsetreg(ANA_CFG_ID, 0, wake); //bit 0 - SLEEP_EN, bit 1 - DEEP_SLEEP=0/SLEEP=1, bit 3 wake on CS
}
