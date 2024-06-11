/*
 * sniffer_tag.c
 *
 *  Created on: May 2, 2024
 *      Author: uqommdev
 */

#include "sniffer_tag.h"

static const char *TAG_MESSAGES[] = { "NO_RESPONSE", "NO_RXCG_DETECTED",
		"RX_FRAME_TIMEOUT", "RX_PREAMBLE_DETECTION_TIMEOUT", "RX_CRC_VALID",
		"RX_ERROR", "RX_DATA_ZERO", "RX_NO_COMMAND", "TX_ERROR",
		"HUMAN_DISTANCE_OK", "END_READINGS", "DISCOVERY",
		"SEND_TIMESTAMP_QUERY", "TAG_SET_SLEEP_MODE", "UNKNOWN" };

static void compute_distance(TAG_t *tag, const uint8_t *rx_buffer,
		int poll_rx_offset, int resp_tx_offset);
static void serialize_tag(TAG_t *tag, uint8_t *buffer);
static TAG_STATUS_t transmit_and_receive(uint8_t *tx_buffer,
		uint8_t tx_buffer_size, uint8_t *rx_buffer, uint32_t *rx_buffer_size);
static void parse_received_data(TAG_t *tag, const uint8_t *rx_buffer);
static TAG_STATUS_t setup_and_transmit(TAG_t *tag, uint8_t *tx_buffer,
		uint8_t tx_buffer_size, uint8_t *rx_buffer, uint32_t *rx_buffer_size);
static TAG_STATUS_t handle_received_command(TAG_t *tag,
		const uint8_t *rx_buffer);
static TAG_STATUS_t setup_and_transmit_for_timestamp_query(TAG_t *tag,
		uint8_t *tx_buffer, uint8_t *rx_buffer, uint32_t *rx_buffer_size);

SPI_HW_t* init_uwb_device(SPI_HandleTypeDef *hspi, GPIO_TypeDef *nssPort,
		uint16_t nssPin, GPIO_TypeDef *nrstPort, uint16_t nrstPin) {

	/* Default communication configuration. We use default non-STS DW mode. */
	dwt_config_t dwt_cfg = { 5, /* Channel number. */
	DWT_PLEN_128, /* Preamble length. Used in TX only. */
	DWT_PAC8, /* Preamble acquisition chunk size. Used in RX only. */
	9, /* TX preamble code. Used in TX only. */
	9, /* RX preamble code. Used in RX only. */
	1, /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
	DWT_BR_6M8, /* Data rate. */
	DWT_PHRMODE_STD, /* PHY header mode. */
	DWT_PHRRATE_STD, /* PHY header rate. */
	(129 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
	DWT_STS_MODE_OFF, /* STS disabled */
	DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
	DWT_PDOA_M0 /* PDOA mode off */
	};
	/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
	 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 8 below. */
	static dwt_txconfig_t dwt_tx_cfg = { 0x34, /* PG delay. */
	0xfdfdfdfd, /* TX power. */
	0x0 /*PG count*/
	};

	SPI_HW_t *spi_hw = malloc(sizeof(SPI_HW_t));
	if (spi_hw == NULL)
		Error_Handler();

	spi_hw->spi = hspi;
	spi_hw->nrstPin = nrstPin;
	spi_hw->nrstPort = nrstPort;
	spi_hw->nssPin = nssPin;
	spi_hw->nssPort = nssPort;

	HAL_GPIO_WritePin(spi_hw->nrstPort, spi_hw->nrstPin, GPIO_PIN_RESET);/* Target specific drive of RSTn line into DW IC low for a period. */
	HAL_Delay(1);
	HAL_GPIO_WritePin(spi_hw->nrstPort, spi_hw->nrstPin, GPIO_PIN_SET);
	hw = spi_hw;
	if (tag_init(&dwt_cfg, &dwt_tx_cfg, pdw3000local, DEV_UWB3000F27, RATE_6M8)
			== 1)
		Error_Handler();

	return (spi_hw);
}

void reset_TAG_values(TAG_t *tag) {
	tag->id = 0;
	tag->readings = 0;
	tag->command = 0;
	tag->distance = NULL;
	// Initialize Measurement_data_t variables to zero
	tag->temperature.calibrated = 0;
	tag->temperature.raw = 0;
	tag->temperature.real = 0.0;

	tag->battery_voltage.calibrated = 0;
	tag->battery_voltage.raw = 0;
	tag->battery_voltage.real = 0.0;

	tag->distance_a.value = 0;
	for (int i = 0; i < DISTANCE_READINGS / 2; i++) {
		tag->distance_a.readings[i] = 0.0;
	}

	tag->distance_a.error_times = 0;
	tag->distance_a.counter = 0;

	tag->distance_b.value = 0;
	for (int i = 0; i < DISTANCE_READINGS / 2; i++) {
		tag->distance_b.readings[i] = 0.0;
	}
	tag->distance_b.error_times = 0;
	tag->distance_b.counter = 0;
}

TAG_STATUS_t tag_discovery(TAG_t *tag) {
	uint8_t tx_buffer[TX_DISCOVERY_SIZE] = { 0 };
	uint8_t rx_buffer[FRAME_LEN_MAX_EX];
	uint32_t rx_buffer_size = 0;
	tag->command = TAG_ID_QUERY;

	tx_buffer[0] = tag->command;

	TAG_STATUS_t status_reg = setup_and_transmit(tag, tx_buffer,
	TX_DISCOVERY_SIZE, rx_buffer, &rx_buffer_size);
	if (status_reg != TAG_RX_CRC_VALID) {
		return status_reg;
	}

	if (tag->command == rx_buffer[0]) {
		parse_received_data(tag, rx_buffer);
		return (TAG_SEND_TIMESTAMP_QUERY);
	}
	return (TAG_RX_NO_COMMAND);
}

static TAG_STATUS_t setup_and_transmit(TAG_t *tag, uint8_t *tx_buffer,
		uint8_t tx_buffer_size, uint8_t *rx_buffer, uint32_t *rx_buffer_size) {
	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS_6M8);
	dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS_6M8);
	dwt_setpreambledetecttimeout(PRE_TIMEOUT_6M8);

	return transmit_and_receive(tx_buffer, tx_buffer_size, rx_buffer,
			rx_buffer_size);
}

static void parse_received_data(TAG_t *tag, const uint8_t *rx_buffer) {
	tag->id = *(const uint32_t*) (rx_buffer + 1);
	compute_distance(tag, rx_buffer, POLL_RX_OFFSET, RESP_TX_OFFSET);
	tag->battery_voltage.raw = *(const uint8_t*) (rx_buffer
			+ BATTERY_VOLTAGE_RAW_OFFSET);
	tag->battery_voltage.calibrated = *(const uint8_t*) (rx_buffer
			+ BATTERY_VOLTAGE_CALIBRATED_OFFSET);
	tag->temperature.raw =
			*(const uint8_t*) (rx_buffer + TEMPERATURE_RAW_OFFSET);
	tag->temperature.calibrated = *(const uint8_t*) (rx_buffer
			+ TEMPERATURE_CALIBRATED_OFFSET);
}

TAG_STATUS_t tag_send_timestamp_query(TAG_t *tag) {
	assert_param(tag != NULL);

	uint8_t tx_buffer[TX_BUFFER_SIZE] = { 0 };
	uint8_t rx_buffer[FRAME_LEN_MAX_EX];
	uint32_t rx_buffer_size = 0;

	tx_buffer[0] = tag->command;
	*(uint32_t*) (tx_buffer + sizeof(tag->command)) = tag->id;

	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS_6M8);
	dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS_6M8);
	dwt_setpreambledetecttimeout(PRE_TIMEOUT_6M8);

	TAG_STATUS_t status_reg = setup_and_transmit_for_timestamp_query(tag,
			tx_buffer, rx_buffer, &rx_buffer_size);
	if (status_reg != TAG_RX_CRC_VALID) {
		tag->distance->error_times++;
		return status_reg;
	}

	tag->command = rx_buffer[0];
	return handle_received_command(tag, rx_buffer);
}

static TAG_STATUS_t handle_received_command(TAG_t *tag,
		const uint8_t *rx_buffer) {
	const int poll_rx_offset = 1;
	const int resp_tx_offset = 1 + 4;
	switch (tag->command) {
	case TAG_TIMESTAMP_QUERY:
		compute_distance(tag, (uint8_t*) rx_buffer, poll_rx_offset,
				resp_tx_offset);
		return TAG_SEND_TIMESTAMP_QUERY;
	case TAG_SET_SLEEP_MODE:
		compute_distance(tag, (uint8_t*) rx_buffer, poll_rx_offset,
				resp_tx_offset);
		return TAG_END_READINGS;
	default:
		return TAG_RX_NO_COMMAND;
	}
}

static TAG_STATUS_t setup_and_transmit_for_timestamp_query(TAG_t *tag,
		uint8_t *tx_buffer, uint8_t *rx_buffer, uint32_t *rx_buffer_size) {
	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS_6M8);
	dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS_6M8);
	dwt_setpreambledetecttimeout(PRE_TIMEOUT_6M8);

	return transmit_and_receive(tx_buffer, TX_BUFFER_SIZE, rx_buffer,
			rx_buffer_size);
}

static TAG_STATUS_t transmit_and_receive(uint8_t *tx_buffer,
		uint8_t tx_buffer_size, uint8_t *rx_buffer, uint32_t *rx_buffer_size) {
	dwt_writetxdata(tx_buffer_size, tx_buffer, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(tx_buffer_size + FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging. */

	if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) == DWT_ERROR) {
		/* Clear RX error/timeout events in the DW IC status register. */
		dwt_write32bitreg(SYS_STATUS_ID,
				SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK);
		return TAG_TX_ERROR;
	}

	TAG_STATUS_t status_reg = wait_rx_data();
	if (status_reg != TAG_RX_CRC_VALID) {
		return status_reg;
	}

	*rx_buffer_size = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
	if (*rx_buffer_size == 0) {
		return TAG_RX_DATA_ZERO;
	}

	dwt_readrxdata(rx_buffer, (uint16_t) *rx_buffer_size, 0);
	return TAG_RX_CRC_VALID;
}

static void compute_distance(TAG_t *tag, const uint8_t *rx_buffer,
		int poll_rx_offset, int resp_tx_offset) {
	// Get timestamps embedded in response message
	uint32_t poll_rx_timestamp = *(uint32_t*) (rx_buffer + poll_rx_offset);
	uint32_t resp_tx_timestamp = *(uint32_t*) (rx_buffer + resp_tx_offset);

	// Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates
	uint64_t rtd_init = get_rx_timestamp_u64() - get_tx_timestamp_u64();
	uint64_t rtd_resp = resp_tx_timestamp - poll_rx_timestamp;

	// Read carrier integrator value and calculate clock offset ratio
	float clockOffsetRatio = dwt_readclockoffset() / (float) (1 << 26);

	// Hold copies of computed time of flight and distance here for reference
	double tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0)
			* DWT_TIME_UNITS;
	tag->distance->readings[tag->distance->counter++] = tof * SPEED_OF_LIGHT;
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

		/* Clear T&XFRS event. */
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

	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS_6M8);
	dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS_6M8);
	dwt_setpreambledetecttimeout(PRE_TIMEOUT_6M8);
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

uint32_t status_reg;
TAG_STATUS_t wait_rx_data() {
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

void debug(TAG_t tag, TAG_STATUS_t status) {
	/* Format the string directly into the dynamically allocated buffer */
	char dist_str[300] = { 0 };
	int size = 0;

	/* Check if status value is within the valid range */
	if (status >= TAG_UNKNOWN) {
		status = TAG_UNKNOWN;
	}

	size =
			sprintf(dist_str,
					"{message: %s},{ID: 0x%08X},{times: %lu},{error_a:%u},{error_b:%u},{distance_a: %0.2f},{distance_b: %0.2f},{battery_voltage: %u},{temperature: %u}",
					TAG_MESSAGES[status], (int) tag.id,
					(unsigned long) tag.readings,
					tag.distance_a.error_times,
					tag.distance_b.error_times,
					(float) tag.distance_a.readings[tag.distance_a.counter - 1],
					(float) tag.distance_b.readings[tag.distance_b.counter - 1],
					tag.battery_voltage.real, tag.temperature.real);

	HAL_UART_Transmit(&huart1, (uint8_t*) dist_str, size, HAL_MAX_DELAY);
	//if (status == TAG_DISCOVERY || status == TAG_SEND_TIMESTAMP_QUERY)
	if (status == TAG_DISCOVERY)
		HAL_UART_Transmit(&huart1, (uint8_t*) "\r", 1, HAL_MAX_DELAY);
	else
		HAL_UART_Transmit(&huart1, (uint8_t*) "\n\r", 2, HAL_MAX_DELAY);

}
void debug_status(TAG_STATUS_t status) {
	/* Format the string directly into the dynamically allocated buffer */
	char dist_str[300] = { 0 };
	int size = 0;

	/* Check if status value is within the valid range */
	if (status >= TAG_UNKNOWN) {
		status = TAG_UNKNOWN;
	}

	size = sprintf(dist_str, "\r\n{message: %s}", TAG_MESSAGES[status]);

	HAL_UART_Transmit(&huart1, (uint8_t*) dist_str, size,
	HAL_MAX_DELAY);
	//if (status == TAG_DISCOVERY || status == TAG_SEND_TIMESTAMP_QUERY)
	if (status == TAG_DISCOVERY)
		HAL_UART_Transmit(&huart1, (uint8_t*) "\r", 1, HAL_MAX_DELAY);
	else
		HAL_UART_Transmit(&huart1, (uint8_t*) "\n\r", 2, HAL_MAX_DELAY);

}

void debug_tag(TAG_t tag) {
	/* Format the string directly into the dynamically allocated buffer */
	char dist_str[300] = { 0 };
	int size = 0;
	size =
			sprintf(dist_str,
					"{ID: 0x%08X},{times: %lu},{distance_a: %u},{distance_b: %u},{battery_voltage: %u},{temperature: %u}",
					(int) tag.id, (unsigned long) tag.readings,
					tag.distance_a.value, tag.distance_b.value,
					tag.battery_voltage.real, tag.temperature.real);

	HAL_UART_Transmit(&huart1, (uint8_t*) dist_str, size,
	HAL_MAX_DELAY);

}

void print_all_tags(TAG_List *list, TAG_STATUS_t status) {
	HAL_UART_Transmit(&huart1, (uint8_t*) "\n####\n\r", (uint16_t) 7,
	HAL_MAX_DELAY);

	if (list->head == NULL) {
		HAL_UART_Transmit(&huart1, (uint8_t*) "No tag detected\n\r####\n\r",
				(uint16_t) 23,
				HAL_MAX_DELAY);
		return;
	}

	char buffer[50];
	snprintf(buffer, sizeof(buffer), "Total tags: %d\n\r", list->count);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, (uint16_t) strlen(buffer),
	HAL_MAX_DELAY);

	TAG_Node *current = list->head;

	while (current != NULL) {
		debug_tag(current->tag);
		HAL_UART_Transmit(&huart1, (uint8_t*) "\n\r", (uint16_t) 2,
		HAL_MAX_DELAY);
		current = current->next;
	}

	HAL_UART_Transmit(&huart1, (uint8_t*) "####\n\r", (uint16_t) 6,
	HAL_MAX_DELAY);
}
void print_serialized_tags(TAG_List *list) {
	if (list->head == NULL) {
		HAL_UART_Transmit(&huart1, (uint8_t*) "No tag detected\n\r",
				(uint16_t) 18, HAL_MAX_DELAY);
		return;
	}

	// Allocate buffer for serialized tags
	uint8_t buffer[SERIALIZED_TAG_SIZE * list->count];
	serialize_tag_list(list, buffer);

	// Print each serialized tag in hexadecimal format separated by "\n"
	uint8_t *current = buffer;
	char hex_output[SERIALIZED_TAG_SIZE * 2 + 1]; // Each byte is represented by 2 hex digits + null terminator

	for (int i = 0; i < list->count; ++i) {
		for (int j = 0; j < SERIALIZED_TAG_SIZE; ++j) {
			snprintf(&hex_output[j * 2], 3, "%02X", current[j]);
		}

		HAL_UART_Transmit(&huart1, (uint8_t*) hex_output,
				(uint16_t) (SERIALIZED_TAG_SIZE * 2), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t*) "\n\r\r", 2,
		HAL_MAX_DELAY);
		current += SERIALIZED_TAG_SIZE;
	}
	HAL_UART_Transmit(&huart1, (uint8_t*) "####\n\r", (uint16_t) 6,
	HAL_MAX_DELAY);
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
			* 105) + 200;
}

// Function to insert a new TAG_t node into the linked list

void insert_tag(TAG_List *list, TAG_t new_tag) {
	TAG_Node *current = list->head;
	TAG_Node *prev = NULL;
	uint8_t found = 0;

	while (current != NULL) {
		if (current->tag.id == new_tag.id) {
			// ID found, replace the TAG_t structure
			current->tag = new_tag;
			found = 1;
			break;
		}
		prev = current;
		current = current->next;
	}

	if (!found) {
		// ID not found, create a new node at the end
		TAG_Node *new_node = (TAG_Node*) malloc(sizeof(TAG_Node));
		new_node->tag = new_tag;
		new_node->next = NULL;

		if (prev == NULL) {
			// Insert at the beginning if the list is empty
			list->head = new_node;
		} else {
			prev->next = new_node;
		}
		list->count++;
	}
}

void delete_tag(TAG_List *list, uint32_t id) {
	TAG_Node *current = list->head;
	TAG_Node *prev = NULL;

	while (current != NULL && current->tag.id != id) {
		prev = current;
		current = current->next;
	}

	if (current == NULL) {
		// ID not found
		return;
	}

	if (prev == NULL) {
		// Node to be deleted is the head
		list->head = current->next;
	} else {
		prev->next = current->next;
	}

	free(current);
	list->count--;
}

// Function to free the entire linked list
void free_tag_list(TAG_List *list) {
	TAG_Node *current = list->head;
	TAG_Node *next;

	while (current != NULL) {
		next = current->next;
		free(current);
		current = next;
	}

	list->head = NULL;
	list->count = 0;
}

static void serialize_tag(TAG_t *tag, uint8_t *buffer) {
	int offset = 0;

	memcpy(buffer + offset, &tag->id, sizeof(tag->id));
	offset += sizeof(tag->id);

	memcpy(buffer + offset, &tag->distance_a.value,
			sizeof(tag->distance_a.value));
	offset += sizeof(tag->distance_a.value);

	memcpy(buffer + offset, &tag->distance_b.value,
			sizeof(tag->distance_b.value));
	offset += sizeof(tag->distance_b.value);

	memcpy(buffer + offset, &tag->temperature.real,
			sizeof(tag->temperature.real));
	offset += sizeof(tag->temperature.real);

	memcpy(buffer + offset, &tag->battery_voltage.real,
			sizeof(tag->battery_voltage.real));
	offset += sizeof(tag->battery_voltage.real);
}

void serialize_tag_list(TAG_List *list, uint8_t *buffer) {
	int offset = 0;
	TAG_Node *current = list->head;
	while (current != NULL) {
		serialize_tag(&current->tag, buffer + offset);
		offset += sizeof(current->tag.id)
				+ sizeof(current->tag.distance_a.value)
				+ sizeof(current->tag.distance_b.value)
				+ sizeof(current->tag.temperature.real)
				+ sizeof(current->tag.battery_voltage.real);
		current = current->next;
	}
}

void print_tx_hex(uint8_t *tx, uint16_t length) {
	char hex_output[length * 2 + 1]; // Each byte is represented by 2 hex digits + null terminator

	for (uint16_t i = 0; i < length; ++i) {
		snprintf(&hex_output[i * 2], 3, "%02X", tx[i]);
	}

	HAL_UART_Transmit(&huart1, (uint8_t*) hex_output, (uint16_t) (length * 2),
	HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, (uint8_t*) "\n\r####\n\r", (uint16_t) 8,
	HAL_MAX_DELAY);
}

