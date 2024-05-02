/*
 * sniffer_tag.c
 *
 *  Created on: May 2, 2024
 *      Author: uqommdev
 */

#include "sniffer_tag.h"

void handle_sniffer_tag(Distance_t *distance) {
	int rets;
	TAG_t tag;
	uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E',
			0x21 };
	uint8_t rx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A',
			0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t tx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E',
			0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint32_t status_reg = 0;
	uint8_t dist_str[30];
	uint8_t frame_seq_nb;
	uint8_t rx_buffer[RX_BUF_LEN] = { 0 };
	/* Write frame data to DW IC and prepare transmission. See NOTE 9 below. */
	tx_poll_msg[ALL_MSG_SN_IDX] = (uint8_t) frame_seq_nb;
	dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(sizeof(tx_poll_msg) + FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging. */

	/* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
	 * set by dwt_setrxaftertxdelay() has elapsed. */
	rets = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
	if (rets == DWT_SUCCESS) {
		/* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 10 below. */
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID))
				& (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO
						| SYS_STATUS_ALL_RX_ERR))) {
		};

		if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
			uint32_t frame_len;

			/* Clear good RX frame event and TX frame sent in the DW IC status register. */
			dwt_write32bitreg(SYS_STATUS_ID,
					SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

			/* A frame has been received, read it into the local buffer. */
			frame_len =
			dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
			if (frame_len <= RX_BUF_LEN) {
				dwt_readrxdata(rx_buffer, (uint16_t) frame_len, 0);
			}
			/* Check that the frame is the expected response from the companion "DS TWR responder" example.
			 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
			rx_buffer[ALL_MSG_SN_IDX] = 0;
			if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {
				tag.distance = calculate_tag_distance(rx_buffer, distance);
				send_message_with_timestamps(tx_final_msg, sizeof(tx_final_msg),
						frame_seq_nb);
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
				uint16_t size = sprintf((char*) dist_str, "distance:%.2f[m]\n\r",
						tag.distance);
				HAL_UART_Transmit(&huart1, dist_str, (uint16_t) size,
				HAL_MAX_DELAY);
				memset(rx_buffer, 0, RX_BUF_LEN);

			}
		} else {
			/* Clear RX error/timeout events in the DW IC status register. */
			dwt_write32bitreg(SYS_STATUS_ID,
					SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK);
		}
	}
}
