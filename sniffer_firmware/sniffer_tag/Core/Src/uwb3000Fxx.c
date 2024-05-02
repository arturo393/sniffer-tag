#include <stdio.h>
#include <math.h>
#include <uwb3000Fxx.h>
#include "deca_regs.h"

int dwt_local_data_init(dwt_local_data_t *dwt_local_data) {
	//uint16_t otp_addr;
	//uint32_t devid; // @suppress("Line comments")
	uint32_t ldo_tune_lo;
	uint32_t ldo_tune_hi;

	dwt_local_data->dblbuffon = DBL_BUFF_OFF; // Double buffer mode off by default / clear the flag
	dwt_local_data->sleep_mode = DWT_RUNSAR; // Configure RUN_SAR on wake by default as it is needed when running PGF_CAL
	dwt_local_data->spicrc = 0;
	dwt_local_data->stsconfig = 0; //STS off
	dwt_local_data->vBatP = 0;
	dwt_local_data->tempP = 0;

	dwt_local_data->cbTxDone = NULL;
	dwt_local_data->cbRxOk = NULL;
	dwt_local_data->cbRxTo = NULL;
	dwt_local_data->cbRxErr = NULL;
	dwt_local_data->cbSPIRdy = NULL;
	dwt_local_data->cbSPIErr = NULL;

	// Read and validate device ID return -1 if not recognised
	if (dwt_check_dev_id() != DWT_SUCCESS) {
		return DWT_ERROR;
	}

	ldo_tune_lo = dwt_otp_read(LDOTUNELO_ADDRESS);
	ldo_tune_hi = dwt_otp_read(LDOTUNEHI_ADDRESS);

	dwt_local_data->bias_tune = (dwt_otp_read(BIAS_TUNE_ADDRESS) >> 16)
			& BIAS_CTRL_BIAS_MASK;

	if ((ldo_tune_lo != 0) && (ldo_tune_hi != 0)
			&& (dwt_local_data->bias_tune != 0)) {
		dwt_or16bitoffsetreg(OTP_CFG_ID, 0, LDO_BIAS_KICK);
		dwt_and_or16bitoffsetreg(BIAS_CTRL_ID, 0,
				(uint16_t)~BIAS_CTRL_BIAS_MASK, dwt_local_data->bias_tune);
	}

	// Read DGC_CFG from OTP
	if (dwt_otp_read(DGC_TUNE_ADDRESS) == DWT_DGC_CFG0) {
		dwt_local_data->dgc_otp_set = DWT_DGC_LOAD_FROM_OTP;
	} else {
		dwt_local_data->dgc_otp_set = DWT_DGC_LOAD_FROM_SW;
	}

	// Load Part and Lot ID from OTP
	dwt_local_data->partID = dwt_otp_read(PARTID_ADDRESS);
	dwt_local_data->lotID = dwt_otp_read(LOTID_ADDRESS);
	dwt_local_data->vBatP = (uint8_t) dwt_otp_read(VBAT_ADDRESS);
	dwt_local_data->tempP = (uint8_t) dwt_otp_read(VTEMP_ADDRESS);
	if (dwt_local_data->tempP == 0) { //if the reference temperature has not been programmed in OTP (early eng samples) set to default value

		dwt_local_data->tempP = 0x85; //@temp of 20 deg
	}

	if (dwt_local_data->vBatP == 0) { //if the reference voltage has not been programmed in OTP (early eng samples) set to default value

		dwt_local_data->vBatP = 0x74;  //@Vref of 3.0V
	}
	dwt_local_data->otprev = (uint8_t) dwt_otp_read(OTPREV_ADDRESS);
	dwt_local_data->init_xtrim = dwt_otp_read(XTRIM_ADDRESS) & 0x7f;
	if (dwt_local_data->init_xtrim == 0) {
		dwt_local_data->init_xtrim = 0x2E; //set default value
	}
	dwt_write8bitoffsetreg(XTAL_ID, 0, dwt_local_data->init_xtrim);

	return DWT_SUCCESS;

} // end dwt_initialise()

uint8_t tag_init(dwt_config_t *dwt_config, dwt_txconfig_t *dwt_txconfig,
		dwt_local_data_t *dwt_local_data, uint8_t device, uint8_t rate) {
	uint32_t check_idle_rc_ticks;
	uint16_t check_idle_rc_timeout = 300;

	HAL_Delay(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC
	check_idle_rc_ticks = HAL_GetTick();
	while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
		if (HAL_GetTick() - check_idle_rc_ticks > check_idle_rc_timeout)
			return (1);

	if (dwt_local_data_init(dwt_local_data) == DWT_ERROR) {
		HAL_Delay(1000);
		return (1);
	}

	dwt_txconfig->power = GAIN_30DB;

	if (dwt_config2(dwt_config, dwt_local_data)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
	{
		HAL_Delay(1000);
		return (1);
	}
	dwt_configuretxrf(dwt_txconfig);
	/* Set the antenna delay. Modify the parameters to adjust the distance error */
	if (device == DEV_UWB3000F27) {
		dwt_settxantennadelay(TX_ANT_DLY_HP);
		dwt_setrxantennadelay(RX_ANT_DLY_HP);
	} else {
		dwt_settxantennadelay(TX_ANT_DLY_LP);
		dwt_setrxantennadelay(RX_ANT_DLY_LP);
	}

	if (rate == RATE_6M8) {
		dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS_6M8);
		dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS_6M8);
		dwt_setpreambledetecttimeout(PRE_TIMEOUT_6M8);
	} else {
		dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS_850K);
		dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS_850K);
		dwt_setpreambledetecttimeout(PRE_TIMEOUT_850K);
	}

	/* If the UWB3000F27 module is used, DWT_LNA_ENABLE and DWT_PA_ENABLE must be enabled; otherwise, the power amplifier circuit cannot be started */
	dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE | DWT_TXRX_EN);
	dwt_setfinegraintxseq(0);

	dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

	return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function provides the main API for the configuration of the
 * DW3000 and this low-level driver.  The input is a pointer to the data structure
 * of type dwt_config_t that holds all the configurable items.
 * The dwt_config_t structure shows which ones are supported
 *
 * input parameters
 * @param config    -   pointer to the configuration structure, which contains the device configuration data.
 *
 * return DWT_SUCCESS or DWT_ERROR
 * Note: If the RX calibration routine fails the device receiver performance will be severely affected,
 * the application should reset device and try again
 *
 */
int dwt_config2(dwt_config_t *config, dwt_local_data_t *dwt_local_data) {

	uint8_t channel = config->chan, cnt, flag;
	uint32_t temp;
	uint8_t scp = ((config->rxCode > 24) || (config->txCode > 24)) ? 1 : 0;
	uint8_t mode = (config->phrMode == DWT_PHRMODE_EXT) ?
	SYS_CFG_PHR_MODE_BIT_MASK :
															0;
	uint16_t sts_len;
	int error = DWT_SUCCESS;
	uint16_t sts_length_factors[STS_LEN_SUPPORTED] = { 1024, 1448, 2048, 2896,
			4096, 5793, 8192 };

	int preamble_len;

	switch (config->txPreambLength) {
	case DWT_PLEN_32:
		preamble_len = 32;
		break;
	case DWT_PLEN_64:
		preamble_len = 64;
		break;
	case DWT_PLEN_72:
		preamble_len = 72;
		break;
	case DWT_PLEN_128:
		preamble_len = 128;
		break;
	default:
		preamble_len = 256;
		break;
	}

	dwt_local_data->sleep_mode &= (~(DWT_ALT_OPS | DWT_SEL_OPS3)); //clear the sleep mode ALT_OPS bit
	dwt_local_data->longFrames = config->phrMode;
	sts_len = (uint16_t) GET_STS_REG_SET_VALUE((uint16_t )(config->stsLength));
	dwt_local_data->ststhreshold = (int16_t) ((((uint32_t) sts_len) * 8)
			* STSQUAL_THRESH_64);
	dwt_local_data->stsconfig = config->stsMode;

	/////////////////////////////////////////////////////////////////////////
	//SYS_CFG
	//clear the PHR Mode, PHR Rate, STS Protocol, SDC, PDOA Mode,
	//then set the relevant bits according to configuration of the PHR Mode, PHR Rate, STS Protocol, SDC, PDOA Mode,
	dwt_modify32bitoffsetreg(SYS_CFG_ID, 0,
			~(SYS_CFG_PHR_MODE_BIT_MASK | SYS_CFG_PHR_6M8_BIT_MASK
					| SYS_CFG_CP_SPC_BIT_MASK | SYS_CFG_PDOA_MODE_BIT_MASK
					| SYS_CFG_CP_SDC_BIT_MASK),
			((uint32_t) config->pdoaMode) << SYS_CFG_PDOA_MODE_BIT_OFFSET
					| ((uint16_t) config->stsMode & DWT_STS_CONFIG_MASK)
							<< SYS_CFG_CP_SPC_BIT_OFFSET
					| (SYS_CFG_PHR_6M8_BIT_MASK
							& ((uint32_t) config->phrRate
									<< SYS_CFG_PHR_6M8_BIT_OFFSET)) | mode);

	if (scp) {
		//configure OPS tables for SCP mode
		dwt_local_data->sleep_mode |= DWT_ALT_OPS | DWT_SEL_OPS1; //configure correct OPS table is kicked on wakeup
		dwt_modify32bitoffsetreg(OTP_CFG_ID, 0, ~(OTP_CFG_OPS_ID_BIT_MASK),
		DWT_OPSET_SCP | OTP_CFG_OPS_KICK_BIT_MASK);

		dwt_write32bitoffsetreg(IP_CONFIG_LO_ID, 0, IP_CONFIG_LO_SCP); //Set this if Ipatov analysis is used in SCP mode
		dwt_write32bitoffsetreg(IP_CONFIG_HI_ID, 0, IP_CONFIG_HI_SCP);

		dwt_write32bitoffsetreg(STS_CONFIG_LO_ID, 0, STS_CONFIG_LO_SCP);
		dwt_write8bitoffsetreg(STS_CONFIG_HI_ID, 0, STS_CONFIG_HI_SCP);
	} else //
	{
		uint16_t sts_mnth;
		if (config->stsMode != DWT_STS_MODE_OFF) {

			//configure CIA STS lower bound
			if ((config->pdoaMode == DWT_PDOA_M1)
					|| (config->pdoaMode == DWT_PDOA_M0)) {
				//In PDOA mode 1, number of accumulated symbols is the whole length of the STS
				sts_mnth = get_sts_mnth(
						sts_length_factors[(uint8_t) (config->stsLength)],
						CIA_MANUALLOWERBOUND_TH_64, 3);
			} else {
				//In PDOA mode 3 number of accumulated symbols is half of the length of STS symbols
				sts_mnth = get_sts_mnth(
						sts_length_factors[(uint8_t) (config->stsLength)],
						CIA_MANUALLOWERBOUND_TH_64, 4);
			}

			preamble_len += (sts_len) * 8;

			dwt_modify16bitoffsetreg(STS_CONFIG_LO_ID, 2,
					(uint16_t) ~(STS_CONFIG_LO_STS_MAN_TH_BIT_MASK >> 16),
					sts_mnth & 0x7F);

		}

		//configure OPS tables for non-SCP mode
		if (preamble_len >= 256) {
			dwt_local_data->sleep_mode |= DWT_ALT_OPS | DWT_SEL_OPS0;
			dwt_modify32bitoffsetreg(OTP_CFG_ID, 0, ~(OTP_CFG_OPS_ID_BIT_MASK),
			DWT_OPSET_LONG | OTP_CFG_OPS_KICK_BIT_MASK);
		} else {
			dwt_modify32bitoffsetreg(OTP_CFG_ID, 0, ~(OTP_CFG_OPS_ID_BIT_MASK),
			DWT_OPSET_SHORT | OTP_CFG_OPS_KICK_BIT_MASK);
		}

	}

	dwt_modify8bitoffsetreg(DTUNE0_ID, 0,
			(uint8_t) ~DTUNE0_PRE_PAC_SYM_BIT_MASK, config->rxPAC);

	dwt_write8bitoffsetreg(STS_CFG0_ID, 0, (uint8_t) (sts_len - 1)); /*Starts from 0 that is why -1*/

	if (config->txPreambLength == DWT_PLEN_72) {
		dwt_setplenfine(8); //value 8 sets fine preamble length to 72 symbols - this is needed to set 72 length.
	} else {
		dwt_setplenfine(0); //clear the setting in the FINE_PLEN register.
	}

	if ((config->stsMode & DWT_STS_MODE_ND) == DWT_STS_MODE_ND) {
		//configure lower preamble detection threshold for no data STS mode
		dwt_write32bitoffsetreg(DTUNE3_ID, 0, PD_THRESH_NO_DATA);
	} else {
		//configure default preamble detection threshold for other modes
		dwt_write32bitoffsetreg(DTUNE3_ID, 0, PD_THRESH_DEFAULT);
	}

	/////////////////////////////////////////////////////////////////////////
	//CHAN_CTRL
	temp = dwt_read32bitoffsetreg(CHAN_CTRL_ID, 0);
	temp &= (~(CHAN_CTRL_RX_PCODE_BIT_MASK | CHAN_CTRL_TX_PCODE_BIT_MASK
			| CHAN_CTRL_SFD_TYPE_BIT_MASK | CHAN_CTRL_RF_CHAN_BIT_MASK));

	if (channel == 9)
		temp |= CHAN_CTRL_RF_CHAN_BIT_MASK;

	temp |= (CHAN_CTRL_RX_PCODE_BIT_MASK
			& ((uint32_t) config->rxCode << CHAN_CTRL_RX_PCODE_BIT_OFFSET));
	temp |= (CHAN_CTRL_TX_PCODE_BIT_MASK
			& ((uint32_t) config->txCode << CHAN_CTRL_TX_PCODE_BIT_OFFSET));
	temp |= (CHAN_CTRL_SFD_TYPE_BIT_MASK
			& ((uint32_t) config->sfdType << CHAN_CTRL_SFD_TYPE_BIT_OFFSET));

	dwt_write32bitoffsetreg(CHAN_CTRL_ID, 0, temp);

	/////////////////////////////////////////////////////////////////////////
	//TX_FCTRL
	// Set up TX Preamble Size, PRF and Data Rate
	dwt_modify32bitoffsetreg(TX_FCTRL_ID, 0,
			~(TX_FCTRL_TXBR_BIT_MASK | TX_FCTRL_TXPSR_BIT_MASK),
			((uint32_t) config->dataRate << TX_FCTRL_TXBR_BIT_OFFSET)
					| ((uint32_t) config->txPreambLength)
							<< TX_FCTRL_TXPSR_BIT_OFFSET);

	//DTUNE (SFD timeout)
	// Don't allow 0 - SFD timeout will always be enabled
	if (config->sfdTO == 0) {
		config->sfdTO = DWT_SFDTOC_DEF;
	}
	dwt_write16bitoffsetreg(DTUNE0_ID, 2, config->sfdTO);

	///////////////////////
	// RF
	if (channel == 9) {
		// Setup TX analog for ch9
		dwt_write32bitoffsetreg(TX_CTRL_HI_ID, 0, RF_TXCTRL_CH9);
		dwt_write16bitoffsetreg(PLL_CFG_ID, 0, RF_PLL_CFG_CH9);
		// Setup RX analog for ch9
		dwt_write32bitoffsetreg(RX_CTRL_HI_ID, 0, RF_RXCTRL_CH9);
	} else {
		// Setup TX analog for ch5
		dwt_write32bitoffsetreg(TX_CTRL_HI_ID, 0, RF_TXCTRL_CH5);
		dwt_write16bitoffsetreg(PLL_CFG_ID, 0, RF_PLL_CFG_CH5);
	}

	dwt_write8bitoffsetreg(LDO_RLOAD_ID, 1, LDO_RLOAD_VAL_B1);
	dwt_write8bitoffsetreg(TX_CTRL_LO_ID, 2, RF_TXCTRL_LO_B2);
	dwt_write8bitoffsetreg(PLL_CAL_ID, 0, RF_PLL_CFG_LD); // Extend the lock delay

	//Verify PLL lock bit is cleared
	dwt_write8bitoffsetreg(SYS_STATUS_ID, 0, SYS_STATUS_CP_LOCK_BIT_MASK);

	///////////////////////
	// auto cal the PLL and change to IDLE_PLL state
	dwt_setdwstate(DWT_DW_IDLE);

	for (flag = 1, cnt = 0; cnt < MAX_RETRIES_FOR_PLL; cnt++) {
		//deca_usleep(DELAY_20uUSec);
		HAL_Delay(1);
		if ((dwt_read8bitoffsetreg(SYS_STATUS_ID, 0)
				& SYS_STATUS_CP_LOCK_BIT_MASK)) {    //PLL is locked
			flag = 0;
			break;
		}
	}

	if (flag) {
		return DWT_ERROR;
	}

	if ((config->rxCode >= 9) && (config->rxCode <= 24)) //only enable DGC for PRF 64
			{
		//load RX LUTs
		/* If the OTP has DGC info programmed into it, do a manual kick from OTP. */
		if (dwt_local_data->dgc_otp_set == DWT_DGC_LOAD_FROM_OTP) {
			_dwt_kick_dgc_on_wakeup((int8_t) channel);
		}
		/* Else we manually program hard-coded values into the DGC registers. */
		else {
			dwt_configmrxlut(channel);
		}
		dwt_modify16bitoffsetreg(DGC_CFG_ID, 0x0,
				(uint16_t) ~DGC_CFG_THR_64_BIT_MASK,
				DWT_DGC_CFG << DGC_CFG_THR_64_BIT_OFFSET);
	} else {
		dwt_and8bitoffsetreg(DGC_CFG_ID, 0x0,
				(uint8_t)~DGC_CFG_RX_TUNE_EN_BIT_MASK);
	}

	///////////////////////
	// PGF
	error = dwt_pgf_cal(1); //if the RX calibration routine fails the device receiver performance will be severely affected, the application should reset and try again

	return error;
} // end dwt_configure()

double calculate_tag_distance(uint8_t *rx_buffer, Distance_t *distance) {
	uint32_t poll_rx_ts, resp_tx_ts;
	uint64_t poll_tx_timestamp;
	uint64_t resp_rx_timestamp;
	uint64_t final_tx_timestamp;
	int32_t rtd_init, rtd_resp;
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
	resp_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_rx_ts);
	resp_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_tx_ts);

	/* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
	rtd_init = resp_rx_timestamp - poll_tx_timestamp;
	rtd_resp = resp_tx_ts - poll_rx_ts;

	/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
	double tof;
	tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0)
			* DWT_TIME_UNITS;
	distance->value = tof * SPEED_OF_LIGHT;
	//The data were smoothed and filtered
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

uint8_t send_message_with_timestamps(uint8_t *tx_final_msg, uint8_t size,
		uint32_t frame_seq_nb) {

	uint64_t poll_tx_timestamp;
	uint64_t resp_rx_timestamp;
	uint64_t final_tx_timestamp;
	uint32_t final_tx_time;
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
	/* Write and send final message. See NOTE 9 below. */
	tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
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
		frame_seq_nb++;
	}
	return frame_seq_nb;
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

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief function to read the OTP memory.
 *
 * input parameters
 * @param address - address to read at
 *
 * output parameters
 *tx
 * returns the 32bit of read data
 */
uint32_t dwt_otp_read(uint16_t address) {
	uint32_t ret_data = 0;

	// Set manual access mode
	dwt_write16bitoffsetreg(OTP_CFG_ID, 0, 0x0001);
	// set the address
	dwt_write16bitoffsetreg(OTP_ADDR_ID, 0, address);
	// Assert the read strobe
	dwt_write16bitoffsetreg(OTP_CFG_ID, 0, 0x0002);
	// attempt a read from OTP address
	ret_data = dwt_read32bitoffsetreg(OTP_RDATA_ID, 0);

	// Return the 32bit of read data
	return ret_data;
}

const uint16_t sts_length_factors[STS_LEN_SUPPORTED] = { 1024, 1448, 2048, 2896,
		4096, 5793, 8192 };

// -------------------------------------------------------------------------------------------------------------------
// Internal functions prototypes for controlling and configuring the device
//
static void dwt_force_clocks(int clocks);
static uint32_t _dwt_otpread(uint16_t address);      // Read non-volatile memory
static void _dwt_otpprogword32(uint32_t data, uint16_t address); // Program the non-volatile memory

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to read/write to the DW3000 device registers
 *
 * input parameters:
 * @param recordNumber  - ID of register file or buffer being accessed
 * @param index         - byte index into register file or buffer being accessed
 * @param length        - number of bytes being written
 * @param buffer        - pointer to buffer containing the 'length' bytes to be written
 * @param rw            - DW3000_SPI_WR_BIT/DW3000_SPI_RD_BIT
 *
 * no return value
 */
void dwt_xfer3000(const uint32_t regFileID, //0x0, 0x04-0x7F ; 0x10000, 0x10004, 0x10008-0x1007F; 0x20000 etc
		const uint16_t indx,     //sub-index, calculated from regFileID 0..0x7F,
		const uint16_t length, uint8_t *buffer, const spi_modes_e mode) {
	/*
	 char str[16] = {0};
	 sprintf(str, "%x, %x\r\n", regFileID, indx);
	 uart_transmit(str, strlen(str));
	 Sleep(10);
	 */
	uint8_t header[2];           // Buffer to compose header in
	uint16_t cnt = 0;             // Counter for length of a header

	uint16_t reg_file = 0x1F & ((regFileID + indx) >> 16);
	uint16_t reg_offset = 0x7F & (regFileID + indx);

	assert(reg_file <= 0x1F);
	assert(reg_offset <= 0x7F);
	assert(length < 0x3100);
	assert(
			mode == DW3000_SPI_WR_BIT || mode == DW3000_SPI_RD_BIT
					|| mode == DW3000_SPI_AND_OR_8
					|| mode == DW3000_SPI_AND_OR_16
					|| mode == DW3000_SPI_AND_OR_32);

	// Write message header selecting WRITE operation and addresses as appropriate
	uint16_t addr;
	addr = (uint16_t) ((reg_file << 9) | (reg_offset << 2));

	header[0] = (uint8_t) ((mode | addr) >> 8); //  & 0xFF; //bit7 + addr[4:0] + sub_addr[6:6]
	header[1] = (uint8_t) (addr | (mode & 0x03)); // & 0xFF; //EAM: subaddr[5:0]+ R/W/AND_OR

	if (/*reg_offset == 0 && */length == 0) { /* Fast Access Commands (FAC)
	 * only write operation is possible for this mode
	 * bit_7=one is W operation, bit_6=zero: FastAccess command, bit_[5..1] addr, bits_0=one: MODE of FastAccess
	 */
		assert(mode == DW3000_SPI_WR_BIT);

		header[0] = (uint8_t) ((DW3000_SPI_WR_BIT >> 8) | (regFileID << 1)
				| DW3000_SPI_FAC);
		cnt = 1;
	} else if (reg_offset == 0 /*&& length > 0*/
			&& (mode == DW3000_SPI_WR_BIT || mode == DW3000_SPI_RD_BIT)) { /* Fast Access Commands with Read/Write support (FACRW)
			 * bit_7 is R/W operation, bit_6=zero: FastAccess command, bit_[5..1] addr, bits_0=zero: MODE of FastAccess
			 */
		header[0] |= DW3000_SPI_FARW;
		cnt = 1;
	} else { /* Extended Address Mode with Read/Write support (EAMRW)
	 * b[0] = bit_7 is R/W operation, bit_6 one = ExtendedAddressMode;
	 * b[1] = addr<<2 | (mode&0x3)
	 */
		header[0] |= DW3000_SPI_EAMRW;
		cnt = 2;
	}

	switch (mode) {
	case DW3000_SPI_AND_OR_8:
	case DW3000_SPI_AND_OR_16:
	case DW3000_SPI_AND_OR_32:
	case DW3000_SPI_WR_BIT: {
		uint8_t crc8 = 0;
		if (pdw3000local->spicrc != DWT_SPI_CRC_MODE_NO) {
			//generate 8 bit CRC
			crc8 = dwt_generatecrc8(header, cnt, 0);
			crc8 = dwt_generatecrc8(buffer, length, crc8);

			//_dbg_printf("NONE SPI writetospiwithcrc\n");
			// Write it to the SPI
			//writetospiwithcrc(cnt, header, length, buffer, crc8);
		} else {

			// Write it to the SPI
			write(cnt, header, length, buffer);
		}
		break;
	}
	case DW3000_SPI_RD_BIT: {
		read(cnt, header, length, buffer);

		//check that the SPI read has correct CRC-8 byte
		//also don't do for SPICRC_CFG_ID register itself to prevent infinite recursion
		if ((pdw3000local->spicrc == DWT_SPI_CRC_MODE_WRRD)
				&& (regFileID != SPICRC_CFG_ID)) {
			uint8_t crc8, dwcrc8;
			//generate 8 bit CRC from the read data
			crc8 = dwt_generatecrc8(header, cnt, 0);
			crc8 = dwt_generatecrc8(buffer, length, crc8);

			//read the CRC that was generated in the DW3000 for the read transaction
			dwcrc8 = dwt_read8bitoffsetreg(SPICRC_CFG_ID, 0);

			//if the two CRC don't match report SPI read error
			//potential problem in callback if it will try to read/write SPI with CRC again.
			if (crc8 != dwcrc8) {
				if (pdw3000local->cbSPIRDErr != NULL)
					pdw3000local->cbSPIRDErr();
			}

		}
		break;
	}
	default:
		while (1)
			;
		break;
	}

} // end dwt_xfer3000()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to write to the DW3000 device registers
 *
 * input parameters:
 * @param recordNumber  - ID of register file or buffer being accessed
 * @param index         - byte index into register file or buffer being accessed
 * @param length        - number of bytes being written
 * @param buffer        - pointer to buffer containing the 'length' bytes to be written
 *
 * output parameters
 *
 * no return value
 */
//static
void dwt_writetodevice(uint32_t regFileID, uint16_t index, uint16_t length,
		uint8_t *buffer) {
	dwt_xfer3000(regFileID, index, length, buffer, DW3000_SPI_WR_BIT);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  This function wakeup device by an IO pin. DW3000 SPI_CS or WAKEUP pins can be used for this.
 *         wakeup_device_with_io() which is external to this file and is platform dependant and it should be modified to
 *         toggle the correct pin depending on the HW/MCU connections with DW3000.
 *
 * @param None
 *
 * output parameters
 *
 * no return value
 */
void dwt_wakeup_ic(void) {
	wakeup_device_with_io();
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to read from the DW3000 device registers
 *
 * @param recordNumber  - ID of register file or buffer being accessed
 * @param index         - byte index into register file or buffer being accessed
 * @param length        - number of bytes being read
 * @param buffer        - pointer to buffer in which to return the read data.
 *
 * output parameters
 *
 * no return value
 */
//static
void dwt_readfromdevice(uint32_t regFileID, uint16_t index, uint16_t length,
		uint8_t *buffer) {
	dwt_xfer3000(regFileID, index, length, buffer, DW3000_SPI_RD_BIT);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to read 32-bit value from the DW3000 device registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 *
 * output parameters
 *
 * returns 32 bit register value
 */
uint32_t dwt_read32bitoffsetreg(uint32_t regFileID, uint16_t regOffset) {
	int j;
	uint32_t regval = 0;
	uint8_t buffer[4];

	dwt_readfromdevice(regFileID, regOffset, 4, buffer); // Read 4 bytes (32-bits) register into buffer

	for (j = 3; j >= 0; j--) {
		regval = (regval << 8) + buffer[j];
	}

	return (regval);

} // end dwt_read32bitoffsetreg()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to read 16-bit value from the DW3000 device registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 *
 * output parameters
 *
 * returns 16 bit register value
 */
uint16_t dwt_read16bitoffsetreg(uint32_t regFileID, uint16_t regOffset) {
	uint16_t regval = 0;
	uint8_t buffer[2];

	dwt_readfromdevice(regFileID, regOffset, 2, buffer); // Read 2 bytes (16-bits) register into buffer

	regval = (uint16_t) ((uint16_t) buffer[1] << 8) + buffer[0];
	return regval;

} // end dwt_read16bitoffsetreg()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to read an 8-bit value from the DW3000 device registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 *
 * output parameters
 *
 * returns 8-bit register value
 */
uint8_t dwt_read8bitoffsetreg(uint32_t regFileID, uint16_t regOffset) {
	uint8_t regval;

	dwt_readfromdevice(regFileID, regOffset, 1, &regval);

	return regval;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to write 32-bit value to the DW3000 device registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 * @param regval    - the value to write
 *
 * output parameters
 *
 * no return value
 */
void dwt_write32bitoffsetreg(uint32_t regFileID, uint16_t regOffset,
		uint32_t regval) {
	int j;
	uint8_t buffer[4];

	for (j = 0; j < 4; j++) {
		buffer[j] = (uint8_t) regval;
		regval >>= 8;
	}

	dwt_writetodevice(regFileID, regOffset, 4, buffer);
} // end dwt_write32bitoffsetreg()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to write 16-bit value to the DW3000 device registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 * @param regval    - the value to write
 *
 * output parameters
 *
 * no return value
 */
void dwt_write16bitoffsetreg(uint32_t regFileID, uint16_t regOffset,
		uint16_t regval) {
	uint8_t buffer[2];

	buffer[0] = (uint8_t) regval;
	buffer[1] = regval >> 8;

	dwt_writetodevice(regFileID, regOffset, 2, buffer);
} // end dwt_write16bitoffsetreg()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to write an 8-bit value to the DW3000 device registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 * @param regval    - the value to write
 *
 * output parameters
 *
 * no return value
 */
void dwt_write8bitoffsetreg(uint32_t regFileID, uint16_t regOffset,
		uint8_t regval) {
	//uint8_t   buf[1];
	//buf[0] = regval;
	dwt_writetodevice(regFileID, regOffset, 1, &regval);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to modify a 32-bit value to the DW3000 device registers
 *
 * input parameters:
 * @param regFileID :   ID of register file or buffer being accessed
 * @param regOffset :   the index into register file or buffer being accessed
 * @param regval_and:   the value to AND to register
 * @param regval_or :   the value to OR to register
 * @output          :   no return value
 */
void dwt_modify32bitoffsetreg(const uint32_t regFileID,
		const uint16_t regOffset, const uint32_t _and, const uint32_t _or) {
	uint8_t buf[8];
	buf[0] = (uint8_t) _and;    //       &0xFF;
	buf[1] = (uint8_t) (_and >> 8);    //  &0xFF;
	buf[2] = (uint8_t) (_and >> 16);    // &0xFF;
	buf[3] = (uint8_t) (_and >> 24);    // &0xFF;
	buf[4] = (uint8_t) _or;    //        &0xFF;
	buf[5] = (uint8_t) (_or >> 8);    //   &0xFF;
	buf[6] = (uint8_t) (_or >> 16);    //  &0xFF;
	buf[7] = (uint8_t) (_or >> 24);    //  &0xFF;
	dwt_xfer3000(regFileID, regOffset, sizeof(buf), buf, DW3000_SPI_AND_OR_32);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to modify a 16-bit value to the DW3000 device registers
 *
 * input parameters:
 * @param regFileID :   ID of register file or buffer being accessed
 * @param regOffset :   the index into register file or buffer being accessed
 * @param regval_and:   the value to AND to register
 * @param regval_or :   the value to OR to register
 * @output          :   no return value
 */
void dwt_modify16bitoffsetreg(const uint32_t regFileID,
		const uint16_t regOffset, const uint16_t _and, const uint16_t _or) {
	uint8_t buf[4];
	buf[0] = (uint8_t) _and;    //       &0xFF;
	buf[1] = (uint8_t) (_and >> 8);    //  &0xFF;
	buf[2] = (uint8_t) _or;    //        &0xFF;
	buf[3] = (uint8_t) (_or >> 8);    //   &0xFF;
	dwt_xfer3000(regFileID, regOffset, sizeof(buf), buf, DW3000_SPI_AND_OR_16);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to modify a 8-bit value to the DW3000 device registers
 *
 * input parameters:
 * @param regFileID :   ID of register file or buffer being accessed
 * @param regOffset :   the index into register file or buffer being accessed
 * @param regval_and:   the value to AND to register
 * @param regval_or :   the value to OR to register
 * @output          :   no return value
 */
void dwt_modify8bitoffsetreg(const uint32_t regFileID, const uint16_t regOffset,
		const uint8_t _and, const uint8_t _or) {
	uint8_t buf[2];
	buf[0] = _and;
	buf[1] = _or;
	dwt_xfer3000(regFileID, regOffset, sizeof(buf), buf, DW3000_SPI_AND_OR_8);
}

static
void _dwt_crc8init(void) {
	uint8_t remainder;
	int dividend;

	/*
	 * Compute the remainder of each possible dividend.
	 */
	for (dividend = 0; dividend < 256; ++dividend) {
		/*
		 * Start with the dividend followed by zeros.
		 */
		remainder = (uint8_t) dividend;

		/*
		 * Perform modulo-2 division, a bit at a time.
		 */
		for (uint8_t bit = 8; bit > 0; --bit) {
			/*
			 * Try to divide the current data bit.
			 */
			if (remainder & TOPBIT) {
				remainder = (uint8_t) (remainder << 1) ^ POLYNOMIAL;
			} else {
				remainder = (uint8_t) (remainder << 1);
			}
		}

		/*
		 * Store the result into the table.
		 */
		crcTable[dividend] = remainder;
	}

} /* _dwt_crc8init() */

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to calculate 8-bit CRC, it uses 100000111 polynomial (i.e. P(x) = x^8+ x^2+ x^1+ x^0)
 * this function has been optimized to use crcTable[] and calculate the CRC on byte by byte basis.
 *
 * input parameters:
 * @param byteArray         - data to calculate CRC for
 * @param len               - length of byteArray
 * @param crcRemainderInit  - the remainder is the crc, also it is initially set to the initialisation value for CRC calculation
 *
 * output parameters
 *
 * returns 8-bit calculate CRC value
 */
uint8_t dwt_generatecrc8(const uint8_t *byteArray, int len,
		uint8_t crcRemainderInit) {
	uint8_t data;
	int byte;

	/*
	 * Divide the message by the polynomial, a byte at a time.
	 */
	for (byte = 0; byte < len; ++byte) {
		data = byteArray[byte] ^ crcRemainderInit;
		crcRemainderInit = crcTable[data];    // ^ (crcRemainderInit << 8);
	}

	/*
	 * The final remainder is the CRC.
	 */
	return (crcRemainderInit);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to enable SPI CRC check in DW3000
 *
 * input parameters
 * @param crc_mode - if set to DWT_SPI_CRC_MODE_WR then SPI CRC checking will be performed in DW3000 on each SPI write
 *                   last byte of the SPI write transaction needs to be the 8-bit CRC, if it does not match
 *                   the one calculated by DW3000 SPI CRC ERROR event will be set in the status register (SYS_STATUS_SPICRC)
 *
 * @param spireaderr_cb - this needs to contain the callback function pointer which will be called when SPI read error
 *                        is detected (when the DW3000 generated CRC does not match the one calculated by  dwt_generatecrc8
 *                        following the SPI read transaction)
 *
 * output parameters
 *
 * no return value
 */
void dwt_enablespicrccheck(dwt_spi_crc_mode_e crc_mode,
		dwt_spierrcb_t spireaderr_cb) {
	if (crc_mode != DWT_SPI_CRC_MODE_NO) //enable CRC check in DW3000
			{
		dwt_or8bitoffsetreg(SYS_CFG_ID, 0, SYS_CFG_SPI_CRC_BIT_MASK);

		if (crc_mode == DWT_SPI_CRC_MODE_WRRD) //enable CRC generation on SPI read transaction which the DW3000 will store in SPICRC_CFG_ID register
				{
			pdw3000local->cbSPIRDErr = spireaderr_cb;
		}
		//initialise the crc calculation lookup table
		_dwt_crc8init();
	} else {
		dwt_and8bitoffsetreg(SYS_CFG_ID, 0, (uint8_t)~SYS_CFG_SPI_CRC_BIT_MASK);
	}
	pdw3000local->spicrc = crc_mode;
}

static
void _dwt_prog_ldo_and_bias_tune(void) {
	dwt_or16bitoffsetreg(OTP_CFG_ID, 0, LDO_BIAS_KICK);
	dwt_and_or16bitoffsetreg(BIAS_CTRL_ID, 0, (uint16_t)~BIAS_CTRL_BIAS_MASK,
			pdw3000local->bias_tune);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function is a static function used to 'kick' the desired operating parameter set (OPS) table upon wakeup from sleep.
 *        It will load the required OPS table configuration based upon what OPS table was set to be used in dwt_configure().
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */

void _dwt_kick_ops_table_on_wakeup(void) {
	/* Restore OPS table config and kick. */
	/* Correct sleep mode should be set by dwt_configure() */

	/* Using the mask of all available OPS table options, check for OPS table options in the sleep mode mask */
	switch (pdw3000local->sleep_mode
			& (DWT_ALT_OPS | DWT_SEL_OPS0 | DWT_SEL_OPS1 | DWT_SEL_OPS2
					| DWT_SEL_OPS3)) {
	/* If preamble length >= 256 and set by dwt_configure(), the OPS table should be kicked off like so upon wakeup. */
	case (DWT_ALT_OPS | DWT_SEL_OPS0):
		dwt_modify32bitoffsetreg(OTP_CFG_ID, 0, ~(OTP_CFG_OPS_ID_BIT_MASK),
				DWT_OPSET_LONG | OTP_CFG_OPS_KICK_BIT_MASK);
		break;
		/* If SCP mode is enabled by dwt_configure(), the OPS table should be kicked off like so upon wakeup. */
	case (DWT_ALT_OPS | DWT_SEL_OPS1):
		dwt_modify32bitoffsetreg(OTP_CFG_ID, 0, ~(OTP_CFG_OPS_ID_BIT_MASK),
				DWT_OPSET_SCP | OTP_CFG_OPS_KICK_BIT_MASK);
		break;
	default:
		break;
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function is a static function used to 'kick' the DGC upon wakeup from sleep. It will load the
 *        required DGC configuration from OTP based upon what channel was set to be used in dwt_configure().
 *
 * input parameters
 * @param channel - specifies the operating channel (e.g. 5 or 9)
 *
 * output parameters
 *
 * no return value
 */

void _dwt_kick_dgc_on_wakeup(int8_t channel) {
	/* The DGC_SEL bit must be set to '0' for channel 5 and '1' for channel 9 */
	if (channel == 5) {
		dwt_modify32bitoffsetreg(OTP_CFG_ID, 0, ~(OTP_CFG_DGC_SEL_BIT_MASK),
				(DWT_DGC_SEL_CH5 << OTP_CFG_DGC_SEL_BIT_OFFSET)
						| OTP_CFG_DGC_KICK_BIT_MASK);
	} else if (channel == 9) {
		dwt_modify32bitoffsetreg(OTP_CFG_ID, 0, ~(OTP_CFG_DGC_SEL_BIT_MASK),
				(DWT_DGC_SEL_CH9 << OTP_CFG_DGC_SEL_BIT_OFFSET)
						| OTP_CFG_DGC_KICK_BIT_MASK);
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function initialises the DW3000 transceiver:
 * it reads its DEV_ID register (address 0x00) to verify the IC is one supported
 * by this software (e.g. DW3000 32-bit device ID value is 0xDECA03xx).  Then it
 * does any initial once only device configurations needed for use and initialises
 * as necessary any static data items belonging to this low-level driver.
 *
 * NOTES:
 * 1.it also reads and applies LDO and BIAS tune and crystal trim values from OTP memory
 * 2.it is assumed this function is called after a reset or on power up of the DW3000
 *
 * input parameters
 * @param mode - mask which defines which OTP values to read.
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_initialise(int mode) {
	//uint16_t otp_addr;
	//uint32_t devid;
	uint32_t ldo_tune_lo;
	uint32_t ldo_tune_hi;

	pdw3000local->dblbuffon = DBL_BUFF_OFF; // Double buffer mode off by default / clear the flag
	pdw3000local->sleep_mode = DWT_RUNSAR; // Configure RUN_SAR on wake by default as it is needed when running PGF_CAL
	pdw3000local->spicrc = 0;
	pdw3000local->stsconfig = 0; //STS off
	pdw3000local->vBatP = 0;
	pdw3000local->tempP = 0;

	pdw3000local->cbTxDone = NULL;
	pdw3000local->cbRxOk = NULL;
	pdw3000local->cbRxTo = NULL;
	pdw3000local->cbRxErr = NULL;
	pdw3000local->cbSPIRdy = NULL;
	pdw3000local->cbSPIErr = NULL;

	// Read and validate device ID return -1 if not recognised
	if (dwt_check_dev_id() != DWT_SUCCESS) {
		return DWT_ERROR;
	}

	//Read LDO_TUNE and BIAS_TUNE from OTP
	ldo_tune_lo = _dwt_otpread(LDOTUNELO_ADDRESS);
	ldo_tune_hi = _dwt_otpread(LDOTUNEHI_ADDRESS);
	pdw3000local->bias_tune = (_dwt_otpread(BIAS_TUNE_ADDRESS) >> 16)
			& BIAS_CTRL_BIAS_MASK;

	if ((ldo_tune_lo != 0) && (ldo_tune_hi != 0)
			&& (pdw3000local->bias_tune != 0)) {
		_dwt_prog_ldo_and_bias_tune();
	}

	// Read DGC_CFG from OTP
	if (_dwt_otpread(DGC_TUNE_ADDRESS) == DWT_DGC_CFG0) {
		pdw3000local->dgc_otp_set = DWT_DGC_LOAD_FROM_OTP;
	} else {
		pdw3000local->dgc_otp_set = DWT_DGC_LOAD_FROM_SW;
	}

	// Load Part and Lot ID from OTP
	if (mode & DWT_READ_OTP_PID)
		pdw3000local->partID = _dwt_otpread(PARTID_ADDRESS);
	if (mode & DWT_READ_OTP_LID)
		pdw3000local->lotID = _dwt_otpread(LOTID_ADDRESS);
	if (mode & DWT_READ_OTP_BAT)
		pdw3000local->vBatP = (uint8_t) _dwt_otpread(VBAT_ADDRESS);
	if (mode & DWT_READ_OTP_TMP)
		pdw3000local->tempP = (uint8_t) _dwt_otpread(VTEMP_ADDRESS);

	if (pdw3000local->tempP == 0) //if the reference temperature has not been programmed in OTP (early eng samples) set to default value
			{
		pdw3000local->tempP = 0x85; //@temp of 20 deg
	}

	if (pdw3000local->vBatP == 0) //if the reference voltage has not been programmed in OTP (early eng samples) set to default value
			{
		pdw3000local->vBatP = 0x74;  //@Vref of 3.0V
	}

	pdw3000local->otprev = (uint8_t) _dwt_otpread(OTPREV_ADDRESS);

	pdw3000local->init_xtrim = _dwt_otpread(XTRIM_ADDRESS) & 0x7f;
	if (pdw3000local->init_xtrim == 0) {
		pdw3000local->init_xtrim = 0x2E; //set default value
	}
	dwt_write8bitoffsetreg(XTAL_ID, 0, pdw3000local->init_xtrim);

	return DWT_SUCCESS;

} // end dwt_initialise()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function can place DW3000 into IDLE/IDLE_PLL or IDLE_RC mode when it is not actively in TX or RX.
 *
 * input parameters
 * @param state - DWT_DW_IDLE (1) to put DW3000 into IDLE/IDLE_PLL state; DWT_DW_INIT (0) to put DW3000 into INIT_RC state;
 *                DWT_DW_IDLE_RC (2) to put DW3000 into IDLE_RC state.
 *
 * output parameters none
 *
 * no return value
 */
void dwt_setdwstate(int state) {
	if (state == DWT_DW_IDLE) // Set the auto INIT2IDLE bit so that DW3000 enters IDLE mode before switching clocks to system_PLL
	//NOTE: PLL should be configured prior to this, and the device should be in IDLE_RC (if the PLL does not lock device will remain in IDLE_RC)
	{
		//switch clock to auto - if coming here from INIT_RC the clock will be FOSC/4, need to switch to auto prior to setting auto INIT2IDLE bit
		dwt_force_clocks(FORCE_CLK_AUTO);
		dwt_or8bitoffsetreg(SEQ_CTRL_ID, 0x01, SEQ_CTRL_AINIT2IDLE_BIT_MASK>>8);
	} else if (state == DWT_DW_IDLE_RC) //Change state to IDLE_RC and clear auto INIT2IDLE bit
	{
		//switch clock to FOSC
		dwt_or8bitoffsetreg(CLK_CTRL_ID, 0, FORCE_SYSCLK_FOSC);
		//clear the auto INIT2IDLE bit and set FORCE2INIT
		dwt_modify32bitoffsetreg(SEQ_CTRL_ID, 0x0,
				(uint32_t) ~SEQ_CTRL_AINIT2IDLE_BIT_MASK,
				SEQ_CTRL_FORCE2INIT_BIT_MASK);
		//clear force bits (device will stay in IDLE_RC)
		dwt_and8bitoffsetreg(SEQ_CTRL_ID, 0x2,
				(uint8_t) ~(SEQ_CTRL_FORCE2INIT_BIT_MASK>>16));
		//switch clock to auto
		dwt_force_clocks(FORCE_CLK_AUTO);
	} else
	//NOTE: the SPI rate needs to be <= 7MHz as device is switching to INIT_RC state
	{
		dwt_or8bitoffsetreg(CLK_CTRL_ID, 0, FORCE_SYSCLK_FOSCDIV4);
		//clear the auto INIT2IDLE bit and set FORCE2INIT
		dwt_modify32bitoffsetreg(SEQ_CTRL_ID, 0x0,
				(uint32_t) ~SEQ_CTRL_AINIT2IDLE_BIT_MASK,
				SEQ_CTRL_FORCE2INIT_BIT_MASK);
		dwt_and8bitoffsetreg(SEQ_CTRL_ID, 0x2,
				(uint8_t) ~(SEQ_CTRL_FORCE2INIT_BIT_MASK>>16));
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to enable GPIO clocks. The clocks are needed to ensure correct GPIO operation
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void dwt_enablegpioclocks(void) {
	dwt_or32bitoffsetreg(CLK_CTRL_ID, 0, CLK_CTRL_GPIO_CLK_EN_BIT_MASK);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to return the read OTP revision
 *
 * NOTE: dwt_initialise() must be called prior to this function so that it can return a relevant value.
 *
 * input parameters
 *
 * output parameters
 *
 * returns the read OTP revision value
 */
uint8_t dwt_otprevision(void) {
	return pdw3000local->otprev;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function enables/disables the fine grain TX sequencing (this is enabled by default in the DW3000).
 *
 * input parameters
 * @param enable - 1 to enable fine grain TX sequencing, 0 to disable it.
 *
 * output parameters none
 *
 * no return value
 */
void dwt_setfinegraintxseq(int enable) {
	if (enable) {
		dwt_write32bitoffsetreg(PWR_UP_TIMES_LO_ID, 2, PMSC_TXFINESEQ_ENABLE);
	} else {
		dwt_write32bitoffsetreg(PWR_UP_TIMES_LO_ID, 2, PMSC_TXFINESEQ_DISABLE);
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to enable GPIO for external LNA or PA functionality - HW dependent, consult the DW3000 User Manual.
 *        This can also be used for debug as enabling TX and RX GPIOs is quite handy to monitor DW3000's activity.
 *
 * NOTE: Enabling PA functionality requires that fine grain TX sequencing is deactivated. This can be done using
 *       dwt_setfinegraintxseq().
 *
 * input parameters
 * @param lna_pa - bit field: bit 0 if set will enable LNA functionality,
 *                          : bit 1 if set will enable PA functionality,
 *                          : to disable LNA/PA set the bits to 0
 * output parameters
 *
 * no return value
 */
void dwt_setlnapamode(int lna_pa) {
	uint32_t gpio_mode = dwt_read32bitreg(GPIO_MODE_ID);
	gpio_mode &= (~(GPIO_MODE_MSGP0_MODE_BIT_MASK
			| GPIO_MODE_MSGP1_MODE_BIT_MASK | GPIO_MODE_MSGP4_MODE_BIT_MASK
			| GPIO_MODE_MSGP5_MODE_BIT_MASK | GPIO_MODE_MSGP6_MODE_BIT_MASK)); //clear GPIO 4, 5, 6, configuration
	if (lna_pa & DWT_LNA_ENABLE) {
		gpio_mode |= GPIO_PIN6_EXTRX;
	}
	if (lna_pa & DWT_PA_ENABLE) {
		gpio_mode |= (GPIO_PIN4_EXTDA | GPIO_PIN5_EXTTX);
	}
	if (lna_pa & DWT_TXRX_EN) {
		gpio_mode |= (GPIO_PIN0_EXTTXE | GPIO_PIN1_EXTRXE);
	}

	dwt_write32bitreg(GPIO_MODE_ID, gpio_mode);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Returns the PG delay value of the TX
 *
 * input parameters
 *
 * output parameters
 *
 * returns uint8_t
 */
uint8_t dwt_readpgdelay(void) {
	return dwt_read8bitoffsetreg(TX_CTRL_HI_ID, 0);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to return the read V measured @ 3.3 V value recorded in OTP address 0x8 (VBAT_ADDRESS)
 *
 * NOTE: dwt_initialise() must be called prior to this function so that it can return a relevant value.
 *
 * input parameters
 *
 * output parameters
 *
 * returns the 8 bit V bat value as programmed in the factory
 */
uint8_t dwt_geticrefvolt(void) {
	return pdw3000local->vBatP;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to return the read T measured @ 23 C value recorded in OTP address 0x9 (VTEMP_ADDRESS)
 *
 * NOTE: dwt_initialise() must be called prior to this function so that it can return a relevant value.
 *
 * input parameters
 *
 * output parameters
 *
 * returns the 8 bit V temp value as programmed in the factory
 */
uint8_t dwt_geticreftemp(void) {
	return pdw3000local->tempP;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to return the read part ID of the device
 *
 * NOTE: dwt_initialise() must be called prior to this function so that it can return a relevant value.
 *
 * input parameters
 *
 * output parameters
 *
 * returns the 32 bit part ID value as programmed in the factory
 */
uint32_t dwt_getpartid(void) {
	return pdw3000local->partID;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to return the read lot ID of the device
 *
 * NOTE: dwt_initialise() must be called prior to this function so that it can return a relevant value.
 *
 * input parameters
 *
 * output parameters
 *
 * returns the 32 bit lot ID value as programmed in the factory
 */
uint32_t dwt_getlotid(void) {
	return pdw3000local->lotID;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to return the read device type and revision information of the DW3000 device (MP part is 0xDECA0300)
 *
 * input parameters
 *
 * output parameters
 *
 * returns the read value which for DW3000 is 0xDECA0312/0xDECA0302
 */
uint32_t dwt_readdevid(void) {
	return dwt_read32bitoffsetreg(DEV_ID_ID, 0);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function provides the API for the configuration of the TX spectrum
 * including the power and pulse generator delay. The input is a pointer to the data structure
 * of type dwt_txconfig_t that holds all the configurable items.
 *
 * input parameters
 * @param config    -   pointer to the txrf configuration structure, which contains the tx rf config data
 *                      If config->PGcount == 0 the PGdelay value will be used, else the PG calibration will run
 *
 * output parameters
 *
 * no return value
 */
void dwt_configuretxrf(dwt_txconfig_t *config) {
	if (config->PGcount == 0) {
		// Configure RF TX PG_DELAY
		dwt_write8bitoffsetreg(TX_CTRL_HI_ID, 0, config->PGdly);
	} else {
		uint8_t channel = 5;
		if (dwt_read8bitoffsetreg(CHAN_CTRL_ID, 0) & 0x1) {
			channel = 9;
		}
		dwt_calcbandwidthadj(config->PGcount, channel);
	}

	// Configure TX power
	dwt_write32bitreg(TX_POWER_ID, config->power);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function configures the STS AES 128 bit key value.
 * the default value is [31:00]c9a375fa,
 *                      [63:32]8df43a20,
 *                      [95:64]b5e5a4ed,
 *                     [127:96]0738123b
 *
 * input parameters
 * @param pStsKey - the pointer to the structure of dwt_sts_cp_key_t type, which holds the AES128 key value to generate STS
 *
 * output parameters
 *
 * no return value
 */
void dwt_configurestskey(dwt_sts_cp_key_t *pStsKey) {
	dwt_write32bitreg(STS_KEY0_ID, pStsKey->key0);
	dwt_write32bitreg(STS_KEY1_ID, pStsKey->key1);
	dwt_write32bitreg(STS_KEY2_ID, pStsKey->key2);
	dwt_write32bitreg(STS_KEY3_ID, pStsKey->key3);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function configures the STS AES 128 bit initial value, the default value is 1, i.e. DW3000 reset value is 1.
 *
 * input parameters
 * @param pStsIv - the pointer to the structure of dwt_sts_cp_iv_t type, which holds the IV value to generate STS
 *
 * output parameters
 *
 * no return value
 */
void dwt_configurestsiv(dwt_sts_cp_iv_t *pStsIv) {
	dwt_write32bitreg(STS_IV0_ID, pStsIv->iv0);
	dwt_write32bitreg(STS_IV1_ID, pStsIv->iv1);
	dwt_write32bitreg(STS_IV2_ID, pStsIv->iv2);
	dwt_write32bitreg(STS_IV3_ID, pStsIv->iv3);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function re-loads the STS initial value
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void dwt_configurestsloadiv(void) {
	dwt_or8bitoffsetreg(STS_CTRL_ID, 0, STS_CTRL_LOAD_IV_BIT_MASK);
}

uint16_t get_sts_mnth(uint16_t cipher, uint8_t threshold, uint8_t shift_val) {
	uint32_t value;
	uint16_t mod_val;

	value = cipher * (uint32_t) threshold;
	if (shift_val == 3) {
		value *= SQRT_FACTOR; //Factor to sqrt(2)
		value >>= SQRT_SHIFT_VAL;
	}

	mod_val = value % MOD_VALUE + HALF_MOD;
	value >>= SHIFT_VALUE;
	/* Check if modulo greater than MOD_VALUE, if yes add 1 */
	if (mod_val >= MOD_VALUE)
		value += 1;

	return (uint16_t) value;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function sets the default values of the lookup tables depending on the channel selected.
 *
 * input parameters
 * @param[in] channel - Channel that the device will be transmitting/receiving on.
 *
 * no return value
 */
void dwt_configmrxlut(int channel) {
	uint32_t lut0, lut1, lut2, lut3, lut4, lut5, lut6 = 0;

	if (channel == 5) {
		lut0 = (uint32_t) CH5_DGC_LUT_0;
		lut1 = (uint32_t) CH5_DGC_LUT_1;
		lut2 = (uint32_t) CH5_DGC_LUT_2;
		lut3 = (uint32_t) CH5_DGC_LUT_3;
		lut4 = (uint32_t) CH5_DGC_LUT_4;
		lut5 = (uint32_t) CH5_DGC_LUT_5;
		lut6 = (uint32_t) CH5_DGC_LUT_6;
	} else {
		lut0 = (uint32_t) CH9_DGC_LUT_0;
		lut1 = (uint32_t) CH9_DGC_LUT_1;
		lut2 = (uint32_t) CH9_DGC_LUT_2;
		lut3 = (uint32_t) CH9_DGC_LUT_3;
		lut4 = (uint32_t) CH9_DGC_LUT_4;
		lut5 = (uint32_t) CH9_DGC_LUT_5;
		lut6 = (uint32_t) CH9_DGC_LUT_6;
	}
	dwt_write32bitoffsetreg(DGC_LUT_0_CFG_ID, 0x0, lut0);
	dwt_write32bitoffsetreg(DGC_LUT_1_CFG_ID, 0x0, lut1);
	dwt_write32bitoffsetreg(DGC_LUT_2_CFG_ID, 0x0, lut2);
	dwt_write32bitoffsetreg(DGC_LUT_3_CFG_ID, 0x0, lut3);
	dwt_write32bitoffsetreg(DGC_LUT_4_CFG_ID, 0x0, lut4);
	dwt_write32bitoffsetreg(DGC_LUT_5_CFG_ID, 0x0, lut5);
	dwt_write32bitoffsetreg(DGC_LUT_6_CFG_ID, 0x0, lut6);
	dwt_write32bitoffsetreg(DGC_CFG0_ID, 0x0, DWT_DGC_CFG0);
	dwt_write32bitoffsetreg(DGC_CFG1_ID, 0x0, DWT_DGC_CFG1);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function needs to be called after device is woken up from DEEPSLEEP/SLEEP state, to restore the
 * configuration which has not been automatically restored from AON
 *
 * input parameters
 *
 * return DWT_SUCCESS
 *
 */
void dwt_restoreconfig(void) {
	uint8_t channel = 5;
	uint16_t chan_ctrl;

	if (pdw3000local->bias_tune != 0) {
		_dwt_prog_ldo_and_bias_tune();
	}
	dwt_write8bitoffsetreg(LDO_RLOAD_ID, 1, LDO_RLOAD_VAL_B1);
	/*Restoring indirect access register B configuration as this is not preserved when device is in DEEPSLEEP/SLEEP state.
	 * Indirect access register B is configured to point to the "Double buffer diagnostic SET 2"*/
	dwt_write32bitreg(INDIRECT_ADDR_B_ID, (BUF1_RX_FINFO >> 16));
	dwt_write32bitreg(ADDR_OFFSET_B_ID, BUF1_RX_FINFO & 0xffff);

	/* Restore OPS table configuration */
	_dwt_kick_ops_table_on_wakeup();

	chan_ctrl = dwt_read16bitoffsetreg(CHAN_CTRL_ID, 0);

	//assume RX code is the same as TX (e.g. we will not RX on 16 MHz or SCP and TX on 64 MHz)
	if ((((chan_ctrl >> CHAN_CTRL_TX_PCODE_BIT_OFFSET)
			& CHAN_CTRL_TX_PCODE_BIT_MASK) >= 9)
			&& (((chan_ctrl >> CHAN_CTRL_TX_PCODE_BIT_OFFSET)
					& CHAN_CTRL_TX_PCODE_BIT_MASK) <= 24)) //only enable DGC for PRF 64
			{
		if (chan_ctrl & 0x1) {
			channel = 9;
		}

		/* If the OTP has DGC info programmed into it, do a manual kick from OTP. */
		if (pdw3000local->dgc_otp_set == DWT_DGC_LOAD_FROM_OTP) {
			_dwt_kick_dgc_on_wakeup((int8_t) channel);
		}
		/* Else we manually program hard-coded values into the DGC registers. */
		else {
			dwt_configmrxlut(channel);
		}
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function configures STS mode: e.g. DWT_STS_MODE_OFF, DWT_STS_MODE_1 etc
 * The dwt_configure should be called prior to this to configure other parameters
 *
 * input parameters
 * @param stsMode    -   e.g. DWT_STS_MODE_OFF, DWT_STS_MODE_1 etc.
 *
 * return DWT_SUCCESS
 *
 */
void dwt_configurestsmode(uint8_t stsMode) {

	pdw3000local->stsconfig = stsMode;

	/////////////////////////////////////////////////////////////////////////
	//SYS_CFG
	//clear the PHR Mode, PHR Rate, STS Protocol, SDC, PDOA Mode,
	//then set the relevant bits according to configuration of the PHR Mode, PHR Rate, STS Protocol, SDC, PDOA Mode,
	dwt_modify32bitoffsetreg(SYS_CFG_ID, 0,
			~(SYS_CFG_CP_SPC_BIT_MASK | SYS_CFG_CP_SDC_BIT_MASK),
			(uint32_t) ((uint16_t) stsMode & DWT_STS_CONFIG_MASK)
					<< SYS_CFG_CP_SPC_BIT_OFFSET);

	if ((stsMode & DWT_STS_MODE_ND) == DWT_STS_MODE_ND) {
		//configure lower preamble detection threshold for no data STS mode
		dwt_write32bitoffsetreg(DTUNE3_ID, 0, PD_THRESH_NO_DATA);
	} else {
		dwt_write32bitoffsetreg(DTUNE3_ID, 0, PD_THRESH_DEFAULT);
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function provides the main API for the configuration of the
 * DW3000 and this low-level driver.  The input is a pointer to the data structure
 * of type dwt_config_t that holds all the configurable items.
 * The dwt_config_t structure shows which ones are supported
 *
 * input parameters
 * @param config    -   pointer to the configuration structure, which contains the device configuration data.
 *
 * return DWT_SUCCESS or DWT_ERROR
 * Note: If the RX calibration routine fails the device receiver performance will be severely affected,
 * the application should reset device and try again
 *
 */
int dwt_configure(dwt_config_t *config) {
	uint8_t chan = config->chan, cnt, flag;
	uint32_t temp;
	uint8_t scp = ((config->rxCode > 24) || (config->txCode > 24)) ? 1 : 0;
	uint8_t mode =
			(config->phrMode == DWT_PHRMODE_EXT) ?
					SYS_CFG_PHR_MODE_BIT_MASK : 0;
	uint16_t sts_len;
	int error = DWT_SUCCESS;

#ifdef DWT_API_ERROR_CHECK
    assert((config->dataRate == DWT_BR_6M8) || (config->dataRate == DWT_BR_850K));
    assert(config->rxPAC <= DWT_PAC4);
    assert((chan == 5) || (chan == 9));
    assert((config->txPreambLength == DWT_PLEN_32) || (config->txPreambLength == DWT_PLEN_64) || (config->txPreambLength == DWT_PLEN_72) || (config->txPreambLength == DWT_PLEN_128) || (config->txPreambLength == DWT_PLEN_256)
           || (config->txPreambLength == DWT_PLEN_512) || (config->txPreambLength == DWT_PLEN_1024) || (config->txPreambLength == DWT_PLEN_1536)
           || (config->txPreambLength == DWT_PLEN_2048) || (config->txPreambLength == DWT_PLEN_4096));
    assert((config->phrMode == DWT_PHRMODE_STD) || (config->phrMode == DWT_PHRMODE_EXT));
    assert((config->phrRate == DWT_PHRRATE_STD) || (config->phrRate == DWT_PHRRATE_DTA));
    assert((config->pdoaMode == DWT_PDOA_M0) || (config->pdoaMode == DWT_PDOA_M1) || (config->pdoaMode == DWT_PDOA_M3));
	assert(((config->stsMode & DWT_STS_CONFIG_MASK) == DWT_STS_MODE_OFF)
        || ((config->stsMode & DWT_STS_CONFIG_MASK) == DWT_STS_MODE_1)
        || ((config->stsMode & DWT_STS_CONFIG_MASK) == DWT_STS_MODE_2)
        || ((config->stsMode & DWT_STS_CONFIG_MASK) == DWT_STS_MODE_ND)
        || ((config->stsMode & DWT_STS_CONFIG_MASK) == DWT_STS_MODE_SDC)
        || ((config->stsMode & DWT_STS_CONFIG_MASK) == (DWT_STS_MODE_1 | DWT_STS_MODE_SDC))
        || ((config->stsMode & DWT_STS_CONFIG_MASK) == (DWT_STS_MODE_2 | DWT_STS_MODE_SDC))
        || ((config->stsMode & DWT_STS_CONFIG_MASK) == (DWT_STS_MODE_ND | DWT_STS_MODE_SDC))
        || ((config->stsMode & DWT_STS_CONFIG_MASK) == DWT_STS_CONFIG_MASK));
#endif
	int preamble_len;

	switch (config->txPreambLength) {
	case DWT_PLEN_32:
		preamble_len = 32;
		break;
	case DWT_PLEN_64:
		preamble_len = 64;
		break;
	case DWT_PLEN_72:
		preamble_len = 72;
		break;
	case DWT_PLEN_128:
		preamble_len = 128;
		break;
	default:
		preamble_len = 256;
		break;
	}

	pdw3000local->sleep_mode &= (~(DWT_ALT_OPS | DWT_SEL_OPS3)); //clear the sleep mode ALT_OPS bit
	pdw3000local->longFrames = config->phrMode;
	sts_len = (uint16_t) GET_STS_REG_SET_VALUE((uint16_t )(config->stsLength));
	pdw3000local->ststhreshold = (int16_t) ((((uint32_t) sts_len) * 8)
			* STSQUAL_THRESH_64);
	pdw3000local->stsconfig = config->stsMode;

	/////////////////////////////////////////////////////////////////////////
	//SYS_CFG
	//clear the PHR Mode, PHR Rate, STS Protocol, SDC, PDOA Mode,
	//then set the relevant bits according to configuration of the PHR Mode, PHR Rate, STS Protocol, SDC, PDOA Mode,
	dwt_modify32bitoffsetreg(SYS_CFG_ID, 0,
			~(SYS_CFG_PHR_MODE_BIT_MASK | SYS_CFG_PHR_6M8_BIT_MASK
					| SYS_CFG_CP_SPC_BIT_MASK | SYS_CFG_PDOA_MODE_BIT_MASK
					| SYS_CFG_CP_SDC_BIT_MASK),
			((uint32_t) config->pdoaMode) << SYS_CFG_PDOA_MODE_BIT_OFFSET
					| ((uint16_t) config->stsMode & DWT_STS_CONFIG_MASK)
							<< SYS_CFG_CP_SPC_BIT_OFFSET
					| (SYS_CFG_PHR_6M8_BIT_MASK
							& ((uint32_t) config->phrRate
									<< SYS_CFG_PHR_6M8_BIT_OFFSET)) | mode);

	if (scp) {
		//configure OPS tables for SCP mode
		pdw3000local->sleep_mode |= DWT_ALT_OPS | DWT_SEL_OPS1; //configure correct OPS table is kicked on wakeup
		dwt_modify32bitoffsetreg(OTP_CFG_ID, 0, ~(OTP_CFG_OPS_ID_BIT_MASK),
				DWT_OPSET_SCP | OTP_CFG_OPS_KICK_BIT_MASK);

		dwt_write32bitoffsetreg(IP_CONFIG_LO_ID, 0, IP_CONFIG_LO_SCP); //Set this if Ipatov analysis is used in SCP mode
		dwt_write32bitoffsetreg(IP_CONFIG_HI_ID, 0, IP_CONFIG_HI_SCP);

		dwt_write32bitoffsetreg(STS_CONFIG_LO_ID, 0, STS_CONFIG_LO_SCP);
		dwt_write8bitoffsetreg(STS_CONFIG_HI_ID, 0, STS_CONFIG_HI_SCP);
	} else //
	{
		uint16_t sts_mnth;
		if (config->stsMode != DWT_STS_MODE_OFF) {

			//configure CIA STS lower bound
			if ((config->pdoaMode == DWT_PDOA_M1)
					|| (config->pdoaMode == DWT_PDOA_M0)) {
				//In PDOA mode 1, number of accumulated symbols is the whole length of the STS
				sts_mnth = get_sts_mnth(
						sts_length_factors[(uint8_t) (config->stsLength)],
						CIA_MANUALLOWERBOUND_TH_64, 3);
			} else {
				//In PDOA mode 3 number of accumulated symbols is half of the length of STS symbols
				sts_mnth = get_sts_mnth(
						sts_length_factors[(uint8_t) (config->stsLength)],
						CIA_MANUALLOWERBOUND_TH_64, 4);
			}

			preamble_len += (sts_len) * 8;

			dwt_modify16bitoffsetreg(STS_CONFIG_LO_ID, 2,
					(uint16_t) ~(STS_CONFIG_LO_STS_MAN_TH_BIT_MASK >> 16),
					sts_mnth & 0x7F);

		}

		//configure OPS tables for non-SCP mode
		if (preamble_len >= 256) {
			pdw3000local->sleep_mode |= DWT_ALT_OPS | DWT_SEL_OPS0;
			dwt_modify32bitoffsetreg(OTP_CFG_ID, 0, ~(OTP_CFG_OPS_ID_BIT_MASK),
					DWT_OPSET_LONG | OTP_CFG_OPS_KICK_BIT_MASK);
		} else {
			dwt_modify32bitoffsetreg(OTP_CFG_ID, 0, ~(OTP_CFG_OPS_ID_BIT_MASK),
					DWT_OPSET_SHORT | OTP_CFG_OPS_KICK_BIT_MASK);
		}

	}

	dwt_modify8bitoffsetreg(DTUNE0_ID, 0,
			(uint8_t) ~DTUNE0_PRE_PAC_SYM_BIT_MASK, config->rxPAC);

	dwt_write8bitoffsetreg(STS_CFG0_ID, 0, (uint8_t) (sts_len - 1)); /*Starts from 0 that is why -1*/

	if (config->txPreambLength == DWT_PLEN_72) {
		dwt_setplenfine(8); //value 8 sets fine preamble length to 72 symbols - this is needed to set 72 length.
	} else {
		dwt_setplenfine(0); //clear the setting in the FINE_PLEN register.
	}

	if ((config->stsMode & DWT_STS_MODE_ND) == DWT_STS_MODE_ND) {
		//configure lower preamble detection threshold for no data STS mode
		dwt_write32bitoffsetreg(DTUNE3_ID, 0, PD_THRESH_NO_DATA);
	} else {
		//configure default preamble detection threshold for other modes
		dwt_write32bitoffsetreg(DTUNE3_ID, 0, PD_THRESH_DEFAULT);
	}

	/////////////////////////////////////////////////////////////////////////
	//CHAN_CTRL
	temp = dwt_read32bitoffsetreg(CHAN_CTRL_ID, 0);
	temp &= (~(CHAN_CTRL_RX_PCODE_BIT_MASK | CHAN_CTRL_TX_PCODE_BIT_MASK
			| CHAN_CTRL_SFD_TYPE_BIT_MASK | CHAN_CTRL_RF_CHAN_BIT_MASK));

	if (chan == 9)
		temp |= CHAN_CTRL_RF_CHAN_BIT_MASK;

	temp |= (CHAN_CTRL_RX_PCODE_BIT_MASK
			& ((uint32_t) config->rxCode << CHAN_CTRL_RX_PCODE_BIT_OFFSET));
	temp |= (CHAN_CTRL_TX_PCODE_BIT_MASK
			& ((uint32_t) config->txCode << CHAN_CTRL_TX_PCODE_BIT_OFFSET));
	temp |= (CHAN_CTRL_SFD_TYPE_BIT_MASK
			& ((uint32_t) config->sfdType << CHAN_CTRL_SFD_TYPE_BIT_OFFSET));

	dwt_write32bitoffsetreg(CHAN_CTRL_ID, 0, temp);

	/////////////////////////////////////////////////////////////////////////
	//TX_FCTRL
	// Set up TX Preamble Size, PRF and Data Rate
	dwt_modify32bitoffsetreg(TX_FCTRL_ID, 0,
			~(TX_FCTRL_TXBR_BIT_MASK | TX_FCTRL_TXPSR_BIT_MASK),
			((uint32_t) config->dataRate << TX_FCTRL_TXBR_BIT_OFFSET)
					| ((uint32_t) config->txPreambLength)
							<< TX_FCTRL_TXPSR_BIT_OFFSET);

	//DTUNE (SFD timeout)
	// Don't allow 0 - SFD timeout will always be enabled
	if (config->sfdTO == 0) {
		config->sfdTO = DWT_SFDTOC_DEF;
	}
	dwt_write16bitoffsetreg(DTUNE0_ID, 2, config->sfdTO);

	///////////////////////
	// RF
	if (chan == 9) {
		// Setup TX analog for ch9
		dwt_write32bitoffsetreg(TX_CTRL_HI_ID, 0, RF_TXCTRL_CH9);
		dwt_write16bitoffsetreg(PLL_CFG_ID, 0, RF_PLL_CFG_CH9);
		// Setup RX analog for ch9
		dwt_write32bitoffsetreg(RX_CTRL_HI_ID, 0, RF_RXCTRL_CH9);
	} else {
		// Setup TX analog for ch5
		dwt_write32bitoffsetreg(TX_CTRL_HI_ID, 0, RF_TXCTRL_CH5);
		dwt_write16bitoffsetreg(PLL_CFG_ID, 0, RF_PLL_CFG_CH5);
	}

	dwt_write8bitoffsetreg(LDO_RLOAD_ID, 1, LDO_RLOAD_VAL_B1);
	dwt_write8bitoffsetreg(TX_CTRL_LO_ID, 2, RF_TXCTRL_LO_B2);
	dwt_write8bitoffsetreg(PLL_CAL_ID, 0, RF_PLL_CFG_LD); // Extend the lock delay

	//Verify PLL lock bit is cleared
	dwt_write8bitoffsetreg(SYS_STATUS_ID, 0, SYS_STATUS_CP_LOCK_BIT_MASK);

	///////////////////////
	// auto cal the PLL and change to IDLE_PLL state
	dwt_setdwstate(DWT_DW_IDLE);

	for (flag = 1, cnt = 0; cnt < MAX_RETRIES_FOR_PLL; cnt++) {
		//deca_usleep(DELAY_20uUSec);
		HAL_Delay(1);
		if ((dwt_read8bitoffsetreg(SYS_STATUS_ID, 0)
				& SYS_STATUS_CP_LOCK_BIT_MASK)) {    //PLL is locked
			flag = 0;
			break;
		}
	}

	if (flag) {
		return DWT_ERROR;
	}

	if ((config->rxCode >= 9) && (config->rxCode <= 24)) //only enable DGC for PRF 64
			{
		//load RX LUTs
		/* If the OTP has DGC info programmed into it, do a manual kick from OTP. */
		if (pdw3000local->dgc_otp_set == DWT_DGC_LOAD_FROM_OTP) {
			_dwt_kick_dgc_on_wakeup((int8_t) chan);
		}
		/* Else we manually program hard-coded values into the DGC registers. */
		else {
			dwt_configmrxlut(chan);
		}
		dwt_modify16bitoffsetreg(DGC_CFG_ID, 0x0,
				(uint16_t) ~DGC_CFG_THR_64_BIT_MASK,
				DWT_DGC_CFG << DGC_CFG_THR_64_BIT_OFFSET);
	} else {
		dwt_and8bitoffsetreg(DGC_CFG_ID, 0x0,
				(uint8_t)~DGC_CFG_RX_TUNE_EN_BIT_MASK);
	}

	///////////////////////
	// PGF
	error = dwt_pgf_cal(1); //if the RX calibration routine fails the device receiver performance will be severely affected, the application should reset and try again

	return (error);
} // end dwt_configure()

/*! ------------------------------------------------------------------------------------------------------------------
 *
 * @brief This function runs the PGF calibration. This is needed prior to reception.
 * Note: If the RX calibration routine fails the device receiver performance will be severely affected, the application should reset and try again
 *
 * input parameters
 * @param ldoen    -   if set to 1 the function will enable LDOs prior to calibration and disable afterwards.
 *
 * return result of PGF calibration (DWT_ERROR/-1 = error)
 *
 */
int dwt_pgf_cal(int ldoen) {
	int temp;
	uint16_t val;

	//PGF needs LDOs turned on - ensure PGF LDOs are enabled
	if (ldoen == 1) {
		val = dwt_read16bitoffsetreg(LDO_CTRL_ID, 0);

		dwt_or16bitoffsetreg(LDO_CTRL_ID, 0,
				( LDO_CTRL_LDO_VDDIF2_EN_BIT_MASK | LDO_CTRL_LDO_VDDMS3_EN_BIT_MASK | LDO_CTRL_LDO_VDDMS1_EN_BIT_MASK));
	}

	//Run PGF Cal
	temp = dwt_run_pgfcal();

	//Turn off RX LDOs if previously off
	if (ldoen == 1) {
		dwt_and16bitoffsetreg(LDO_CTRL_ID, 0, val); // restore LDO values
	}
	return temp;
}

/*! ------------------------------------------------------------------------------------------------------------------
 *
 * @brief This function runs the PGF calibration. This is needed prior to reception.
 *
 * input parameters
 *
 * return result of PGF calibration (DWT_ERROR/-1 = error)
 *
 */
int dwt_run_pgfcal(void) {
	int result = DWT_SUCCESS;
	uint32_t data;
	uint32_t val = 0;
	uint8_t cnt, flag;
	//put into cal mode
	//Turn on delay mode
	data = (((uint32_t) 0x02) << RX_CAL_CFG_COMP_DLY_BIT_OFFSET)
			| (RX_CAL_CFG_CAL_MODE_BIT_MASK & 0x1);
	dwt_write32bitoffsetreg(RX_CAL_CFG_ID, 0x0, data);
	// Trigger PGF Cal
	dwt_or8bitoffsetreg(RX_CAL_CFG_ID, 0x0, RX_CAL_CFG_CAL_EN_BIT_MASK);

	for (flag = 1, cnt = 0; cnt < MAX_RETRIES_FOR_PGF; cnt++) {
		deca_usleep(DELAY_20uUSec);
		if (dwt_read8bitoffsetreg(RX_CAL_STS_ID, 0x0) == 1) { //PGF cal is complete
			flag = 0;
			break;
		}
	}
	if (flag) {
		result = DWT_ERROR;
	}

	// Put into normal mode
	dwt_write8bitoffsetreg(RX_CAL_CFG_ID, 0x0, 0);
	dwt_write8bitoffsetreg(RX_CAL_STS_ID, 0x0, 1); //clear the status
	dwt_or8bitoffsetreg(RX_CAL_CFG_ID, 0x2, 0x1); //enable reading
	val = dwt_read32bitoffsetreg(RX_CAL_RESI_ID, 0x0);
	if (val == ERR_RX_CAL_FAIL) {
		//PGF I Cal Fail
		result = DWT_ERROR;
	}
	val = dwt_read32bitoffsetreg(RX_CAL_RESQ_ID, 0x0);
	if (val == ERR_RX_CAL_FAIL) {
		//PGF Q Cal Fail
		result = DWT_ERROR;
	}

	return result;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This API function writes the antenna delay (in time units) to RX registers
 *
 * input parameters:
 * @param rxDelay - this is the total (RX) antenna delay value, which
 *                          will be programmed into the RX register
 *
 * output parameters
 *
 * no return value
 */
void dwt_setrxantennadelay(uint16_t rxDelay) {
	// Set the RX antenna delay for auto TX timestamp adjustment
	dwt_write16bitoffsetreg(CIA_CONF_ID, 0, rxDelay);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This API function writes the antenna delay (in time units) to TX registers
 *
 * input parameters:
 * @param txDelay - this is the total (TX) antenna delay value, which
 *                          will be programmed into the TX delay register
 *
 * output parameters
 *
 * no return value
 */
void dwt_settxantennadelay(uint16_t txDelay) {
	// Set the TX antenna delay for auto TX timestamp adjustment
	dwt_write16bitoffsetreg(TX_ANTD_ID, 0, txDelay);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This API function writes the supplied TX data into the DW3000's
 * TX buffer.  The input parameters are the data length in bytes and a pointer
 * to those data bytes.
 *
 * input parameters
 * @param txDataLength   - This is the total length of data (in bytes) to write to the tx buffer.
 *                         Note: the size of tx buffer is 1024 bytes.
 *                         The standard PHR mode allows to transmit frames of up to 127 bytes (including 2 byte CRC)
 *                         The extended PHR mode allows to transmit frames of up to 1023 bytes (including 2 byte CRC)
 *                         if > 127 is programmed, DWT_PHRMODE_EXT needs to be set in the phrMode configuration
 *                         see dwt_configure function
 * @param txDataBytes    - Pointer to the user�s buffer containing the data to send.
 * @param txBufferOffset - This specifies an offset in the DW IC�s TX Buffer at which to start writing data.
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_writetxdata(uint16_t data_size, uint8_t *data, uint16_t offset) {
	if ((offset + data_size) < TX_BUFFER_MAX_LEN) {
		/* Directly write the data to the IC TX buffer */
		if (offset <= REG_DIRECT_OFFSET_MAX_LEN)
			dwt_xfer3000(TX_BUFFER_ID, offset, data_size, data,
					DW3000_SPI_WR_BIT);

		else {
			/* Program the indirect offset register A for specified offset to TX buffer */
			dwt_write32bitreg(INDIRECT_ADDR_A_ID, (TX_BUFFER_ID >> 16));
			dwt_write32bitreg(ADDR_OFFSET_A_ID, offset);

			/* Indirectly write the data to the IC TX buffer */
			dwt_xfer3000(INDIRECT_POINTER_A_ID, 0, data_size, data,
					DW3000_SPI_WR_BIT);

		}
		return DWT_SUCCESS;
	} else
		return DWT_ERROR;
} // end dwt_writetxdata()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This API function configures the TX frame control register before the transmission of a frame
 *
 * input parameters:
 * @param txFrameLength - this is the length of TX message (including the 2 byte CRC) - max is 1023
 *                              NOTE: standard PHR mode allows up to 127 bytes
 *                              if > 127 is programmed, DWT_PHRMODE_EXT needs to be set in the phrMode configuration
 *                              see dwt_configure function
 * @param txBufferOffset - the offset in the tx buffer to start writing the data
 * @param ranging - 1 if this is a ranging frame, else 0
 *
 * output parameters
 *
 * no return value
 */
void dwt_writetxfctrl(uint16_t txFrameLength, uint16_t txBufferOffset,
		uint8_t ranging) {
	uint32_t reg32;
#ifdef DWT_API_ERROR_CHECK
    assert((pdw3000local->longFrames && (txFrameLength <= EXT_FRAME_LEN)) ||\
           (txFrameLength <= STD_FRAME_LEN));
#endif

	//DW3000/3700 - if offset is > 127, 128 needs to be added before data is written, this will be subtracted internally
	//prior to writing the data
	if (txBufferOffset <= 127) {
		// Write the frame length to the TX frame control register
		reg32 =
				txFrameLength
						| ((uint32_t) (txBufferOffset)
								<< TX_FCTRL_TXB_OFFSET_BIT_OFFSET)
						| ((uint32_t) ranging << TX_FCTRL_TR_BIT_OFFSET);
		dwt_modify32bitoffsetreg(TX_FCTRL_ID, 0,
				~(TX_FCTRL_TXB_OFFSET_BIT_MASK | TX_FCTRL_TR_BIT_MASK
						| TX_FCTRL_TXFLEN_BIT_MASK), reg32);
	} else {
		// Write the frame length to the TX frame control register
		reg32 = txFrameLength
				| ((uint32_t) (txBufferOffset + DWT_TX_BUFF_OFFSET_ADJUST)
						<< TX_FCTRL_TXB_OFFSET_BIT_OFFSET)
				| ((uint32_t) ranging << TX_FCTRL_TR_BIT_OFFSET);
		dwt_modify32bitoffsetreg(TX_FCTRL_ID, 0,
				~(TX_FCTRL_TXB_OFFSET_BIT_MASK | TX_FCTRL_TR_BIT_MASK
						| TX_FCTRL_TXFLEN_BIT_MASK), reg32);
		reg32 = dwt_read8bitoffsetreg(SAR_CTRL_ID, 0); //DW3000/3700 - need to read this to load the correct TX buffer offset value
	}

} // end dwt_writetxfctrl()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This API function is used to configure frame preamble length, the frame premable length can be
 * configured in steps of 8, from 16 to 2048 symbols. If a non-zero value is configured, then the TXPSR_PE setting is ignored.
 *
 * input parameters:
 * @param preambleLength - sets the length of the preamble, value of 0 disables this setting and the length of the
 *                         frame will be dependent on the TXPSR_PE setting as configured by dwt_configure function
 *
 * output parameters
 *
 * no return value
 */
void dwt_setplenfine(uint8_t preambleLength) {
	dwt_write8bitoffsetreg(TX_FCTRL_HI_ID, 1, preambleLength);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the data from the RX scratch buffer, from an offset location given by offset parameter.
 *
 * input parameters
 * @param buffer - the buffer into which the data will be read
 * @param length - the length of data to read (in bytes)
 * @param rxBufferOffset - the offset in the rx buffer from which to read the data
 *
 * output parameters
 *
 * no return value
 */
void dwt_read_rx_scratch_data(uint8_t *buffer, uint16_t length,
		uint16_t rxBufferOffset) {
	//!!Check later if needs range protection.

	/* Directly read data from the IC to the buffer */
	dwt_readfromdevice(SCRATCH_RAM_ID, rxBufferOffset, length, buffer);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the data from the RX buffer, from an offset location give by offset parameter
 *
 * input parameters
 * @param buffer - the buffer into which the data will be read
 * @param length - the length of data to read (in bytes)
 * @param rxBufferOffset - the offset in the rx buffer from which to read the data
 *
 * output parameters
 *
 * no return value
 */
void dwt_readrxdata(uint8_t *buffer, uint16_t length, uint16_t rxBufferOffset) {
	uint32_t rx_buff_addr;

	if (pdw3000local->dblbuffon == DBL_BUFF_ACCESS_BUFFER_1) //if the flag is 0x3 we are reading from RX_BUFFER_1
	{
		rx_buff_addr = RX_BUFFER_1_ID;
	} else //reading from RX_BUFFER_0 - also when non-double buffer mode
	{
		rx_buff_addr = RX_BUFFER_0_ID;
	}

	if ((rxBufferOffset + length) <= RX_BUFFER_MAX_LEN) {
		if (rxBufferOffset <= REG_DIRECT_OFFSET_MAX_LEN) {
			/* Directly read data from the IC to the buffer */
			dwt_readfromdevice(rx_buff_addr, rxBufferOffset, length, buffer);
		} else {
			/* Program the indirect offset registers B for specified offset to RX buffer */
			dwt_write32bitreg(INDIRECT_ADDR_A_ID, (rx_buff_addr >> 16));
			dwt_write32bitreg(ADDR_OFFSET_A_ID, rxBufferOffset);

			/* Indirectly read data from the IC to the buffer */
			dwt_readfromdevice(INDIRECT_POINTER_A_ID, 0, length, buffer);
		}
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the 18 bit data from the Accumulator buffer, from an offset location give by offset parameter
 *        for 18 bit complex samples, each sample is 6 bytes (3 real and 3 imaginary)
 *
 *
 * NOTE: Because of an internal memory access delay when reading the accumulator the first octet output is a dummy octet
 *       that should be discarded. This is true no matter what sub-index the read begins at.
 *
 * input parameters
 * @param buffer - the buffer into which the data will be read
 * @param length - the length of data to read (in bytes)
 * @param accOffset - the offset in the acc buffer from which to read the data, this is a complex sample index
 *                    e.g. to read 10 samples starting at sample 100
 *                    buffer would need to be >= 10*6 + 1, length is 61 (1 is for dummy), accOffset is 100
 *
 * output parameters
 *
 * no return value
 */
void dwt_readaccdata(uint8_t *buffer, uint16_t length, uint16_t accOffset) {
	// Force on the ACC clocks if we are sequenced
	dwt_or16bitoffsetreg(CLK_CTRL_ID, 0x0,
			CLK_CTRL_ACC_MCLK_EN_BIT_MASK | CLK_CTRL_ACC_CLK_EN_BIT_MASK);

	if ((accOffset + length) <= ACC_BUFFER_MAX_LEN) {
		if (accOffset <= REG_DIRECT_OFFSET_MAX_LEN) {
			/* Directly read data from the IC to the buffer */
			dwt_readfromdevice(ACC_MEM_ID, accOffset, length, buffer);
		} else {
			/* Program the indirect offset registers B for specified offset to ACC */
			dwt_write32bitreg(INDIRECT_ADDR_A_ID, (ACC_MEM_ID >> 16));
			dwt_write32bitreg(ADDR_OFFSET_A_ID, accOffset);

			/* Indirectly read data from the IC to the buffer */
			dwt_readfromdevice(INDIRECT_POINTER_A_ID, 0, length, buffer);
		}
	} else {
		assert(0);
	}

	// Revert clocks back
	dwt_and16bitoffsetreg(CLK_CTRL_ID, 0x0,
			(uint16_t)~(CLK_CTRL_ACC_MCLK_EN_BIT_MASK | CLK_CTRL_ACC_CLK_EN_BIT_MASK));
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the crystal offset (relating to the frequency offset of the far DW3000 device compared to this one)
 *        Note: the returned signed 16-bit number should be divided by by 2^26 to get ppm offset.
 *
 * input parameters - NONE
 *
 * return value - the (int12) signed offset value. (s[-15:-26])
 *                A positive value means the local RX clock is running faster than the remote TX device.
 */
int16_t dwt_readclockoffset(void) {
	uint16_t regval = 0;

	switch (pdw3000local->dblbuffon) //if the flag is non zero - we are either accessing RX_BUFFER_0 or RX_BUFFER_1
	{
	case DBL_BUFF_ACCESS_BUFFER_1:
		//!!! Assumes that Indirect pointer register B was already set. This is done in the dwt_setdblrxbuffmode when mode is enabled.
		regval =
				dwt_read16bitoffsetreg(INDIRECT_POINTER_B_ID,
						(BUF1_CIA_DIAG_0 - BUF1_RX_FINFO)) & CIA_DIAG_0_COE_PPM_BIT_MASK;
		break;
	case DBL_BUFF_ACCESS_BUFFER_0:
		regval = dwt_read16bitoffsetreg(BUF0_CIA_DIAG_0,
				0) & CIA_DIAG_0_COE_PPM_BIT_MASK;
		break;
	default:
		regval = dwt_read16bitoffsetreg(CIA_DIAG_0_ID,
				0) & CIA_DIAG_0_COE_PPM_BIT_MASK;
		break;
	}

	if (regval & B11_SIGN_EXTEND_TEST) {
		regval |= B11_SIGN_EXTEND_MASK; // sign extend bit #12 to the whole short
	}

	return (int16_t) regval;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the RX carrier integrator value (relating to the frequency offset of the TX node)
 *
 * NOTE: This is a 21-bit signed quantity, the function sign extends the most significant bit, which is bit #20
 *       (numbering from bit zero) to return a 32-bit signed integer value.
 *
 * input parameters - NONE
 *
 * return value - the (int32_t) signed carrier integrator value.
 *                A positive value means the local RX clock is running slower than the remote TX device.
 */
int32_t dwt_readcarrierintegrator(void) {
	uint32_t regval = 0;

	int j;
	uint8_t buffer[DRX_CARRIER_INT_LEN];

	/* Read 3 bytes into buffer (21-bit quantity) */
	dwt_readfromdevice(DRX_DIAG3_ID, 0, DRX_CARRIER_INT_LEN, buffer);         //

	for (j = 2; j >= 0; j--) // arrange the three bytes into an unsigned integer value
			{
		regval = (regval << 8) + buffer[j];
	}

	if (regval & B20_SIGN_EXTEND_TEST) {
		regval |= B20_SIGN_EXTEND_MASK; // sign extend bit #20 to whole word
	}

	return (int32_t) regval; // cast unsigned value to signed quantity.
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function reads the STS signal quality index
 *
 * input parameters
 * @param rxStsQualityIndex - the (int16_t) signed STS quality index value.
 *
 * output parameters
 * return value - >=0 for good and < 0 if bad STS quality.
 *
 * Note: For the 64 MHz PRF if value is >= 90% of the STS length then we can assume good STS reception.
 *       Otherwise the STS timestamp may not be accurate.
 */
int dwt_readstsquality(int16_t *rxStsQualityIndex) {
	uint16_t preambleCount;

	//read STS preamble count value
	preambleCount = dwt_read16bitoffsetreg(STS_STS_ID,
			0) & STS_STS_ACC_QUAL_BIT_MASK; //  dwt_read16bitoffsetreg(CP_PRNG_ID, CP_STS_OFFSET) & CP_ACC_CP_QUAL_MASK;

	if (preambleCount & STS_ACC_CP_QUAL_SIGNTST)
		preambleCount |= STS_ACC_CP_QUAL_SIGNEXT;

	*rxStsQualityIndex = (int16_t) preambleCount;

	//determine if the STS Rx quality is good or bad (return >=0 for good and < 0 if bad)
	return (int) ((int16_t) preambleCount - pdw3000local->ststhreshold);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function reads the STS status
 *
 * input parameters
 * @param stsStatus - the (uint16_t) STS status value. 9 bits of this buffer are populated with various STS statuses. The
 *                    remaining 7 bits are ignored.
 * @param sts_num   - 0 for 1st STS, 1 for 2nd STS (2nd only valid when PDOA Mode 3 is used)
 *
 * output parameters
 * return value DWT_SUCCESS for good/valid STS status, DWT_ERROR if bad STS status.
 */
int dwt_readstsstatus(uint16_t *stsStatus, int sts_num) {
	int ret = DWT_SUCCESS;
	uint32_t stsStatusRegAdd = (sts_num == 1) ? BUF0_STS1_STAT : BUF0_STS_STAT;
	uint32_t stsStatusRegAddN = (sts_num == 1) ? STS1_TOA_HI_ID : STS_TOA_HI_ID;

	switch (pdw3000local->dblbuffon) //check if in double buffer mode and if so which buffer host is currently accessing
	{
	case DBL_BUFF_ACCESS_BUFFER_1:
		//!!! Assumes that Indirect pointer register B was already set. This is done in the dwt_setdblrxbuffmode when mode is enabled.
		*stsStatus = dwt_read16bitoffsetreg(INDIRECT_POINTER_B_ID,
				(uint16_t) (stsStatusRegAdd - BUF0_RX_FINFO + 2)) >> 7;
		break;
	case DBL_BUFF_ACCESS_BUFFER_0:
		*stsStatus = (dwt_read16bitoffsetreg(stsStatusRegAdd, 2) >> 7);
		break;
	default:
		*stsStatus = (dwt_read16bitoffsetreg(stsStatusRegAddN, 2) >> 7);
		break;
	}

	//determine if the STS is ok
	if (*stsStatus != 0 /*& DWT_SFD_COUNT_WARN*/)
		ret = DWT_ERROR;

	return ret;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function reads the RX signal quality diagnostic data
 *
 * input parameters
 * @param diagnostics - diagnostic structure pointer, this will contain the diagnostic data read from the DW3000
 *
 * output parameters
 *
 * no return value
 */
void dwt_readdiagnostics(dwt_rxdiag_t *diagnostics) {
	int i;
	int offset_0xd;
	int offset_buff = BUF0_RX_FINFO;
	uint8_t temp[DB_MAX_DIAG_SIZE]; //address from 0xC0000 to 0xD0068 (108*2 bytes) - when using normal mode, or 232 length for max logging when in Double Buffer mode

	//minimal diagnostics - 40 bytes

	switch (pdw3000local->dblbuffon) //check if in double buffer mode and if so which buffer host is currently accessing
	{
	case DBL_BUFF_ACCESS_BUFFER_1:
		offset_buff = BUF1_RX_FINFO;
	__attribute__ ((fallthrough));
	//no break
	case DBL_BUFF_ACCESS_BUFFER_0:

		if (pdw3000local->dblbuffon == DBL_BUFF_ACCESS_BUFFER_1) {
			/* Program the indirect offset registers B for specified offset to swinging set buffer B */
			//!!! Assumes that Indirect pointer register B was already set. This is done in the dwt_setdblrxbuffmode when mode is enabled.
			/* Indirectly read data from the IC to the buffer */
			if (pdw3000local->cia_diagnostic & DW_CIA_DIAG_LOG_MAX) {
				dwt_readfromdevice(INDIRECT_POINTER_B_ID, 0, DB_MAX_DIAG_SIZE,
						temp);
			} else if (pdw3000local->cia_diagnostic & DW_CIA_DIAG_LOG_MID) {
				dwt_readfromdevice(INDIRECT_POINTER_B_ID, 0, DB_MID_DIAG_SIZE,
						temp);
			} else {
				dwt_readfromdevice(INDIRECT_POINTER_B_ID, 0, DB_MIN_DIAG_SIZE,
						temp);
			}
		} else {
			if (pdw3000local->cia_diagnostic & DW_CIA_DIAG_LOG_MAX) {
				dwt_readfromdevice((uint32_t) offset_buff, 0, DB_MAX_DIAG_SIZE,
						temp);
			} else if (pdw3000local->cia_diagnostic & DW_CIA_DIAG_LOG_MID) {
				dwt_readfromdevice((uint32_t) offset_buff, 0, DB_MID_DIAG_SIZE,
						temp);
			} else {
				dwt_readfromdevice((uint32_t) offset_buff, 0, DB_MIN_DIAG_SIZE,
						temp);
			}
		}

		for (i = 0; i < (CIA_I_RX_TIME_LEN + 1); i++) {
			diagnostics->tdoa[i] = temp[i + BUF0_TDOA - BUF0_RX_FINFO]; // timestamp difference of the 2 STS RX timestamps
		}

		diagnostics->xtalOffset = ((int16_t) temp[BUF0_CIA_DIAG_0
				- BUF0_RX_FINFO + 1] << 8
				| temp[BUF0_CIA_DIAG_0 - BUF0_RX_FINFO]) & 0x1FFF; // Estimated xtal offset of remote device

		diagnostics->pdoa = ((int16_t) temp[BUF0_PDOA - BUF0_RX_FINFO + 3] << 8
				| temp[BUF0_PDOA - BUF0_RX_FINFO + 2]) & 0x3FFF; // phase difference of the 2 STS POAs (signed in [1:-11])
		if (diagnostics->pdoa & 0x2000)
			diagnostics->pdoa |= 0xC000; //sign extend

		diagnostics->ipatovAccumCount = ((uint16_t) temp[BUF0_IP_DIAG_12
				- BUF0_RX_FINFO + 1] << 8
				| temp[BUF0_IP_DIAG_12 - BUF0_RX_FINFO]) & 0xFFF; // Number accumulated symbols [11:0] for Ipatov sequence

		if (pdw3000local->cia_diagnostic & DW_CIA_DIAG_LOG_MIN)
			break;

		for (i = 0; i < CIA_I_RX_TIME_LEN; i++) {
			diagnostics->ipatovRxTime[i] = temp[i + BUF0_IP_TS - BUF0_RX_FINFO]; // RX timestamp from Ipatov sequence
			diagnostics->stsRxTime[i] = temp[i + BUF0_STS_TS - BUF0_RX_FINFO]; // RX timestamp from STS
			diagnostics->sts2RxTime[i] = temp[i + BUF0_STS1_TS - BUF0_RX_FINFO]; // RX timestamp from STS1
		}
		diagnostics->ipatovRxStatus = temp[BUF0_IP_TS - BUF0_RX_FINFO
				+ CIA_C_STAT_OFFSET];      // RX status info for Ipatov sequence
		diagnostics->ipatovPOA =
				(uint16_t) ((uint16_t) temp[BUF0_IP_TS - BUF0_RX_FINFO + 2] << 8
						| temp[BUF0_IP_TS - BUF0_RX_FINFO + 1]); // Phase of arrival as computed from the Ipatov CIR (signed rad*2-12)

		diagnostics->stsRxStatus = temp[BUF0_STS_TS - BUF0_RX_FINFO
				+ CIA_C_STAT_OFFSET];       // RX status info for STS
		diagnostics->stsPOA = (uint16_t) ((uint16_t) temp[BUF0_STS_TS
				- BUF0_RX_FINFO + 2] << 8
				| temp[BUF0_STS_TS - BUF0_RX_FINFO + 1]); // Phase of arrival as computed from the STS 1 CIR (signed rad*2-12)

		diagnostics->sts2RxStatus = temp[BUF0_STS1_TS - BUF0_RX_FINFO
				+ CIA_C_STAT_OFFSET];       // RX status info for STS1
		diagnostics->sts2POA = (uint16_t) ((uint16_t) temp[BUF0_STS1_TS
				- BUF0_RX_FINFO + 2] << 8
				| temp[BUF0_STS1_TS - BUF0_RX_FINFO + 1]); // Phase of arrival as computed from the STS 1 CIR (signed rad*2-12)

		if (pdw3000local->cia_diagnostic & DW_CIA_DIAG_LOG_MID)
			break;

		diagnostics->ciaDiag1 = ((uint32_t) temp[BUF0_CIA_DIAG_1 - BUF0_RX_FINFO
				+ 3] << 24
				| (uint32_t) temp[BUF0_CIA_DIAG_1 - BUF0_RX_FINFO + 2] << 16
				| (uint32_t) temp[BUF0_CIA_DIAG_1 - BUF0_RX_FINFO + 1] << 8
				| (uint32_t) temp[BUF0_CIA_DIAG_1 - BUF0_RX_FINFO])
				& 0x1FFFFFFF; // Diagnostics common to both sequences (carrier integrator [28:8] and resampler delay [7:0])

		//IP
		diagnostics->ipatovPeak = ((uint32_t) temp[BUF0_IP_DIAG_0
				- BUF0_RX_FINFO + 3] << 24
				| (uint32_t) temp[BUF0_IP_DIAG_0 - BUF0_RX_FINFO + 2] << 16
				| (uint32_t) temp[BUF0_IP_DIAG_0 - BUF0_RX_FINFO + 1] << 8
				| (uint32_t) temp[BUF0_IP_DIAG_0 - BUF0_RX_FINFO]) & 0x7FFFFFFF; // index [30:21] and amplitude [20:0] of peak sample in Ipatov sequence CIR
		diagnostics->ipatovPower = ((uint32_t) temp[BUF0_IP_DIAG_1
				- BUF0_RX_FINFO + 3] << 24
				| (uint32_t) temp[BUF0_IP_DIAG_1 - BUF0_RX_FINFO + 2] << 16
				| (uint32_t) temp[BUF0_IP_DIAG_1 - BUF0_RX_FINFO + 1] << 8
				| (uint32_t) temp[BUF0_IP_DIAG_1 - BUF0_RX_FINFO]) & 0x1FFFF; // channel area allows estimation [16:0] of channel power for the Ipatov sequence
		diagnostics->ipatovF1 = ((uint32_t) temp[BUF0_IP_DIAG_2 - BUF0_RX_FINFO
				+ 3] << 24
				| (uint32_t) temp[BUF0_IP_DIAG_2 - BUF0_RX_FINFO + 2] << 16
				| (uint32_t) temp[BUF0_IP_DIAG_2 - BUF0_RX_FINFO + 1] << 8
				| (uint32_t) temp[BUF0_IP_DIAG_2 - BUF0_RX_FINFO]) & 0x3FFFFF; // F1 for Ipatov sequence [21:0]
		diagnostics->ipatovF2 = ((uint32_t) temp[BUF0_IP_DIAG_3 - BUF0_RX_FINFO
				+ 3] << 24
				| (uint32_t) temp[BUF0_IP_DIAG_3 - BUF0_RX_FINFO + 2] << 16
				| (uint32_t) temp[BUF0_IP_DIAG_3 - BUF0_RX_FINFO + 1] << 8
				| (uint32_t) temp[BUF0_IP_DIAG_3 - BUF0_RX_FINFO]) & 0x3FFFFF; // F2 for Ipatov sequence [21:0]
		diagnostics->ipatovF3 = ((uint32_t) temp[BUF0_IP_DIAG_4 - BUF0_RX_FINFO
				+ 3] << 24
				| (uint32_t) temp[BUF0_IP_DIAG_4 - BUF0_RX_FINFO + 2] << 16
				| (uint32_t) temp[BUF0_IP_DIAG_4 - BUF0_RX_FINFO + 1] << 8
				| (uint32_t) temp[BUF0_IP_DIAG_4 - BUF0_RX_FINFO]) & 0x3FFFFF; // F3 for Ipatov sequence [21:0]
		diagnostics->ipatovFpIndex =
				(uint16_t) ((uint16_t) temp[BUF0_IP_DIAG_8 - BUF0_RX_FINFO + 1]
						<< 8 | temp[BUF0_IP_DIAG_8 - BUF0_RX_FINFO]); // First path index [15:0] for Ipatov sequence

		//CP 1
		diagnostics->stsPeak = ((uint32_t) temp[BUF0_STS_DIAG_0 - BUF0_RX_FINFO
				+ 3] << 24
				| (uint32_t) temp[BUF0_STS_DIAG_0 - BUF0_RX_FINFO + 2] << 16
				| (uint32_t) temp[BUF0_STS_DIAG_0 - BUF0_RX_FINFO + 1] << 8
				| (uint32_t) temp[BUF0_STS_DIAG_0 - BUF0_RX_FINFO])
				& 0x3FFFFFFF; // index [29:21] and amplitude [20:0] of peak sample in STS CIR
		diagnostics->stsPower = (uint16_t) ((uint16_t) temp[BUF0_STS_DIAG_1
				- BUF0_RX_FINFO + 1] << 8
				| temp[BUF0_STS_DIAG_1 - BUF0_RX_FINFO]); // channel area allows estimation of channel power for the STS
		diagnostics->stsF1 = ((uint32_t) temp[BUF0_STS_DIAG_2 - BUF0_RX_FINFO
				+ 3] << 24
				| (uint32_t) temp[BUF0_STS_DIAG_2 - BUF0_RX_FINFO + 2] << 16
				| (uint32_t) temp[BUF0_STS_DIAG_2 - BUF0_RX_FINFO + 1] << 8
				| (uint32_t) temp[BUF0_STS_DIAG_2 - BUF0_RX_FINFO]) & 0x3FFFFF; // F1 for STS [21:0]
		diagnostics->stsF2 = ((uint32_t) temp[BUF0_STS_DIAG_3 - BUF0_RX_FINFO
				+ 3] << 24
				| (uint32_t) temp[BUF0_STS_DIAG_3 - BUF0_RX_FINFO + 2] << 16
				| (uint32_t) temp[BUF0_STS_DIAG_3 - BUF0_RX_FINFO + 1] << 8
				| (uint32_t) temp[BUF0_STS_DIAG_3 - BUF0_RX_FINFO]) & 0x3FFFFF; // F2 for STS [21:0]
		diagnostics->stsF3 = ((uint32_t) temp[BUF0_STS_DIAG_4 - BUF0_RX_FINFO
				+ 3] << 24
				| (uint32_t) temp[BUF0_STS_DIAG_4 - BUF0_RX_FINFO + 2] << 16
				| (uint32_t) temp[BUF0_STS_DIAG_4 - BUF0_RX_FINFO + 1] << 8
				| (uint32_t) temp[BUF0_STS_DIAG_4 - BUF0_RX_FINFO]) & 0x3FFFFF; // F3 for STS [21:0]
		diagnostics->stsFpIndex = ((uint16_t) temp[BUF0_STS_DIAG_8
				- BUF0_RX_FINFO + 1] << 8
				| temp[BUF0_STS_DIAG_8 - BUF0_RX_FINFO]) & 0x7FFF; // First path index [14:0] for STS
		diagnostics->stsAccumCount = ((uint16_t) temp[BUF0_STS_DIAG_12
				- BUF0_RX_FINFO + 1] << 8
				| temp[BUF0_STS_DIAG_12 - BUF0_RX_FINFO]) & 0xFFF; // Number accumulated symbols [11:0] for STS

		//CP 2
		diagnostics->sts2Peak = ((uint32_t) temp[BUF0_STS1_DIAG_0
				- BUF0_RX_FINFO + 3] << 24
				| (uint32_t) temp[BUF0_STS1_DIAG_0 - BUF0_RX_FINFO + 2] << 16
				| (uint32_t) temp[BUF0_STS1_DIAG_0 - BUF0_RX_FINFO + 1] << 8
				| (uint32_t) temp[BUF0_STS1_DIAG_0 - BUF0_RX_FINFO])
				& 0x3FFFFFFF; // index [29:21] and amplitude [20:0] of peak sample in STS CIR
		diagnostics->sts2Power = (uint16_t) ((uint16_t) temp[BUF0_STS1_DIAG_1
				- BUF0_RX_FINFO + 1] << 8
				| temp[BUF0_STS1_DIAG_1 - BUF0_RX_FINFO]); // channel area allows estimation of channel power for the STS
		diagnostics->sts2F1 = ((uint32_t) temp[BUF0_STS1_DIAG_2 - BUF0_RX_FINFO
				+ 3] << 24
				| (uint32_t) temp[BUF0_STS1_DIAG_2 - BUF0_RX_FINFO + 2] << 16
				| (uint32_t) temp[BUF0_STS1_DIAG_2 - BUF0_RX_FINFO + 1] << 8
				| (uint32_t) temp[BUF0_STS1_DIAG_2 - BUF0_RX_FINFO]) & 0x3FFFFF; // F1 for STS [21:0]
		diagnostics->sts2F2 = ((uint32_t) temp[BUF0_STS1_DIAG_3 - BUF0_RX_FINFO
				+ 3] << 24
				| (uint32_t) temp[BUF0_STS1_DIAG_3 - BUF0_RX_FINFO + 2] << 16
				| (uint32_t) temp[BUF0_STS1_DIAG_3 - BUF0_RX_FINFO + 1] << 8
				| (uint32_t) temp[BUF0_STS1_DIAG_3 - BUF0_RX_FINFO]) & 0x3FFFFF; // F2 for STS [21:0]
		diagnostics->sts2F3 = ((uint32_t) temp[BUF0_STS1_DIAG_4 - BUF0_RX_FINFO
				+ 3] << 24
				| (uint32_t) temp[BUF0_STS1_DIAG_4 - BUF0_RX_FINFO + 2] << 16
				| (uint32_t) temp[BUF0_STS1_DIAG_4 - BUF0_RX_FINFO + 1] << 8
				| (uint32_t) temp[BUF0_STS1_DIAG_4 - BUF0_RX_FINFO]) & 0x3FFFFF; // F3 for STS [21:0]
		diagnostics->sts2FpIndex = ((uint16_t) temp[BUF0_STS1_DIAG_8
				- BUF0_RX_FINFO + 1] << 8
				| temp[BUF0_STS1_DIAG_8 - BUF0_RX_FINFO]) & 0x7FFF; // First path index [14:0] for STS
		diagnostics->sts2AccumCount = ((uint16_t) temp[BUF0_STS1_DIAG_12
				- BUF0_RX_FINFO + 1] << 8
				| temp[BUF0_STS1_DIAG_12 - BUF0_RX_FINFO]) & 0xFFF; // Number accumulated symbols [11:0] for STS

		break;

	default:  //double buffer is off

		if (pdw3000local->cia_diagnostic & DW_CIA_DIAG_LOG_ALL) {
			dwt_readfromdevice(IP_TOA_LO_ID, 0, 108, temp); //read form 0xC0000 space  (108 bytes)
			dwt_readfromdevice(STS_DIAG_4_ID, 0, 108, &temp[108]); //read from 0xD0000 space  (108 bytes)
		} else {
			dwt_readfromdevice(IP_TOA_LO_ID, 0, 40, temp);
		}

		for (i = 0; i < CIA_I_RX_TIME_LEN; i++) {
			diagnostics->ipatovRxTime[i] = temp[i]; // RX timestamp from Ipatov sequence
			diagnostics->stsRxTime[i] = temp[i + STS_TOA_LO_ID - IP_TOA_LO_ID]; // RX timestamp from STS
			diagnostics->sts2RxTime[i] =
					temp[i + STS1_TOA_LO_ID - IP_TOA_LO_ID]; // RX timestamp from STS1
			diagnostics->tdoa[i] = temp[i + CIA_TDOA_0_ID - IP_TOA_LO_ID]; // timestamp difference of the 2 STS RX timestamps
		}
		diagnostics->tdoa[5] = temp[5 + CIA_TDOA_0_ID - IP_TOA_LO_ID];

		diagnostics->ipatovRxStatus = temp[IP_TOA_HI_ID - IP_TOA_LO_ID
				+ CIA_I_STAT_OFFSET];      // RX status info for Ipatov sequence
		diagnostics->ipatovPOA =
				(uint16_t) ((uint16_t) temp[IP_TOA_HI_ID - IP_TOA_LO_ID + 2]
						<< 8 | temp[IP_TOA_HI_ID - IP_TOA_LO_ID + 1]); // Phase of arrival as computed from the Ipatov CIR (signed rad*2-12)

		diagnostics->stsRxStatus = ((uint16_t) temp[STS_TOA_HI_ID - IP_TOA_LO_ID
				+ CIA_C_STAT_OFFSET + 1]
				| temp[STS_TOA_HI_ID - IP_TOA_LO_ID + CIA_C_STAT_OFFSET]) >> 7; // RX status info for STS
		diagnostics->stsPOA = (uint16_t) ((uint16_t) temp[STS_TOA_HI_ID
				- IP_TOA_LO_ID + 2] << 8
				| temp[STS_TOA_HI_ID - IP_TOA_LO_ID + 1]); // Phase of arrival as computed from the STS 1 CIR (signed rad*2-12)

		diagnostics->sts2RxStatus = ((uint16_t) temp[STS1_TOA_HI_ID
				- IP_TOA_LO_ID + CIA_C_STAT_OFFSET + 1]
				| temp[STS_TOA_HI_ID - IP_TOA_LO_ID + CIA_C_STAT_OFFSET]) >> 7; // RX status info for STS
		diagnostics->sts2POA = (uint16_t) ((uint16_t) temp[STS1_TOA_HI_ID
				- IP_TOA_LO_ID + 2] << 8
				| temp[STS1_TOA_HI_ID - IP_TOA_LO_ID + 1]); // Phase of arrival as computed from the STS 1 CIR (signed rad*2-12)

		diagnostics->pdoa = ((int16_t) temp[CIA_TDOA_1_PDOA_ID - IP_TOA_LO_ID
				+ 3] << 8 | temp[CIA_TDOA_1_PDOA_ID - IP_TOA_LO_ID + 2])
				& 0x3FFF; // phase difference of the 2 STS POAs (signed in [1:-11])
		if (diagnostics->pdoa & 0x2000)
			diagnostics->pdoa |= 0xC000; //sign extend

		diagnostics->xtalOffset = ((int16_t) temp[CIA_DIAG_0_ID - IP_TOA_LO_ID
				+ 1] << 8 | temp[CIA_DIAG_0_ID - IP_TOA_LO_ID]) & 0x1FFF; // Estimated xtal offset of remote device

		diagnostics->ciaDiag1 = ((uint32_t) temp[CIA_DIAG_1_ID - IP_TOA_LO_ID
				+ 3] << 24
				| (uint32_t) temp[CIA_DIAG_1_ID - IP_TOA_LO_ID + 2] << 16
				| (uint32_t) temp[CIA_DIAG_1_ID - IP_TOA_LO_ID + 1] << 8
				| (uint32_t) temp[CIA_DIAG_1_ID - IP_TOA_LO_ID]) & 0x1FFFFFFF; // Diagnostics common to both sequences

		if ((pdw3000local->cia_diagnostic & DW_CIA_DIAG_LOG_ALL) == 0)
			break; //break here is only logging minimal diagnositcs

		//IP
		diagnostics->ipatovPeak = ((uint32_t) temp[IP_DIAG_0_ID - IP_TOA_LO_ID
				+ 3] << 24
				| (uint32_t) temp[IP_DIAG_0_ID - IP_TOA_LO_ID + 2] << 16
				| (uint32_t) temp[IP_DIAG_0_ID - IP_TOA_LO_ID + 1] << 8
				| (uint32_t) temp[IP_DIAG_0_ID - IP_TOA_LO_ID]) & 0x7FFFFFFF; // index [30:21] and amplitude [20:0] of peak sample in Ipatov sequence CIR
		diagnostics->ipatovPower = ((uint32_t) temp[IP_DIAG_1_ID - IP_TOA_LO_ID
				+ 3] << 24
				| (uint32_t) temp[IP_DIAG_1_ID - IP_TOA_LO_ID + 2] << 16
				| (uint32_t) temp[IP_DIAG_1_ID - IP_TOA_LO_ID + 1] << 8
				| (uint32_t) temp[IP_DIAG_1_ID - IP_TOA_LO_ID]) & 0x1FFFF; // channel area allows estimation [16:0] of channel power for the Ipatov sequence
		diagnostics->ipatovF1 =
				((uint32_t) temp[IP_DIAG_2_ID - IP_TOA_LO_ID + 3] << 24
						| (uint32_t) temp[IP_DIAG_2_ID - IP_TOA_LO_ID + 2] << 16
						| (uint32_t) temp[IP_DIAG_2_ID - IP_TOA_LO_ID + 1] << 8
						| (uint32_t) temp[IP_DIAG_2_ID - IP_TOA_LO_ID])
						& 0x3FFFFF;    // F1 for Ipatov sequence [21:0]
		diagnostics->ipatovF2 =
				((uint32_t) temp[IP_DIAG_3_ID - IP_TOA_LO_ID + 3] << 24
						| (uint32_t) temp[IP_DIAG_3_ID - IP_TOA_LO_ID + 2] << 16
						| (uint32_t) temp[IP_DIAG_3_ID - IP_TOA_LO_ID + 1] << 8
						| (uint32_t) temp[IP_DIAG_3_ID - IP_TOA_LO_ID])
						& 0x3FFFFF;    // F2 for Ipatov sequence [21:0]
		diagnostics->ipatovF3 =
				((uint32_t) temp[IP_DIAG_4_ID - IP_TOA_LO_ID + 3] << 24
						| (uint32_t) temp[IP_DIAG_4_ID - IP_TOA_LO_ID + 2] << 16
						| (uint32_t) temp[IP_DIAG_4_ID - IP_TOA_LO_ID + 1] << 8
						| (uint32_t) temp[IP_DIAG_4_ID - IP_TOA_LO_ID])
						& 0x3FFFFF;    // F3 for Ipatov sequence [21:0]
		diagnostics->ipatovFpIndex = (uint16_t) ((uint16_t) temp[IP_DIAG_8_ID
				- IP_TOA_LO_ID + 1] << 8 | temp[IP_DIAG_8_ID - IP_TOA_LO_ID]); // First path index [15:0] for Ipatov sequence
		diagnostics->ipatovAccumCount = ((uint16_t) temp[IP_DIAG_12_ID
				- IP_TOA_LO_ID + 1] << 8 | temp[IP_DIAG_12_ID - IP_TOA_LO_ID])
				& 0xFFF; // Number accumulated symbols [11:0] for Ipatov sequence

		//STS 1
		diagnostics->stsPeak =
				((uint32_t) temp[STS_DIAG_0_ID - IP_TOA_LO_ID + 3] << 24
						| (uint32_t) temp[STS_DIAG_0_ID - IP_TOA_LO_ID + 2]
								<< 16
						| (uint32_t) temp[STS_DIAG_0_ID - IP_TOA_LO_ID + 1] << 8
						| (uint32_t) temp[STS_DIAG_0_ID - IP_TOA_LO_ID])
						& 0x3FFFFFFF; // index [29:21] and amplitude [20:0] of peak sample in STS CIR
		diagnostics->stsPower = (uint16_t) ((uint16_t) temp[STS_DIAG_1_ID
				- IP_TOA_LO_ID + 1] << 8 | temp[STS_DIAG_1_ID - IP_TOA_LO_ID]); // channel area allows estimation of channel power for the STS
		diagnostics->stsF1 = ((uint32_t) temp[STS_DIAG_2_ID - IP_TOA_LO_ID + 3]
				<< 24 | (uint32_t) temp[STS_DIAG_2_ID - IP_TOA_LO_ID + 2] << 16
				| (uint32_t) temp[STS_DIAG_2_ID - IP_TOA_LO_ID + 1] << 8
				| (uint32_t) temp[STS_DIAG_2_ID - IP_TOA_LO_ID]) & 0x3FFFFF; // F1 for STS [21:0]
		diagnostics->stsF2 = ((uint32_t) temp[STS_DIAG_3_ID - IP_TOA_LO_ID + 3]
				<< 24 | (uint32_t) temp[STS_DIAG_3_ID - IP_TOA_LO_ID + 2] << 16
				| (uint32_t) temp[STS_DIAG_3_ID - IP_TOA_LO_ID + 1] << 8
				| (uint32_t) temp[STS_DIAG_3_ID - IP_TOA_LO_ID]) & 0x3FFFFF; // F2 for STS [21:0]

		offset_0xd = 0x6c; // there are 0x6C bytes in 0xC0000 base before we enter 0xD0000

		diagnostics->stsF3 =
				((uint32_t) temp[STS_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd + 3]
						<< 24
						| (uint32_t) temp[STS_DIAG_4_ID - STS_DIAG_4_ID
								+ offset_0xd + 2] << 16
						| (uint32_t) temp[STS_DIAG_4_ID - STS_DIAG_4_ID
								+ offset_0xd + 1] << 8
						| (uint32_t) temp[STS_DIAG_4_ID - STS_DIAG_4_ID
								+ offset_0xd]) & 0x3FFFFF; // F3 for STS [21:0]
		diagnostics->stsFpIndex = ((uint16_t) temp[STS_DIAG_8_ID - STS_DIAG_4_ID
				+ offset_0xd + 1] << 8
				| temp[STS_DIAG_8_ID - STS_DIAG_4_ID + offset_0xd]) & 0x7FFF; // First path index [14:0] for STS
		diagnostics->stsAccumCount = ((uint16_t) temp[STS_DIAG_12_ID
				- STS_DIAG_4_ID + offset_0xd + 1] << 8
				| temp[STS_DIAG_12_ID - STS_DIAG_4_ID + offset_0xd]) & 0xFFF; // Number accumulated symbols [11:0] for STS

		//STS 2
		diagnostics->sts2Peak = ((uint32_t) temp[STS1_DIAG_0_ID - STS_DIAG_4_ID
				+ offset_0xd + 3] << 24
				| (uint32_t) temp[STS1_DIAG_0_ID - STS_DIAG_4_ID + offset_0xd
						+ 2] << 16
				| (uint32_t) temp[STS1_DIAG_0_ID - STS_DIAG_4_ID + offset_0xd
						+ 1] << 8
				| (uint32_t) temp[STS1_DIAG_0_ID - STS_DIAG_4_ID + offset_0xd])
				& 0x3FFFFFFF; // index [29:21] and amplitude [20:0] of peak sample in STS CIR
		diagnostics->sts2Power = (uint16_t) ((uint16_t) temp[STS1_DIAG_1_ID
				- STS_DIAG_4_ID + offset_0xd + 1] << 8
				| temp[STS1_DIAG_1_ID - STS_DIAG_4_ID + offset_0xd]); // channel area allows estimation of channel power for the STS
		diagnostics->sts2F1 = ((uint32_t) temp[STS1_DIAG_2_ID - STS_DIAG_4_ID
				+ offset_0xd + 3] << 24
				| (uint32_t) temp[STS1_DIAG_2_ID - STS_DIAG_4_ID + offset_0xd
						+ 2] << 16
				| (uint32_t) temp[STS1_DIAG_2_ID - STS_DIAG_4_ID + offset_0xd
						+ 1] << 8
				| (uint32_t) temp[STS1_DIAG_2_ID - STS_DIAG_4_ID + offset_0xd])
				& 0x3FFFFF; // F1 for STS [21:0]
		diagnostics->sts2F2 = ((uint32_t) temp[STS1_DIAG_3_ID - STS_DIAG_4_ID
				+ offset_0xd + 3] << 24
				| (uint32_t) temp[STS1_DIAG_3_ID - STS_DIAG_4_ID + offset_0xd
						+ 2] << 16
				| (uint32_t) temp[STS1_DIAG_3_ID - STS_DIAG_4_ID + offset_0xd
						+ 1] << 8
				| (uint32_t) temp[STS1_DIAG_3_ID - STS_DIAG_4_ID + offset_0xd])
				& 0x3FFFFF; // F2 for STS [21:0]
		diagnostics->sts2F3 = ((uint32_t) temp[STS1_DIAG_4_ID - STS_DIAG_4_ID
				+ offset_0xd + 3] << 24
				| (uint32_t) temp[STS1_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd
						+ 2] << 16
				| (uint32_t) temp[STS1_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd
						+ 1] << 8
				| (uint32_t) temp[STS1_DIAG_4_ID - STS_DIAG_4_ID + offset_0xd])
				& 0x3FFFFF; // F3 for STS [21:0]
		diagnostics->sts2FpIndex = ((uint16_t) temp[STS1_DIAG_8_ID
				- STS_DIAG_4_ID + offset_0xd + 1] << 8
				| temp[STS1_DIAG_8_ID - STS_DIAG_4_ID + offset_0xd]) & 0x7FFF; // First path index [14:0] for STS
		diagnostics->sts2AccumCount = ((uint16_t) temp[STS1_DIAG_12_ID
				- STS_DIAG_4_ID + offset_0xd + 1] << 8
				| temp[STS1_DIAG_12_ID - STS_DIAG_4_ID + offset_0xd]) & 0xFFF; // Number accumulated symbols [11:0] for STS
		break;
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the TX timestamp (adjusted with the programmed antenna delay)
 *
 * input parameters
 * @param timestamp - a pointer to a 5-byte buffer which will store the read TX timestamp time
 *
 * output parameters - the timestamp buffer will contain the value after the function call
 *
 * no return value
 */
void dwt_readtxtimestamp(uint8_t *timestamp) {
	dwt_readfromdevice(TX_TIME_LO_ID, 0, TX_TIME_TX_STAMP_LEN, timestamp); // Read bytes directly into buffer
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the high 32-bits of the TX timestamp (adjusted with the programmed antenna delay)
 *
 * input parameters
 *
 * output parameters
 *
 * returns high 32-bits of TX timestamp
 */
uint32_t dwt_readtxtimestamphi32(void) {
	return dwt_read32bitoffsetreg(TX_TIME_LO_ID, 1); // Offset is 1 to get the 4 upper bytes out of 5
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the low 32-bits of the TX timestamp (adjusted with the programmed antenna delay)
 *
 * input parameters
 *
 * output parameters
 *
 * returns low 32-bits of TX timestamp
 */
uint32_t dwt_readtxtimestamplo32(void) {
	return dwt_read32bitreg(TX_TIME_LO_ID); // Read TX TIME as a 32-bit register to get the 4 lower bytes out of 5
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the PDOA result, it is the phase difference between either the Ipatov and STS POA (in PDOA mode 1),
 *  or the two STS POAs (in PDOA mode 3), depending on the PDOA mode of operation. (POA - Phase Of Arrival)
 *
 * NOTE: To convert to degrees: float pdoa_deg = ((float)pdoa / (1 << 11)) * 180 / M_PI
 *
 * input parameters
 *
 * output parameters - the PDOA result (signed in [1:-11] radian units)
 *
 * no return value
 */
int16_t dwt_readpdoa(void) {
	int16_t pdoa;

	switch (pdw3000local->dblbuffon) //check if in double buffer mode and if so which buffer host is currently accessing
	{
	case DBL_BUFF_ACCESS_BUFFER_1:
		//!!! Assumes that Indirect pointer register B was already set. This is done in the dwt_setdblrxbuffmode when mode is enabled.
		pdoa = dwt_read16bitoffsetreg(INDIRECT_POINTER_B_ID,
				BUF1_PDOA - BUF1_RX_FINFO + 2)
				& (CIA_TDOA_1_PDOA_PDOA_BIT_MASK >> 16);
		break;
	case DBL_BUFF_ACCESS_BUFFER_0:
		pdoa = dwt_read16bitoffsetreg(BUF0_PDOA, 2)
				& (CIA_TDOA_1_PDOA_PDOA_BIT_MASK >> 16);
		break;
	default:
		pdoa = dwt_read16bitoffsetreg(CIA_TDOA_1_PDOA_ID, 2)
				& (CIA_TDOA_1_PDOA_PDOA_BIT_MASK >> 16); // phase difference of the 2 POAs
		break;
	}

	if ((unsigned long) pdoa & B12_SIGN_EXTEND_TEST)
		pdoa |= B12_SIGN_EXTEND_MASK; //sign extend
	return pdoa;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function is used to read the TDOA (Time Difference On Arrival). The TDOA value that is read from the
 * register is 41-bits in length. However, 6 bytes (or 48 bits) are read from the register. The remaining 7 bits at
 * the 'top' of the 6 bytes that are not part of the TDOA value should be set to zero and should not interfere with
 * rest of the 41-bit value. However, there is no harm in masking the returned value.
 *
 * input parameters
 *
 * output parameters
 * @param tdoa: time difference on arrival - buffer of 6 bytes that will be filled with TDOA value by calling this function
 *
 * no return value
 */
void dwt_readtdoa(uint8_t *tdoa) {
	// timestamp difference of the 2 cipher RX timestamps
	dwt_readfromdevice(CIA_TDOA_0_ID, 0, CIA_TDOA_LEN, tdoa);
	tdoa[5] &= 0x01; // TDOA value is 41 bits long. You will need to read 6 bytes and mask the highest byte with 0x01
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the RX timestamp (adjusted time of arrival)
 *
 * input parameters
 * @param timestamp - a pointer to a 5-byte buffer which will store the read RX timestamp time
 *
 * output parameters - the timestamp buffer will contain the value after the function call
 *
 * no return value
 */
void dwt_readrxtimestamp(uint8_t *timestamp) {
	switch (pdw3000local->dblbuffon) //check if in double buffer mode and if so which buffer host is currently accessing
	{
	case DBL_BUFF_ACCESS_BUFFER_1:
		//!!! Assumes that Indirect pointer register B was already set. This is done in the dwt_setdblrxbuffmode when mode is enabled.
		dwt_readfromdevice(INDIRECT_POINTER_B_ID, BUF1_RX_TIME - BUF1_RX_FINFO,
				RX_TIME_RX_STAMP_LEN, timestamp);
		break;
	case DBL_BUFF_ACCESS_BUFFER_0:
		dwt_readfromdevice(BUF0_RX_TIME, 0, RX_TIME_RX_STAMP_LEN, timestamp);
		break;
	default:
		dwt_readfromdevice(RX_TIME_0_ID, 0, RX_TIME_RX_STAMP_LEN, timestamp); // Get the adjusted time of arrival
		break;
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the raw RX timestamp (RMARKER time) before any CIA first path analysis adjustments
 *
 * input parameters
 * @param timestamp - a pointer to a 4-byte buffer which will store the read RX timestamp time
 *
 * output parameters - the timestamp buffer will contain the value after the function call
 *
 * no return value
 */
void dwt_readrxtimestampunadj(uint8_t *timestamp) {
	timestamp[0] = 0;
	dwt_readfromdevice(RX_TIME_RAW_ID, 0, RX_TIME_RX_STAMP_LEN - 1,
			&timestamp[1]);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the RX timestamp (adjusted time of arrival) w.r.t. Ipatov CIR
 *
 * input parameters
 * @param timestamp - a pointer to a 5-byte buffer which will store the read RX timestamp time
 *
 * output parameters - the timestamp buffer will contain the value after the function call
 *
 * no return value
 */
void dwt_readrxtimestamp_ipatov(uint8_t *timestamp) {
	switch (pdw3000local->dblbuffon) //check if in double buffer mode and if so which buffer host is currently accessing
	{
	case DBL_BUFF_ACCESS_BUFFER_1:
		//!!! Assumes that Indirect pointer register B was already set. This is done in the dwt_setdblrxbuffmode when mode is enabled.
		dwt_readfromdevice(INDIRECT_POINTER_B_ID, BUF1_IP_TS - BUF1_RX_FINFO,
				CIA_I_RX_TIME_LEN, timestamp);
		break;
	case DBL_BUFF_ACCESS_BUFFER_0:
		dwt_readfromdevice(BUF0_IP_TS, 0, CIA_I_RX_TIME_LEN, timestamp);
		break;
	default:
		dwt_readfromdevice(IP_TOA_LO_ID, 0, CIA_I_RX_TIME_LEN, timestamp); // Get the adjusted time of arrival w.r.t. Ipatov CIR
		break;
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the RX timestamp (adjusted time of arrival) w.r.t. STS CIR
 *
 * input parameters
 * @param timestamp - a pointer to a 5-byte buffer which will store the read RX timestamp time
 *
 * output parameters - the timestamp buffer will contain the value after the function call
 *
 * no return value
 */
void dwt_readrxtimestamp_sts(uint8_t *timestamp) {
	switch (pdw3000local->dblbuffon) //check if in double buffer mode and if so which buffer host is currently accessing
	{
	case DBL_BUFF_ACCESS_BUFFER_1:
		//!!! Assumes that Indirect pointer register B was already set. This is done in the dwt_setdblrxbuffmode when mode is enabled.
		dwt_readfromdevice(INDIRECT_POINTER_B_ID, BUF1_STS_TS - BUF1_RX_FINFO,
				CIA_C_RX_TIME_LEN, timestamp);
		break;
	case DBL_BUFF_ACCESS_BUFFER_0:
		dwt_readfromdevice(BUF0_STS_TS, 0, CIA_C_RX_TIME_LEN, timestamp);
		break;
	default:
		dwt_readfromdevice(STS_TOA_LO_ID, 0, CIA_C_RX_TIME_LEN, timestamp); // Get the adjusted time of arrival w.r.t. STS CIR
		break;
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the high 32-bits of the RX timestamp (adjusted with the programmed antenna delay)
 *
 * input parameters
 *
 * output parameters
 *
 * returns high 32-bits of RX timestamp
 */
uint32_t dwt_readrxtimestamphi32(void) {
	return dwt_read32bitoffsetreg(RX_TIME_0_ID, 1); // Offset is 1 to get the 4 upper bytes out of 5 byte tiemstamp
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the low 32-bits of the RX timestamp (adjusted with the programmed antenna delay)
 *
 * input parameters
 *
 * output parameters
 *
 * returns low 32-bits of RX timestamp
 */
uint32_t dwt_readrxtimestamplo32(void) {
	return dwt_read32bitreg(RX_TIME_0_ID); // Read RX TIME as a 32-bit register to get the 4 lower bytes out of 5 byte timestamp
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the high 32-bits of the system time
 *
 * input parameters
 *
 * output parameters
 *
 * returns high 32-bits of system time timestamp
 */
uint32_t dwt_readsystimestamphi32(void) {
	return dwt_read32bitreg(SYS_TIME_ID);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the system time
 *
 * input parameters
 * @param timestamp - a pointer to a 4-byte buffer which will store the read system time
 *
 * output parameters
 * @param timestamp - the timestamp buffer will contain the value after the function call
 *
 * no return value
 */
void dwt_readsystime(uint8_t *timestamp) {
	dwt_readfromdevice(SYS_TIME_ID, 0, SYS_TIME_LEN, timestamp);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to enable the frame filtering - (the default option is to
 * accept any data and ACK frames with correct destination address)
 *
 * input parameters
 * @param enabletype (bitmask) - enables/disables the frame filtering and configures 802.15.4 type
 *       DWT_FF_ENABLE_802_15_4      0x2             // use 802.15.4 filtering rules
 *       DWT_FF_DISABLE              0x0             // disable FF
 * @param filtermode (bitmask) - configures the frame filtering options according to
 *       DWT_FF_BEACON_EN            0x001           // beacon frames allowed
 *       DWT_FF_DATA_EN              0x002           // data frames allowed
 *       DWT_FF_ACK_EN               0x004           // ack frames allowed
 *       DWT_FF_MAC_EN               0x008           // mac control frames allowed
 *       DWT_FF_RSVD_EN              0x010           // reserved frame types allowed
 *       DWT_FF_MULTI_EN             0x020           // multipurpose frames allowed
 *       DWT_FF_FRAG_EN              0x040           // fragmented frame types allowed
 *       DWT_FF_EXTEND_EN            0x080           // extended frame types allowed
 *       DWT_FF_COORD_EN             0x100           // behave as coordinator (can receive frames with no dest address (PAN ID has to match))
 *       DWT_FF_IMPBRCAST_EN         0x200           // allow MAC implicit broadcast
 *       DWT_FF_LE0_PEND             0x400           // Data pending for device at LE0 address. see dwt_configure_le_address for more info
 *       DWT_FF_LE1_PEND             0x800           // Data pending for device at LE1 address. see dwt_configure_le_address for more info
 *       DWT_FF_LE2_PEND             0x1000           // Data pending for device at LE2 address. see dwt_configure_le_address for more info
 *       DWT_FF_LE3_PEND             0x2000          // Data pending for device at LE3 address. see dwt_configure_le_address for more info
 *       DWT_SSADRAPE                0x4000          //Short Source Address Data Request ACK with PEND Enable
 *       DWT_LSADRAPE                0x8000          //Long Source Address Data Request ACK with PEND Enable
 * output parameters
 *
 * no return value
 */
void dwt_configureframefilter(uint16_t enabletype, uint16_t filtermode) {
	if (enabletype == DWT_FF_ENABLE_802_15_4) {
		dwt_or8bitoffsetreg(SYS_CFG_ID, 0, (uint8_t)(SYS_CFG_FFEN_BIT_MASK));
		dwt_write16bitoffsetreg(ADR_FILT_CFG_ID, 0, filtermode);
	} else {
		// Disable frame filter
		dwt_and8bitoffsetreg(SYS_CFG_ID, 0,
				(uint8_t)(~(SYS_CFG_FFEN_BIT_MASK)));
		// Clear the configuration
		dwt_write16bitoffsetreg(ADR_FILT_CFG_ID, 0, 0x0);
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to set the PAN ID
 *
 * input parameters
 * @param panID - this is the PAN ID
 *
 * output parameters
 *
 * no return value
 */
void dwt_setpanid(uint16_t panID) {
	// PAN ID is high 16 bits of register
	dwt_write16bitoffsetreg(PANADR_ID, PANADR_PAN_ID_BYTE_OFFSET, panID);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to set 16-bit (short) address
 *
 * input parameters
 * @param shortAddress - this sets the 16 bit short address
 *
 * output parameters
 *
 * no return value
 */
void dwt_setaddress16(uint16_t shortAddress) {
	// Short address into low 16 bits
	dwt_write16bitoffsetreg(PANADR_ID, PANADR_SHORTADDR_BIT_OFFSET,
			shortAddress);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to set the EUI 64-bit (long) address
 *
 * input parameters
 * @param eui64 - this is the pointer to a buffer that contains the 64bit address
 *
 * output parameters
 *
 * no return value
 */
void dwt_seteui(uint8_t *eui64) {
	dwt_writetodevice(EUI_64_LO_ID, 0, 0x8, eui64);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to get the EUI 64-bit from the DW3000
 *
 * input parameters
 * @param eui64 - this is the pointer to a buffer that will contain the read 64-bit EUI value
 *
 * output parameters
 *
 * no return value
 */
void dwt_geteui(uint8_t *eui64) {
	dwt_readfromdevice(EUI_64_LO_ID, 0, 0x8, eui64);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read from AON memory
 *
 * input parameters
 * @param aon_address - this is the address of the memory location to read
 *
 * output parameters - None
 *
 * returns 8-bits read from given AON memory address
 */
uint8_t dwt_aon_read(uint16_t aon_address) {
	dwt_write16bitoffsetreg(AON_ADDR_ID, 0x0, aon_address); // Set short AON address for read
	dwt_write8bitoffsetreg(AON_CTRL_ID, 0x0,
			(AON_CTRL_DCA_ENAB_BIT_MASK | AON_CTRL_DCA_READ_EN_BIT_MASK));
	dwt_write8bitoffsetreg(AON_CTRL_ID, 0x0, 0x0); // Clear all enabled bits
	return dwt_read8bitoffsetreg(AON_RDATA_ID, 0x0); //Return the data that was read
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to write to AON memory
 *
 * @param aon_address - this is the address of the memory location to write
 * @param aon_write_data - this is the data to write
 *
 * output parameters - None
 *
 * no return value
 *
 */
void dwt_aon_write(uint16_t aon_address, uint8_t aon_write_data) {
	uint8_t temp = 0;
	if (aon_address >= 0x100)
		temp = AON_CTRL_DCA_WRITE_HI_EN_BIT_MASK;
	dwt_write16bitoffsetreg(AON_ADDR_ID, 0x0, aon_address); // Set AON address for write
	dwt_write8bitoffsetreg(AON_WDATA_ID, 0x0, aon_write_data); // Set write data
	dwt_write8bitoffsetreg(AON_CTRL_ID, 0x0,
			(temp | AON_CTRL_DCA_ENAB_BIT_MASK | AON_CTRL_DCA_WRITE_EN_BIT_MASK)); //Enable write
	dwt_write8bitoffsetreg(AON_CTRL_ID, 0x0, 0x0); // Clear all enabled bits
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the OTP data from given address into provided array
 *
 * input parameters
 * @param address - this is the OTP address to read from
 * @param array - this is the pointer to the array into which to read the data
 * @param length - this is the number of 32 bit words to read (array needs to be at least this length)
 *
 * output parameters
 *
 * no return value
 */
void dwt_otpread(uint16_t address, uint32_t *array, uint8_t length) {
	uint8_t i;

	for (i = 0; i < length; i++) {
		array[i] = _dwt_otpread(address + i);
	}

	return;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief function to read the OTP memory.
 *
 * input parameters
 * @param address - address to read at
 *
 * output parameters
 *
 * returns the 32bit of read data
 */
uint32_t _dwt_otpread(uint16_t address) {
	uint32_t ret_data = 0;

	// Set manual access mode
	dwt_write16bitoffsetreg(OTP_CFG_ID, 0, 0x0001);
	// set the address
	dwt_write16bitoffsetreg(OTP_ADDR_ID, 0, address);
	// Assert the read strobe
	dwt_write16bitoffsetreg(OTP_CFG_ID, 0, 0x0002);
	// attempt a read from OTP address
	ret_data = dwt_read32bitoffsetreg(OTP_RDATA_ID, 0);

	// Return the 32bit of read data
	return ret_data;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief For each value to send to OTP bloc, following two register writes are required as shown below
 *
 * @param val: 16-bit value to write to the OTP block
 */
void __dwt_otp_write_wdata_id_reg(int16_t val) {
	/* Pull the CS high to enable user interface for programming */
	/* 'val' is ignored in this instance by the OTP block */
	dwt_write16bitoffsetreg(OTP_WDATA_ID, 0, (uint16_t) (0x0200 | val));
	/* Send the relevant command to the OTP block */
	dwt_write16bitoffsetreg(OTP_WDATA_ID, 0, (uint16_t) (0x0000 | val));
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief function to program the OTP memory.
 * Note the address is only 11 bits long.
 *
 * input parameters
 * @param data - data to write to given address
 * @param address - address to write to
 *
 * output parameters
 *
 * returns None
 */
void _dwt_otpprogword32(uint32_t data, uint16_t address) {
	//uint32_t rd_buf;
	uint16_t wr_buf[4];
	//uint8_t otp_done;

	// Read current register value
	uint32_t ldo_tune = dwt_read32bitoffsetreg(LDO_TUNE_HI_ID, 0);
	// Set VDDHV_TX LDO to max
	dwt_or32bitoffsetreg(LDO_TUNE_HI_ID, 0,
			LDO_TUNE_HI_LDO_HVAUX_TUNE_BIT_MASK);

	// configure mode for programming
	dwt_write16bitoffsetreg(OTP_CFG_ID, 0, 0x018);

	// Select fast programming
	__dwt_otp_write_wdata_id_reg(0x0025);

	// Apply instruction to write the address
	__dwt_otp_write_wdata_id_reg(0x0002);
	__dwt_otp_write_wdata_id_reg(0x01fc);

	// Now sending the OTP address data (2 bytes)
	wr_buf[0] = 0x0100 | (address & 0xff);
	__dwt_otp_write_wdata_id_reg((int16_t) wr_buf[0]);

	// Write data (upper byte of address)
	__dwt_otp_write_wdata_id_reg(0x0100);

	// Clean up
	__dwt_otp_write_wdata_id_reg(0x0000);

	// Apply instruction  to write data
	__dwt_otp_write_wdata_id_reg(0x0002);
	__dwt_otp_write_wdata_id_reg(0x01c0);

	// Write the data
	wr_buf[0] = 0x100 | ((data >> 24) & 0xff);
	wr_buf[1] = 0x100 | ((data >> 16) & 0xff);
	wr_buf[2] = 0x100 | ((data >> 8) & 0xff);
	wr_buf[3] = 0x100 | (data & 0xff);
	__dwt_otp_write_wdata_id_reg((int16_t) wr_buf[3]);
	__dwt_otp_write_wdata_id_reg((int16_t) wr_buf[2]);
	__dwt_otp_write_wdata_id_reg((int16_t) wr_buf[1]);
	__dwt_otp_write_wdata_id_reg((int16_t) wr_buf[0]);

	// Clean up
	__dwt_otp_write_wdata_id_reg(0x0000);

	//Enter prog mode
	__dwt_otp_write_wdata_id_reg(0x003a);
	__dwt_otp_write_wdata_id_reg(0x01ff);
	__dwt_otp_write_wdata_id_reg(0x010a);
	// Clean up
	__dwt_otp_write_wdata_id_reg(0x0000);

	/*
	 // Enable state/status output
	 __dwt_otp_write_wdata_id_reg(0x003a);
	 __dwt_otp_write_wdata_id_reg(0x01bf);
	 __dwt_otp_write_wdata_id_reg(0x0100);
	 */

	//Start prog mode
	__dwt_otp_write_wdata_id_reg(0x003a);
	__dwt_otp_write_wdata_id_reg(0x0101);
	dwt_write16bitoffsetreg(OTP_WDATA_ID, 0, 0x0002); // Different to previous one
	dwt_write16bitoffsetreg(OTP_WDATA_ID, 0, 0x0000);

	/*
	 read status after programm command.
	 The for loop will exit once the status indicates programming is complete or if it reaches the max 1000 iterations.
	 1000 is more than sufficient for max OTP programming delay and max supported DW3000 SPI rate.
	 Instead a delay of 2ms (as commented out below) can be used.
	 Burn time is about 1.76ms
	 */

	/*uint16_t	i;

	 for (i = 0; i < 1000; i++)
	 {
	 rd_buf = dwt_read32bitoffsetreg(OTP_STATUS_ID, 0);

	 if (!(rd_buf & OTP_STATUS_OTP_PROG_DONE_BIT_MASK))
	 {
	 break;
	 }
	 }*/

	HAL_Delay(2); //Uncomment this command if you don't want to use the loop above. It will take more time than the loop above.

	// Stop prog mode
	__dwt_otp_write_wdata_id_reg(0x003a);
	__dwt_otp_write_wdata_id_reg(0x0102);
	dwt_write16bitoffsetreg(OTP_WDATA_ID, 0, 0x0002); // Different to previous one
	dwt_write16bitoffsetreg(OTP_WDATA_ID, 0, 0x0000);

	// configure mode for reading
	dwt_write16bitoffsetreg(OTP_CFG_ID, 0, 0x0000);

	// Restore LDO tune register
	dwt_write32bitoffsetreg(LDO_TUNE_HI_ID, 0, ldo_tune);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to program 32-bit value into the DW3000 OTP memory.
 *
 * input parameters
 * @param value - this is the 32-bit value to be programmed into OTP
 * @param address - this is the 16-bit OTP address into which the 32-bit value is programmed
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_otpwriteandverify(uint32_t value, uint16_t address) {
	//program the word
	_dwt_otpprogword32(value, address);

	//check it is programmed correctly
	if (_dwt_otpread(address) == value) {
		return DWT_SUCCESS;
	} else {
		return DWT_ERROR;
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function puts the device into deep sleep or sleep. dwt_configuresleep() should be called first
 * to configure the sleep and on-wake/wake-up parameters
 *
 * input parameters
 * @param idle_rc - if this is set to DWT_DW_IDLE_RC, the auto INIT2IDLE bit will be cleared prior to going to sleep
 *                  thus after wake-up device will stay in IDLE_RC state
 *
 * output parameters
 *
 * no return value
 */
void dwt_entersleep(int idle_rc) {
	//clear auto INIT2IDLE bit if required
	if (idle_rc == DWT_DW_IDLE_RC) {
		dwt_and8bitoffsetreg(SEQ_CTRL_ID, 0x1,
				(uint8_t) ~(SEQ_CTRL_AINIT2IDLE_BIT_MASK>>8));
	}

	// Copy config to AON - upload the new configuration
	dwt_write8bitoffsetreg(AON_CTRL_ID, 0, 0);
	dwt_write8bitoffsetreg(AON_CTRL_ID, 0, AON_CTRL_ARRAY_SAVE_BIT_MASK);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief sets the sleep counter to new value, this function programs the sleep counter top 16-bits [27:12]
 *
 * NOTE: this function needs to be run before dwt_configuresleep
 *
 * input parameters
 * @param sleepcnt - this it value of the sleep counter to program
 *
 * output parameters
 *
 * no return value
 */
void dwt_configuresleepcnt(uint16_t sleepcnt) {

	dwt_aon_write(AON_SLPCNT_LO, (uint8_t) (sleepcnt));
	dwt_aon_write(AON_SLPCNT_HI, (uint8_t) (sleepcnt >> 8));

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief calibrates the local oscillator as its frequency can vary between 15 and 34kHz depending on temp and voltage
 *
 * NOTE: this function needs to be run before dwt_configuresleepcnt, so that we know what the counter units are
 *
 * input parameters
 *
 * output parameters
 *
 * returns the number of XTAL cycles per low-power oscillator cycle. LP OSC frequency = 38.4 MHz/return value
 */
uint16_t dwt_calibratesleepcnt(void) {
	uint16_t temp = 0;

	// Enable VDDPLL for reference clock
	dwt_or8bitoffsetreg(LDO_CTRL_ID, 0, LDO_CTRL_LDO_VDDPLL_EN_BIT_MASK);
	// Clear any previous cal settings
	dwt_aon_write(AON_SLPCNT_CAL_CTRL, 0x00);
	// Run cal
	dwt_aon_write(AON_SLPCNT_CAL_CTRL, 0x04);
	HAL_Delay(2); //need to wait for at least 1 LP OSC period at slowest frequency of 15kHz =~ 66 us
	// Read the Cal value from AON
	temp = dwt_aon_read(AON_SLPCNT_CAL_LO);
	temp = (uint16_t) ((temp) | (dwt_aon_read(AON_SLPCNT_CAL_HI) << 8));
	// Clear cal
	dwt_aon_write(AON_SLPCNT_CAL_CTRL, 0x00);
	// Disable VDDPLL for reference clock
	dwt_and8bitoffsetreg(LDO_CTRL_ID, 0,
			(uint8_t)~LDO_CTRL_LDO_VDDPLL_EN_BIT_MASK);
	return (temp);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief configures the device for both DEEP_SLEEP and SLEEP modes, and on-wake mode
 * i.e. before entering the sleep, the device should be programmed for TX or RX, then upon "waking up" the TX/RX settings
 * will be preserved and the device can immediately perform the desired action TX/RX
 *
 * NOTE: e.g. Tag operation - after deep sleep, the device needs to just load the TX buffer and send the frame
 *
 *
 *      mode:
 *      DWT_PGFCAL       0x0800
 *      DWT_GOTORX       0x0200
 *      DWT_GOTOIDLE     0x0100
 *      DWT_SEL_OPS      0x0040 | 0x0080
 *      DWT_LOADOPS      0x0020
 *      DWT_LOADLDO      0x0010
 *      DWT_LOADDGC      0x0008
 *      DWT_LOADBIAS     0x0004
 *      DWT_RUNSAR       0x0002
 *      DWT_CONFIG       0x0001 - download the AON array into the HIF (configuration download)
 *
 *      wake: wake up parameters
 *      DWT_SLP_CNT_RPT  0x40 - sleep counter loop after expiration
 *      DWT_PRESRVE_SLP  0x20 - allows for SLEEP_EN bit to be "preserved", although it will self-clear on wake up
 *      DWT_WAKE_WK      0x10 - wake up on WAKEUP PIN
 *      DWT_WAKE_CS      0x8 - wake up on chip select
 *      DWT_BR_DET       0x4 - enable brownout detector during sleep/deep sleep
 *      DWT_SLEEP        0x2 - enable sleep
 *      DWT_SLP_EN       0x1 - enable sleep/deep sleep functionality
 *
 * input parameters
 * @param mode - config on-wake parameters
 * @param wake - config wake up parameters
 *
 * output parameters
 *
 * no return value
 */
void dwt_configuresleep(uint16_t mode, uint8_t wake) {
	// Add predefined sleep settings before writing the mode
	pdw3000local->sleep_mode |= mode;
	dwt_write16bitoffsetreg(AON_DIG_CFG_ID, 0, pdw3000local->sleep_mode);

	dwt_write8bitoffsetreg(ANA_CFG_ID, 0, wake); //bit 0 - SLEEP_EN, bit 1 - DEEP_SLEEP=0/SLEEP=1, bit 3 wake on CS
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function clears the AON configuration in DW3000
 *
 * input parameters:
 *
 * output parameters
 *
 * no return value
 */
void dwt_clearaonconfig(void) {
	// Clear any AON auto download bits (as reset will trigger AON download)
	dwt_write16bitoffsetreg(AON_DIG_CFG_ID, 0, 0x00);
	// Clear the wake-up configuration
	dwt_write8bitoffsetreg(ANA_CFG_ID, 0, 0x00);
	// Upload the new configuration
	// Copy config to AON - upload the new configuration
	dwt_write8bitoffsetreg(AON_CTRL_ID, 0, 0);
	dwt_write8bitoffsetreg(AON_CTRL_ID, 0, AON_CTRL_ARRAY_SAVE_BIT_MASK);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief sets the auto TX to sleep bit. This means that after a frame
 * transmission the device will enter deep sleep mode. The dwt_configuresleep() function
 * needs to be called before this to configure the on-wake settings
 *
 * NOTE: the IRQ line has to be low/inactive (i.e. no pending events)
 *
 * input parameters
 * @param enable - 1 to configure the device to enter deep sleep after TX, 0 - disables the configuration
 *
 * output parameters
 *
 * no return value
 */
void dwt_entersleepaftertx(int enable) {
	// Set the auto TX -> sleep bit
	if (enable) {
		dwt_or16bitoffsetreg(SEQ_CTRL_ID, 0, SEQ_CTRL_ATX2SLP_BIT_MASK);
	} else {
		dwt_and16bitoffsetreg(SEQ_CTRL_ID, 0,
				(uint16_t)~SEQ_CTRL_ATX2SLP_BIT_MASK);
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this reads the device ID and checks if it is the right one
 *
 * input parameters
 * None
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_check_dev_id(void) {
	uint32_t dev_id;
	dev_id = dwt_readdevid();

	if (!((DWT_C0_PDOA_DEV_ID == dev_id) || (DWT_C0_DEV_ID == dev_id))) {
		return DWT_ERROR;
	}

	return DWT_SUCCESS;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function enables CIA diagnostic data. When turned on the following registers will be logged:
 * IP_TOA_LO, IP_TOA_HI, STS_TOA_LO, STS_TOA_HI, STS1_TOA_LO, STS1_TOA_HI, CIA_TDOA_0, CIA_TDOA_1_PDOA, CIA_DIAG_0, CIA_DIAG_1
 *
 * input parameters
 * @param enable_mask :     DW_CIA_DIAG_LOG_MAX (0x8)   //CIA to copy to swinging set a maximum set of diagnostic registers in Double Buffer mode
 *                          DW_CIA_DIAG_LOG_MID (0x4)   //CIA to copy to swinging set a medium set of diagnostic registers in Double Buffer mode
 *                          DW_CIA_DIAG_LOG_MIN (0x2)   //CIA to copy to swinging set a minimal set of diagnostic registers in Double Buffer mode
 *                          DW_CIA_DIAG_LOG_ALL (0x1)   //CIA to log all diagnostic registers
 *                          DW_CIA_DIAG_LOG_MIN (0x0)   //CIA to log reduced set of diagnostic registers
 *
 * output parameters
 *
 * no return value
 */
void dwt_configciadiag(uint8_t enable_mask) {
	if (enable_mask & DW_CIA_DIAG_LOG_ALL) {
		dwt_and8bitoffsetreg(CIA_CONF_ID, 2, (uint8_t)~(CIA_DIAGNOSTIC_OFF));
	} else {
		dwt_or8bitoffsetreg(CIA_CONF_ID, 2, CIA_DIAGNOSTIC_OFF);
	}

	dwt_write8bitoffsetreg(RDB_DIAG_MODE_ID, 0, enable_mask >> 1);

	pdw3000local->cia_diagnostic = enable_mask;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This call enables the auto-ACK feature. If the responseDelayTime (parameter) is 0, the ACK will be sent a.s.a.p.
 * otherwise it will be sent with a programmed delay (in symbols), max is 255.
 * NOTE: needs to have frame filtering enabled as well
 *
 * input parameters
 * @param responseDelayTime - if non-zero the ACK is sent after this delay, max is 255.
 * @param enable - enables or disables the auto-ACK feature
 *
 * output parameters
 *
 * no return value
 */
void dwt_enableautoack(uint8_t responseDelayTime, int enable) {
	// Set auto ACK reply delay
	dwt_write8bitoffsetreg(ACK_RESP_ID, 3, responseDelayTime); // In symbols

	// Enable AUTO ACK
	if (enable) {
		dwt_or32bitoffsetreg(SYS_CFG_ID, 0,
				SYS_CFG_AUTO_ACK_BIT_MASK | SYS_CFG_FAST_AAT_EN_BIT_MASK); //set the AUTO_ACK bit
	} else {
		dwt_and16bitoffsetreg(SYS_CFG_ID, 0,
				(uint16_t)(~SYS_CFG_AUTO_ACK_BIT_MASK)); //clear the AUTO_ACK bit
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This API sends issues a command to the device that the specific RX buff is free for frame reception,
 * it will also update the dblbuffon flag/status to the next buffer
 *
 * input parameters
 * @param None
 *
 * output parameters
 *
 * no return value
 */
void dwt_signal_rx_buff_free(void) {
	dwt_writefastCMD(CMD_DB_TOGGLE);

	//update the status
	if (pdw3000local->dblbuffon == DBL_BUFF_ACCESS_BUFFER_1) {
		pdw3000local->dblbuffon = DBL_BUFF_ACCESS_BUFFER_0; //next buffer is RX_BUFFER_0
	} else {
		pdw3000local->dblbuffon = DBL_BUFF_ACCESS_BUFFER_1; //next buffer is RX_BUFFER_1
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This call enables the double receive buffer mode
 *
 * input parameters
 * @param dbl_buff_state - enum variable for enabling/disabling double buffering mode
 * @param dbl_buff_mode - enum variable for Receiver Auto-Re-enable
 *
 * output parameters
 *
 * no return value
 */
void dwt_setdblrxbuffmode(dwt_dbl_buff_state_e dbl_buff_state,
		dwt_dbl_buff_mode_e dbl_buff_mode) {
	uint32_t or_val = 0, and_val = (uint32_t) -1;

	if (dbl_buff_state == DBL_BUF_STATE_EN) {
		and_val = ~(SYS_CFG_DIS_DRXB_BIT_MASK);
		pdw3000local->dblbuffon = DBL_BUFF_ACCESS_BUFFER_0; //the host will access RX_BUFFER_0 initially (on 1st reception after enable)
		//Updating indirect address here to save time setting it inside the interrupt(in order to read BUF1_RX_FINFO)..
		//Pay attention that after sleep, this register needs to be set again.
		dwt_write32bitreg(INDIRECT_ADDR_B_ID, (BUF1_RX_FINFO >> 16));
		dwt_write32bitreg(ADDR_OFFSET_B_ID, BUF1_RX_FINFO & 0xffff);
	} else {
		or_val = SYS_CFG_DIS_DRXB_BIT_MASK;
		pdw3000local->dblbuffon = DBL_BUFF_OFF;
	}

	if (dbl_buff_mode == DBL_BUF_MODE_AUTO) {
		or_val |= SYS_CFG_RXAUTR_BIT_MASK;
	} else {
		and_val &= (~SYS_CFG_RXAUTR_BIT_MASK); //Clear the needed bit
	}

	dwt_and_or32bitoffsetreg(SYS_CFG_ID, 0, and_val, or_val);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This sets the receiver turn on delay time after a transmission of a frame
 *
 * input parameters
 * @param rxDelayTime - (20 bits) - the delay is in UWB microseconds
 *
 * output parameters
 *
 * no return value
 */
void dwt_setrxaftertxdelay(uint32_t rxDelayTime) {
	uint32_t val = dwt_read32bitreg(ACK_RESP_ID); // Read ACK_RESP_T_ID register

	val &= (~ACK_RESP_W4R_TIM_BIT_MASK); // Clear the timer (19:0)

	val |= (rxDelayTime & ACK_RESP_W4R_TIM_BIT_MASK); // In UWB microseconds (e.g. turn the receiver on 20uus after TX)

	dwt_write32bitoffsetreg(ACK_RESP_ID, 0, val);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function is used to register the different callbacks called when one of the corresponding event occurs.
 *
 * NOTE: Callbacks can be undefined (set to NULL). In this case, dwt_isr() will process the event as usual but the 'null'
 * callback will not be called.
 *
 * input parameters
 * @param cbTxDone - the pointer to the TX confirmation event callback function
 * @param cbRxOk - the pointer to the RX good frame event callback function
 * @param cbRxTo - the pointer to the RX timeout events callback function
 * @param cbRxErr - the pointer to the RX error events callback function
 * @param cbSPIErr - the pointer to the SPI error events callback function
 * @param cbSPIRdy - the pointer to the SPI ready events callback function
 *
 * output parameters
 *
 * no return value
 */
void dwt_setcallbacks(dwt_cb_t cbTxDone, dwt_cb_t cbRxOk, dwt_cb_t cbRxTo,
		dwt_cb_t cbRxErr, dwt_cb_t cbSPIErr, dwt_cb_t cbSPIRdy) {
	pdw3000local->cbTxDone = cbTxDone;
	pdw3000local->cbRxOk = cbRxOk;
	pdw3000local->cbRxTo = cbRxTo;
	pdw3000local->cbRxErr = cbRxErr;
	pdw3000local->cbSPIErr = cbSPIErr;
	pdw3000local->cbSPIRdy = cbSPIRdy;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function checks if the IRQ line is active - this is used instead of interrupt handler
 *
 * input parameters
 *
 * output parameters
 *
 * return value is 1 if the IRQS bit is set and 0 otherwise
 */
uint8_t dwt_checkirq(void) {
	/* Reading the lower byte only is enough for this operation */
	return (dwt_read8bitoffsetreg(SYS_STATUS_ID, 0) & SYS_STATUS_IRQS_BIT_MASK);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function checks if the DW3000 is in IDLE_RC state
 *
 * input parameters
 *
 * output parameters
 *
 * return value is 1 if the IDLE_RC bit is set and 0 otherwise
 */
uint8_t dwt_checkidlerc(void) {
	//HAL_Delay(2); /* wait 2 ms for DW IC to get into IDLE_RC state */
	/* Poll DW IC until IDLE_RC event set. This means that DW IC is in IDLE_RC state and ready */
	uint32_t reg = 0;
	reg = ((uint32_t) dwt_read16bitoffsetreg(SYS_STATUS_ID, 2) << 16);

	return ((reg & (SYS_STATUS_RCINIT_BIT_MASK)) == (SYS_STATUS_RCINIT_BIT_MASK));
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is the DW3000's general Interrupt Service Routine. It will process/report the following events:
 *          - RXFR + no data mode (through cbRxOk callback, but set datalength to 0)
 *          - RXFCG (through cbRxOk callback)
 *          - TXFRS (through cbTxDone callback)
 *          - RXRFTO/RXPTO (through cbRxTo callback)
 *          - RXPHE/RXFCE/RXRFSL/RXSFDTO/AFFREJ/LDEERR/LCSSERR (through cbRxTo cbRxErr)
 *          -
 *        For all events, corresponding interrupts are cleared and necessary resets are performed. In addition, in the RXFCG case,
 *        received frame information and frame control are read before calling the callback. If double buffering is activated, it
 *        will also toggle between reception buffers once the reception callback processing has ended.
 *
 *        /!\ This version of the ISR supports double buffering but does not support automatic RX re-enabling!
 *
 * NOTE:  In PC based system using (Cheetah or ARM) USB to SPI converter there can be no interrupts, however we still need something
 *        to take the place of it and operate in a polled way. In an embedded system this function should be configured to be triggered
 *        on any of the interrupts described above.

 * input parameters
 *
 * output parameters
 *
 * no return value
 */

void dwt_isr(void) {

	//Read Fast Status register
	uint8_t fstat = dwt_read8bitoffsetreg(FINT_STAT_ID, 0);
	uint32_t status = dwt_read32bitreg(SYS_STATUS_ID); // Read status register low 32bits
	pdw3000local->cbData.status = status;
	if ((pdw3000local->stsconfig & DWT_STS_MODE_ND) == DWT_STS_MODE_ND) //cannot use FSTAT when in no data mode...
	{

		if (status & SYS_STATUS_RXFR_BIT_MASK) {
			fstat |= FINT_STAT_RXOK_BIT_MASK;
		}
	}
	// Handle System panic confirmation event
	// AES_ERR|SPICRCERR|BRNOUT|SPI_UNF|SPI_OVR|CMD_ERR|SPI_COLLISION|PLLHILO
	if (fstat & FINT_STAT_SYS_PANIC_BIT_MASK) {
		pdw3000local->cbData.status_hi = dwt_read16bitoffsetreg(
				SYS_STATUS_HI_ID, 0);

		// Handle SPI CRC error event, which was due to an SPI write CRC error
		// Handle SPI error events (if this has happened, the last SPI transaction has not completed correctly, the device should be reset)
		if ((pdw3000local->spicrc
				&& (pdw3000local->cbData.status & SYS_STATUS_SPICRCE_BIT_MASK))
				|| (pdw3000local->cbData.status_hi
						& (SYS_STATUS_HI_SPIERR_BIT_MASK
								| SYS_STATUS_HI_SPI_UNF_BIT_MASK
								| SYS_STATUS_HI_SPI_OVF_BIT_MASK))) {
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_SPICRCE_BIT_MASK);
			dwt_write16bitoffsetreg(SYS_STATUS_HI_ID, 0,
					(SYS_STATUS_HI_SPIERR_BIT_MASK
							| SYS_STATUS_HI_SPI_UNF_BIT_MASK
							| SYS_STATUS_HI_SPI_OVF_BIT_MASK)); // Clear SPI error event bits
			// Call the corresponding callback if present
			if (pdw3000local->cbSPIErr != NULL) {
				pdw3000local->cbSPIErr(&pdw3000local->cbData);
			}
		}

		// Handle Fast CMD errors event, the means the last CMD did not execute (e.g. it was given while device was already executing previous)
		if (pdw3000local->cbData.status_hi & SYS_STATUS_HI_CMD_ERR_BIT_MASK) {
			dwt_write16bitoffsetreg(SYS_STATUS_HI_ID, 0,
					SYS_STATUS_HI_CMD_ERR_BIT_MASK); // Clear CMD error event bit
			// Call the corresponding callback if present
			/*if(pdw3000local->cbCMDErr != NULL)
			 {
			 pdw3000local->cbCMDErr(&pdw3000local->cbData);
			 }*/
		}

		//AES_ERR, BRNOUT, PLLHILO not handled here ...
	}

	// Handle TX frme sent confirmation event
	if (fstat & FINT_STAT_TXOK_BIT_MASK) {
		// Clear TX events after the callback - this lets the host schedule another TX/RX inside the callback
		dwt_write8bitoffsetreg(SYS_STATUS_ID, 0, (uint8_t) SYS_STATUS_ALL_TX); // Clear TX event bits to clear the interrupt

		// Call the corresponding callback if present
		if (pdw3000local->cbTxDone != NULL) {
			pdw3000local->cbTxDone(&pdw3000local->cbData);
		}

	}

	// SPI ready and IDLE_RC bit gets set when device powers on, or on wake up
	if (fstat & FINT_STAT_SYS_EVENT_BIT_MASK) {
		//pdw3000local->cbData.status_hi = dwt_read16bitreg(SYS_STATUS_HI_ID);

		// Call the corresponding callback if present
		if (pdw3000local->cbSPIRdy != NULL) {
			pdw3000local->cbSPIRdy(&pdw3000local->cbData);
		}
		// Clear SPI RDY events after the callback - this lets the host read the SYS_STATUS register inside the callback
		dwt_write16bitoffsetreg(SYS_STATUS_ID, 2,
				(uint16_t) ((SYS_STATUS_RCINIT_BIT_MASK
						| SYS_STATUS_SPIRDY_BIT_MASK) >> 16)); // Clear the bit to clear the interrupt

		//VTDET, GPIO, not handled here ...
	}

	// Handle RX ok events
	if (fstat & FINT_STAT_RXOK_BIT_MASK) {
		uint32_t cia_err = 0;

		pdw3000local->cbData.rx_flags = 0;

		if (pdw3000local->dblbuffon) // if in double buffer mode
		{
			uint8_t statusDB = dwt_read8bitoffsetreg(RDB_STATUS_ID, 0);

			if (pdw3000local->dblbuffon == DBL_BUFF_ACCESS_BUFFER_1) //If accessing the second buffer (RX_BUFFER_B then read second nibble of the DB status reg)
			{
				statusDB >>= 4;
			}
			//setting the relevant bits in the main status register according to DB status register
			if (statusDB & RDB_STATUS_RXFCG0_BIT_MASK)
				status |= SYS_STATUS_RXFCG_BIT_MASK;
			if (statusDB & RDB_STATUS_RXFR0_BIT_MASK)
				status |= SYS_STATUS_RXFR_BIT_MASK;
			if (statusDB & RDB_STATUS_CIADONE0_BIT_MASK)
				status |= SYS_STATUS_CIADONE_BIT_MASK;
		}
		//update the status based on the DB RX events
		pdw3000local->cbData.status = status;
		//clear LDE error (as we do not want to go back into cbRxErr)
		if (status & SYS_STATUS_CIAERR_BIT_MASK) {
			pdw3000local->cbData.rx_flags |= DWT_CB_DATA_RX_FLAG_CER;
			cia_err = SYS_STATUS_CIAERR_BIT_MASK;
		} else {
			if (status & SYS_STATUS_CIADONE_BIT_MASK) {
				pdw3000local->cbData.rx_flags |= DWT_CB_DATA_RX_FLAG_CIA;
			}
		}

		if (status & SYS_STATUS_CPERR_BIT_MASK) {
			pdw3000local->cbData.rx_flags |= DWT_CB_DATA_RX_FLAG_CPER;
			cia_err |= SYS_STATUS_CPERR_BIT_MASK;
		}

		// When using No Data STS mode we do not get RXFCG but RXFR
		if ((status & SYS_STATUS_RXFR_BIT_MASK)
				&& ((pdw3000local->stsconfig & DWT_STS_MODE_ND)
						== DWT_STS_MODE_ND)) {
			pdw3000local->cbData.rx_flags |= DWT_CB_DATA_RX_FLAG_ND;
			pdw3000local->cbData.datalength = 0;

			cia_err |= SYS_STATUS_RXFCE_BIT_MASK; // Clear FCE

		} else
		// Handle RX good frame event
		if (status & SYS_STATUS_RXFCG_BIT_MASK) {
			uint16_t finfo16;

			// Read frame info - Only the first two bytes of the register are used here.
			switch (pdw3000local->dblbuffon) //check if in double buffer mode and if so which buffer host is currently accessing
			{
			case DBL_BUFF_ACCESS_BUFFER_1: //accessing frame info relating to the second buffer (RX_BUFFER_1)
				dwt_write8bitoffsetreg(RDB_STATUS_ID, 0,
						RDB_STATUS_CLEAR_BUFF1_EVENTS); //clear DB status register bits corresponding to RX_BUFFER_1
				finfo16 = dwt_read16bitoffsetreg(INDIRECT_POINTER_B_ID, 0);
				break;
			case DBL_BUFF_ACCESS_BUFFER_0: //accessing frame info relating to the first buffer (RX_BUFFER_0)
				dwt_write8bitoffsetreg(RDB_STATUS_ID, 0,
						RDB_STATUS_CLEAR_BUFF0_EVENTS); //clear DB status register bits corresponding to RX_BUFFER_0
				finfo16 = dwt_read16bitoffsetreg(BUF0_RX_FINFO, 0);
				break;
			default: //accessing frame info relating to the second buffer (RX_BUFFER_0) (single buffer mode)
				finfo16 = dwt_read16bitoffsetreg(RX_FINFO_ID, 0);
				break;
			}

			// Report frame length - Standard frame length up to 127, extended frame length up to 1023 bytes
			if (pdw3000local->longFrames == 0) {
				pdw3000local->cbData.datalength = finfo16
						& RX_FINFO_STD_RXFLEN_MASK;
			} else {
				pdw3000local->cbData.datalength = finfo16
						& RX_FINFO_RXFLEN_BIT_MASK;
			}

			// Report ranging bit
			if (finfo16 & RX_FINFO_RNG_BIT_MASK) {
				pdw3000local->cbData.rx_flags |= DWT_CB_DATA_RX_FLAG_RNG;
			}

		}

		dwt_write32bitreg(SYS_STATUS_ID, cia_err | SYS_STATUS_ALL_RX_GOOD); // Clear all status bits relating to good reception

		// Call the corresponding callback if present
		if (pdw3000local->cbRxOk != NULL) {
			pdw3000local->cbRxOk(&pdw3000local->cbData);
		}

		if (pdw3000local->dblbuffon) //check if in double buffer mode and if so which buffer host is currently accessing
		{
			// Free up the current buffer - let the device know that it can receive into this buffer again
			dwt_signal_rx_buff_free();
		}

	}

	// RXFCE&~DISFCE|RXPHE|RXFSL|ARFE|RXSTO|RXOVRR. Real errored frame received, so ignore FCE if disabled
	// Handle RX errors events
	if (fstat & FINT_STAT_RXERR_BIT_MASK) {
		// Clear RX error events before the callback - this lets the host renable the receiver inside the callback
		dwt_write32bitoffsetreg(SYS_STATUS_ID, 0, SYS_STATUS_ALL_RX_ERR); // Clear RX error event bits

		// Call the corresponding callback if present
		if (pdw3000local->cbRxErr != NULL) {
			pdw3000local->cbRxErr(&pdw3000local->cbData);
		}

	}

	// Handle RX Timeout event (PTO and FWTO)
	if (fstat & FINT_STAT_RXTO_BIT_MASK) {
		// Clear RX TO events before the callback - this lets the host renable the receiver inside the callback
		dwt_write8bitoffsetreg(SYS_STATUS_ID, 2,
				(uint8_t) (SYS_STATUS_ALL_RX_TO >> 16)); // Clear RX timeout event bits (PTO, RFTO)

		// Call the corresponding callback if present
		if (pdw3000local->cbRxTo != NULL) {
			pdw3000local->cbRxTo(&pdw3000local->cbData);
		}

	}

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to set up Tx/Rx GPIOs which could be used to control LEDs
 * Note: not completely IC dependent, also needs board with LEDS fitted on right I/O lines
 *       this function enables GPIOs 2 and 3 which are connected to LED3 and LED4 on EVB1000
 *
 * input parameters
 * @param mode - this is a bit field interpreted as follows:
 *          - bit 0: 1 to enable LEDs, 0 to disable them
 *          - bit 1: 1 to make LEDs blink once on init. Only valid if bit 0 is set (enable LEDs)
 *          - bit 2 to 7: reserved
 *
 * output parameters none
 *
 * no return value
 */
void dwt_setleds(uint8_t mode) {
	uint32_t reg;
	if (mode & DWT_LEDS_ENABLE) {
		// Set up MFIO for LED output.
		dwt_modify32bitoffsetreg(GPIO_MODE_ID, 0,
				~(GPIO_MODE_MSGP3_MODE_BIT_MASK | GPIO_MODE_MSGP2_MODE_BIT_MASK),
				(GPIO_PIN2_RXLED | GPIO_PIN3_TXLED));

		// Enable LP Oscillator to run from counter and turn on de-bounce clock.
		dwt_or32bitoffsetreg(CLK_CTRL_ID, 0,
				(CLK_CTRL_GPIO_DCLK_EN_BIT_MASK | CLK_CTRL_LP_CLK_EN_BIT_MASK));

		// Enable LEDs to blink and set default blink time.
		reg = LED_CTRL_BLINK_EN_BIT_MASK | DWT_LEDS_BLINK_TIME_DEF;
		// Make LEDs blink once if requested.
		if (mode & DWT_LEDS_INIT_BLINK) {
			reg |= LED_CTRL_FORCE_TRIGGER_BIT_MASK;
		}
		dwt_write32bitreg(LED_CTRL_ID, reg);
		// Clear force blink bits if needed.
		if (mode & DWT_LEDS_INIT_BLINK) {
			reg &= (~LED_CTRL_FORCE_TRIGGER_BIT_MASK);
			dwt_write32bitreg(LED_CTRL_ID, reg);
		}
	} else {
		// Clear the GPIO bits that are used for LED control.
		dwt_and32bitoffsetreg(GPIO_MODE_ID, 0,
				~(GPIO_MODE_MSGP2_MODE_BIT_MASK | GPIO_MODE_MSGP3_MODE_BIT_MASK));
		dwt_and16bitoffsetreg(LED_CTRL_ID, 0,
				(uint16_t) ~LED_CTRL_BLINK_EN_BIT_MASK);
	}

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief function to enable/disable clocks to particular digital blocks/system
 *
 * input parameters
 * @param clocks - set of clocks to enable/disable
 *
 * output parameters none
 *
 * no return value
 */
static
void dwt_force_clocks(int clocks) {

	if (clocks == FORCE_CLK_SYS_TX) {
		uint16_t regvalue0 = CLK_CTRL_TX_BUF_CLK_ON_BIT_MASK
				| CLK_CTRL_RX_BUF_CLK_ON_BIT_MASK;

		//SYS_CLK_SEL = PLL
		regvalue0 |= ((uint16_t) FORCE_SYSCLK_PLL)
				<< CLK_CTRL_SYS_CLK_SEL_BIT_OFFSET;

		//TX_CLK_SEL = ON
		regvalue0 |= ((uint16_t) FORCE_CLK_PLL)
				<< CLK_CTRL_TX_CLK_SEL_BIT_OFFSET;

		dwt_write16bitoffsetreg(CLK_CTRL_ID, 0x0, regvalue0);

	}

	if (clocks == FORCE_CLK_AUTO) {
		//Restore auto clock mode
		dwt_write16bitoffsetreg(CLK_CTRL_ID, 0x0, (uint16_t) DWT_AUTO_CLKS); //we only need to restore the low 16 bits as they are the only ones to change as a result of  FORCE_CLK_SYS_TX
	}

} // end dwt_force_clocks()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This API function configures the reference time used for relative timing of delayed sending and reception.
 * The value is at a 8ns resolution.
 *
 * input parameters
 * @param reftime - the reference time (which together with DX_TIME or TX timestamp or RX timestamp time is used to define a
 * transmission time or delayed RX on time)
 *
 * output parameters none
 *
 * no return value
 */
void dwt_setreferencetrxtime(uint32_t reftime) {
	dwt_write32bitoffsetreg(DREF_TIME_ID, 0, reftime); // Note: bit 0 of this register is ignored
} // end dwt_setreferencetrxtime()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This API function configures the delayed transmit time or the delayed RX on time
 * The value is at a 8ns resolution.
 *
 * input parameters
 * @param starttime - the TX/RX start time (the 32 bits should be the high 32 bits of the system time at which to send the message,
 * or at which to turn on the receiver)
 *
 * output parameters none
 *
 * no return value
 */
void dwt_setdelayedtrxtime(uint32_t starttime) {
	dwt_write32bitoffsetreg(DX_TIME_ID, 0, starttime); // Note: bit 0 of this register is ignored
} // end dwt_setdelayedtrxtime()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This call initiates the transmission, input parameter indicates which TX mode is used see below
 *
 * input parameters:
 * @param mode - if mode = DWT_START_TX_IMMEDIATE - immediate TX (no response expected)
 *               if mode = DWT_START_TX_DELAYED - delayed TX (no response expected)  at specified time (time in DX_TIME register)
 *               if mode = DWT_START_TX_DLY_REF - delayed TX (no response expected)  at specified time (time in DREF_TIME register + any time in DX_TIME register)
 *               if mode = DWT_START_TX_DLY_RS  - delayed TX (no response expected)  at specified time (time in RX_TIME_0 register + any time in DX_TIME register)
 *               if mode = DWT_START_TX_DLY_TS  - delayed TX (no response expected)  at specified time (time in TX_TIME_LO register + any time in DX_TIME register)
 *               if mode = DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED - immediate TX (response expected - so the receiver will be automatically turned on after TX is done)
 *               if mode = DWT_START_TX_DELAYED/DLY_* | DWT_RESPONSE_EXPECTED - delayed TX (response expected - so the receiver will be automatically turned on after TX is done)
 *               if mode = DWT_START_TX_CCA - Send the frame if no preamble detected within PTO time
 *               if mode = DWT_START_TX_CCA  | DWT_RESPONSE_EXPECTED - Send the frame if no preamble detected within PTO time and then enable RX*
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error (e.g. a delayed transmission will be cancelled if the delayed time has passed)
 */
int dwt_starttx(uint8_t mode) {
	int retval = DWT_SUCCESS;
	uint16_t checkTxOK = 0;
	uint32_t sys_state;

	if ((mode & DWT_START_TX_DELAYED) || (mode & DWT_START_TX_DLY_REF)
			|| (mode & DWT_START_TX_DLY_RS) || (mode & DWT_START_TX_DLY_TS)) {
		if (mode & DWT_START_TX_DELAYED) //delayed TX
		{
			if (mode & DWT_RESPONSE_EXPECTED) {
				dwt_writefastCMD(CMD_DTX_W4R);
			} else {
				dwt_writefastCMD(CMD_DTX);
			}
		} else if (mode & DWT_START_TX_DLY_RS) //delayed TX WRT RX timestamp
		{
			if (mode & DWT_RESPONSE_EXPECTED) {
				dwt_writefastCMD(CMD_DTX_RS_W4R);
			} else {
				dwt_writefastCMD(CMD_DTX_RS);
			}
		} else if (mode & DWT_START_TX_DLY_TS) //delayed TX WRT TX timestamp
		{
			if (mode & DWT_RESPONSE_EXPECTED) {
				dwt_writefastCMD(CMD_DTX_TS_W4R);
			} else {
				dwt_writefastCMD(CMD_DTX_TS);
			}
		} else  //delayed TX WRT reference time
		{
			if (mode & DWT_RESPONSE_EXPECTED) {
				dwt_writefastCMD(CMD_DTX_REF_W4R);
			} else {
				dwt_writefastCMD(CMD_DTX_REF);
			}
		}

		checkTxOK = dwt_read8bitoffsetreg(SYS_STATUS_ID, 3); // Read at offset 3 to get the upper 2 bytes out of 5
		if ((checkTxOK & (SYS_STATUS_HPDWARN_BIT_MASK >> 24)) == 0) // Transmit Delayed Send set over Half a Period away.
				{
			sys_state = dwt_read32bitreg(SYS_STATE_LO_ID);
			if (sys_state == DW_SYS_STATE_TXERR) {
				//uart_transmit("TXE", 3);
				dwt_writefastCMD(CMD_TXRXOFF);
				retval = DWT_ERROR; // Failed !
			} else {
				retval = DWT_SUCCESS; // All okay
			}
		} else {
			//uart_transmit("HPDWARN", 7);
			dwt_writefastCMD(CMD_TXRXOFF);
			retval = DWT_ERROR; // Failed !

			//optionally could return error, and still send the frame at indicated time
			//then if the application want to cancel the sending this can be done in a separate command.
		}
	} else if (mode & DWT_START_TX_CCA) {
		if (mode & DWT_RESPONSE_EXPECTED) {
			dwt_writefastCMD(CMD_CCA_TX_W4R);
		} else {
			dwt_writefastCMD(CMD_CCA_TX);
		}
	} else {
		if (mode & DWT_RESPONSE_EXPECTED) {
			dwt_writefastCMD(CMD_TX_W4R);
		} else {
			dwt_writefastCMD(CMD_TX);
		}
	}

	return retval;

} // end dwt_starttx()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to turn off the transceiver
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void dwt_forcetrxoff(void) {
	decaIrqStatus_t stat;
	// Need to beware of interrupts occurring in the middle of following command cycle
	// We can disable the radio, but before the status is cleared an interrupt can be set (e.g. the
	// event has just happened before the radio was disabled)
	// thus we need to disable interrupt during this operation
	stat = decamutexon();

	dwt_writefastCMD(CMD_TXRXOFF);

	// Enable/restore interrupts again...
	decamutexoff(stat);
} // end deviceforcetrxoff()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief enable/disable and configure SNIFF mode.
 *
 * SNIFF mode is a low-power reception mode where the receiver is sequenced on and off instead of being on all the time.
 * The time spent in each state (on/off) is specified through the parameters below.
 * See DW3000 User Manual section 4.5 "Low-Power SNIFF mode" for more details.
 *
 * input parameters:
 * @param enable - 1 to enable SNIFF mode, 0 to disable. When 0, all other parameters are not taken into account.
 * @param timeOn - duration of receiver ON phase, expressed in multiples of PAC size. The counter automatically adds 1 PAC
 *                 size to the value set. Min value that can be set is 1 (i.e. an ON time of 2 PAC size), max value is 15.
 * @param timeOff - duration of receiver OFF phase, expressed in multiples of 128/125 �s (~1 �s). Max value is 255.
 *
 * output parameters
 *
 * no return value
 */
void dwt_setsniffmode(int enable, uint8_t timeOn, uint8_t timeOff) {
	if (enable) {
		/* Configure ON/OFF times and enable PLL2 on/off sequencing by SNIFF mode. */
		uint16_t sniff_reg = (((uint16_t) timeOff << 8) | timeOn)
				& (RX_SNIFF_SNIFF_OFF_BIT_MASK | RX_SNIFF_SNIFF_ON_BIT_MASK);
		dwt_write16bitoffsetreg(RX_SNIFF_ID, 0, sniff_reg);
	} else {
		/* Clear ON/OFF times and disable PLL2 on/off sequencing by SNIFF mode. */
		dwt_write16bitoffsetreg(RX_SNIFF_ID, 0, 0x0000);
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This call turns on the receiver, can be immediate or delayed (depending on the mode parameter). In the case of a
 * "late" error the receiver will only be turned on if the DWT_IDLE_ON_DLY_ERR is not set.
 * The receiver will stay turned on, listening to any messages until
 * it either receives a good frame, an error (CRC, PHY header, Reed Solomon) or  it times out (SFD, Preamble or Frame).
 *
 * input parameters
 * @param mode - this can be one of the following allowed values:
 *
 * DWT_START_RX_IMMEDIATE      0x00    Enable the receiver immediately
 * DWT_START_RX_DELAYED        0x01    Set up delayed RX, if "late" error triggers, then the RX will be enabled immediately
 * DWT_IDLE_ON_DLY_ERR         0x02    If delayed RX failed due to "late" error then if this
 flag is set the RX will not be re-enabled immediately, and device will be in IDLE when function exits
 * DWT_START_RX_DLY_REF        0x04    Enable the receiver at specified time (time in DREF_TIME register + any time in DX_TIME register)
 * DWT_START_RX_DLY_RS         0x08    Enable the receiver at specified time (time in RX_TIME_0 register + any time in DX_TIME register)
 * DWT_START_RX_DLY_TS         0x10    Enable the receiver at specified time (time in TX_TIME_LO register + any time in DX_TIME register)

 * e.g.
 * (DWT_START_RX_DELAYED | DWT_IDLE_ON_DLY_ERR) 0x03 used to disable re-enabling of receiver if delayed RX failed due to "late" error
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error (e.g. a delayed receive enable will be too far in the future if delayed time has passed)
 */
int dwt_rxenable(int mode) {
	uint8_t temp1;

	if (mode == DWT_START_RX_IMMEDIATE) {
		dwt_writefastCMD(CMD_RX);
	} else //delayed RX
	{
		switch (mode & ~DWT_IDLE_ON_DLY_ERR) {
		case DWT_START_RX_DELAYED:
			dwt_writefastCMD(CMD_DRX);
			break;
		case DWT_START_RX_DLY_REF:
			dwt_writefastCMD(CMD_DRX_REF);
			break;
		case DWT_START_RX_DLY_RS:
			dwt_writefastCMD(CMD_DRX_RS);
			break;
		case DWT_START_RX_DLY_TS:
			dwt_writefastCMD(CMD_DRX_TS);
			break;
		default:
			return DWT_ERROR; // return error
		}

		temp1 = dwt_read8bitoffsetreg(SYS_STATUS_ID, 3); // Read 1 byte at offset 3 to get the 4th byte out of 5
		if ((temp1 & (SYS_STATUS_HPDWARN_BIT_MASK >> 24)) != 0) // if delay has passed do immediate RX on unless DWT_IDLE_ON_DLY_ERR is true
				{
			dwt_writefastCMD(CMD_TXRXOFF);

			if ((mode & DWT_IDLE_ON_DLY_ERR) == 0) // if DWT_IDLE_ON_DLY_ERR not set then re-enable receiver
					{
				dwt_writefastCMD(CMD_RX);
			}
			return DWT_ERROR; // return warning indication
		}
	}

	return DWT_SUCCESS;
} // end dwt_rxenable()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This call enables RX timeout (SY_STAT_RFTO event)
 *
 * input parameters
 * @param time - how long the receiver remains on from the RX enable command
 *               The time parameter used here is in 1.0256 us (512/499.2MHz) units
 *               If set to 0 the timeout is disabled.
 *
 * output parameters
 *
 * no return value
 */
void dwt_setrxtimeout(uint32_t time) {
	if (time > 0) {
		dwt_write32bitoffsetreg(RX_FWTO_ID, 0, time);

		dwt_or16bitoffsetreg(SYS_CFG_ID, 0, SYS_CFG_RXWTOE_BIT_MASK); //set the RX FWTO bit
	} else {
		dwt_and16bitoffsetreg(SYS_CFG_ID, 0,
				(uint16_t)(~SYS_CFG_RXWTOE_BIT_MASK)); //clear the RX FWTO bit
	}
} // end dwt_setrxtimeout()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This call enables preamble timeout (SY_STAT_RXPTO event)
 *
 * input parameters
 * @param  timeout - Preamble detection timeout, expressed in multiples of PAC size. The counter automatically adds 1 PAC
 *                   size to the value set. Min value that can be set is 1 (i.e. a timeout of 2 PAC size).
 *
 * output parameters
 *
 * no return value
 */
void dwt_setpreambledetecttimeout(uint16_t timeout) {
	dwt_write16bitoffsetreg(DTUNE1_ID, 0, timeout);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function enables the specified events to trigger an interrupt.
 * The following events can be found in SYS_ENABLE_LO and SYS_ENABLE_HI registers.
 *
 *
 * input parameters:
 * @param bitmask_lo - sets the events in SYS_ENABLE_LO_ID register which will generate interrupt
 * @param bitmask_hi - sets the events in SYS_ENABLE_HI_ID register which will generate interrupt
 * @param operation  - if set to DWT_ENABLE_INT additional interrupts as selected in the bitmask are enabled
 *                   - if set to DWT_ENABLE_INT_ONLY the interrupts in the bitmask are forced to selected state -
 *                      i.e. the mask is written to the register directly.
 *                   - otherwise (if set to DWT_DISABLE_INT) clear the interrupts as selected in the bitmask
 * output parameters
 *
 * no return value
 */
void dwt_setinterrupt(uint32_t bitmask_lo, uint32_t bitmask_hi,
		dwt_INT_options_e INT_options) {
	decaIrqStatus_t stat;

	// Need to beware of interrupts occurring in the middle of following read modify write cycle
	stat = decamutexon();

	if (INT_options == DWT_ENABLE_INT_ONLY) {
		dwt_write32bitreg(SYS_ENABLE_LO_ID, bitmask_lo); // New value
		dwt_write32bitreg(SYS_ENABLE_HI_ID, bitmask_hi); // New value
	} else {
		if (INT_options == DWT_ENABLE_INT) {
			dwt_or32bitoffsetreg(SYS_ENABLE_LO_ID, 0, bitmask_lo); //Set the bits
			dwt_or32bitoffsetreg(SYS_ENABLE_HI_ID, 0, bitmask_hi); //Set the bits
		} else {
			dwt_and32bitoffsetreg(SYS_ENABLE_LO_ID, 0,
					(uint32_t )(~bitmask_lo)); // Clear the bits
			dwt_and32bitoffsetreg(SYS_ENABLE_HI_ID, 0,
					(uint32_t )(~bitmask_hi)); // Clear the bits
		}
	}

	decamutexoff(stat);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to enable/disable the event counter in the IC
 *
 * input parameters
 * @param - enable - 1 enables (and reset), 0 disables the event counters
 * output parameters
 *
 * no return value
 */
void dwt_configeventcounters(int enable) {
	// Need to clear and disable, can't just clear
	dwt_write8bitoffsetreg(EVC_CTRL_ID, 0x0,
			(uint8_t) (EVC_CTRL_EVC_CLR_BIT_MASK));

	if (enable) {
		dwt_write8bitoffsetreg(EVC_CTRL_ID, 0x0,
				(uint8_t) (EVC_CTRL_EVC_EN_BIT_MASK)); // Enable
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the event counters in the IC
 *
 * input parameters
 * @param counters - pointer to the dwt_deviceentcnts_t structure which will hold the read data
 *
 * output parameters
 *
 * no return value
 */
void dwt_readeventcounters(dwt_deviceentcnts_t *counters) {
	uint32_t temp;

	temp = dwt_read32bitoffsetreg(EVC_COUNT0_ID, 0); // Read sync loss (27-16), PHE (11-0)
	counters->PHE = temp & 0xFFF;
	counters->RSL = (temp >> 16) & 0xFFF;

	temp = dwt_read32bitoffsetreg(EVC_COUNT1_ID, 0); // Read CRC bad (27-16), CRC good (11-0)
	counters->CRCG = temp & 0xFFF;
	counters->CRCB = (temp >> 16) & 0xFFF;

	temp = dwt_read32bitoffsetreg(EVC_COUNT2_ID, 0); // Overruns (23-16), address errors (7-0)
	counters->ARFE = (uint8_t) temp;
	counters->OVER = (uint8_t) (temp >> 16);

	temp = dwt_read32bitoffsetreg(EVC_COUNT3_ID, 0); // Read PTO (27-16), SFDTO (11-0)
	counters->PTO = (temp >> 16) & 0xFFF;
	counters->SFDTO = temp & 0xFFF;

	temp = dwt_read32bitoffsetreg(EVC_COUNT4_ID, 0); // Read TXFRAME (27-16), RX TO (7-0)
	counters->TXF = (temp >> 16) & 0xFFF;
	counters->RTO = (uint8_t) temp;

	temp = dwt_read32bitoffsetreg(EVC_COUNT5_ID, 0); // Read half period warning (7-0) events
	counters->HPW = (uint8_t) temp;
	counters->CRCE = (uint8_t) (temp >> 16); // SPI CRC errors (23-16) warning events

	temp = dwt_read32bitoffsetreg(EVC_COUNT6_ID, 0); //Preamble rejections (11-0) events
	counters->PREJ = temp & 0xFFF;

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function resets the DW3000
 *
 * NOTE: SPI rate must be <= 7MHz before a call to this function as the device will use FOSC/4 as part of internal reset
 *
 * input parameters:
 *
 * output parameters
 *
 * no return value
 */
void dwt_softreset(void) {
	//clear any AON configurations (this will leave the device at FOSC/4, thus we need low SPI rate)
	dwt_clearaonconfig();

	//make sure the new AON array config has been set
	deca_sleep(1);

	//need to make sure clock is not PLL as the PLL will be switched off as part of reset
	dwt_or8bitoffsetreg(CLK_CTRL_ID, 0, FORCE_SYSCLK_FOSC);

	// Reset HIF, TX, RX and PMSC
	dwt_write8bitoffsetreg(SOFT_RST_ID, 0, DWT_RESET_ALL);

	// DW3000 needs a 10us sleep to let clk PLL lock after reset - the PLL will automatically lock after the reset
	// Could also have polled the PLL lock flag, but then the SPI needs to be <= 7MHz !! So a simple delay is easier
	deca_sleep(1);

	//reset buffer to process RX_BUFFER_0 next - if in double buffer mode (clear bit 1 if set)
	pdw3000local->dblbuffon = DBL_BUFF_ACCESS_BUFFER_0;
	pdw3000local->sleep_mode = 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to adjust the crystal frequency
 *
 * input parameters:
 * @param   value - crystal trim value (in range 0x0 to 0x3F) 64 steps  (~1.65ppm per step)
 *
 * output parameters
 *
 * no return value
 */
void dwt_setxtaltrim(uint8_t value) {
	uint8_t reg_val =
			((value & XTAL_TRIM_BIT_MASK) >> XTAL_XTAL_TRIM_BIT_OFFSET);
	pdw3000local->init_xtrim = reg_val;
	dwt_write8bitoffsetreg(XTAL_ID, 0, reg_val);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function returns the current value of XTAL trim that has been applied. Following dwt_initialise this will
 *        be either the value read in OTP memory or a default value.
 *
 *
 * input parameters
 *
 * output parameters
 *
 * returns the XTAL trim value set upon initialisation
 */
uint8_t dwt_getxtaltrim(void) {
	return pdw3000local->init_xtrim;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function will disable TX LDOs and allow TX blocks to be manually turned off by dwt_disable_rftx_blocks
 *
 * input parameters
 * @param[in] switch_config - specifies whether the switch needs to be restored to auto.
 *
 * output parameters
 * None
 *
 */
static
void dwt_disable_rf_tx(uint8_t switch_config) {
	//Turn off TX LDOs
	dwt_write32bitoffsetreg(LDO_CTRL_ID, 0, 0x00000000);

	//Disable RF blocks for TX (configure RF_ENABLE_ID reg)
	dwt_write32bitoffsetreg(RF_ENABLE_ID, 0, 0x00000000);

	if (switch_config) {
		//Restore the TXRX switch to auto
		dwt_write32bitoffsetreg(RF_SWITCH_CTRL_ID, 0x0, TXRXSWITCH_AUTO);
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function will enable TX LDOs and allow TX blocks to be manually turned on by dwt_enable_rftx_blocks for a given channel
 *
 * input parameters
 * @param[in] channel - specifies the operating channel (e.g. 5 or 9)
 * @param[in] switch_config - specifies whether the switch needs to be configured for TX
 *
 * output parameters
 *
 */
static
void dwt_enable_rf_tx(uint32_t channel, uint8_t switch_control) {
	//Turn on TX LDOs
	dwt_or32bitoffsetreg(LDO_CTRL_ID, 0,
			(LDO_CTRL_LDO_VDDHVTX_VREF_BIT_MASK | LDO_CTRL_LDO_VDDHVTX_EN_BIT_MASK));
	dwt_or32bitoffsetreg(LDO_CTRL_ID, 0,
			(LDO_CTRL_LDO_VDDTX2_VREF_BIT_MASK | LDO_CTRL_LDO_VDDTX1_VREF_BIT_MASK | LDO_CTRL_LDO_VDDTX2_EN_BIT_MASK | LDO_CTRL_LDO_VDDTX1_EN_BIT_MASK));

	//Enable RF blocks for TX (configure RF_ENABLE_ID reg)
	if (channel == SEL_CHANNEL5) {
		dwt_or32bitoffsetreg(RF_ENABLE_ID, 0,
				(RF_ENABLE_TX_SW_EN_BIT_MASK | RF_ENABLE_TX_CH5_BIT_MASK | RF_ENABLE_TX_EN_BIT_MASK | RF_ENABLE_TX_EN_BUF_BIT_MASK | RF_ENABLE_TX_BIAS_EN_BIT_MASK));
	} else {
		dwt_or32bitoffsetreg(RF_ENABLE_ID, 0,
				(RF_ENABLE_TX_SW_EN_BIT_MASK | RF_ENABLE_TX_EN_BIT_MASK | RF_ENABLE_TX_EN_BUF_BIT_MASK | RF_ENABLE_TX_BIAS_EN_BIT_MASK));
	}

	if (switch_control) {
		//configure the TXRX switch for TX mode
		dwt_write32bitoffsetreg(RF_SWITCH_CTRL_ID, 0x0, TXRXSWITCH_TX);
	}

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function will enable a repeated continuous waveform on the device
 *
 * input parameters:
 * @param cw_enable: CW mode enable
 * @param cw_mode_config: CW configuration mode.
 *
 * output parameters:
 *
 */
void dwt_repeated_cw(int cw_enable, int cw_mode_config) {

	//Turn off TX Seq
	dwt_setfinegraintxseq(0);

	if (cw_mode_config > 0xF)
		cw_mode_config = 0xF;
	if ((cw_enable > 3) || (cw_enable < 1))
		cw_enable = 4;

	dwt_write32bitoffsetreg(TX_TEST_ID, 0x0, 0x10 >> cw_enable);
	dwt_write32bitoffsetreg(PG_TEST_ID, 0x0,
			(uint32_t) (cw_mode_config << ((cw_enable - 1) * 4)));
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function enables repeated frames to be generated given a frame repetition rate.
 *
 * input parameters:
 * @param framerepetitionrate - Value specifying the rate at which frames will be repeated.
 *                            If the value is less than the frame duration, the frames are sent
 *                            back-to-back.
 *
 * output parameters:
 * None
 *
 * No return value
 */
void dwt_repeated_frames(uint32_t framerepetitionrate) {
	//Enable repeated frames
	dwt_or8bitoffsetreg(TEST_CTRL0_ID, 0x0, TEST_CTRL0_TX_PSTM_BIT_MASK);

	if (framerepetitionrate < 4) {
		framerepetitionrate = 4;
	}
	dwt_write32bitreg(DX_TIME_ID, framerepetitionrate);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function disables the automatic sequencing of the tx-blocks for a specific channel.
 *
 * input parameters:
 * @param[in] chan - specifies the operating channel (e.g. 5 or 9)
 *
 * output parameters:
 * None
 *
 * No return value
 */
static
void dwt_enable_rftx_blocks(uint32_t channel) {
	if (channel == SEL_CHANNEL5) {
		dwt_or32bitoffsetreg(RF_CTRL_MASK_ID, 0,
				(RF_ENABLE_TX_SW_EN_BIT_MASK | RF_ENABLE_TX_CH5_BIT_MASK | RF_ENABLE_TX_EN_BIT_MASK | RF_ENABLE_TX_EN_BUF_BIT_MASK | RF_ENABLE_TX_BIAS_EN_BIT_MASK));
	} else if (channel == SEL_CHANNEL9) {
		dwt_or32bitoffsetreg(RF_CTRL_MASK_ID, 0,
				(RF_ENABLE_TX_SW_EN_BIT_MASK | RF_ENABLE_TX_EN_BIT_MASK | RF_ENABLE_TX_EN_BUF_BIT_MASK | RF_ENABLE_TX_BIAS_EN_BIT_MASK));
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function enables the automatic sequencing of the tx-blocks for a specific channel.
 *
 * input parameters:
 * None
 *
 * output parameters:
 * None
 *
 * No return value
 */
static
void dwt_disable_rftx_blocks(void) {
	dwt_write32bitoffsetreg(RF_CTRL_MASK_ID, 0, 0x00000000);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function sets the DW3000 to transmit cw signal at specific channel frequency
 *
 * input parameters:
 * @param channel - specifies the operating channel (e.g. 5 or 9)
 *
 * output parameters
 *
 * no return value
 */
void dwt_configcwmode(uint8_t channel) {
	dwt_enable_rf_tx(channel, 1);
	dwt_enable_rftx_blocks(channel);
	dwt_force_clocks(FORCE_CLK_SYS_TX);
	dwt_repeated_cw(1, 0xF);    //PulseGen Channel 1, full power
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function sets the DW3000 to continuous tx frame mode for regulatory approvals testing.
 *
 * input parameters:
 * @param framerepetitionrate - This is a 32-bit value that is used to set the interval between transmissions.
 *  The minimum value is 4. The units are approximately 8 ns. (or more precisely 512/(499.2e6*128) seconds)).
 * @param channel - specifies the operating channel (e.g. 5 or 9)
 *
 * output parameters
 *
 * no return value
 */
void dwt_configcontinuousframemode(uint32_t framerepetitionrate,
		uint8_t channel) {
	//NOTE: dwt_configure and dwt_configuretxrf must be called before a call to this API
	dwt_enable_rf_tx(channel, 1);
	dwt_enable_rftx_blocks(channel);
	dwt_force_clocks(FORCE_CLK_SYS_TX);
	dwt_repeated_frames(framerepetitionrate);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function reads the raw battery voltage and temperature values of the DW IC.
 * The values read here will be the current values sampled by DW IC AtoD converters.
 *
 *
 * input parameters:
 *
 * output parameters
 *
 * returns  (temp_raw<<8)|(vbat_raw)
 */
uint16_t dwt_readtempvbat(void) {
	uint8_t vbat_raw = 0;
	uint8_t temp_raw = 0;
	uint8_t ldo_ctrl;
	uint16_t wr_buf;

	/* Enable MS2 LDO - this is needed to read SAR if device is in IDLE_RC state, this LDO is on in IDLE_PLL state */
	ldo_ctrl = dwt_read8bitoffsetreg(LDO_CTRL_ID, 0);
	dwt_or8bitoffsetreg(LDO_CTRL_ID, 0, LDO_CTRL_LDO_VDDMS2_EN_BIT_MASK);

	/*
	 * SAR can read Vbat correctly by default. However, in order to read the Tsense value,
	 * the appropriate bit must be set in the SAR_TEST register.
	 */

	// Enable the TSENSE
	dwt_write8bitoffsetreg(SAR_TEST_ID, 0, SAR_TEST_SAR_RDEN_BIT_MASK);

	// Reading All SAR inputs
	dwt_write8bitoffsetreg(SAR_CTRL_ID, SAR_CTRL_SAR_START_BIT_OFFSET,
			SAR_CTRL_SAR_START_BIT_MASK);

	// Wait until SAR conversion is complete.
	while (!(dwt_read8bitoffsetreg(SAR_STATUS_ID,
			SAR_STATUS_SAR_DONE_BIT_OFFSET) & SAR_STATUS_SAR_DONE_BIT_MASK))
		;

	// Read voltage and temperature.
	wr_buf = dwt_read16bitoffsetreg(SAR_READING_ID,
			SAR_READING_SAR_READING_VBAT_BIT_OFFSET);

	vbat_raw = (uint8_t) (wr_buf & 0x00ff);
	temp_raw = (uint8_t) (wr_buf >> 8);

	// Clear SAR enable
	dwt_write8bitoffsetreg(SAR_CTRL_ID, SAR_CTRL_SAR_START_BIT_OFFSET, 0x00);

	// Disable the TSENSE
	dwt_write8bitoffsetreg(SAR_TEST_ID, 0, 0x0 << SAR_TEST_SAR_RDEN_BIT_OFFSET);

	// restore LDO control register
	dwt_write8bitoffsetreg(LDO_CTRL_ID, 0, ldo_ctrl);

	return (uint16_t) ((temp_raw << 8) | (vbat_raw));
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function takes in a raw temperature value and applies the conversion factor
 * to give true temperature. The dwt_initialise needs to be called before call to this to
 * ensure pdw3000local->tempP contains the SAR_LTEMP value from OTP.
 *
 * input parameters:
 * @param raw_temp - this is the 8-bit raw temperature value as read by dwt_readtempvbat
 *
 * output parameters:
 *
 * returns: temperature sensor value
 */
float dwt_convertrawtemperature(uint8_t raw_temp) {
	float realtemp;

	// the User Manual formula is: Temperature (�C) = ( (SAR_LTEMP ?OTP_READ(Vtemp @ 20�C) ) x 1.05)        // Vtemp @ 20�C
	realtemp = (float) ((raw_temp - pdw3000local->tempP) * 1.05f) + 20.0f;
	return realtemp;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function takes in a raw voltage value and applies the conversion factor
 * to give true voltage. The dwt_initialise needs to be called before call to this to
 * ensure pdw3000local->vBatP contains the SAR_LVBAT value from OTP
 *
 * input parameters:
 * @param raw_voltage - this is the 8-bit raw voltage value as read by dwt_readtempvbat
 *
 * output parameters:
 *
 * returns: voltage sensor value
 */
float dwt_convertrawvoltage(uint8_t raw_voltage) {
	float realvolt;

	// Bench measurements gives approximately: VDDBAT = sar_read * Vref / max_code * 16x_atten   - assume Vref @ 3.0V
	realvolt = (float) ((float) (raw_voltage - pdw3000local->vBatP) * 0.4f * 16
			/ 255) + 3.0f;
	//realvolt = ((float)raw_voltage * 0.4f / 255) * 16;
	return realvolt;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function reads the temperature of the DW3000 that was sampled
 * on waking from Sleep/Deepsleep. They are not current values, but read on last
 * wakeup if DWT_RUNSAR bit is set in mode parameter of dwt_configuresleep
 *
 * input parameters:
 *
 * output parameters:
 *
 * returns: 8-bit raw temperature sensor value
 */
uint8_t dwt_readwakeuptemp(void) {
	return dwt_read8bitoffsetreg(SAR_READING_ID, 1);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function reads the battery voltage of the DW3000 that was sampled
 * on waking from Sleep/Deepsleep. They are not current values, but read on last
 * wakeup if DWT_RUNSAR bit is set in mode parameter of dwt_configuresleep
 *
 * input parameters:
 *
 * output parameters:
 *
 * returns: 8-bit raw battery voltage sensor value
 */
uint8_t dwt_readwakeupvbat(void) {
	return dwt_read8bitoffsetreg(SAR_READING_ID, 0);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function determines the adjusted bandwidth setting (PG_DELAY bitfield setting)
 * of the DW3000. The adjustment is a result of DW3000 internal PG cal routine, given a target count value it will try to
 * find the PG delay which gives the closest count value.
 * Manual sequencing of TX blocks and TX clocks need to be enabled for either channel 5 or 9.
 * This function presumes that the PLL is already in the IDLE state. Please configure the PLL to IDLE
 * state before calling this function, by calling dwt_configure.
 *
 * input parameters:
 * @param target_count - uint16_t - the PG count target to reach in order to correct the bandwidth
 * @param channel - int - The channel to configure for the corrected bandwidth (5 or 9)
 *
 * output parameters:
 * returns: (uint8_t) The setting that was written to the PG_DELAY register (when calibration completed)
 */
uint8_t dwt_calcbandwidthadj(uint16_t target_count, int channel) {
	// Force system clock to FOSC/4 and TX clocks on and enable RF blocks
	dwt_force_clocks(FORCE_CLK_SYS_TX);
	dwt_enable_rf_tx((uint32_t) channel, 0);
	dwt_enable_rftx_blocks((uint32_t) channel);

	// Write to the PG target before kicking off PG auto-cal with given target value
	dwt_write16bitoffsetreg(PG_CAL_TARGET_ID, 0x0,
			target_count & PG_CAL_TARGET_TARGET_BIT_MASK);
	// Run PG count cal
	dwt_or8bitoffsetreg(PGC_CTRL_ID, 0x0,
			(uint8_t)(PGC_CTRL_PGC_START_BIT_MASK | PGC_CTRL_PGC_AUTO_CAL_BIT_MASK));
	// Wait for calibration to complete
	while (dwt_read8bitoffsetreg(PGC_CTRL_ID, 0) & PGC_CTRL_PGC_START_BIT_MASK)
		;

	//Restore clocks to AUTO and turn off TX blocks
	dwt_disable_rftx_blocks();
	dwt_disable_rf_tx(0);
	dwt_force_clocks(FORCE_CLK_AUTO);

	return (dwt_read8bitoffsetreg(TX_CTRL_HI_ID, 0)
			& TX_CTRL_HI_TX_PG_DELAY_BIT_MASK);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this function calculates the value in the pulse generator counter register (PGC_STATUS) for a given PG_DELAY
 * This is used to take a reference measurement, and the value recorded as the reference is used to adjust the
 * bandwidth of the device when the temperature changes. This function presumes that the PLL is already in the IDLE
 * state.
 *
 * input parameters:
 * @param pgdly - uint8_t - the PG_DELAY (max value 63) to set (to control bandwidth), and to find the corresponding count value for
 * @param channel - int - The channel to configure for the corrected bandwith (5 or 9)
 *
 * output parameters:
 * returns (uint16_t) - The count value calculated from the provided PG_DELAY value (from PGC_STATUS) - used as reference
 * for later bandwidth adjustments
 */
uint16_t dwt_calcpgcount(uint8_t pgdly, int channel) {
	uint16_t count = 0;

	// Force system clock to FOSC/4 and TX clocks on
	dwt_force_clocks(FORCE_CLK_SYS_TX);
	dwt_enable_rf_tx((uint32_t) channel, 0);
	dwt_enable_rftx_blocks((uint32_t) channel);

	dwt_write8bitoffsetreg(TX_CTRL_HI_ID, TX_CTRL_HI_TX_PG_DELAY_BIT_OFFSET,
			pgdly & TX_CTRL_HI_TX_PG_DELAY_BIT_MASK);

	// Run count cal
	dwt_or8bitoffsetreg(PGC_CTRL_ID, 0x0, PGC_CTRL_PGC_START_BIT_MASK);
	// Wait for calibration to complete
	while (dwt_read8bitoffsetreg(PGC_CTRL_ID, 0) & PGC_CTRL_PGC_START_BIT_MASK)
		;
	count =
			dwt_read16bitoffsetreg(PGC_STATUS_ID,
					PGC_STATUS_PG_DELAY_COUNT_BIT_OFFSET) & PGC_STATUS_PG_DELAY_COUNT_BIT_MASK;

	dwt_disable_rftx_blocks();
	dwt_disable_rf_tx(0);
	dwt_force_clocks(FORCE_CLK_AUTO);

	return count;
}

/* AES block */

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief   This function provides the API for the configuration of the AES block before first usage.
 * @param   pCfg    - pointer to the configuration structure, which contains the AES configuration data.
 *
 * output parameters
 *
 * no return value
 */
void dwt_configure_aes(const dwt_aes_config_t *pCfg) {
	uint16_t tmp;

	tmp = (uint16_t) pCfg->mode;
	tmp |= ((uint32_t) pCfg->key_size) << AES_CFG_KEY_SIZE_BIT_OFFSET;
	tmp |= ((uint32_t) pCfg->key_addr) << AES_CFG_KEY_ADDR_BIT_OFFSET;
	tmp |= ((uint32_t) pCfg->key_load) << AES_CFG_KEY_LOAD_BIT_OFFSET;
	tmp |= ((uint32_t) pCfg->key_src) << AES_CFG_KEY_SRC_BIT_OFFSET;
	tmp |= ((uint32_t) pCfg->mic) << AES_CFG_TAG_SIZE_BIT_OFFSET;
	tmp |= ((uint32_t) pCfg->aes_core_type) << AES_CFG_CORE_SEL_BIT_OFFSET;
	tmp |= ((uint32_t) pCfg->aes_key_otp_type) << AES_CFG_KEY_OTP_BIT_OFFSET;

	dwt_write16bitoffsetreg(AES_CFG_ID, 0, tmp);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief   This gets mic size in bytes and convert it to value to write in AES_CFG
 * @param   mic_size_in_bytes - mic size in bytes.
 *
 * @Return  dwt_mic_size_e - reg value number
 */
dwt_mic_size_e dwt_mic_size_from_bytes(uint8_t mic_size_in_bytes) {
	dwt_mic_size_e mic_size = MIC_0;

	if (mic_size_in_bytes != 0) {
		mic_size = (dwt_mic_size_e) ((mic_size_in_bytes >> 1) - 1);
	}
	return mic_size;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief   This function provides the API for the configuration of the AES key before first usage.
 * @param   *key - pointer to the key which will be programmed to the Key register
 *          Note, key register supports only 128-bit keys.
 *
 * output parameters
 *
 * no return value
 */
void dwt_set_keyreg_128(dwt_aes_key_t *key) {
	/* program Key to the register : only 128 bit key can be used */
	dwt_write32bitreg(AES_KEY0_ID, (uint32_t )key->key0);
	dwt_write32bitreg(AES_KEY1_ID, (uint32_t )key->key1);
	dwt_write32bitreg(AES_KEY2_ID, (uint32_t )key->key2);
	dwt_write32bitreg(AES_KEY3_ID, (uint32_t )key->key3);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief   poll AES block waiting for completion of encrypt/decrypt job
 *          It clears all received errors/statuses.
 *          This is not a safe function as it is waiting for AES_STS_AES_DONE_BIT_MASK forever.
 *
 * @return  AES_STS_ID status
 */
static uint8_t dwt_wait_aes_poll(void) {
	uint8_t tmp;
	do {
		tmp = dwt_read8bitoffsetreg(AES_STS_ID, 0);
	} while (!(tmp & (AES_STS_AES_DONE_BIT_MASK | AES_STS_TRANS_ERR_BIT_MASK)));

	dwt_write8bitoffsetreg(AES_STS_ID, 0, tmp); //clear all bits which were set as a result of AES operation

	return (tmp & 0x3F);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function update the core IV regs according to the core type
 *
 * DW3000 IC stores the nonce in AES_IV0_ID to AES_IV3_ID registers.
 * DW3000 IC, when operating in CCM* AES mode expects the nonce to be programmed into 4 words as follows:
 * AES_IV0_ID[0] = nonce[10]
 * AES_IV0_ID[1] = nonce[9]
 * AES_IV0_ID[2] = nonce[8]
 * AES_IV0_ID[3] = nonce[7]
 * AES_IV1_ID[0] = nonce[6]
 * AES_IV1_ID[1] = nonce[5]
 * AES_IV1_ID[2] = nonce[4]
 * AES_IV1_ID[3] = nonce[3]
 * AES_IV2_ID[0] = nonce[2]
 * AES_IV2_ID[1] = nonce[1]
 * AES_IV2_ID[2] = nonce[0]
 * AES_IV2_ID[3] = don't care
 * AES_IV3_ID[0] = payload_length[0]
 * AES_IV3_ID[1] = payload_length[1]
 * AES_IV3_ID[2] = nonce[12]
 * AES_IV3_ID[3] = nonce[11]
 *
 * @param nonce - Pointer to the nonce
 * @param payload - Length of data payload to encrypt/decrypt
 *
 */
static
void dwt_update_nonce_CCM(uint8_t *nonce, uint16_t payload) {
	uint8_t iv[16];
	iv[0] = nonce[10];
	iv[1] = nonce[9];
	iv[2] = nonce[8];
	iv[3] = nonce[7];
	iv[4] = nonce[6];
	iv[5] = nonce[5];
	iv[6] = nonce[4];
	iv[7] = nonce[3];
	iv[8] = nonce[2];
	iv[9] = nonce[1];
	iv[10] = nonce[0];
	iv[11] = 0x00;    //don't care
	iv[12] = (uint8_t) payload;
	iv[13] = (uint8_t) (payload >> 8);
	iv[14] = nonce[12];
	iv[15] = nonce[11];

	dwt_writetodevice(AES_IV0_ID, 0, 16, iv);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function update the nonce-IV regs when using GCM AES core type
 *
 * DW3000 IC, when operating in GCM AES mode expects the nonce to be programmed into 3 words as follows:
 * LSB (of nonce array) sent first. Nonce array is made up of 12 bytes.
 *
 * @param nonce - Pointer to the nonce value to set
 *
 */
void dwt_update_nonce_GCM(uint8_t *nonce) {
	dwt_writetodevice(AES_IV0_ID, 0, 12, nonce);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief   This function provides the API for the job of encrypt/decrypt the data block
 *
 *          128 bit key shall be pre-loaded with dwt_set_aes_key()
 *          dwt_configure_aes
 *
 *          supports AES_KEY_Src_Register mode only
 *          packet sizes < 127
 *          note, the "nonce" shall be unique for every transaction
 * @param job - pointer to AES job, contains data info and encryption info.
 * @param core_type - Core type
 *
 * @return  AES_STS_ID status bits
 *
 *
 */
int8_t dwt_do_aes(dwt_aes_job_t *job, dwt_aes_core_type_e core_type) {
	uint32_t tmp, dest_reg;
	uint16_t allow_size;
	uint8_t ret;
	dwt_aes_src_port_e src_port;
	dwt_aes_dst_port_e dst_port;

	if (job->mic_size == MIC_ERROR) {
		return ERROR_WRONG_MIC_SIZE;
	}

	/* program Initialization Vector
	 * */
	if (core_type == AES_core_type_GCM) {
		dwt_update_nonce_GCM(job->nonce);
	} else {
		dwt_update_nonce_CCM(job->nonce, job->payload_len);
	}

	/* write data to be encrypted.
	 * for decryption data should be present in the src buffer
	 * */
	tmp = job->header_len + job->payload_len;

	if (job->mode == AES_Encrypt) {
		if (job->src_port == AES_Src_Scratch) {
			allow_size = SCRATCH_BUFFER_MAX_LEN;
			dest_reg = SCRATCH_RAM_ID;
		} else {
			allow_size = TX_BUFFER_MAX_LEN;
			dest_reg = TX_BUFFER_ID;
		}
	} else if (job->mode == AES_Decrypt) {
		if (job->dst_port == AES_Dst_Scratch) {
			allow_size = SCRATCH_BUFFER_MAX_LEN;
		} else {
			allow_size = RX_BUFFER_MAX_LEN;
		}
	} else {
		return ERROR_WRONG_MODE;
	}

	if (tmp > (allow_size - job->mic_size - (unsigned int) FCS_LEN)) {
		return ERROR_DATA_SIZE;
	}

	if (job->mode == AES_Encrypt) {
		dwt_writetodevice(dest_reg, 0, job->header_len, job->header); /*!< non-encrypted header */
		dwt_writetodevice(dest_reg, job->header_len, job->payload_len,
				job->payload); /*!< data to be encrypted */
	}

	/* Set SRC and DST ports in memory.
	 * Current implementation uses frames started from start of the desired port
	 * */
	src_port = job->src_port;
	if ((job->src_port == AES_Src_Rx_buf_0)
			|| (job->src_port == AES_Src_Rx_buf_1)) {
		if (pdw3000local->dblbuffon == DBL_BUFF_ACCESS_BUFFER_1) //if the flag is 0x3 we are reading from RX_BUFFER_1
		{
			src_port = AES_Src_Rx_buf_1;
		} else {
			src_port = AES_Src_Rx_buf_0;
		}
	}

	dst_port = job->dst_port;
	if ((job->dst_port == AES_Dst_Rx_buf_0)
			|| (job->dst_port == AES_Dst_Rx_buf_1)) {
		if (pdw3000local->dblbuffon == DBL_BUFF_ACCESS_BUFFER_1) //if the flag is 0x3 we are reading from RX_BUFFER_1
		{
			dst_port = AES_Dst_Rx_buf_1;
		} else {
			dst_port = AES_Dst_Rx_buf_0;
		}
	} else if (job->dst_port == AES_Dst_STS_key) {
		if (job->payload_len > STS_LEN_128BIT) //when writing to STS registers (destination port) payload cannot exceed 16 bytes
		{
			return ERROR_PAYLOAD_SIZE;
		}
	}

	tmp = (((uint32_t) src_port) << DMA_CFG0_SRC_PORT_BIT_OFFSET)
			|\
 (((uint32_t) dst_port) << DMA_CFG0_DST_PORT_BIT_OFFSET);

	dwt_write32bitreg(DMA_CFG0_ID, tmp);

	/* fill header length and payload length - the only known information of the frame.
	 * Note, the payload length does not include MIC and FCS lengths.
	 * So, if overall rx_frame length is 100 and header length is 10, MIC is 16 and FCS is 2,
	 * then payload length is 100-10-16-2 = 72 bytes.
	 * */
	tmp = (DMA_CFG1_HDR_SIZE_BIT_MASK
			& (((uint32_t) job->header_len) << DMA_CFG1_HDR_SIZE_BIT_OFFSET))
			|\
 (DMA_CFG1_PYLD_SIZE_BIT_MASK
					& (((uint32_t) job->payload_len)
							<< DMA_CFG1_PYLD_SIZE_BIT_OFFSET));

	dwt_write32bitreg(DMA_CFG1_ID, tmp);

	/* start AES action encrypt/decrypt */
	dwt_write8bitoffsetreg(AES_START_ID, 0, AES_START_AES_START_BIT_MASK);
	ret = dwt_wait_aes_poll();

	/* Read plain header and decrypted payload on correct AES decryption
	 * and if instructed to do so, i.e. if job->mode == AES_Decrypt and
	 * job->header or job->payload addresses are exist
	 * */

	if (((ret & ~(AES_STS_RAM_EMPTY_BIT_MASK | AES_STS_RAM_FULL_BIT_MASK))
			== AES_STS_AES_DONE_BIT_MASK) && (job->mode == AES_Decrypt)) {
		uint32_t read_addr;

		if ((job->dst_port == AES_Dst_Rx_buf_0)
				|| (job->dst_port == AES_Dst_Rx_buf_1)) {
			if (pdw3000local->dblbuffon == DBL_BUFF_ACCESS_BUFFER_1) //if the flag is 0x3 we are reading from RX_BUFFER_1
			{
				read_addr = RX_BUFFER_1_ID;
			} else {
				read_addr = RX_BUFFER_0_ID;
			}
		} else if (job->dst_port == AES_Dst_Tx_buf) {
			read_addr = TX_BUFFER_ID;
		} else {
			read_addr = SCRATCH_RAM_ID;
		}

		if (job->header != NULL) {
			if (job->header_len) {
				dwt_readfromdevice(read_addr, 0, job->header_len, job->header);
			}
		}

		if (job->payload != NULL) {
			if (job->payload_len) {
				dwt_readfromdevice(read_addr, job->header_len, job->payload_len,
						job->payload);
			}
		}
	}
	return ((int8_t) ret);
}

/*! ------------------------------------------------------------------------------------------------------------------
 *
 * @brief   This function is used to write a 16 bit address to a desired Low-Energy device (LE) address. For frame pending to function when
 * the correct bits are set in the frame filtering configuration via the dwt_configureframefilter. See dwt_configureframefilter for more details.
 *
 * @param addr - the address value to be written to the selected LE register
 * @param leIndex - Low-Energy device (LE) address to write to
 *
 * no return value
 *
 */
void dwt_configure_le_address(uint16_t addr, int leIndex) {
	switch (leIndex) {
	case 0:
		dwt_write16bitoffsetreg(LE_PEND_01_ID, 0, addr);
		break;
	case 1:
		dwt_write16bitoffsetreg(LE_PEND_01_ID, 2, addr);
		break;
	case 2:
		dwt_write16bitoffsetreg(LE_PEND_23_ID, 0, addr);
		break;
	case 3:
		dwt_write16bitoffsetreg(LE_PEND_23_ID, 2, addr);
		break;
	default:
		break;
	}

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function configures SFD type only: e.g. IEEE 4a - 8, DW-8, DW-16, or IEEE 4z -8 (binary)
 * The dwt_configure should be called prior to this to configure other parameters
 *
 * input parameters
 * @param sfdType    -   e.g. DWT_SFD_IEEE_4A, DWT_SFD_DW_8, DWT_SFD_DW_16, DWT_SFD_IEEE_4Z
 *
 * return none
 *
 */
void dwt_configuresfdtype(uint8_t sfdType) {
	dwt_modify32bitoffsetreg(CHAN_CTRL_ID, 0,
			(uint32_t) (~CHAN_CTRL_SFD_TYPE_BIT_MASK),
			(CHAN_CTRL_SFD_TYPE_BIT_MASK
					& ((uint32_t) sfdType << CHAN_CTRL_SFD_TYPE_BIT_OFFSET)));
}

/* ===============================================================================================
 List of expected (known) device ID handled by this software
 ===============================================================================================

 // as defined in the deca_device_api.h

 ===============================================================================================
 */

HAL_StatusTypeDef write(uint16_t headerLength, uint8_t *headerBuffer,
		uint32_t bodylength, uint8_t *bodyBuffer) {
	HAL_StatusTypeDef res;
	uint8_t buf[100] = { 0 };
	int i;
	int j;
	for (i = 0; i < headerLength; i++)
		buf[i] = headerBuffer[i];

	for (j = 0; j < bodylength; j++)
		buf[i + j] = bodyBuffer[j];

	HAL_GPIO_WritePin(hw->nssPort, hw->nssPin, GPIO_PIN_RESET); // pull the pin low
	res = HAL_SPI_Transmit(hw->spi, buf, i + j, 0xffff);
	HAL_GPIO_WritePin(hw->nssPort, hw->nssPin, GPIO_PIN_SET); // pull the pin high

	return (res);
}

HAL_StatusTypeDef read(uint16_t headerLength, const uint8_t *headerBuffer,
		uint32_t readlength, uint8_t *readBuffer) {
	HAL_StatusTypeDef res;
	HAL_GPIO_WritePin(hw->nssPort, hw->nssPin, GPIO_PIN_RESET); // pull the pin low
	res = HAL_SPI_Transmit(hw->spi, (uint8_t*) headerBuffer, headerLength,
			0xffff);
	if (res == HAL_OK)
		res = HAL_SPI_Receive(hw->spi, readBuffer, readlength, 0xffff);
	HAL_GPIO_WritePin(hw->nssPort, hw->nssPin, GPIO_PIN_SET); // pull the pin high
	return (res);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn check_for_status_errors()
 *
 * @brief This function is used to get a value to increase the delay timer by dependent on the current TX preamble length set.
 *
 * @param reg: uint32_t value representing the current status register value.
 * @param errors: pointer to a uint32_t buffer that contains the sum of different errors logged during program operation.
 *
 * @return none
 */
void check_for_status_errors(uint32_t reg, uint32_t *errors) {
	uint16_t stsStatus = 0;

	if (!(reg & SYS_STATUS_RXFCG_BIT_MASK)) {
		errors[BAD_FRAME_ERR_IDX] += 1;
	}

	if (reg & SYS_STATUS_RXFSL_BIT_MASK) {
		errors[RSE_ERR_IDX] += 1;
	}

	if (reg & SYS_STATUS_RXPHE_BIT_MASK) {
		errors[PHE_ERR_IDX] += 1;
	}

	if (reg & SYS_STATUS_RXPTO_BIT_MASK) {
		errors[PTO_ERR_IDX] += 1;
	}

	if (reg & SYS_STATUS_ARFE_BIT_MASK) {
		errors[ARFE_ERR_IDX] += 1;
	}

	if ((reg & SYS_STATUS_RXFR_BIT_MASK)
			&& !(reg & SYS_STATUS_RXFCG_BIT_MASK)) {
		errors[CRC_ERR_IDX] += 1;
	}

	if ((reg & SYS_STATUS_RXFTO_BIT_MASK) || (reg & SYS_STATUS_ALL_RX_TO)) {
		errors[RTO_ERR_IDX] += 1;
	}

	if (reg & SYS_STATUS_RXSTO_BIT_MASK) {
		errors[SFDTO_ERR_IDX] += 1;
	}

	if (reg & SYS_STATUS_CPERR_BIT_MASK) {
		// There is a general STS error
		errors[STS_PREAMBLE_ERR] += 1;

		// Get the status for a more detailed error reading of what went wrong with the STS
		dwt_readstsstatus(&stsStatus, 0);
		if (stsStatus & 0x100) {
			// Peak growth rate warning
			errors[STS_PEAK_GROWTH_RATE_ERR] += 1;
		}
		if (stsStatus & 0x080) {
			// ADC count warning
			errors[STS_ADC_COUNT_ERR] += 1;
		}
		if (stsStatus & 0x040) {
			// SFD count warning
			errors[STS_SFD_COUNT_ERR] += 1;
		}
		if (stsStatus & 0x020) {
			// Late first path estimation
			errors[STS_LATE_FIRST_PATH_ERR] += 1;
		}
		if (stsStatus & 0x010) {
			// Late coarse estimation
			errors[STS_LATE_COARSE_EST_ERR] += 1;
		}
		if (stsStatus & 0x008) {
			// Coarse estimation empty
			errors[STS_COARSE_EST_EMPTY_ERR] += 1;
		}
		if (stsStatus & 0x004) {
			// High noise threshold
			errors[STS_HIGH_NOISE_THREASH_ERR] += 1;
		}
		if (stsStatus & 0x002) {
			// Non-triangle
			errors[STS_NON_TRIANGLE_ERR] += 1;
		}
		if (stsStatus & 0x001) {
			// Logistic regression failed
			errors[STS_LOG_REG_FAILED_ERR] += 1;
		}
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_delay_time_txpreamble()
 *
 * @brief This function is used to get a value to increase the delay timer by dependent on the current TX preamble length set.
 *
 * @param None
 *
 * @return delay_time - a uint32_t value indicating the required increase needed to delay the time by.
 */
uint32_t get_rx_delay_time_txpreamble(dwt_config_t *config_options) {
	uint32_t delay_time = 0;
	/* Standard delay values for preamble lengths of 32, 64, 72 & 128 should be adequate.
	 * Additional time deues for preamble lengths of 32, 64, 72 & 128 should be adequate.
	 * Additional time delay will be needed for larger preamble lengths.
	 * Delay required is dependent on the preamble length as it increases the frame length. */
	switch (config_options->txPreambLength) {
	case DWT_PLEN_256:
		delay_time += 128; /* 256 - 128 */
		break;
	case DWT_PLEN_512:
		delay_time += 384; /* 512 - 128 */
		break;
	case DWT_PLEN_1024:
		delay_time += 896; /* 1024 - 128 */
		break;
	case DWT_PLEN_1536:
		delay_time += 1408; /* 1536 - 128 */
		break;
	case DWT_PLEN_2048:
		delay_time += 1920; /* 2048 - 128 */
		break;
	case DWT_PLEN_4096:
		delay_time += 3968; /* 4096 - 128 */
		break;
	case DWT_PLEN_32:
	case DWT_PLEN_64:
	case DWT_PLEN_72:
	case DWT_PLEN_128:
	default:
		break;
	}

	return delay_time;
}
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_delay_time_data_rate()
 *
 * @brief This function is used to get a value to increase the delay timer by dependent on the current data rate set.
 *
 * @param None
 *
 * @return delay_time - a uint32_t value indicating the required increase needed to delay the time by.
 */
uint32_t get_rx_delay_time_data_rate(dwt_config_t *config_options) {
	uint32_t delay_time = 0;
	/*
	 * If data rate is set to 850k (slower rate),
	 * increase the delay time
	 */
	switch (config_options->dataRate) {
	case DWT_BR_850K:
		delay_time += 200;
		break;
	case DWT_BR_6M8:
	default:
		break;
	}

	return delay_time;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn set_delayed_rx_time()
 *
 * @brief This function is used to set the delayed RX time before running dwt_rxenable()
 *
 * @param delay - This is a defined delay value (usually POLL_TX_TO_RESP_RX_DLY_UUS)
 * @param config_options - pointer to dwt_config_t configuration structure that is in use at the time this function
 *                         is called.
 *
 * @return None
 */
void set_delayed_rx_time(uint32_t delay, dwt_config_t *config_options) {
	uint32_t delay_time = delay;

	switch (config_options->txPreambLength) {
	case DWT_PLEN_32:
		delay_time -= 32;
		break;
	case DWT_PLEN_64:
		delay_time -= 64;
		break;
	case DWT_PLEN_72:
		delay_time -= 72;
		break;
	case DWT_PLEN_128:
		delay_time -= 128;
		break;
	case DWT_PLEN_256:
		delay_time -= 256;
		break;
	case DWT_PLEN_512:
		delay_time -= 512;
		break;
	case DWT_PLEN_1024:
		delay_time -= 1024;
		break;
	case DWT_PLEN_1536:
		delay_time -= 1536;
		break;
	case DWT_PLEN_2048:
	case DWT_PLEN_4096:
		delay_time -= 2048;
		break;
	default:
		break;
	}

	/* Length of the STS effects the size of the frame also.
	 * This means the delay required is greater for larger STS lengths. */
	delay_time += ((1 << (config_options->stsLength + 2)) * 8);

	dwt_setdelayedtrxtime((uint32_t) ((delay_time * UUS_TO_DWT_TIME) >> 8));
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn set_resp_rx_timeout()
 *
 * @brief This function is used to set the RX timeout value
 *
 * @param delay - This is a defined delay value (usually RESP_RX_TIMEOUT_UUS)
 * @param config_options - pointer to dwt_config_t configuration structure that is in use at the time this function
 *                         is called.
 *
 * @return None
 */
void set_resp_rx_timeout(uint32_t delay, dwt_config_t *config_options) {
	/*
	 * The program will need to adjust the timeout value depending on the size of the frame
	 * Different sized frames require different time delays.
	 */
	uint32_t delay_time = delay + get_rx_delay_time_data_rate(config_options)
			+ get_rx_delay_time_txpreamble(config_options) + 500;

	/* Length of the STS effects the size of the frame also.
	 * This means the delay required is greater for larger STS lengths. */
	switch (config_options->stsLength) {
	case DWT_STS_LEN_256:
	case DWT_STS_LEN_512:
	case DWT_STS_LEN_1024:
	case DWT_STS_LEN_2048:
		delay_time += ((1 << (config_options->stsLength + 2)) * 8);
		break;
	case DWT_STS_LEN_32:
	case DWT_STS_LEN_64:
	case DWT_STS_LEN_128:
	default:
		break;
	}

	dwt_setrxtimeout(delay_time);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn resync_sts()
 *
 * @brief This function is used to resync the counter values used by the STS.
 *        The counter value is sent over via the TX device during a "poll" message.
 *        This counter value is used to make the RX device align with the same counter value
 *        so that the STS aligns again.
 *
 * @param newCount - The 32 bit value to set the STS to.
 *
 * @return None
 */
void resync_sts(dwt_config_t config_options, uint32_t newCount) {
	// New IV value to use is the original IV0 value plus the count value sent over from the RX,
	// plus half the STS length. This is because the TX device would have read it's current
	// STS count value and then transmitted it. By transmitting, it would have increased it's
	// STS count value by half of the configured STS length.
	// It is important to note that the STS counter will increment by 32 whenever there is a
	// receiver or SFD timeout. This value is consistent regardless of the STS preamble length
	// that is set.
	uint32_t iv_value = newCount;

	iv_value += ((1 << (config_options.stsLength + 2)) * 8) / 2;

	/* Write the new STS count value to the appropriate register and reload the value into the counter */
	dwt_write32bitreg(STS_IV0_ID, iv_value);
	dwt_configurestsloadiv();
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn resp_msg_get_ts()
 *
 * @brief Read a given timestamp value from the response message. In the timestamp fields of the response message, the
 *        least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to get
 *         ts  timestamp value
 *
 * @return none
 */
void resp_msg_get_ts(uint8_t *ts_field, uint32_t *ts) {
	int i;
	*ts = 0;
	for (i = 0; i < RESP_MSG_TS_LEN; i++) {
		*ts += (uint32_t) ts_field[i] << (i * 8);
	}
}

/*! ------------------------------------------------ues for preamble lengths of 32, 64, 72 & 128 should be adequate.
 * Additional time de------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64_t get_tx_timestamp_u64(void) {
	uint8_t ts_tab[5];
	uint64_t ts = 0;
	int8_t i;
	dwt_readtxtimestamp(ts_tab);
	for (i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64_t get_rx_timestamp_u64(void) {
	uint8_t ts_tab[5];
	uint64_t ts = 0;
	int8_t i;
	dwt_readrxtimestamp(ts_tab);
	for (i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts) {
	uint8_t i;
	*ts = 0;
	for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
		*ts += ((uint32_t) ts_field[i] << (i * 8));
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
void final_msg_set_ts(uint8_t *ts_field, uint64_t ts) {
	uint8_t i;
	for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
		ts_field[i] = (uint8_t) ts;
		ts >>= 8;
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn resp_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the response message with the given value. In the timestamp fields of the
 *        response message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
void resp_msg_set_ts(uint8_t *ts_field, const uint64_t ts) {
	uint8_t i;
	for (i = 0; i < RESP_MSG_TS_LEN; i++) {
		ts_field[i] = (uint8_t) (ts >> (i * 8));
	}
}

void deca_usleep(unsigned int usec) {
	unsigned int i;

	usec *= 12;
	for (i = 0; i < usec; i++) {
		__NOP();
	}
}
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used here for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW IC.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 *    Initiator: |Poll TX| ..... |Resp RX| ........ |Final TX|
 *    Responder: |Poll RX| ..... |Resp TX| ........ |Final RX|
 *                   ^|P RMARKER|                                    - time of Poll TX/RX
 *                                   ^|R RMARKER|                    - time of Resp TX/RX
 *                                                      ^|R RMARKER| - time of Final TX/RX
 *
 *                       <--TDLY->                                   - POLL_TX_TO_RESP_RX_DLY_UUS (RDLY-RLEN)
 *                               <-RLEN->                            - RESP_RX_TIMEOUT_UUS   (length of poll frame)
 *                    <----RDLY------>                               - POLL_RX_TO_RESP_TX_DLY_UUS (depends on how quickly responder
 *                                                                                                                      can turn around and reply)
 *
 *
 *                                        <--T2DLY->                 - RESP_TX_TO_FINAL_RX_DLY_UUS (R2DLY-FLEN)
 *                                                  <-FLEN--->       - FINAL_RX_TIMEOUT_UUS   (length of response frame)
 *                                    <----RDLY--------->            - RESP_RX_TO_FINAL_TX_DLY_UUS (depends on how quickly initiator
 *                                                                                                                      can turn around and reply)
 *
 * EXAMPLE 1: with SPI rate set to 18 MHz (default on this platform), and frame lengths of ~190 us, the delays can be set to:
 *            POLL_RX_TO_RESP_TX_DLY_UUS of 400uus, and RESP_RX_TO_FINAL_TX_DLY_UUS of 400uus (TXtoRX delays are set to 210uus)
 *            reducing the delays further can be achieved by using interrupt to handle the TX/RX events, or other code optimisations/faster SPI
 *
 * EXAMPLE 2: with SPI rate set to 4.5 MHz, and frame lengths of ~190 us, the delays can be set to:
 *            POLL_RX_TO_RESP_TX_DLY_UUS of 550uus, and RESP_RX_TO_FINAL_TX_DLY_UUS of 600uus (TXtoRX delays are set to 360 and 410 uus respectively)
 *
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete response frame sent by the responder at the
 *    6.81 Mbps data rate used (around 200 us).
 * 6. In this example, the DW IC is put into IDLE state after calling dwt_initialise(). This means that a fast SPI rate of up to 38 MHz can be used
 *    thereafter.
 * 7. The preamble timeout allows the receiver to stop listening in situations where preamble is not starting (which might be because the responder is
 *    out of range or did not receive the message to respond to). This saves the power waste of listening for a message that is not coming. We
 *    recommend a minimum preamble timeout of 5 PACs for short range applications and a larger value (e.g. in the range of 50% to 80% of the preamble
 *    length) for more challenging longer range, NLOS or noisy environments.
 * 8. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 9. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW IC. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 10. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW IC User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 11. As we want to send final TX timestamp in the final message, we have to compute it in advance instead of relying on the reading of DW IC
 *     register. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to
 *     response RX timestamp to get final transmission time. The delayed transmission time resolution is 512 device time units which means that the
 *     lower 9 bits of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower
 *     8 bits.
 * 12. In this operation, the high order byte of each 40-bit timestamps is discarded. This is acceptable as those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays (needed in the
 *     time-of-flight computation) can be handled by a 32-bit subtraction.
 * 13. When running this example on the DWK3000 platform with the RESP_RX_TO_FINAL_TX_DLY response delay provided, the dwt_starttx() is always
 *     successful. However, in cases where the delay is too short (or something else interrupts the code flow), then the dwt_starttx() might be issued
 *     too late for the configured start time. The code below provides an example of how to handle this condition: In this case it abandons the
 *     ranging exchange to try another one after 1 second. If this error handling code was not here, a late dwt_starttx() would result in the code
 *     flow getting stuck waiting for a TX frame sent event that will never come. The companion "responder" example (ex_05b) should timeout from
 *     awaiting the "final" and proceed to have its receiver on ready to poll of the following exchange.
 * 14. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW IC API Guide for more details on the DW IC driver functions.
 ****************************************************************************************************************************************************/
