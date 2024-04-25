#include "uwb_examples.h"
#include "key.h"
#include <stdio.h>
#include <math.h>

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* X preamble code. Used in RX only. */
    1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0      /* PDOA mode off */
};

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000


/* Frames used in the ranging process. See NOTE 2 below. */
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW IC's delayed TX function. This includes the
 * frame length of approximately 190 us with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS_6M8 900
#define POLL_RX_TO_RESP_TX_DLY_UUS_850K 900
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW IC's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS_6M8 500
#define RESP_TX_TO_FINAL_RX_DLY_UUS_850K 60
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS_6M8 220
#define FINAL_RX_TIMEOUT_UUS_850K 5000
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT_6M8 5
#define PRE_TIMEOUT_850K 0

/* Timestamps of frames transmission/reception. */
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;
/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static uint8_t dist_str[30];
static double tof;
static double distance;
static double distances[10] = {0};
static uint8_t d_len = 0;
static double new_distances[10] = {0};
static double distances_sum;
static double last_distance;
static uint8_t error_times = 0;
#define MAX_DISTANCE_ERROR	20
/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
static dwt_txconfig_t Config_options = 
{
    0x34,           /* PG delay. */
    0xfdfdfdfd,      /* TX power. */
    0x0             /*PG count*/
};
extern dwt_txconfig_t txconfig_options_ch9;


uint8_t ds_twr_responder_init(void)
{
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC)

    flags.option_timeout = 1;
    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
    { 
        if(flags.option_timeout > 300)
        {
            flags.option_timeout = 0;
            HMISends("loading.L1.txt=\"Init fail.IDLERC\"\xff\xff\xff");
            Sleep(1000);
            HMISends("page menu\xff\xff\xff"); 
            return 1;
        }
    };

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        HMISends("loading.L1.txt=\"Init fail.DWTINIT\"\xff\xff\xff");
        Sleep(1000); 
        HMISends("page menu\xff\xff\xff");
        return 1;
    }    
    
    dwt_structs_init(DS_TWR_RESPINDER, &config, &Config_options);
    
    if(dwt_configure(&config)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    {
        HMISends("loading.L1.txt=\"Init fail.CONFIG\"\xff\xff\xff");
        Sleep(1000);
        HMISends("page menu\xff\xff\xff");
        return 1;
    }

    dwt_configuretxrf(&Config_options);
    
	/* Set the antenna delay. Modify the parameters to adjust the distance error */
    if(dwt_setting_data.dwt_setting_value[DS_TWR_RESPINDER].dev_id == DEV_UWB3000F27)
    {
        dwt_settxantennadelay(TX_ANT_DLY_HP);
        dwt_setrxantennadelay(RX_ANT_DLY_HP);
    }
    else
    {
        dwt_settxantennadelay(TX_ANT_DLY_LP);
        dwt_setrxantennadelay(RX_ANT_DLY_LP);
    }
	
	/* If the UWB3000F27 module is used, DWT_LNA_ENABLE and DWT_PA_ENABLE must be enabled; otherwise, the power amplifier circuit cannot be started */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE | DWT_TXRX_EN);
    dwt_setfinegraintxseq(0);
	
    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
     * Note, in real low power applications the LEDs should not be used. */
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
	d_len = 0;
	error_times = 0;
    flags.ds_twr_timeout = 1;
    HMISends("page rangingresp\xff\xff\xff");
    return 0;
}
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn ds_twr_responder()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
void ds_twr_responder(void)
{
	/* Loop forever responding to ranging requests. */
	dwt_setpreambledetecttimeout(0);
	
	/* Clear reception timeout to start next ranging process. */
	dwt_setrxtimeout(0);
	
	/* Activate reception immediately. */
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
	/* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
	{
		if(hmi_recv() == 1)
			return;
		if(flags.opt_1ms_flag == 10)
		{
			flags.opt_1ms_flag = 0;
			key_scan();
		}
		if(flags.ds_twr_timeout > 3000)
		{
			flags.ds_twr_timeout = 1;
			HMISends("rangingresp.t1.txt=\"???\"\xff\xff\xff");
		}
	};
	if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
	{
		uint32_t frame_len;
		/* Clear good RX frame event in the DW IC status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

		/* A frame has been received, read it into the local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
		if (frame_len <= RX_BUF_LEN)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}
		
		/* Check that the frame is a poll sent by "DS TWR initiator" example.
		 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
		rx_buffer[ALL_MSG_SN_IDX] = 0;
		if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
		{
			uint32_t resp_tx_time;
			int ret;

			/* Retrieve poll reception timestamp. */
			poll_rx_ts = get_rx_timestamp_u64();
			
			
			/* Set send time for response. See NOTE 9 below. */
			if(dwt_setting_data.dwt_setting_value[DS_TWR_RESPINDER].data_rate == RATE_6M8)
				resp_tx_time = (uint32_t)((poll_rx_ts + ((POLL_RX_TO_RESP_TX_DLY_UUS_6M8) * UUS_TO_DWT_TIME)) >> 8);
			else
				resp_tx_time = (uint32_t)((poll_rx_ts + ((POLL_RX_TO_RESP_TX_DLY_UUS_850K) * UUS_TO_DWT_TIME)) >> 8);

			dwt_setdelayedtrxtime(resp_tx_time);
					 

			/* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
			if(dwt_setting_data.dwt_setting_value[DS_TWR_RESPINDER].dev_id == DEV_UWB3000F27)
				resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY_HP;
			else
				resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY_LP;
			
			/* Write all timestamps in the final message. See NOTE 8 below. */
			resp_msg_set_ts(&tx_resp_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_rx_ts);
			resp_msg_set_ts(&tx_resp_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_tx_ts);
			
			if(dwt_setting_data.dwt_setting_value[DS_TWR_RESPINDER].data_rate == RATE_6M8)
			{
				/* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
				dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS_6M8); 
				/* FINAL_RX_TIMEOUT_UUS. */
				dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS_6M8);                
				/* Set preamble timeout for expected frames. See NOTE 6 below. */
				dwt_setpreambledetecttimeout(PRE_TIMEOUT_6M8);
			}
			else
			{
				/* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
				dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS_850K); 
				/* FINAL_RX_TIMEOUT_UUS. */
				dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS_850K);                
				/* Set preamble timeout for expected frames. See NOTE 6 below. */
				dwt_setpreambledetecttimeout(PRE_TIMEOUT_850K);
			}  
			/* Write and send the response message. See NOTE 10 below.*/
			tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
			dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
			dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
			ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);//DWT_START_TX_DELAYED DWT_START_TX_IMMEDIATE
			
			/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
			if (ret == DWT_ERROR)
			{
				flags.target_allow_run_time = DSTWR_RESP_RERUN_INTERVAL;
				flags.time_to_allow_run = 1;
				return;
			}
			
			/* Poll for reception of expected "final" frame or error/timeout. See NOTE 8 below. */
			while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
			{
				
			};
			/* Increment frame sequence number after transmission of the response message (modulo 256). */
			frame_seq_nb++;
			
			if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
			{
				/* Clear good RX frame event and TX frame sent in the DW IC status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

				/* A frame has been received, read it into the local buffer. */
				frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
				if (frame_len <= RX_BUF_LEN)
				{
					dwt_readrxdata(rx_buffer, frame_len, 0);
				}

				/* Check that the frame is a final message sent by "DS TWR initiator" example.
				 * As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
				rx_buffer[ALL_MSG_SN_IDX] = 0;
				
				if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
				{
					uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
					uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
					double Ra, Rb, Da, Db;
					int64_t tof_dtu;

					/* Retrieve response transmission and final reception timestamps. */
					resp_tx_ts = get_tx_timestamp_u64();
					final_rx_ts = get_rx_timestamp_u64();

					/* Get timestamps embedded in the final message. */
					final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
					final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
					final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

					/* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
					poll_rx_ts_32 = (uint32_t)poll_rx_ts;
					resp_tx_ts_32 = (uint32_t)resp_tx_ts;
					final_rx_ts_32 = (uint32_t)final_rx_ts;
					Ra = (double)(resp_rx_ts - poll_tx_ts);
					Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
					Da = (double)(final_tx_ts - resp_rx_ts);
					Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
					tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

					tof = tof_dtu * DWT_TIME_UNITS;
					distance = tof * SPEED_OF_LIGHT;
					//The data were smoothed and filtered
					if(d_len < 10)
					{
						distances[d_len] = distance;
						d_len++;
						distances_sum = 0;
						for(int i = 0; i < d_len; i++)
						{
							distances_sum += distances[i];
						}
						distance = distances_sum / (double)d_len;
					}
					else
					{
						if(fabs(distance-last_distance) < MAX_DISTANCE_ERROR)
						{
							error_times = 0;
							for(int i = 0; i < d_len-1; i++)
							{
								new_distances[i] = distances[i+1];
							}
							new_distances[d_len-1] = distance;
							distances_sum = 0;
							for(int i = 0; i < d_len; i++)
							{
								distances_sum += new_distances[i];
								distances[i] = new_distances[i];
							}
							last_distance = distance;
							distance = distances_sum / (double)d_len;
						}
						else
						{
							distance = last_distance;
							error_times++;
							if(error_times > 20)
							{
								error_times = 0;
								d_len = 0;
							}
						}
					}
					/* Display computed distance on HMI display. */
					sprintf(dist_str, "rangingresp.t1.txt=\"%.2f\"\xff\xff\xff", distance);
					HMISends(dist_str);
					flags.ds_twr_timeout = 1;
				}
			}
			else
			{           
				/* Clear RX error/timeout events in the DW IC status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
			}
		}
	}
	else
	{ 
		/* Clear RX error/timeout events in the DW IC status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
	}
	
	/* Data is sent to the HMI display due to a long period of time without updating the ranging data */
	if(flags.ds_twr_timeout > 3000)
	{
		flags.ds_twr_timeout = 1;
		HMISends("rangingresp.t1.txt=\"???\"\xff\xff\xff");
	}

	/* Changing the value of target_allow_run_time adjusts the interval (in ms) between runs of the example again */
    flags.target_allow_run_time = DSTWR_RESP_RERUN_INTERVAL;
    flags.time_to_allow_run = 1;
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
 *     - byte 11/12: activity parameter, not used for activity code 0x02.
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
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete final frame sent by the responder at the
 *    6.81 Mbps data rate used (around 200 us).
 * 6. The preamble timeout allows the receiver to stop listening in situations where preamble is not starting (which might be because the responder is
 *    out of range or did not receive the message to respond to). This saves the power waste of listening for a message that is not coming. We
 *    recommend a minimum preamble timeout of 5 PACs for short range applications and a larger value (e.g. in the range of 50% to 80% of the preamble
 *    length) for more challenging longer range, NLOS or noisy environments.
 * 7. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW IC User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 9. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to poll RX
 *    timestamp to get response transmission time. The delayed transmission time resolution is 512 device time units which means that the lower 9 bits
 *    of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower 8 bits.
 * 10. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *     automatically appended by the DW IC. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *     work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 11. When running this example on the DWK3000 platform with the POLL_RX_TO_RESP_TX_DLY response delay provided, the dwt_starttx() is always
 *     successful. However, in cases where the delay is too short (or something else interrupts the code flow), then the dwt_starttx() might be issued
 *     too late for the configured start time. The code below provides an example of how to handle this condition: In this case it abandons the
 *     ranging exchange and simply goes back to awaiting another poll message. If this error handling code was not here, a late dwt_starttx() would
 *     result in the code flow getting stuck waiting subsequent RX event that will will never come. The companion "initiator" example (ex_05a) should
 *     timeout from awaiting the "response" and proceed to send another poll in due course to initiate another ranging exchange.
 * 12. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *     subtraction.
 * 13. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW IC API Guide for more details on the DW IC driver functions.
 * 14. In this example, the DW IC is put into IDLE state after calling dwt_initialise(). This means that a fast SPI rate of up to 38 MHz can be used
 *     thereafter.
 * 15. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *     configuration.
 ****************************************************************************************************************************************************/
