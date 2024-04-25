#include "uwb_examples.h"
#include "stdio.h"
#include "key.h"

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0      /* PDOA mode off */
};

/* Default antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* As "TX then wait for a response" example sends a blink message encoded as per the ISO/IEC 24730-62:2013 standard which includes a bit signalling
 * that a response is listened for, this example will respond with a valid frame (that will be ignored anyway) following the same standard. The
 * response is a 21-byte frame composed of the following fields:
 *     - byte 0/1: frame control (0x8C41 to indicate a data frame using 16-bit source addressing and 64-bit destination addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: application ID (0x609A for data frames in this standard).
 *     - byte 5 -> 12: 64-bit destination address.
 *     - byte 13/14: 16-bit source address, hard coded in this example to keep it simple.
 *     - byte 15: function code (0x10 to indicate this is an activity control message).
 *     - byte 16: activity code (0x00 to indicate activity is finished).
 *     - byte 17/18: new tag blink rate.
 *     - byte 19/20: frame check-sum, automatically set by DW IC.  */
static uint8_t tx_msg[] = {0x41, 0x8C, 0, 0x9A, 0x60, 0, 0, 0, 0, 0, 0, 0, 0, 'D', 'W', 0x10, 0x00, 0, 0, 0, 0};
/* Indexes to access to sequence number and destination address of the data frame in the tx_msg array. */
#define DATA_FRAME_SN_IDX 2
#define DATA_FRAME_DEST_IDX 5

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 1000

/* Buffer to store received frame. See NOTE 1 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];
/* Index to access to source address of the blink frame in the rx_buffer array. */
#define BLINK_FRAME_SRC_IDX 2

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
static dwt_txconfig_t Config_options = 
{
    0x34,           /* PG delay. */
    0xfdfdfdfd,      /* TX power. */
    0x0             /*PG count*/
};

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;
/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
static uint16_t frame_len = 0;
static uint16_t tx_cnt = 0;
static uint16_t rx_cnt = 0;
static char msg_str[17] = {0};
void rx_reset_count(void)
{
    tx_cnt = 0;
    rx_cnt = 0;
}

uint8_t simple_rx_init(void)
{
    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)
    flags.option_timeout = 1;
    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
    {
        hmi_recv();
        if(flags.option_timeout > 300)
        {
            flags.option_timeout = 0;
            HMISends("loading.L1.txt=\"Init fail.IDLERC\"\xff\xff\xff");
            Sleep(1000);
            HMISends("page menu\xff\xff\xff"); 
            return 1;
        }
    };
    flags.option_timeout = 0;
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        HMISends("loading.L1.txt=\"Init fail.DWTINIT\"\xff\xff\xff");
        Sleep(1000); 
        HMISends("page menu\xff\xff\xff");
        return 1;
    }

    /* Configure DW IC. */
    dwt_structs_init(SIMPLE_RX, &config, &Config_options);
    if(dwt_configure(&config)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    {
        HMISends("loading.L1.txt=\"Init fail.CONFIG\"\xff\xff\xff");
        Sleep(1000);
        HMISends("page menu\xff\xff\xff");
        return 1;
    }
    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&Config_options);
    
	/* If the UWB3000F27 module is used, DWT_LNA_ENABLE and DWT_PA_ENABLE must be enabled; otherwise, the power amplifier circuit cannot be started */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE | DWT_TXRX_EN);
    dwt_setfinegraintxseq(0);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
        
    /* Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards. */
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK); 
    
    HMISends("page simglerx\xff\xff\xff");
    rx_reset_count();
    return 0;
}
/**
 * Application entry point.
 */
void simple_rx(void)
{
    /* Activate reception immediately. See NOTE 4 below. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE); 

    /* Poll until a frame is properly received or an error occurs. 
     * STATUS register is 5 bytes long but, as the events we are looking at are in the lower bytes of the register, we can use this simplest API
     * function to access it. */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)))
    {
        if(hmi_recv() == 1)
			return;
		if(flags.opt_1ms_flag == 10)
		{
			flags.opt_1ms_flag = 0;
			key_scan();
		}
        //if(flags.key_value == KEY_BACK || flags.key_value == KEY_OK) return;
    };
    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
    {
        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
        if (frame_len <= FRAME_LEN_MAX)
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);
        }
        
        /* Clear good RX frame event in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        
        /* Validate the frame is the one expected as sent by "TX then wait for a response" example. */
        if ((frame_len == 14) && (rx_buffer[0] == 0xC5) && (rx_buffer[10] == 0x43) && (rx_buffer[11] == 0x2))
        {
            rx_cnt++;
            sprintf(msg_str, "t2.txt=\"%d\"\xff\xff\xff", rx_cnt);
            HMISends(msg_str);
            
            int i;

            /* Copy source address of blink in response destination address. */
            for (i = 0; i < 8; i++)
            {
                tx_msg[DATA_FRAME_DEST_IDX + i] = rx_buffer[BLINK_FRAME_SRC_IDX + i];
            }
            /* Write response frame data to DW IC and prepare transmission. See NOTE 6 below.*/
            dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
            dwt_writetxfctrl(sizeof(tx_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */
            

            /* Send the response. */
            dwt_starttx(DWT_START_TX_IMMEDIATE);
            /* Poll DW IC until TX frame sent event set. */
            while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
            { };
            tx_cnt++;
            sprintf(msg_str, "t1.txt=\"%d\"\xff\xff\xff", tx_cnt);
            HMISends(msg_str);
            /* Clear TX frame sent event. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

            /* Increment the data frame sequence number (modulo 256). */
            tx_msg[DATA_FRAME_SN_IDX]++;
        }
    }
    else
    {

        /* Clear RX error events in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    }
	/* Changing the value of target_allow_run_time adjusts the interval (in ms) between runs of the example again */
    flags.target_allow_run_time = SIMGLE_RX_RERUN_INTERVAL;
    flags.time_to_allow_run = 1; /* start counting */
}
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. In this example, maximum frame length is set to 127 bytes which is 802.15.4 UWB standard maximum frame length. DW IC supports an extended
 *    frame length (up to 1023 bytes long) mode which is not used in this example.
 * 2. Manual reception activation is performed here but DW IC offers several features that can be used to handle more complex scenarios or to
 *    optimise system's overall performance (e.g. timeout after a given time, automatic re-enabling of reception in case of errors, etc.).
 * 3. We use polled mode of operation here to keep the example as simple as possible, but RXFCG and error/timeout status events can be used to generate
 *    interrupts. Please refer to DW IC User Manual for more details on "interrupts".
 ****************************************************************************************************************************************************/
