#include "uwb_examples.h"
#include "stdio.h"


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

/* The frame sent in this example is a blink encoded as per the ISO/IEC 24730-62:2013 standard. It is a 14-byte frame composed of the following fields:
 *     - byte 0: frame control (0xC5 to indicate a multipurpose frame using 64-bit addressing).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 *     - byte 10: encoding header (0x43 to indicate no extended ID, temperature, or battery status is carried in the message).
 *     - byte 11: EXT header (0x02 to indicate tag is listening for a response immediately after this message).
 *     - byte 12/13: frame check-sum, automatically set by DW IC. */
static uint8_t tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0x43, 0x02, 0, 0};
/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 1000

/* Delay from end of transmission to activation of reception, expressed in UWB microseconds (1 uus is 512/499.2 microseconds). See NOTE 2 below. */
#define TX_TO_RX_DELAY_UUS 60

/* Receive response timeout, expressed in UWB microseconds. See NOTE 4 below. */
#define RX_RESP_TO_UUS 50000

/* Buffer to store received frame. See NOTE 5 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];

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
static uint16_t frame_len = 0;
static uint16_t tx_cnt = 0;
static uint16_t rx_cnt = 0;
static char msg_str[17] = {0};
void tx_reset_count(void)
{
    tx_cnt = 0;
    rx_cnt = 0;
}
uint8_t simple_tx_init(void)
{
    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)
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
    flags.option_timeout = 0;
	
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        HMISends("loading.L1.txt=\"Init fail.DWTINIT\"\xff\xff\xff");
        Sleep(1000); 
        HMISends("page menu\xff\xff\xff");
        return 1;
    }

    /* Configure DW IC. See NOTE 8 below. */
    dwt_structs_init(SIMPLE_TX, &config, &Config_options);
    if(dwt_configure(&config)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    {
        HMISends("loading.L1.txt=\"Init fail.CONFIG\"\xff\xff\xff");
        Sleep(1000);
        HMISends("page menu\xff\xff\xff");
        return 1;
    }
    
	/* If the UWB3000F27 module is used, DWT_LNA_ENABLE and DWT_PA_ENABLE must be enabled; otherwise, the power amplifier circuit cannot be started */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE | DWT_TXRX_EN);
    dwt_setfinegraintxseq(0);
    
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    
    /* Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards. */
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK) ;

    /* Configure the TX spectrum parameters (power PG delay and PG Count) */
    dwt_configuretxrf(&Config_options);
    
    /* Set delay to turn reception on after transmission of the frame. See NOTE 3 below. */
    dwt_setrxaftertxdelay(TX_TO_RX_DELAY_UUS);

    /* Set response frame timeout. */
    dwt_setrxtimeout(RX_RESP_TO_UUS);
	
    HMISends("page simgletx\xff\xff\xff");
    tx_reset_count();
    return 0;
}
/**
 * Application entry point.
 */
void simple_tx(void)
{
    /* Write frame data to DW3000 and prepare transmission. See NOTE 7 below. */
    dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

    tx_cnt++;
    sprintf(msg_str, "simgletx.t1.txt=\"%d\"\xff\xff\xff", tx_cnt);
    HMISends(msg_str);
    

    /* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    /* We assume that the transmission is achieved normally, now poll for reception of a frame or error/timeout.*/
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    { 
    };

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
    {
        int i;

        /* Clear local RX buffer to avoid having leftovers from previous receptions. This is not necessary but is included here to aid reading
         * the RX buffer. */
        for (i = 0 ; i < FRAME_LEN_MAX; i++ )
        {
            rx_buffer[i] = 0;
        }

        /* A frame has been received, copy it to our local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
        if (frame_len <= FRAME_LEN_MAX)
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);
            rx_cnt++;
            sprintf(msg_str, "simgletx.t2.txt=\"%d\"\xff\xff\xff", rx_cnt);
            HMISends(msg_str);
        }

        /* Clear good RX frame event in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    }
    else
    {
        /* Clear RX error/timeout events in the DW3000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }

    /* Increment the blink frame sequence number (modulo 256). */
    tx_msg[BLINK_FRAME_SN_IDX]++;
	
	/* Changing the value of target_allow_run_time adjusts the interval (in ms) between runs of the example again */
    flags.target_allow_run_time = SIMGLE_TX_RERUN_INTERVAL;
    flags.time_to_allow_run = 1;	/* start counting */
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The device ID is a hard coded constant in the blink to keep the example simple but for a real product every device should have a unique ID.
 *    For development purposes it is possible to generate a DW IC unique ID by combining the Lot ID & Part Number values programmed into the
 *    DW IC during its manufacture. However there is no guarantee this will not conflict with someone else�s implementation. We recommended that
 *    customers buy a block of addresses from the IEEE Registration Authority for their production items. See "EUI" in the DW IC User Manual.
 * 2. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 3. dwt_writetxdata() takes the full size of tx_msg as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW IC. This means that our tx_msg could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 4. We use polled mode of operation here to keep the example as simple as possible, but the TXFRS status event can be used to generate an interrupt.
 *    Please refer to DW IC User Manual for more details on "interrupts".
 * 5. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *    configuration.
 ****************************************************************************************************************************************************/

