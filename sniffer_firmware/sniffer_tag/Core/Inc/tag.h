/*
 * tag.c
 *
 *  Created on: Apr 26, 2024
 *      Author: uqommdev
 */

#ifndef INC_TAG_C_
#define INC_TAG_C_

#include "main.h"
#include "Application.h"
#include "string.h"
#include "math.h"

/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20

#define MAX_DISTANCE_ERROR	20

/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW IC's delayed TX function. This includes the
 * frame length of approximately 190 us with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS_6M8 700
#define RESP_RX_TO_FINAL_TX_DLY_UUS_850K 1500

// OTP addresses definitions
#define LDOTUNELO_ADDRESS (0x04)
#define LDOTUNEHI_ADDRESS (0x05)
#define PARTID_ADDRESS  (0x06)
#define LOTID_ADDRESS   (0x07)
#define VBAT_ADDRESS    (0x08)
#define VTEMP_ADDRESS   (0x09)
#define XTRIM_ADDRESS   (0x1E)
#define OTPREV_ADDRESS  (0x1F)
#define BIAS_TUNE_ADDRESS (0xA)
#define DGC_TUNE_ADDRESS (0x20)

#define CIA_MANUALLOWERBOUND_TH_64  (0x10) //cia lower bound threshold values for 64 MHz PRF
#define STSQUAL_THRESH_64 (0.90f)

#define GAIN_30DB  0xffffffff  // 30db
#define GAIN_27DB  0xfcfcfcfc  // 27db
#define GAIN_24DB  0x7c7c7c7c  // 24db
#define GAIN_21DB  0x48484848  // 21db
#define GAIN_18DB  0x30303030  // 18db
#define GAIN_15DB  0x20202020  // 15db
#define GAIN_12DB  0x14141414  // 12db
#define GAIN_9DB   0x18181818  //  9db
#define GAIN_6DB   0x06060606  //  6db
#define GAIN_3DB   0x04040404  //  3db
#define GAIN_0DB   0x00000000  //  0db

#define DISTANCE_READINGS 10
typedef struct{
	double tof;
	double value;
	double readings[DISTANCE_READINGS];
	double new[DISTANCE_READINGS];
	double sum;
	double last;
	uint8_t error_times;
	uint8_t counter;
}Distance_t;
// -------------------------------------------------------------------------------------------------------------------
// Data for DW3000 Decawave Transceiver control
//
// Structure to hold the device data
typedef struct
{
    uint32_t      partID ;            // IC Part ID - read during initialisation
    uint32_t      lotID ;             // IC Lot ID - read during initialisation
    uint8_t       bias_tune;          // bias tune code
    uint8_t       dgc_otp_set;        // Flag to check if DGC values are programmed in OTP
    uint8_t       vBatP;              // IC V bat read during production and stored in OTP (Vmeas @ 3V3)
    uint8_t       tempP;              // IC temp read during production and stored in OTP (Tmeas @ 23C)
    uint8_t       longFrames ;        // Flag in non-standard long frame mode
    uint8_t       otprev ;            // OTP revision number (read during initialisation)
    uint8_t       init_xtrim;         // initial XTAL trim value read from OTP (or defaulted to mid-range if OTP not programmed)
    uint8_t       dblbuffon;          // Double RX buffer mode and DB status flag
    uint16_t      sleep_mode;         // Used for automatic reloading of LDO tune and microcode at wake-up
    int16_t       ststhreshold;       // Threshold for deciding if received STS is good or bad
    dwt_spi_crc_mode_e   spicrc;      // Use SPI CRC when this flag is true
    uint8_t       stsconfig;          // STS configuration mode
    uint8_t       cia_diagnostic;     // CIA dignostic logging level
    dwt_cb_data_t cbData;             // Callback data structure
    dwt_spierrcb_t cbSPIRDErr;        // Callback for SPI read error events
    dwt_cb_t    cbTxDone;             // Callback for TX confirmation event
    dwt_cb_t    cbRxOk;               // Callback for RX good frame event
    dwt_cb_t    cbRxTo;               // Callback for RX timeout events
    dwt_cb_t    cbRxErr;              // Callback for RX error events
    dwt_cb_t    cbSPIErr;             // Callback for SPI error events
    dwt_cb_t    cbSPIRdy;             // Callback for SPI ready events
} dwt_local_data_t ;

typedef struct{
uint32_t id;
uint32_t timestamp;
double distance;
} TAG_t;

int dwt_local_data_init(dwt_local_data_t* dwt_local_data);
uint8_t tag_initiator_init(dwt_config_t *dwt_config,dwt_txconfig_t *dwt_txconfig,dwt_local_data_t* dwt_local_data);
void tag_initiator(uint8_t*tx_poll_msg);
uint32_t dwt_otp_read(uint16_t address);
int dwt_config2(dwt_config_t *config,dwt_local_data_t *dwt_local_data);
uint16_t get_sts_mnth(uint16_t sts, uint8_t default_threshold, uint8_t shift_val);
void _dwt_kick_dgc_on_wakeup(int8_t channel);
double calculate_tag_distance(uint8_t*rx_buffer,Distance_t* distance);
uint8_t send_message_with_timestamps(uint8_t*tx_final_msg,uint8_t size,uint8_t frame_seq_nb2);
#endif /* INC_TAG_C_ */
