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

int dw3000_local_init(dwt_local_data_t* dwt_local_data, int mode);
uint8_t tag_initiator_init(dwt_config_t *dwt_config,dwt_txconfig_t *dwt_txconfig,dwt_local_data_t* dwt_local_data);
void tag_initiator(void);
uint32_t dwt_otp_read(uint16_t address);
int dwt_config2(dwt_config_t *config,dwt_local_data_t *dwt_local_data);
uint16_t get_sts_mnth(uint16_t sts, uint8_t default_threshold, uint8_t shift_val);
void _dwt_kick_dgc_on_wakeup(int8_t channel);
#endif /* INC_TAG_C_ */
