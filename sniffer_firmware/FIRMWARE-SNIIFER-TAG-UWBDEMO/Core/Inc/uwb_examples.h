#ifndef UWB_EXAMPLE_H
#define UWB_EXAMPLE_H

#include "Application.h"
#include "uwb.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "shared_defines.h"
#include "shared_functions.h"
#include "example_selection.h"
#include "hmi_uart.h"

/* When using the ranging example, if the displayed distance value does not match the actual value, you can adjust the error by modifying here */
#define TX_ANT_DLY_HP 16410
#define RX_ANT_DLY_HP 16410
#define TX_ANT_DLY_LP 16370
#define RX_ANT_DLY_LP 16370

#define SIMGLE_TX_RERUN_INTERVAL	1000
#define SIMGLE_RX_RERUN_INTERVAL	990

#define DSTWR_INIT_RERUN_INTERVAL	100
#define DSTWR_RESP_RERUN_INTERVAL	90

uint8_t simple_tx_init(void);
void simple_tx(void);
void tx_reset_count(void);

uint8_t simple_rx_init(void);
void simple_rx(void);
void rx_reset_count(void);

uint8_t ds_twr_initiator_init(void);
void ds_twr_initiator(void);

uint8_t ds_twr_responder_init(void);
void ds_twr_responder(void);

void dwt_structs_init(uint8_t page, dwt_config_t* dwt_config, dwt_txconfig_t* dwt_txconfig);
#endif
