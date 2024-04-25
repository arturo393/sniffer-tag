#ifndef APPLICATION_H
#define APPLICATION_H

#include "main.h"

/**
  * @brief  UWB Module set options structures definition
  */
enum uwb_device_opts
{
	DEV_UWB3000F27,
	DEV_UWB3000F00
};
enum uwb_rate_opts
{
	RATE_6M8,
	RATE_580K
};
enum uwb_channels
{
	CHANNEL_5,
	CHANNEL_9
};
enum uwb_gain_opts
{
	GAIN_30DB,
	GAIN_27DB,
	GAIN_24DB,
	GAIN_21DB,
	GAIN_18DB,
	GAIN_15DB,
	GAIN_12DB,
	GAIN_9DB,
	GAIN_6DB,
	GAIN_3DB,
	GAIN_0DB
};


enum functions
{
    SIMPLE_TX = 3,
    SIMPLE_RX,
    DS_TWR_INITIATOR,
    DS_TWR_RESPINDER,
    MAX_FUNCTIONS,
};

struct examples_func_t
{
    uint8_t(*initfunction)(void);
    void(*function)(void);
};

struct dwt_setting_value_t
{
    uint8_t channel;
    uint8_t data_rate;
    uint8_t tx_power;
    uint8_t bypass_mode_en;
    uint8_t dev_id;
};


struct dwt_setting_data_t
{
    uint8_t func_idx;
    uint8_t setting_ischanged;
    struct dwt_setting_value_t dwt_setting_value[MAX_FUNCTIONS];
};

extern struct dwt_setting_data_t dwt_setting_data;

void dwt_setting_data_init(void);
uint8_t hmi_recv(void);

#endif

