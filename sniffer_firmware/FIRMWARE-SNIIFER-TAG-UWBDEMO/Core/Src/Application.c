/* ----------------------------------------------------------------------------
 * @file    Application.c
 * @brief   
 *
 * @attention
 *
 * Copyright(c) 2020 NiceRF Wireless Technology Co., Ltd.
 *
 * All rights reserved.
 *
 */
#include "Application.h"
#include "uwb_examples.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "menu.h"

uint8_t running_device;

extern uint8_t recvBuf[100];
extern uint16_t recvBufLen;

extern uint8_t hmi_page;
extern uint8_t cur_opt[MAX_PAGE];

static struct examples_func_t example_func[MAX_FUNCTIONS] = {
    [SIMPLE_TX]={
        simple_tx_init,
        simple_tx,
    },
    [SIMPLE_RX]={
        simple_rx_init,
        simple_rx,
    },
    [DS_TWR_INITIATOR]={
        ds_twr_initiator_init,
        ds_twr_initiator,
    },
    [DS_TWR_RESPINDER]={
        ds_twr_responder_init,
        ds_twr_responder,
    },
};

struct dwt_setting_data_t dwt_setting_data;

/**
  * @brief The preset parameters of the UWB module are initialized
  * @param None
  * @retval None
  */
void dwt_setting_data_init(void)
{
	for(int i = SIMPLE_TX; i < MAX_FUNCTIONS; i++)
	{
		dwt_setting_data.dwt_setting_value[i].dev_id = DEV_UWB3000F27;
		dwt_setting_data.dwt_setting_value[i].channel = CHANNEL_5;
		dwt_setting_data.dwt_setting_value[i].data_rate = RATE_6M8;
		dwt_setting_data.dwt_setting_value[i].tx_power = GAIN_30DB;
	}
}

/**
  * @brief Data from the HMI display is processed here
  * @param None
  * @retval None
  */
uint8_t hmi_recv(void)
{
    if(flags.uart_recv)
    {
        if(recvBuf[recvBufLen-1] == 0xff && recvBuf[recvBufLen-2] == 0xff && recvBuf[recvBufLen-3] == 0xff)
        {
            if(recvBufLen == 5)
            {
                if(recvBuf[0] == 0x00)
                {
                    if(recvBuf[1] == 0xff || recvBuf[1] == 0x00)	
                    {
						/* Exit the running example and return to the menu */
                        reset_DWIC();  	//Reset the module to stop working
                        HMISends("page menu\xff\xff\xff");//Send the command back to the menu to the HMI display
                        flags.function = NULL;
                        flags.time_to_allow_run = 0;
                        flags.func_allow_run = 0;
                        flags.target_allow_run_time = 0;
                    }
                }
                else
                {
					/* Ready to run example */
                    if(recvBuf[0] == 0x01)
                    {
						HMISends("page loading\xff\xff\xff");
                        reset_DWIC(); 
                        running_device = dwt_setting_data.dwt_setting_value[recvBuf[1]].dev_id;
                        //Initialize the UWB module before running the example
						if(example_func[recvBuf[1]].initfunction() == 0)
                        {
                            flags.function = example_func[recvBuf[1]].function;
                            flags.func_allow_run = 1;
                        }
                    }
					/* Ready to run example */
                    else if(recvBuf[0] == 0x02)
                    {
						//While running the example, open the Settings screen, read the current parameters and display them on the HMI display
						int dev_id = dwt_setting_data.dwt_setting_value[recvBuf[1]].dev_id;
                        int channel = dwt_setting_data.dwt_setting_value[recvBuf[1]].channel;
                        int datarate = dwt_setting_data.dwt_setting_value[recvBuf[1]].data_rate;
                        int powgain = dwt_setting_data.dwt_setting_value[recvBuf[1]].tx_power;
                        uint8_t settingstr[20];
						sprintf(settingstr, "deviceid=%d\xff\xff\xff", dev_id);
                        HMISends(settingstr);
                        sprintf(settingstr, "channel=%d\xff\xff\xff", channel);
                        HMISends(settingstr);
                        sprintf(settingstr, "datarage=%d\xff\xff\xff", datarate);
                        HMISends(settingstr);
                        sprintf(settingstr, "powgain=%d\xff\xff\xff", powgain);
                        HMISends(settingstr);
                        HMISends("page setting\xff\xff\xff");
                    }
                } 
            }
            else if(recvBufLen == 10)
            {
                if(recvBuf[0] == 0x02)
                {
                    if(recvBuf[1] == 0xff)
                    {
						/* Global setting. All examples apply this parameter (menu -> setting)*/
                        for(int i = 1; i < MAX_FUNCTIONS; i++)
                        {
                            dwt_setting_data.dwt_setting_value[i].channel = recvBuf[2];
                            dwt_setting_data.dwt_setting_value[i].data_rate = recvBuf[3];
                            dwt_setting_data.dwt_setting_value[i].tx_power = recvBuf[4];
							dwt_setting_data.dwt_setting_value[i].dev_id = recvBuf[6];
                        }
                        HMISends("page menu\xff\xff\xff");
                    }
                    else
                    {
						//Once the parameters are set, rerun the example to apply the parameters (example -> setting)
                        dwt_setting_data.dwt_setting_value[recvBuf[1]].channel = recvBuf[2];
                        dwt_setting_data.dwt_setting_value[recvBuf[1]].data_rate = recvBuf[3];
                        dwt_setting_data.dwt_setting_value[recvBuf[1]].tx_power = recvBuf[4];
						dwt_setting_data.dwt_setting_value[recvBuf[1]].dev_id = recvBuf[6];
                        HMISends("page loading\xff\xff\xff");
                        reset_DWIC(); 
                        running_device = dwt_setting_data.dwt_setting_value[recvBuf[1]].dev_id;
                        if(flags.function == example_func[recvBuf[1]].function)
                        {
                            if(example_func[recvBuf[1]].initfunction() == 1)
                            {
                                flags.function = NULL;
                                flags.func_allow_run = 0;
                            }
                            else
                            {
                                flags.func_allow_run = 1;
                            }
                        }
                        else
                        {
                            flags.function = NULL;
                            HMISends("page menu\xff\xff\xff");
                        }
                    }
                }
            }
            else if(recvBufLen == 6)
            {
				/* Record the current page of the HMI display */
                if(recvBuf[0] == 0xaa && recvBuf[1] == 0xbb)
                {
                    switch(recvBuf[2])
                    {
                        case 0x01:
                            NVIC_SystemReset();
                        break;
                        case 0x02:
                            hmi_page = MENU;
                        break;
                        case 0x04:
                            hmi_page = SIMGLETXCHOOSE;
                        break;
                        case 0x05:
                            hmi_page = RANGINGCHOOSE;
                        break;
                        case 0x08:
                            hmi_page = SETTING;
							cur_opt[hmi_page] = 0;//It resets its highlight position each time you enter the screen
                        break;
                        case 0x0b:
                            hmi_page = SIMGLETX;
                        break;
                        case 0x0c:
                            hmi_page = SIMGLERX;
							flags.target_allow_run_time = SIMGLE_RX_RERUN_INTERVAL;
							flags.time_to_allow_run = 1;
                        break;
                        case 0x0d:
                            hmi_page = RANGINGINIT;
                        break;
                        case 0x0e:
                            hmi_page = RANGINGRESP;
							flags.target_allow_run_time = DSTWR_RESP_RERUN_INTERVAL;
							flags.time_to_allow_run = 1;
                        break;
                        case 0x0f:
                            hmi_page = LOADING;
                        break;
                        case 0x10:
                            hmi_page = ABOUT;
                        break;
						default:break;
                    }
                    page_opts_sethighlight(hmi_page, cur_opt[hmi_page]);
                }
            }
        }
        flags.uart_recv = 0; 
        recvBufLen = 0;
        return 1;
    }
    return 0;
}


void dwt_structs_init(uint8_t func_idx, dwt_config_t* dwt_config, dwt_txconfig_t* dwt_txconfig)
{
    if(dwt_setting_data.dwt_setting_value[func_idx].channel == 1)
        dwt_config->chan = 9;
    else
        dwt_config->chan = 5;
    
    if(dwt_setting_data.dwt_setting_value[func_idx].data_rate == 1)
        dwt_config->dataRate = DWT_BR_850K;
    else
        dwt_config->dataRate = DWT_BR_6M8;
    
	/* You can modify this to run other power gains */
	// The following parameters may not accurately correspond to the corresponding gain and are only an estimate
    uint32_t powervalue[11] = {
        [GAIN_30DB] = 0xffffffff,     //30db
        [GAIN_27DB] = 0xfcfcfcfc,     //27db
        [GAIN_24DB] = 0x7c7c7c7c,     //24db
        [GAIN_21DB] = 0x48484848,     //21db
        [GAIN_18DB] = 0x30303030,     //18db 
        [GAIN_15DB] = 0x20202020,     //15db
        [GAIN_12DB] = 0x14141414,     //12db
        [GAIN_9DB]  = 0x18181818,     //9db
        [GAIN_6DB]  = 0x06060606,     //6db      
        [GAIN_3DB]  = 0x04040404,     //3db
        [GAIN_0DB]  = 0x00000000      //0db
    };    
    dwt_txconfig->power = powervalue[dwt_setting_data.dwt_setting_value[func_idx].tx_power];
}

