/* ----------------------------------------------------------------------------
 * @file    key.c
 * @brief   
 *
 * @attention
 *
 * Copyright(c) 2020 NiceRF Wireless Technology Co., Ltd.
 *
 * All rights reserved.
 *
 */
#include "key.h"
#include "stm32f1xx_hal_gpio.h"
#include "menu.h"

/**
  *  Key GPIO Configuration
  *  Key UP 	------> PB1
  *  Key DOWN   ------> PB2
  *  Key OK 	------> PB10
  *  Key BACK 	------> PB11
  */
void key_scan(void)
{
    uint16_t key_value;
    key_value = (KEY_PORT->IDR)&KEY_UNKNOWN;
    if(key_value == KEY_UNKNOWN)
    {
        flags.key_ispress = 0;
        flags.key_presstime = 0;
        flags.key_value = KEY_UNKNOWN;
    }
    else
    {
        if(!flags.key_ispress)
        {
            if(flags.key_presstime == 0)
                flags.key_presstime = 1;
            if(flags.key_presstime >= 20)
            {
                flags.key_ispress = 1;
                flags.key_value = key_value;
                key_options(flags.key_value);
            }
        }
    }
}

