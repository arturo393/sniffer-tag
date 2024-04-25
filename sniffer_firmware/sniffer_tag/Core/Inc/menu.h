#ifndef MENU_H
#define MENU_H

#include "main.h"

//对应HMI工程文件中的页面顺序
enum hmi_pages
{
    STARTUP = 1,
    MENU,
    LONGTXCHOOSE,
    SIMGLETXCHOOSE,
    RANGINGCHOOSE,
    LOCATIONCHOOSE,
    LOCA_ID_CHOOSE,
    SETTING,
    LONGTXWAVE,
    LONGTXFRAME,
    SIMGLETX,
    SIMGLERX,
    RANGINGINIT,
    RANGINGRESP,
    LOADING,
    ABOUT,
    DRAW,
    MAX_PAGE,
};

void page_opts_sethighlight(enum hmi_pages page, uint8_t opt_value);
void key_options(uint16_t key_value);

#endif
