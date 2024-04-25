#include "menu.h"
#include "key.h"
#include "hmi_uart.h"
#include <string.h>

uint8_t hmi_page = MENU;
uint8_t cur_opt[MAX_PAGE] = {0};

#define ENGLISH 0
#define CHINESE 1
uint8_t language = ENGLISH;

void page_opts_sethighlight(enum hmi_pages page, uint8_t opt_value)
{
    switch(page)
    {
        case MENU:
        {
            if(opt_value == 0)
            {
                if(language == ENGLISH)
                {
                    HMISends("b5.picc=17\xff\xff\xff");
                    HMISends("b1.picc=31\xff\xff\xff");
                    HMISends("b2.picc=17\xff\xff\xff");
                }
                else if(language == CHINESE)
                {
                    HMISends("b5.picc=1\xff\xff\xff");
                    HMISends("b1.picc=30\xff\xff\xff");
                    HMISends("b2.picc=1\xff\xff\xff");
                }
            }
            else if(opt_value == 1)
            {
                if(language == ENGLISH)
                {
                    HMISends("b1.picc=17\xff\xff\xff");
                    HMISends("b2.picc=31\xff\xff\xff");
                    HMISends("b4.picc=17\xff\xff\xff");
                }
                else if(language == CHINESE)
                {
                    HMISends("b1.picc=1\xff\xff\xff");
                    HMISends("b2.picc=30\xff\xff\xff");
                    HMISends("b4.picc=1\xff\xff\xff");
                }
            }
            else if(opt_value == 2)
            {
                if(language == ENGLISH)
                {
                    HMISends("b2.picc=17\xff\xff\xff");
                    HMISends("b4.picc=31\xff\xff\xff");
                    HMISends("b5.picc=17\xff\xff\xff");
                }
                else if(language == CHINESE)
                {
                    HMISends("b2.picc=1\xff\xff\xff");
                    HMISends("b4.picc=30\xff\xff\xff");
                    HMISends("b5.picc=1\xff\xff\xff");
                }
            }
            else if(opt_value == 3)
            {
                if(language == ENGLISH)
                {
                    HMISends("b4.picc=17\xff\xff\xff");
                    HMISends("b5.picc=31\xff\xff\xff");
                    HMISends("b1.picc=17\xff\xff\xff");
                }
                else if(language == CHINESE)
                {
                    HMISends("b4.picc=1\xff\xff\xff");
                    HMISends("b5.picc=30\xff\xff\xff");
                    HMISends("b1.picc=1\xff\xff\xff");
                }
            }
        }
        break;
        case LONGTXCHOOSE:
        {
            if(opt_value == 0)
            {
                if(language == ENGLISH)
                {
                    HMISends("b0.picc=33\xff\xff\xff");
                    HMISends("b1.picc=18\xff\xff\xff");
                }
                else if(language == CHINESE)
                {
                    HMISends("b0.picc=32\xff\xff\xff");
                    HMISends("b1.picc=2\xff\xff\xff");
                }
            }
            else if(opt_value == 1)
            {
                if(language == ENGLISH)
                {
                    HMISends("b0.picc=18\xff\xff\xff");
                    HMISends("b1.picc=33\xff\xff\xff");
                }
                else if(language == CHINESE)
                {
                    HMISends("b0.picc=2\xff\xff\xff");
                    HMISends("b1.picc=32\xff\xff\xff");
                }
            }
        }
        break;
        case SIMGLETXCHOOSE:
        {
            if(opt_value == 0)
            {
                if(language == ENGLISH)
                {
                    HMISends("b0.picc=35\xff\xff\xff");
                    HMISends("b1.picc=19\xff\xff\xff");
                }
                else if(language == CHINESE)
                {
                    HMISends("b0.picc=34\xff\xff\xff");
                    HMISends("b1.picc=3\xff\xff\xff");
                }
            }
            else if(opt_value == 1)
            {
                if(language == ENGLISH)
                {
                    HMISends("b0.picc=19\xff\xff\xff");
                    HMISends("b1.picc=35\xff\xff\xff");
                }
                else if(language == CHINESE)
                {
                    HMISends("b0.picc=3\xff\xff\xff");
                    HMISends("b1.picc=34\xff\xff\xff");
                }
            }
        }
        break;
        case RANGINGCHOOSE:
        {
            if(opt_value == 0)
            {
                if(language == ENGLISH)
                {
                    HMISends("b0.picc=37\xff\xff\xff");
                    HMISends("b1.picc=20\xff\xff\xff");
                }
                else if(language == CHINESE)
                {
                    HMISends("b0.picc=36\xff\xff\xff");
                    HMISends("b1.picc=4\xff\xff\xff");
                }
            }
            else if(opt_value == 1)
            {
                if(language == ENGLISH)
                {
                    HMISends("b0.picc=20\xff\xff\xff");
                    HMISends("b1.picc=37\xff\xff\xff");
                }
                else if(language == CHINESE)
                {
                    HMISends("b0.picc=4\xff\xff\xff");
                    HMISends("b1.picc=36\xff\xff\xff");
                }
            }
        }
        break;
        case LOCATIONCHOOSE:
        {
            if(opt_value == 0)
            {
                if(language == ENGLISH)
                {
                    HMISends("b0.picc=21\xff\xff\xff");
                    HMISends("b1.picc=39\xff\xff\xff");
                }
                else if(language == CHINESE)
                {
                    HMISends("b0.picc=5\xff\xff\xff");
                    HMISends("b1.picc=38\xff\xff\xff");
                }
            }
            else if(opt_value == 1)
            {
                if(language == ENGLISH)
                {
                    HMISends("b0.picc=39\xff\xff\xff");
                    HMISends("b1.picc=21\xff\xff\xff");
                }
                else if(language == CHINESE)
                {
                    HMISends("b0.picc=38\xff\xff\xff");
                    HMISends("b1.picc=5\xff\xff\xff");
                }
                
            }
        }
        break;
        case LOCA_ID_CHOOSE:
        {
            if(opt_value == 0)
                HMISends("click b4,0\xff\xff\xff");
            else if(opt_value == 1)
                HMISends("click b5,0\xff\xff\xff");
            else if(opt_value == 2)
                HMISends("click b6,0\xff\xff\xff");
            else if(opt_value == 3)
                HMISends("click b7,0\xff\xff\xff");
            else if(opt_value == 4)
                HMISends("click b8,0\xff\xff\xff");
            else if(opt_value == 5)
                HMISends("click b9,0\xff\xff\xff");
            else if(opt_value == 6)
                HMISends("click b10,0\xff\xff\xff");
            else if(opt_value == 7)
                HMISends("click b11,0\xff\xff\xff");
            else if(opt_value == 8)
                HMISends("click b12,0\xff\xff\xff");
            else if(opt_value == 9)
                HMISends("click b13,0\xff\xff\xff");
            else if(opt_value == 10)
                HMISends("click b14,0\xff\xff\xff");
            else if(opt_value == 11)
                HMISends("click b15,0\xff\xff\xff");
        }
        break;
        case SETTING:
        {
			if(opt_value == 0)
                HMISends("click b12,0\xff\xff\xff");
            else if(opt_value == 1)
                HMISends("click b13,0\xff\xff\xff");
            else if(opt_value == 2)
                HMISends("click b14,0\xff\xff\xff");
            else if(opt_value == 3)
                HMISends("click b15,0\xff\xff\xff");
            else if(opt_value == 4)
                HMISends("click b16,0\xff\xff\xff");
        }
        break;
        case ABOUT:
        {
            if(opt_value == 0)
                HMISends("click b3,0\xff\xff\xff");
            else if(opt_value == 1)
                HMISends("click b2,0\xff\xff\xff");
        }
        break;
        default:break;
    }
}



void menu_key_options(uint16_t key_value)
{
    switch(key_value)
    {
        case KEY_UP:
        {
            cur_opt[MENU]--;
            if(cur_opt[MENU] == 0xff)
                cur_opt[MENU] = 3;
            page_opts_sethighlight(MENU, cur_opt[MENU]);
        }
        break;
        case KEY_DOWN:
        {
            cur_opt[MENU]++;
            if(cur_opt[MENU] == 4)
                cur_opt[MENU] = 0;
            page_opts_sethighlight(MENU, cur_opt[MENU]);
        }
        break;
        case KEY_OK:
        {
            if(cur_opt[MENU] == 0)
                HMISends("click b1,0\xff\xff\xff");
            else if(cur_opt[MENU] == 1)
                HMISends("click b2,0\xff\xff\xff");
            else if(cur_opt[MENU] == 2)
                HMISends("click b4,0\xff\xff\xff");
            else if(cur_opt[MENU] == 3)
                HMISends("click b5,0\xff\xff\xff");
        }
        break;
        case KEY_BACK:
        break;
        default:break;
    }
}

void longtxchoose_key_options(uint16_t key_value)
{
    switch(key_value)
    {
        case KEY_UP:
        {
            cur_opt[LONGTXCHOOSE]--;
            if(cur_opt[LONGTXCHOOSE] == 0xff)
                cur_opt[LONGTXCHOOSE] = 1;
            page_opts_sethighlight(LONGTXCHOOSE, cur_opt[LONGTXCHOOSE]);
        }
        break;
        case KEY_DOWN:
        {
            cur_opt[LONGTXCHOOSE]++;
            if(cur_opt[LONGTXCHOOSE] == 2)
                cur_opt[LONGTXCHOOSE] = 0;
            page_opts_sethighlight(LONGTXCHOOSE, cur_opt[LONGTXCHOOSE]);
        }
        break;
        case KEY_OK:
        {
            if(cur_opt[LONGTXCHOOSE] == 0)
                HMISends("click b0,0\xff\xff\xff");
            else if(cur_opt[LONGTXCHOOSE] == 1)
                HMISends("click b1,0\xff\xff\xff");
        }
        break;
        case KEY_BACK:
        {
            HMISends("click b2,0\xff\xff\xff");
        }
        break;
        default:break;
    }
}

void simgletxchoose_key_options(uint16_t key_value)
{
    switch(key_value)
    {
        case KEY_UP:
        {
            cur_opt[SIMGLETXCHOOSE]--;
            if(cur_opt[SIMGLETXCHOOSE] == 0xff)
                cur_opt[SIMGLETXCHOOSE] = 1;
            page_opts_sethighlight(SIMGLETXCHOOSE, cur_opt[SIMGLETXCHOOSE]);
        }
        break;
        case KEY_DOWN:
        {
            cur_opt[SIMGLETXCHOOSE]++;
            if(cur_opt[SIMGLETXCHOOSE] == 2)
                cur_opt[SIMGLETXCHOOSE] = 0;
            page_opts_sethighlight(SIMGLETXCHOOSE, cur_opt[SIMGLETXCHOOSE]);
        }
        break;
        case KEY_OK:
        {
            if(cur_opt[SIMGLETXCHOOSE] == 0)
                HMISends("click b0,0\xff\xff\xff");
            else if(cur_opt[SIMGLETXCHOOSE] == 1)
                HMISends("click b1,0\xff\xff\xff");
        }
        break;
        case KEY_BACK:
        {
            HMISends("click b2,0\xff\xff\xff");
        }
        break;
        default:break;
    }
}

void rangingchoose_key_options(uint16_t key_value)
{
    switch(key_value)
    {
        case KEY_UP:
        {
            cur_opt[RANGINGCHOOSE]--;
            if(cur_opt[RANGINGCHOOSE] == 0xff)
                cur_opt[RANGINGCHOOSE] = 1;
            page_opts_sethighlight(RANGINGCHOOSE, cur_opt[RANGINGCHOOSE]);
        }
        break;
        case KEY_DOWN:
        {
            cur_opt[RANGINGCHOOSE]++;
            if(cur_opt[RANGINGCHOOSE] == 2)
                cur_opt[RANGINGCHOOSE] = 0;
            page_opts_sethighlight(RANGINGCHOOSE, cur_opt[RANGINGCHOOSE]);
        }
        break;
        case KEY_OK:
        {
            if(cur_opt[RANGINGCHOOSE] == 0)
                HMISends("click b0,0\xff\xff\xff");
            else if(cur_opt[RANGINGCHOOSE] == 1)
                HMISends("click b1,0\xff\xff\xff");
        }
        break;
        case KEY_BACK:
        {
            HMISends("click b2,0\xff\xff\xff");
        }
        break;
        default:break;
    }
}

void locationchoose_key_options(uint16_t key_value)
{
    switch(key_value)
    {
        case KEY_UP:
        {
            cur_opt[LOCATIONCHOOSE]--;
            if(cur_opt[LOCATIONCHOOSE] == 0xff)
                cur_opt[LOCATIONCHOOSE] = 1;
            page_opts_sethighlight(LOCATIONCHOOSE, cur_opt[LOCATIONCHOOSE]);
        }
        break;
        case KEY_DOWN:
        {
            cur_opt[LOCATIONCHOOSE]++;
            if(cur_opt[LOCATIONCHOOSE] == 2)
                cur_opt[LOCATIONCHOOSE] = 0;
            page_opts_sethighlight(LOCATIONCHOOSE, cur_opt[LOCATIONCHOOSE]);
        }
        break;
        case KEY_OK:
        {
            if(cur_opt[LOCATIONCHOOSE] == 0)
                HMISends("click b1,0\xff\xff\xff");
            else if(cur_opt[LOCATIONCHOOSE] == 1)
                HMISends("click b0,0\xff\xff\xff");
        }
        break;
        case KEY_BACK:
        {
            HMISends("click b2,0\xff\xff\xff");
        }
        break;
        default:break;
    }
}

void loca_id_choose_key_options(uint16_t key_value)
{
    switch(key_value)
    {
        case KEY_UP:
        {
            cur_opt[LOCA_ID_CHOOSE]--;
            if(cur_opt[LOCA_ID_CHOOSE] == 0xff)
                cur_opt[LOCA_ID_CHOOSE] = 11;
            page_opts_sethighlight(LOCA_ID_CHOOSE, cur_opt[LOCA_ID_CHOOSE]);
        }
        break;
        case KEY_DOWN:
        {
            cur_opt[LOCA_ID_CHOOSE]++;
            if(cur_opt[LOCA_ID_CHOOSE] == 12)
                cur_opt[LOCA_ID_CHOOSE] = 0;
            page_opts_sethighlight(LOCA_ID_CHOOSE, cur_opt[LOCA_ID_CHOOSE]);
        }
        break;
        case KEY_OK:
        {
            if(cur_opt[LOCA_ID_CHOOSE] == 0)
                HMISends("click n0,0\xff\xff\xff");
            else if(cur_opt[LOCA_ID_CHOOSE] == 1)
                HMISends("click n1,0\xff\xff\xff");
            else if(cur_opt[LOCA_ID_CHOOSE] == 2)
                HMISends("click n2,0\xff\xff\xff");
            else if(cur_opt[LOCA_ID_CHOOSE] == 3)
                HMISends("click n3,0\xff\xff\xff");
            else if(cur_opt[LOCA_ID_CHOOSE] == 4)
                HMISends("click n4,0\xff\xff\xff");
            else if(cur_opt[LOCA_ID_CHOOSE] == 5)
                HMISends("click n5,0\xff\xff\xff");
            else if(cur_opt[LOCA_ID_CHOOSE] == 6)
                HMISends("click n6,0\xff\xff\xff");
            else if(cur_opt[LOCA_ID_CHOOSE] == 7)
                HMISends("click n7,0\xff\xff\xff");
            else if(cur_opt[LOCA_ID_CHOOSE] == 8)
                HMISends("click b0,0\xff\xff\xff");
            else if(cur_opt[LOCA_ID_CHOOSE] == 9)
                HMISends("click b1,0\xff\xff\xff");
            else if(cur_opt[LOCA_ID_CHOOSE] == 10)
                HMISends("click b3,0\xff\xff\xff");
            else if(cur_opt[LOCA_ID_CHOOSE] == 11)
                HMISends("click b20,0\xff\xff\xff");
        }
        break;
        case KEY_BACK:
        {
            HMISends("click b2,0\xff\xff\xff");
        }
        break;
        default:break;
    }
}

void setting_key_options(uint16_t key_value)
{
    switch(key_value)
    {
        case KEY_UP:
        {
            cur_opt[SETTING]--;
            if(cur_opt[SETTING] == 0xff)
                cur_opt[SETTING] = 4;
            page_opts_sethighlight(SETTING, cur_opt[SETTING]);
        }
        break;
        case KEY_DOWN:
        {
            cur_opt[SETTING]++;
            if(cur_opt[SETTING] == 5)
                cur_opt[SETTING] = 0;
            page_opts_sethighlight(SETTING, cur_opt[SETTING]);
        }
        break;
        case KEY_OK:
        {
            if(cur_opt[SETTING] == 0)
                HMISends("click b11,0\xff\xff\xff");
            else if(cur_opt[SETTING] == 1)
                HMISends("click b3,0\xff\xff\xff");
            else if(cur_opt[SETTING] == 2)
                HMISends("click b5,0\xff\xff\xff");
            else if(cur_opt[SETTING] == 3)
                HMISends("click b7,0\xff\xff\xff");
            else if(cur_opt[SETTING] == 4)
                HMISends("click b0,0\xff\xff\xff");
        }
        break;
        case KEY_BACK:
        {
            HMISends("click b2,0\xff\xff\xff");
        }
        break;
        default:break;
    }
}

void longtxwave_key_options(uint16_t key_value)
{
    switch(key_value)
    {
        case KEY_UP:
        break;
        case KEY_DOWN:
        break;
        case KEY_OK:
        {
            HMISends("click b0,0\xff\xff\xff");
        }
        break;
        case KEY_BACK:
        {
            HMISends("click b2,0\xff\xff\xff");
        }
        break;
        default:break;
    }
}

void longtxframe_key_options(uint16_t key_value)
{
    switch(key_value)
    {
        case KEY_UP:
        break;
        case KEY_DOWN:
        break;
        case KEY_OK:
        {
            HMISends("click b0,0\xff\xff\xff");
        }
        break;
        case KEY_BACK:
        {
            HMISends("click b2,0\xff\xff\xff");
        }
        break;
        default:break;
    }
}

void simgletx_key_options(uint16_t key_value)
{
    switch(key_value)
    {
        case KEY_UP:
        break;
        case KEY_DOWN:
        break;
        case KEY_OK:
        {
            HMISends("click b0,0\xff\xff\xff");
        }
        break;
        case KEY_BACK:
        {
            HMISends("click b2,0\xff\xff\xff");
        }
        break;
        default:break;
    }
}

void simglerx_key_options(uint16_t key_value)
{
    switch(key_value)
    {
        case KEY_UP:
        break;
        case KEY_DOWN:
        break;
        case KEY_OK:
        {
            HMISends("click b0,0\xff\xff\xff");
        }
        break;
        case KEY_BACK:
        {
            HMISends("click b2,0\xff\xff\xff");
        }
        break;
        default:break;
    }
}

void ranginginit_key_options(uint16_t key_value)
{
    switch(key_value)
    {
        case KEY_UP:
        break;
        case KEY_DOWN:
        break;
        case KEY_OK:
        {
            HMISends("click b0,0\xff\xff\xff");
        }
        break;
        case KEY_BACK:
        {
            HMISends("click b2,0\xff\xff\xff");
        }
        break;
        default:break;
    }
}

void rangingresp_key_options(uint16_t key_value)
{
    switch(key_value)
    {
        case KEY_UP:
        break;
        case KEY_DOWN:
        break;
        case KEY_OK:
        {
            HMISends("click b0,0\xff\xff\xff");
        }
        break;
        case KEY_BACK:
        {
            HMISends("click b2,0\xff\xff\xff");
        }
        break;
        default:break;
    }
}

void loading_key_options(uint16_t key_value)
{
    switch(key_value)
    {
        case KEY_UP:
        break;
        case KEY_DOWN:
        break;
        case KEY_OK:
        break;
        case KEY_BACK:
        break;
        default:break;
    }
}

void about_key_options(uint16_t key_value)
{
    switch(key_value)
    {
        case KEY_UP:
        {
            cur_opt[ABOUT]++;
            if(cur_opt[ABOUT] == 2)
                cur_opt[ABOUT] = 0;
            page_opts_sethighlight(ABOUT, cur_opt[ABOUT]);
        }
        break;
        case KEY_DOWN:
        {
            cur_opt[ABOUT]--;
            if(cur_opt[ABOUT] == 0xff)
                cur_opt[ABOUT] = 1;
            page_opts_sethighlight(ABOUT, cur_opt[ABOUT]);
        }
        break;
        case KEY_OK:
        {
            if(cur_opt[ABOUT] == 0)
            {
                language = ENGLISH;
                HMISends("click b1,0\xff\xff\xff");
            }
            else if(cur_opt[ABOUT] == 1)
            {
                language = CHINESE;
                HMISends("click b0,0\xff\xff\xff");
            }
        }
        break;
        case KEY_BACK:
            HMISends("page menu\xff\xff\xff");
        break;
        default:break;
    }
}

void key_options(uint16_t key_value)
{
    if(hmi_page == MENU)
        menu_key_options(key_value);
    else if(hmi_page == LONGTXCHOOSE)
        longtxchoose_key_options(key_value);
    else if(hmi_page == SIMGLETXCHOOSE)
        simgletxchoose_key_options(key_value);
    else if(hmi_page == RANGINGCHOOSE)
        rangingchoose_key_options(key_value);
    else if(hmi_page == LOCATIONCHOOSE)
        locationchoose_key_options(key_value);
    else if(hmi_page == LOCA_ID_CHOOSE)
        loca_id_choose_key_options(key_value);
    else if(hmi_page == SETTING)
        setting_key_options(key_value);
    else if(hmi_page == LONGTXWAVE)
        longtxwave_key_options(key_value);
    else if(hmi_page == LONGTXFRAME)
        longtxframe_key_options(key_value);
    else if(hmi_page == SIMGLETX)
        simgletx_key_options(key_value);
    else if(hmi_page == SIMGLERX)
        simglerx_key_options(key_value);
    else if(hmi_page == RANGINGINIT)
        ranginginit_key_options(key_value);
    else if(hmi_page == RANGINGRESP)
        rangingresp_key_options(key_value);
    else if(hmi_page == ABOUT)
        about_key_options(key_value);
}
