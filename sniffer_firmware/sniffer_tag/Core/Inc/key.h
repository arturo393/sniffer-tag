#ifndef KEY_H
#define KEY_H

#include "main.h"

#define KEY_PORT        GPIOB

#define KEY_UNKNOWN     0x0C06
#define KEY_UP          0x0C04
#define KEY_DOWN        0x0C02
#define KEY_OK          0x0806
#define KEY_BACK        0x0406
   
void key_scan(void);
    
#endif
