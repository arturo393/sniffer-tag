#ifndef HMI_UART_H
#define HMI_UART_H

#include "main.h"

extern uint8_t recvChar;
extern uint8_t recvBuf[100];
extern uint16_t recvBufLen;

void HMISends(uint8_t *buf1);

#endif
