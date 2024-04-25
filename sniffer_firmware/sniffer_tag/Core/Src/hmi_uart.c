/*! ----------------------------------------------------------------------------
 * @file    hmi_uart.c
 * @brief   The microcontroller communicates with HMI display through serial port to realize data interaction
 *
 * @attention
 *
 * Copyright(c) 2020 NiceRF Wireless Technology Co., Ltd.
 *
 * All rights reserved.
 *
 */
#include "hmi_uart.h"
#include <string.h>

uint8_t recvChar;
uint8_t recvBuf[100];
uint16_t recvBufLen = 0;

/**
  *	HMISends
  * @brief Send data to the HMI display
  */
void HMISends(uint8_t *buf1)
{
    HAL_UART_Transmit(&huart1, buf1, strlen((char*)buf1), 0xffff);
}

/*
 * HAL_UART_RxCpltCallback
 * Serial port receive interrupt callback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        recvBuf[recvBufLen++] = recvChar;
		
		//Start receiving timing
        if(flags.uart_recv_time == 0)
            flags.uart_recv_time = 1;
		
        HAL_UART_Receive_IT(&huart1, &recvChar, 1);
    }
}
