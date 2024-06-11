/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
struct flags_t
{
    uint8_t opt_1ms_flag;
    uint8_t key_ispress :1;
    uint16_t key_presstime;
    uint16_t key_value;

    uint8_t uart_recv_time;
    uint8_t uart_recv;

    uint8_t func_allow_run;
    uint16_t target_allow_run_time;
    uint16_t time_to_allow_run;

    uint16_t ds_twr_timeout;

    uint16_t option_timeout;
    void(*function)(void);
};

extern struct flags_t flags;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern SPI_HandleTypeDef hspi1;

extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart1;

extern struct flags_t flags;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DW3000_A_RST_Pin GPIO_PIN_0
#define DW3000_A_RST_GPIO_Port GPIOA
#define DW3000_A_CS_Pin GPIO_PIN_4
#define DW3000_A_CS_GPIO_Port GPIOA
#define DEVICE_SELECT_Pin GPIO_PIN_0
#define DEVICE_SELECT_GPIO_Port GPIOB
#define KEY_UP_Pin GPIO_PIN_1
#define KEY_UP_GPIO_Port GPIOB
#define KEY_DOWN_Pin GPIO_PIN_2
#define KEY_DOWN_GPIO_Port GPIOB
#define KEY_OK_Pin GPIO_PIN_10
#define KEY_OK_GPIO_Port GPIOB
#define KEY_BACK_Pin GPIO_PIN_11
#define KEY_BACK_GPIO_Port GPIOB
#define DW3000_B_CS_Pin GPIO_PIN_12
#define DW3000_B_CS_GPIO_Port GPIOB
#define DW3000_B_RST_Pin GPIO_PIN_8
#define DW3000_B_RST_GPIO_Port GPIOA
#define SX1276_RX_NSS_Pin GPIO_PIN_11
#define SX1276_RX_NSS_GPIO_Port GPIOA
#define SX1276_RX_NRST_Pin GPIO_PIN_12
#define SX1276_RX_NRST_GPIO_Port GPIOA
#define SX1276_TX_NSS_Pin GPIO_PIN_15
#define SX1276_TX_NSS_GPIO_Port GPIOA
#define SX1276_TX_NRST_Pin GPIO_PIN_3
#define SX1276_TX_NRST_GPIO_Port GPIOB
#define LORA_RX_Pin GPIO_PIN_4
#define LORA_RX_GPIO_Port GPIOB
#define LORA_TX_Pin GPIO_PIN_5
#define LORA_TX_GPIO_Port GPIOB
#define SX1276_TX_DIO0_Pin GPIO_PIN_8
#define SX1276_TX_DIO0_GPIO_Port GPIOB
#define SX1276_RX_DIO0_Pin GPIO_PIN_9
#define SX1276_RX_DIO0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
