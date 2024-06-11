/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sniffer_tag.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct flags_t flags;
double *distance_ptr;
TAG_t *tag;
int size = 0;
uint8_t running_device = DEV_UWB3000F27;
SPI_HW_t *hw_a;
SPI_HW_t *hw_b;
SPI_HW_t *hw;
dwt_local_data_t *pdw3000local;
uint8_t crcTable[256];
uint8_t recvChar;
LORA_t lora;
SX1276_HW_t sx1276_hw_tx;
SX1276_HW_t sx1276_hw_rx;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	HAL_DeInit();
	HAL_RCC_DeInit();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	pdw3000local = malloc(sizeof(dwt_local_data_t));

	init_sx1276_hw(&sx1276_hw_tx, &hspi2, SX1276_TX_NSS_GPIO_Port,
	SX1276_TX_NSS_Pin,
	SX1276_TX_NRST_GPIO_Port, SX1276_TX_NRST_Pin,
	SX1276_TX_DIO0_GPIO_Port,
	SX1276_TX_DIO0_Pin);

	init_sx1276_hw(&sx1276_hw_rx, &hspi2, SX1276_RX_NSS_GPIO_Port,
	SX1276_RX_NSS_Pin,
	SX1276_RX_NRST_GPIO_Port, SX1276_RX_NRST_Pin,
	SX1276_RX_DIO0_GPIO_Port,
	SX1276_RX_DIO0_Pin);

	lora_init(&lora, &sx1276_hw_tx, &sx1276_hw_rx);
	TAG_List list = { NULL, 0 };
	SPI_HW_t *hw_a = init_uwb_device(&hspi1, DW3000_A_CS_GPIO_Port,
	DW3000_A_CS_Pin,
	DW3000_A_RST_GPIO_Port, DW3000_A_RST_Pin);
	SPI_HW_t *hw_b = init_uwb_device(&hspi1, DW3000_B_CS_GPIO_Port,
	DW3000_B_CS_Pin,
	DW3000_B_RST_GPIO_Port, DW3000_B_RST_Pin);

	/* Frames used in the ranging process. See NOTE 2 below. */

	/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
	TAG_t tag;
	reset_TAG_values(&tag);

	TAG_STATUS_t tag_status = TAG_DISCOVERY;
	uint32_t lora_send_timeout = 5000;
	uint32_t lora_send_ticks = HAL_GetTick();
	uint32_t query_timeout = 1000;
	uint32_t query_ticks;

	RDSS_status_t rdss_status;

	/* Time-stamps of frames transmission/reception, expressed in device time units. */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {
		if (tag_status == TAG_DISCOVERY) {

			if (hw == hw_a) {
				hw = hw_b;
				tag.distance = &(tag.distance_b);
			} else {
				hw = hw_a;
				tag.distance = &(tag.distance_a);
			}

			tag_status = tag_discovery(&tag);

			if (tag_status != TAG_SEND_TIMESTAMP_QUERY)
				tag_status = TAG_DISCOVERY;
			else
				query_ticks = HAL_GetTick();
			set_battery_voltage(&(tag.battery_voltage));
			set_temperature(&(tag.temperature));
			debug(tag, tag_status);

		} else if (tag_status == TAG_SEND_TIMESTAMP_QUERY) {
			if (tag.readings < DISTANCE_READINGS) {
				if (hw == hw_a) {
					hw = hw_b;
					tag.distance = &(tag.distance_b);
				} else {
					hw = hw_a;
					tag.distance = &(tag.distance_a);
				}
				tag.command = TAG_TIMESTAMP_QUERY;
			}
			if (tag.readings == DISTANCE_READINGS - 2) {
				tag.command = TAG_SET_SLEEP_MODE;
				tag_status = TAG_SEND_SET_SLEEP;
			}
			debug(tag, tag_status);
			tag_status = tag_send_timestamp_query(&tag);

			if (tag_status == TAG_SEND_TIMESTAMP_QUERY) {
				tag.readings++;
				tag_status = TAG_SEND_TIMESTAMP_QUERY;

			} else if (tag_status == TAG_END_READINGS) {
				double distance_a_sum = 0;
				double distance_b_sum = 0;
				for (uint8_t i = 0; i < tag.distance_a.counter; i++)
					distance_a_sum += tag.distance_a.readings[i];

				for (uint8_t i = 0; i < tag.distance_b.counter; i++)
					distance_b_sum += tag.distance_b.readings[i];

				tag.distance_a.value = (uint16_t) ((distance_a_sum * 100)
						/ tag.distance_a.counter);
				tag.distance_b.value = (uint16_t) ((distance_b_sum * 100)
						/ tag.distance_b.counter);
				insert_tag(&list, tag);
				tag_status = TAG_DISCOVERY;
				reset_TAG_values(&tag);
			} else {
				tag_status = TAG_SEND_TIMESTAMP_QUERY;
			}
			debug(tag, tag_status);
			if (HAL_GetTick() - query_ticks > query_timeout) {
				tag_status = TAG_DISCOVERY;
				reset_TAG_values(&tag);
			}

		}

		if (read_lora_packet(&lora) > 0) {
			rdss_status = rdss_validation(lora.rxData, lora.rxSize, 0x00);
			if (rdss_status == DATA_OK) {
				uint8_t *rx = lora.rxData;
				size_t temp_value = 1 + list.count * SERIALIZED_TAG_SIZE; // Or use uint32_t
				uint8_t tag_bytes = (uint8_t) temp_value;
				uint8_t response_length = calculate_frame_length(tag_bytes);
				uint8_t *tx = malloc(sizeof(uint8_t) * response_length);
				if (tx == NULL) {
					Error_Handler();
				}
				memset(tx, 0, response_length);
				memcpy(tx, rx, LORA_DATA_LENGHT_INDEX_1);
				memcpy(tx + LORA_DATA_LENGHT_INDEX_1, &tag_bytes,
						sizeof(tag_bytes));
				uint8_t *tx_data = tx + LORA_DATA_START_INDEX;
				tx_data[0] = list.count;
				tx_data++;
				serialize_tag_list(&list, tx_data);
				uint8_t CRC_INDEX = LORA_DATA_START_INDEX
						+ tx[LORA_DATA_LENGHT_INDEX_1];
				uint8_t END_INDEX = CRC_INDEX + CRC_SIZE;
				setCrc(tx, CRC_INDEX);
				tx[END_INDEX] = LTEL_END_MARK;
				HAL_GPIO_WritePin(LORA_TX_GPIO_Port, LORA_TX_Pin,
						GPIO_PIN_RESET);
				write_lora_RegFifo(lora.txhw, tx, response_length);
				start_lora_transmission(lora.txhw);
				HAL_GPIO_WritePin(LORA_TX_GPIO_Port, LORA_TX_Pin, GPIO_PIN_SET);
				serialize_tag_list(&list, tx_data);
				free(tx);
				print_all_tags(&list, tag_status);
				print_serialized_tags(&list);
				free_tag_list(&list);
				restart_lora_rx_continuous_mode(&lora);
			} else {
				lora_reset(lora.txhw);
				lora_reset(lora.rxhw);
				init_lora_parameters(&lora);
			}
		}

		if (((HAL_GetTick() - lora_send_ticks) > lora_send_timeout)
				&& tag_status == TAG_DISCOVERY) {
			debug_status(tag_status);
			size_t temp_value = 1 + list.count * SERIALIZED_TAG_SIZE; // Or use uint32_t
			uint8_t tag_bytes = (uint8_t) temp_value;
			uint8_t response_length = calculate_frame_length(tag_bytes);
			uint8_t tx[response_length];
			memset(tx, 0, response_length);
			memcpy(tx + LORA_DATA_LENGHT_INDEX_1, &tag_bytes,
					sizeof(tag_bytes));
			uint8_t *tx_data = tx + LORA_DATA_START_INDEX;
			tx_data[0] = list.count;
			tx_data++;
			serialize_tag_list(&list, tx_data);
			uint8_t CRC_INDEX = LORA_DATA_START_INDEX
					+ tx[LORA_DATA_LENGHT_INDEX_1];
			uint8_t END_INDEX = CRC_INDEX + CRC_SIZE;
			setCrc(tx, CRC_INDEX);
			tx[END_INDEX] = LTEL_END_MARK;
			print_all_tags(&list, tag_status);
			print_serialized_tags(&list);
			print_tx_hex(tx, response_length);
			free_tag_list(&list);
			lora_send_ticks = HAL_GetTick();
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7200-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 6;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
	HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 230400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

	HAL_UART_Receive_IT(&huart1, &recvChar, 1);

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DW3000_A_RST_Pin|DW3000_B_RST_Pin|SX1276_RX_NSS_Pin|SX1276_RX_NRST_Pin
                          |SX1276_TX_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DW3000_A_CS_GPIO_Port, DW3000_A_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DW3000_B_CS_GPIO_Port, DW3000_B_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SX1276_TX_NRST_Pin|LORA_RX_Pin|LORA_TX_Pin|SX1276_TX_DIO0_Pin
                          |SX1276_RX_DIO0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DW3000_A_RST_Pin DW3000_B_RST_Pin SX1276_RX_NSS_Pin SX1276_RX_NRST_Pin
                           SX1276_TX_NSS_Pin */
  GPIO_InitStruct.Pin = DW3000_A_RST_Pin|DW3000_B_RST_Pin|SX1276_RX_NSS_Pin|SX1276_RX_NRST_Pin
                          |SX1276_TX_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DW3000_A_CS_Pin */
  GPIO_InitStruct.Pin = DW3000_A_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DW3000_A_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DEVICE_SELECT_Pin KEY_UP_Pin KEY_DOWN_Pin KEY_OK_Pin
                           KEY_BACK_Pin */
  GPIO_InitStruct.Pin = DEVICE_SELECT_Pin|KEY_UP_Pin|KEY_DOWN_Pin|KEY_OK_Pin
                          |KEY_BACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DW3000_B_CS_Pin */
  GPIO_InitStruct.Pin = DW3000_B_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DW3000_B_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SX1276_TX_NRST_Pin LORA_RX_Pin LORA_TX_Pin SX1276_TX_DIO0_Pin
                           SX1276_RX_DIO0_Pin */
  GPIO_InitStruct.Pin = SX1276_TX_NRST_Pin|LORA_RX_Pin|LORA_TX_Pin|SX1276_TX_DIO0_Pin
                          |SX1276_RX_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
