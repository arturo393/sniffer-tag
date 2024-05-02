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
#include "sniffer_tag.h"
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct flags_t flags;
double *distance_ptr;
TAG_t tag;
int size = 0;
uint8_t running_device = DEV_UWB3000F27;
SPI_HW_t *hw;
dwt_local_data_t *pdw3000local;
uint8_t crcTable[256];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	/* USER CODE BEGIN 2 */
	/*
	 uint8_t addr[3] = {0};
	 uint8_t addr_count= 0;
	 for(i=1; i<128; i++)
	 {
	 ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
	 if(ret == HAL_OK)
	 addr[addr_count++]=i;
	 }
	 */
	HAL_GPIO_WritePin(GPIOA, DW3000_RST_Pin, GPIO_PIN_SET);
	/*Local device data, can be an array to support multiple DW3000 testing applications/platforms */

	dwt_local_data_t dwt_local_data;
	pdw3000local = &dwt_local_data;
	/* Default communication configuration. We use default non-STS DW mode. */
	dwt_config_t defatult_dwt_config = { 5, /* Channel number. */
	DWT_PLEN_128, /* Preamble length. Used in TX only. */
	DWT_PAC8, /* Preamble acquisition chunk size. Used in RX only. */
	9, /* TX preamble code. Used in TX only. */
	9, /* RX preamble code. Used in RX only. */
	1, /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
	DWT_BR_6M8, /* Data rate. */
	DWT_PHRMODE_STD, /* PHY header mode. */
	DWT_PHRRATE_STD, /* PHY header rate. */
	(129 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
	DWT_STS_MODE_OFF, /* STS disabled */
	DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
	DWT_PDOA_M0 /* PDOA mode off */
	};

//	dwt_config_t config_options = {
//	    9,                  /* Channel number. */
//	    DWT_PLEN_512,       /* Preamble length. Used in TX only. */
//	    DWT_PAC8,           /* Preamble acquisition chunk size. Used in RX only. */
//	    10,                  /* TX preamble code. Used in TX only. */
//	    10,                  /* RX preamble code. Used in RX only. */
//	    3,                  /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
//	    DWT_BR_6M8,         /* Data rate. */
//	    DWT_PHRMODE_STD,    /* PHY header mode. */
//	    DWT_PHRRATE_STD,    /* PHY header rate. */
//	    (512 + 1 + 8 - 8),  /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
//	    DWT_STS_MODE_1,      /* Mode 1 STS enabled */
//	    DWT_STS_LEN_64,      /* STS length*/
//	    DWT_PDOA_M0         /* PDOA mode off */
//	};

	/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
	 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 8 below. */
	static dwt_txconfig_t defatult_dwt_txconfig = { 0x34, /* PG delay. */
	0xfdfdfdfd, /* TX power. */
	0x0 /*PG count*/
	};
	uint8_t dist_str[30];
	int size;
	tag.hw = malloc(sizeof(SPI_HW_t));
	if (tag.hw == NULL)
		Error_Handler();

	switch (running_device) {

	case DEV_UWB3000F27:
		tag.hw->spi = &hspi1;
		tag.hw->nrstPin = DW3000_RST_Pin;
		tag.hw->nrstPort = DW3000_RST_GPIO_Port;
		tag.hw->nssPin = SPI1_CS_Pin;
		tag.hw->nssPort = SPI1_CS_GPIO_Port;
		size = sprintf((char*) dist_str, "\n\rDEV_UWB3000F27 init\n\r");
		HAL_UART_Transmit(&huart1, dist_str,(uint16_t) size,
		HAL_MAX_DELAY);
		break;
	case DEV_UWB3000F00:
		tag.hw->spi = &hspi2;
		tag.hw->nrstPin = DW3000_RST_RCV_Pin;
		tag.hw->nrstPort = DW3000_RST_RCV_GPIO_Port;
		tag.hw->nssPin = SPI2_CS_Pin;
		tag.hw->nssPort = SPI2_CS_GPIO_Port;
		size = sprintf((char*) dist_str, "\n\rDEV_UWB3000F00 init\n\r");
		HAL_UART_Transmit(&huart1, dist_str,(uint16_t)size,
		HAL_MAX_DELAY);
		break;
	default:
		break;
	}

	hw = tag.hw;

	HAL_GPIO_WritePin(tag.hw->nrstPort, tag.hw->nrstPin, GPIO_PIN_RESET);/* Target specific drive of RSTn line into DW IC low for a period. */
	HAL_Delay(1);
	HAL_GPIO_WritePin(tag.hw->nrstPort, tag.hw->nrstPin, GPIO_PIN_SET);
	if (tag_init(&defatult_dwt_config, &defatult_dwt_txconfig, &dwt_local_data,DEV_UWB3000F00,RATE_6M8)
			== 1)
		Error_Handler();

	/* Frames used in the ranging process. See NOTE 2 below. */
	uint8_t rx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E',
			0x21, 0, 0 };
	uint8_t tx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A',
			0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t rx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E',
			0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


	/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
	uint32_t status_reg = 0;
	uint32_t frame_seq_nb = 0;
	uint8_t rx_buffer[RX_BUF_LEN] = { 0 };
	Distance_t distance;
	distance.counter = 0;
	distance.last = 0;
	distance.error_times = 0;


	/* Time-stamps of frames transmission/reception, expressed in device time units. */

	defatult_dwt_txconfig.power = GAIN_30DB;
	dwt_configuretxrf(&defatult_dwt_txconfig);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (running_device == DEV_UWB3000F27) {
			handle_sniffer_tag(&distance);

		} else if (running_device == DEV_UWB3000F00) {
			/* Loop forever responding to ranging requests. */
			dwt_setpreambledetecttimeout(0);
			/* Clear reception timeout to start next ranging process. */
			dwt_setrxtimeout(0);
			/* Activate reception immediately. */
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
			/* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
			while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID))
					& (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO
							| SYS_STATUS_ALL_RX_ERR))) {
			};

			if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
				uint32_t frame_len;
				/* Clear good RX frame event in the DW IC status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

				/* A frame has been received, read it into the local buffer. */
				memset(rx_buffer, 0, RX_BUF_LEN);
				frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
				if (frame_len <= RX_BUF_LEN) {
					dwt_readrxdata(rx_buffer, (uint16_t) frame_len, 0);
				}

				/* Check that the frame is a poll sent by "DS TWR initiator" example.
				 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
				rx_buffer[ALL_MSG_SN_IDX] = 0;
				if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) {

					uint32_t resp_tx_time;
					uint64_t resp_tx_timestamp;
					uint64_t poll_rx_timestamp;
					int ret;

					/* Retrieve poll reception timestamp. */
					poll_rx_timestamp = get_rx_timestamp_u64();

					/* Set send time for response. See NOTE 9 below. */
					resp_tx_time = (uint32_t) ((poll_rx_timestamp
							+ ((POLL_RX_TO_RESP_TX_DLY_UUS_6M8)
									* UUS_TO_DWT_TIME)) >> 8);

					dwt_setdelayedtrxtime(resp_tx_time);

					/* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
					resp_tx_timestamp = (((uint64_t) (resp_tx_time
							& 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY_HP;

					/* Write all timestamps in the final message. See NOTE 8 below. */
					resp_msg_set_ts(&tx_resp_msg[FINAL_MSG_POLL_TX_TS_IDX],
							poll_rx_timestamp);
					resp_msg_set_ts(&tx_resp_msg[FINAL_MSG_RESP_RX_TS_IDX],
							resp_tx_timestamp);
					/* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
					dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS_6M8);
					/* FINAL_RX_TIMEOUT_UUS. */
					dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS_6M8);
					/* Set preamble timeout for expected frames. See NOTE 6 below. */
					dwt_setpreambledetecttimeout(PRE_TIMEOUT_6M8);
					/* Write and send the response message. See NOTE 10 below.*/
					tx_resp_msg[ALL_MSG_SN_IDX] = (uint8_t) frame_seq_nb;
					dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
					dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
					/*DWT_START_TX_DELAYED DWT_START_TX_IMMEDIATE*/
					ret = dwt_starttx(
					DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

					/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
					if (ret == DWT_SUCCESS) {

						/* Poll for reception of expected "final" frame or error/timeout. See NOTE 8 below. */
						while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID))
								& (SYS_STATUS_RXFCG_BIT_MASK
										| SYS_STATUS_ALL_RX_TO
										| SYS_STATUS_ALL_RX_ERR))) {

						};
						/* Increment frame sequence number after transmission of the response message (modulo 256). */
						frame_seq_nb++;

						if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
							/* Clear good RX frame event and TX frame sent in the DW IC status register. */
							dwt_write32bitreg(SYS_STATUS_ID,
									SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

							/* A frame has been received, read it into the local buffer. */
							frame_len = dwt_read32bitreg(
									RX_FINFO_ID) & FRAME_LEN_MAX_EX;
							if (frame_len <= RX_BUF_LEN) {
								dwt_readrxdata(rx_buffer, (uint16_t) frame_len,
										0);
							}
							for (int i = 0; i < (int) frame_len; i++) {
								sprintf((char*) dist_str, "%02X ",
										rx_buffer[i]); // Print with leading zeros for 2-digit format
								HAL_UART_Transmit(&huart1, dist_str,
										(uint16_t) 2,
										HAL_MAX_DELAY);
							}
							sprintf((char*) dist_str, "\n\r");
							HAL_UART_Transmit(&huart1, dist_str, (uint16_t) 2,
							HAL_MAX_DELAY);

							/* Check that the frame is a final message sent by "DS TWR initiator" example.
							 * As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
							rx_buffer[ALL_MSG_SN_IDX] = 0;

							if (memcmp(rx_buffer, rx_final_msg,
							ALL_MSG_COMMON_LEN) == 0) {
								uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
								uint32_t poll_rx_ts_32, resp_tx_ts_32,
										final_rx_ts_32;
								double Ra, Rb, Da, Db;
								int64_t tof_dtu;
								uint64_t final_rx_timestamp;

								/* Retrieve response transmission and final reception timestamps. */
								resp_tx_timestamp = get_tx_timestamp_u64();
								final_rx_timestamp = get_rx_timestamp_u64();

								/* Get timestamps embedded in the final message. */
								final_msg_get_ts(
										&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX],
										&poll_tx_ts);
								final_msg_get_ts(
										&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX],
										&resp_rx_ts);
								final_msg_get_ts(
										&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX],
										&final_tx_ts);

								/* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
								poll_rx_ts_32 = (uint32_t) poll_rx_timestamp;
								resp_tx_ts_32 = (uint32_t) resp_tx_timestamp;
								final_rx_ts_32 = (uint32_t) final_rx_timestamp;
								Ra = (double) (resp_rx_ts - poll_tx_ts);
								Rb = (double) (final_rx_ts_32 - resp_tx_ts_32);
								Da = (double) (final_tx_ts - resp_rx_ts);
								Db = (double) (resp_tx_ts_32 - poll_rx_ts_32);
								tof_dtu = (int64_t) ((Ra * Rb - Da * Db)
										/ (Ra + Rb + Da + Db));

								double tof;
								tof = (double) tof_dtu * DWT_TIME_UNITS;
								distance.value = tof * SPEED_OF_LIGHT;
								/*The data were smoothed and filtered */
								if (distance.counter < 10) {
									distance.readings[distance.counter] =
											distance.value;
									distance.counter++;
									distance.sum = 0;
									for (int i = 0; i < distance.counter; i++) {
										distance.sum += distance.readings[i];
									}
									distance.value = distance.sum
											/ (double) distance.counter;
								} else {
									if (fabs(
											distance.value
													- distance.last) < MAX_DISTANCE_ERROR) {
										distance.error_times = 0;
										for (int i = 0;
												i < distance.counter - 1; i++) {
											distance.new[i] =
													distance.readings[i + 1];
										}
										distance.new[distance.counter - 1] =
												distance.value;
										distance.sum = 0;
										for (int i = 0; i < distance.counter;
												i++) {
											distance.sum += distance.new[i];
											distance.readings[i] =
													distance.new[i];
										}
										distance.last = distance.value;
										distance.value = distance.sum
												/ (double) distance.counter;
									} else {
										distance.value = distance.last;
										distance.error_times++;
										if (distance.error_times > 20) {
											distance.error_times = 0;
											distance.counter = 0;
										}
									}
								}
								tag.distance = distance.value;
								/* Display computed distance on HMI display. */
								size = sprintf((char*) dist_str,
										"distance:%.2f\n\r", tag.distance);
								HAL_UART_Transmit(&huart1, dist_str,
										(uint16_t) size, HAL_MAX_DELAY);
								flags.ds_twr_timeout = 1;
							}
						} else {
							/* Clear RX error/timeout events in the DW IC status register. */
							dwt_write32bitreg(SYS_STATUS_ID,
									SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
						}
					}
				}
			} else {
				/* Clear RX error/timeout events in the DW IC status register. */
				dwt_write32bitreg(SYS_STATUS_ID,
						SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
			}

			/* Data is sent to the HMI display due to a long period of time without updating the ranging data */
			if (flags.ds_twr_timeout > 3000) {
				flags.ds_twr_timeout = 1;
				//HMISends("rangingresp.t1.txt=\"???\"\xff\xff\xff");

			}

			/* Changing the value of target_allow_run_time adjusts the interval (in ms) between runs of the example again */
			//flags.target_allow_run_time = DSTWR_RESP_RERUN_INTERVAL;
			flags.time_to_allow_run = 1;
		}

		/* Target specific drive of RSTn line into DW IC low for a period. */

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

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
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
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
static void MX_SPI2_Init(void) {

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
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
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
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 160 - 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 320 - 1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
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
uint8_t recvChar;
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, DW3000_RST_Pin | DW3000_RST_RCV_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : DW3000_RST_Pin DW3000_RST_RCV_Pin */
	GPIO_InitStruct.Pin = DW3000_RST_Pin | DW3000_RST_RCV_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI1_CS_Pin */
	GPIO_InitStruct.Pin = SPI1_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DEVICE_SELECT_Pin KEY_UP_Pin KEY_DOWN_Pin KEY_OK_Pin
	 KEY_BACK_Pin */
	GPIO_InitStruct.Pin = DEVICE_SELECT_Pin | KEY_UP_Pin | KEY_DOWN_Pin
			| KEY_OK_Pin | KEY_BACK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI2_CS_Pin */
	GPIO_InitStruct.Pin = SPI2_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
