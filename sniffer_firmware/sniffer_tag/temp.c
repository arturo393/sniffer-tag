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
	/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
	 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 8 below. */
	static dwt_txconfig_t defatult_dwt_txconfig = { 0x34, /* PG delay. */
	0xfdfdfdfd, /* TX power. */
	0x0 /*PG count*/
	};

	char dist_str[100];
	int size;

	hw_a = malloc(sizeof(SPI_HW_t));
	if (hw_a == NULL)
		Error_Handler();

	hw_a->spi = &hspi1;
	hw_a->nrstPin = DW3000_RST_Pin;
	hw_a->nrstPort = DW3000_RST_GPIO_Port;
	hw_a->nssPin = SPI1_CS_Pin;
	hw_a->nssPort = SPI1_CS_GPIO_Port;

	size = sprintf((char*) dist_str, "\n\rDEV_UWB3000F27_A init\n\r");
	HAL_UART_Transmit(&huart1, (uint8_t*) dist_str, (uint16_t) size,
	HAL_MAX_DELAY);

	HAL_GPIO_WritePin(hw_a->nrstPort, hw_a->nrstPin, GPIO_PIN_RESET);/* Target specific drive of RSTn line into DW IC low for a period. */
	HAL_Delay(1);
	HAL_GPIO_WritePin(hw_a->nrstPort, hw_a->nrstPin, GPIO_PIN_SET);
	hw = hw_a;
	if (tag_init(&defatult_dwt_config, &defatult_dwt_txconfig, &dwt_local_data,
			running_device, RATE_6M8) == 1)
		Error_Handler();

	hw_b = malloc(sizeof(SPI_HW_t));
	if (hw_b == NULL)
		Error_Handler();

	hw_b->spi = &hspi1;
	hw_b->nrstPin = DW3000_RST_RCV_Pin;
	hw_b->nrstPort = DW3000_RST_RCV_GPIO_Port;
	hw_b->nssPin = SPI2_CS_Pin;
	hw_b->nssPort = SPI2_CS_GPIO_Port;

	size = sprintf((char*) dist_str, "\n\rDEV_UWB3000F27_B init\n\r");
	HAL_UART_Transmit(&huart1, (uint8_t*) dist_str, (uint16_t) size,
	HAL_MAX_DELAY);

	HAL_GPIO_WritePin(hw_b->nrstPort, hw_b->nrstPin, GPIO_PIN_RESET);/* Target specific drive of RSTn line into DW IC low for a period. */
	HAL_Delay(1);
	HAL_GPIO_WritePin(hw_b->nrstPort, hw_b->nrstPin, GPIO_PIN_SET);
	hw = hw_b;
	if (tag_init(&defatult_dwt_config, &defatult_dwt_txconfig, &dwt_local_data,
			running_device, RATE_6M8) == 1)
		Error_Handler();
