/**
 * Author Wojciech Domski <Wojciech.Domski@gmail.com>
 * www: www.Domski.pl
 *
 * work based on DORJI.COM sample code and
 * https://github.com/realspinner/SX1278_LoRa
 */

#include "SX1278.h"

#define DIOMAPPING1TX (DIO0_TX_DONE | DIO1_RX_TIMEOUT | DIO2_FHSS_CHANGE_CHANNEL | DIO3_VALID_HEADER)
#define FLAGSMOODETX (0xff & ~(TX_DONE_MASK))
#define CLEARIRQ 0xff
#define DIOMAPPING1RX (DIO0_RX_DONE | DIO1_RX_TIMEOUT | DIO2_FHSS_CHANGE_CHANNEL | DIO3_VALID_HEADER)
#define FLAGSMOODERX (0xff & ~(RX_DONE_MASK) & ~(PAYLOAD_CRC_ERROR_MASK))
#define SYNCWORD 0x12
#define POWER SX1278_POWER_17DBM
#define CRC_SUM CRC_ENABLE
#define OVERCURRENTPROTECT 0x0B // Default value
#define LNAGAIN 0x23 // Highest gain & Boost on
#define AGCAUTON 12
#define SYMBTIMEOUTLSB 0x08
#define PREAMBLELENGTHMSB 0x00
#define PREAMBLELENGTHLSB 6
#define FHSSVALUE 0x07
#define LORA_SLEEP_MODE (LORA_MODE_ACTIVATION | LOW_FREQUENCY_MODE | SLEEP)
#define LORA_RX_CONTINUOUS_MODE (LORA_MODE_ACTIVATION | LOW_FREQUENCY_MODE | RX_CONTINUOUS)
#define LORA_TX_MODE (LORA_MODE_ACTIVATION | LOW_FREQUENCY_MODE | TX)
#define DEFAULT_TX_ADDR 0x80
#define flags 0x40

// Configure the registers and their values
const RegisterConfig_t config[] = { { RegSyncWord, SYNCWORD }, { LR_RegPaConfig,
POWER }, { LR_RegOcp, OVERCURRENTPROTECT }, { LR_RegLna, LNAGAIN }, {
LR_RegModemConfig3, AGCAUTON }, { LR_RegSymbTimeoutLsb, SYMBTIMEOUTLSB }, {
LR_RegPreambleMsb, PREAMBLELENGTHMSB },
		{ LR_RegPreambleLsb, PREAMBLELENGTHLSB }, {
		LR_RegHopPeriod, FHSSVALUE } };

void lora_init(LORA_t *lora, SX1276_HW_t *sx1276_hw_tx,
		SX1276_HW_t *sx1276_hw_rx) {

	// Assert that the pointers are not NULL
	assert_param(lora != NULL);
	assert_param(sx1276_hw_tx != NULL);
	assert_param(sx1276_hw_rx != NULL);

	// Zero out the structure fields
	lora->id = 0;
	lora->ufreq = 0;
	lora->dfreq = 0;
	lora->sf = (SPREAD_FACTOR_t) 0;
	lora->bw = (LORABW_t) 0;
	lora->cr = (CODING_RATE_t) 0;
	lora->mode = (OPERATING_MODE_t) 0;
	lora->lastTxTime = 0;
	lora->lastRxTim = 0;
	lora->writeTime = 0;
	memset(lora->rxData, 0, sizeof(lora->rxData));
	lora->rxSize = 0;
	lora->txhw = NULL;
	lora->rxhw = NULL;

	EEPROM_Read(K24C02_PAGE_ADDR(0), SF_OFFSET, (uint8_t*) &lora->sf,
			sizeof(lora->sf));
	if (lora->sf < SF_6 || lora->sf > SF_12) {
		lora->sf = SF_10;
	}
	EEPROM_Read(K24C02_PAGE_ADDR(0), BW_OFFSET, (uint8_t*) &lora->bw,
			sizeof(lora->bw));
	if (lora->bw < LORABW_7_8KHZ || lora->bw > LORABW_500KHZ) {
		lora->bw = LORABW_62_5KHZ;
	}
	EEPROM_Read(K24C02_PAGE_ADDR(0), CR_OFFSET, (uint8_t*) &lora->cr,
			sizeof(lora->cr));

	if (lora->cr < LORA_CR_4_5 || lora->cr > LORA_CR_4_8) {
		lora->cr = LORA_CR_4_6;
	}

	EEPROM_Read(K24C02_PAGE_ADDR(1), ID_OFFSET, (uint8_t*) &(lora->id),
			sizeof(lora->id));
	if (lora->id == -1) {
		lora->id = 0;
	}

	EEPROM_Read(K24C02_PAGE_ADDR(2), DL_OFFSET, (uint8_t*) &lora->dfreq,
			sizeof(lora->dfreq));
	if (lora->dfreq < DOWNLINK_FREQ_MIN || lora->dfreq > DOWNLINK_FREQ_MAX) {
		lora->dfreq = DOWNLINK_FREQ;
	}

	EEPROM_Read(K24C02_PAGE_ADDR(3), UL_OFFSET, (uint8_t*) &lora->ufreq,
			sizeof(lora->ufreq));
	if (lora->ufreq < UPLINK_FREQ_MIN || lora->ufreq > UPLINK_FREQ_MAX) {
		lora->ufreq = UPLINK_FREQ;
	}

	lora->rxhw = sx1276_hw_rx;
	lora->txhw = sx1276_hw_tx;

	init_lora_parameters(lora);

}

uint8_t readReg(SX1276_HW_t *hw, uint8_t address) {
	uint8_t rec = 0;
	HAL_StatusTypeDef res;
	HAL_GPIO_WritePin(hw->nssPort, hw->nssPin, GPIO_PIN_RESET); // pull the pin low
	HAL_Delay(1);
	res = HAL_SPI_Transmit(hw->spi, &address, 1, 100);  // send address
	res = HAL_SPI_Receive(hw->spi, &rec, 1, 100);  // receive 6 bytes data
	HAL_Delay(1);
	HAL_GPIO_WritePin(hw->nssPort, hw->nssPin, GPIO_PIN_SET); // pull the pin high
	if (res != HAL_OK)
		Error_Handler();
	return (rec);
}

void writeReg(SX1276_HW_t *hw, uint8_t address, const uint8_t *cmd,
		uint8_t lenght) {
	HAL_StatusTypeDef res;
	if (lenght > 4)
		return;
	uint8_t tx_data[5] = { 0 };
	tx_data[0] = address | 0x80;
	int j = 0;
	for (int i = 1; i <= lenght; i++) {
		tx_data[i] = cmd[j++];
	}
	HAL_GPIO_WritePin(hw->nssPort, hw->nssPin, GPIO_PIN_RESET); // pull the pin low
	res = HAL_SPI_Transmit(hw->spi, tx_data, lenght + 1, 1000);
	HAL_GPIO_WritePin(hw->nssPort, hw->nssPin, GPIO_PIN_SET); // pull the pin high
	if (res != HAL_OK)
		Error_Handler();
	//HAL_Delay(10);
}

void setFreqReg(SX1276_HW_t *hw, uint32_t frequency) {
	uint8_t regOpMode = LORA_MODE_ACTIVATION | LOW_FREQUENCY_MODE | STANDBY;
	uint64_t freq = ((uint64_t) frequency << 19) / FXOSC;
	uint8_t freq_reg[3];
	freq_reg[0] = (uint8_t) (freq >> 16);
	freq_reg[1] = (uint8_t) (freq >> 8);
	freq_reg[2] = (uint8_t) (freq >> 0);
	writeReg(hw, LR_RegOpMode, &regOpMode, 1);
	HAL_Delay(1);
	writeReg(hw, LR_RegFrMsb, freq_reg, sizeof(freq_reg));
}

void setMode(SX1276_HW_t *hw, uint32_t freq, uint8_t dioMapping,
		uint8_t irqFlagsMask) {
	uint8_t regOpMode = LORA_MODE_ACTIVATION | LOW_FREQUENCY_MODE | STANDBY;
	uint64_t freqReal = ((uint64_t) freq << 19) / FXOSC;
	uint8_t freq_reg[3];
	freq_reg[0] = (uint8_t) (freqReal >> 16);
	freq_reg[1] = (uint8_t) (freqReal >> 8);
	freq_reg[2] = (uint8_t) (freqReal >> 0);
	uint8_t clear_irq[] = { CLEARIRQ };
	writeReg(hw, LR_RegOpMode, &regOpMode, 1);
	HAL_Delay(1);
	writeReg(hw, LR_RegFrMsb, freq_reg, sizeof(freq_reg));
	writeReg(hw, LR_RegDioMapping1, &dioMapping, 1);
	writeReg(hw, LR_RegIrqFlags, clear_irq, 1);
	writeReg(hw, LR_RegIrqFlagsMask, &irqFlagsMask, 1);
}

void setTxMode(SX1276_HW_t *hw, uint32_t freq) {
	setMode(hw, freq, DIOMAPPING1TX, FLAGSMOODETX);
}

void setRxMode(SX1276_HW_t *hw, uint32_t freq) {
	setMode(hw, freq, 0x00, FLAGSMOODERX);
}

void writeLoRaParams(LORA_t *loRa) {
	uint8_t headerMode;
	uint8_t symbTimeoutMsb;
	uint8_t sleep_mode_cmd[] = { LORA_SLEEP_MODE };
	writeReg(loRa->txhw, LR_RegOpMode, sleep_mode_cmd, 1);
	writeReg(loRa->rxhw, LR_RegOpMode, sleep_mode_cmd, 1);

	HAL_Delay(15);
	if (loRa->sf == SF_6) {
		headerMode = IMPLICIT;
		symbTimeoutMsb = 0x03;
		uint8_t tmp;
		tmp = readReg(loRa->txhw, LR_RegDetectOptimize);
		tmp = readReg(loRa->rxhw, LR_RegDetectOptimize);
		tmp &= 0xF8;
		tmp |= 0x05;
		writeReg(loRa->txhw, LR_RegDetectOptimize, &tmp, 1);
		writeReg(loRa->rxhw, LR_RegDetectOptimize, &tmp, 1);
		tmp = 0x0C;
		writeReg(loRa->txhw, LR_RegDetectionThreshold, &tmp, 1);
		writeReg(loRa->rxhw, LR_RegDetectionThreshold, &tmp, 1);
	} else {
		headerMode = EXPLICIT;
		symbTimeoutMsb = 0x00;
	}

	uint8_t cmd = 0;
	cmd = loRa->bw << 4;
	cmd += loRa->cr << 1;
	cmd += headerMode;
	writeReg(loRa->txhw, LR_RegModemConfig1, &cmd, 1); //Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)
	writeReg(loRa->rxhw, LR_RegModemConfig1, &cmd, 1);
	cmd = loRa->sf << 4;
	cmd += CRC_SUM << 2;
	cmd += symbTimeoutMsb;
	writeReg(loRa->txhw, LR_RegModemConfig2, &cmd, 1);
	writeReg(loRa->rxhw, LR_RegModemConfig2, &cmd, 1);
}

void writeCommon(SX1276_HW_t *hw) {
	uint8_t cmd[] = { LORA_SLEEP_MODE };
	writeReg(hw, LR_RegOpMode, cmd, 1);
	HAL_Delay(15);
	for (uint8_t i = 0; i < sizeof(config) / sizeof(config[0]); i++) {
		writeReg(hw, config[i].reg, &config[i].value, 1);
	}
}

void restartRegFifoAddrPtr(SX1276_HW_t *hw) {
	uint8_t zero = 0;
	uint8_t cmd[] = { LORA_SLEEP_MODE };
	writeReg(hw, LR_RegOpMode, cmd, 1);
	HAL_Delay(15);
	writeReg(hw, LR_RegFifoAddrPtr, &zero, 1);
}

void startRxContinuous(SX1276_HW_t *hw, uint8_t payloadLength) {
	uint8_t addr = 0;
	uint8_t cmd[] = { LORA_SLEEP_MODE };
	writeReg(hw, LR_RegOpMode, cmd, 1);
	//writeReg(hw, LR_RegPayloadLength, &(payloadLength), 1); //RegPayloadLength 21byte
	writeReg(hw, LR_RegFifoAddrPtr, &addr, 1); //RegFifoAddrPtr
	cmd[0] = LORA_RX_CONTINUOUS_MODE;
	writeReg(hw, LR_RegOpMode, cmd, 1);

}

uint8_t lora_rx_done_old(LORA_t *loRa) {

	SX1276_HW_t *hw = loRa->rxhw;
	HAL_GPIO_WritePin(LORA_RX_LED_GPIO_Port, LORA_RX_LED_Pin, GPIO_PIN_SET);
	uint8_t regflags = readReg(hw, LR_RegIrqFlags);
	if ((regflags & flags) != 0x40)
		return (0);
	uint8_t clear_irq[] = { CLEARIRQ };
	writeReg(hw, LR_RegIrqFlags, clear_irq, 1);
	loRa->rxSize = readReg(hw, LR_RegRxNbBytes); //Number for received bytes
	HAL_GPIO_WritePin(LORA_RX_LED_GPIO_Port, LORA_RX_LED_Pin, GPIO_PIN_RESET);
	return (loRa->rxSize);
}

void init_sx1276_hw(SX1276_HW_t *sx1276_hw, SPI_HandleTypeDef *hspi,
		GPIO_TypeDef *nssPort, uint16_t nssPin, GPIO_TypeDef *nrstPort,
		uint16_t nrstPin, GPIO_TypeDef *dio0Port, uint16_t dio0Pin) {
	// Ensure that the pointers are not NULL
	assert_param(sx1276_hw != NULL);
	assert_param(hspi != NULL);
	assert_param(nssPort != NULL);
	assert_param(nrstPort != NULL);
	assert_param(dio0Port != NULL);

	sx1276_hw->nssPin = nssPin;
	sx1276_hw->nssPort = nssPort;
	sx1276_hw->nrstPin = nrstPin;
	sx1276_hw->nrstPort = nrstPort;
	sx1276_hw->dio0Port = dio0Port;
	sx1276_hw->dio0Pin = dio0Pin;
	sx1276_hw->spi = hspi;
	HAL_GPIO_WritePin(nssPort, nssPin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(nrstPort, nrstPin, GPIO_PIN_SET);
}

void read_lora_LR_RegFifo(SX1276_HW_t *hw, uint8_t *data_fifo,
		uint8_t data_size) {

	uint8_t addr = LR_RegFifo;
	HAL_GPIO_WritePin(hw->nssPort, hw->nssPin, GPIO_PIN_RESET); // pull the pin low
	HAL_Delay(1);
	HAL_SPI_Transmit(hw->spi, &addr, 1, 100); // send address
	HAL_SPI_Receive(hw->spi, data_fifo, data_size, 100); // receive 6 bytes data
	HAL_Delay(1);
	HAL_GPIO_WritePin(hw->nssPort, hw->nssPin, GPIO_PIN_SET); // pull the pin high
}

void restart_lora_rx_continuous_mode(LORA_t *lora) {
	memset(lora->rxData, 0, 300);
	lora->rxSize = 0;
	startRxContinuous(lora->rxhw, RECEIVE_PAYLOAD_LENGTH);
}

uint8_t read_lora_packet(LORA_t *lora) {

    assert_param(lora != NULL);
    assert_param(lora->rxhw != NULL);

	if (HAL_GPIO_ReadPin(lora->rxhw->dio0Port, lora->rxhw->dio0Pin)
			== GPIO_PIN_RESET)
		return (0);

	HAL_GPIO_WritePin(LORA_RX_LED_GPIO_Port, LORA_RX_LED_Pin, GPIO_PIN_SET);
	uint8_t irq_flags = readReg(lora->rxhw, LR_RegIrqFlags);
	if ((irq_flags & flags) != 0x40)
		return (0);
	uint8_t clear_irq = CLEARIRQ;
	writeReg(lora->rxhw, LR_RegIrqFlags, &clear_irq, 1);
	lora->rxSize = readReg(lora->rxhw, LR_RegRxNbBytes); //Number for received bytes
	uint8_t addr = LR_RegFifo;
	HAL_GPIO_WritePin(lora->rxhw->nssPort, lora->rxhw->nssPin, GPIO_PIN_RESET); // pull the pin low
	//HAL_Delay(1);
	HAL_SPI_Transmit(lora->rxhw->spi, &addr, 1, 100); // send address
	HAL_SPI_Receive(lora->rxhw->spi, lora->rxData, lora->rxSize, 100); // receive 6 bytes data
	//HAL_Delay(1);
	HAL_GPIO_WritePin(lora->rxhw->nssPort, lora->rxhw->nssPin, GPIO_PIN_SET); // pull the pin high
	HAL_GPIO_WritePin(LORA_RX_LED_GPIO_Port, LORA_RX_LED_Pin, GPIO_PIN_RESET);
	return (1);
}

uint8_t write_lora_RegFifo(SX1276_HW_t *hw, uint8_t *data, uint8_t data_length) {

	if (data_length == 0)
		return (-1);
	uint8_t addr[] = { DEFAULT_TX_ADDR };
	writeReg(hw, LR_RegPayloadLength, &(data_length), 1);
	writeReg(hw, LR_RegFifoAddrPtr, addr, 1);
	for (int i = 0; i < data_length; i++)
		writeReg(hw, LR_RegFifo, data + i, 1);
	return (0);
}

SX1278_Status_t start_lora_transmission(SX1276_HW_t *hw) {
	uint8_t irqFlags = 0;
	uint32_t timeStart = HAL_GetTick();
	uint8_t cmd[] = { LORA_TX_MODE };
	writeReg(hw, LR_RegOpMode, cmd, 1);

	while (1) {
		irqFlags = readReg(hw, LR_RegIrqFlags);
		if (HAL_GPIO_ReadPin(hw->dio0Port, hw->dio0Pin) || (irqFlags & 0x08)) {
			uint8_t cmd = 0xFF;
			writeReg(hw, LR_RegIrqFlags, &cmd, 1);
			return (TX_DONE);
		}
		if (HAL_GetTick() - timeStart > LORA_SEND_TIMEOUT) {
			return (TX_TIMEOUT);
		}
	}
}

void lora_reset(SX1276_HW_t *hw) {
	HAL_GPIO_WritePin(hw->nrstPort, hw->nrstPin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(hw->nrstPort, hw->nrstPin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(hw->nrstPort, hw->nrstPin, GPIO_PIN_SET);
	HAL_Delay(100);
}

void init_lora_parameters(LORA_t *lora) {
	writeCommon(lora->txhw);
	writeCommon(lora->rxhw);
	setTxMode(lora->txhw, lora->ufreq);
	setRxMode(lora->rxhw, lora->dfreq);
	writeLoRaParams(lora);
	startRxContinuous(lora->rxhw, RECEIVE_PAYLOAD_LENGTH);
}
