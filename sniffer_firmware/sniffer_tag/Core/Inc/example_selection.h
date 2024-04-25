/*! ----------------------------------------------------------------------------
 * @file    example_selection.h
 * @brief   Example selection is configured here
 *
 * @attention
 *
 * Copyright 2013 - 2020(c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#ifndef TEST_SELECTION_
#define TEST_SELECTION_

#ifdef __cplusplus
extern "C" {
#endif

//Enable the needed example/test. Please enable only one example/test!


//#define TEST_READING_DEV_ID							//(01.读取DevID例程)
#define TEST_SIMPLE_TX								//(02.简单Tx发送例程)
//#define TEST_SIMPLE_TX_PDOA							//(08.简单PDoA Tx发送例程)
//#define TEST_TX_SLEEP									//(03.发送进入睡眠模式例程)
//#define TEST_TX_SLEEP_IDLE_RC							//(发送进入休眠模式IDLE_RC例程)
//#define TEST_TX_SLEEP_AUTO							//(04.发送自动进入休眠例程)
//#define TEST_TX_SLEEP_TIMED							//(05.发送进入休眠内部唤醒例程)
//#define TEST_TX_WITH_CCA								//(06.发送信道评估例程)

//#define TEST_SIMPLE_RX								//(10.简单Rx接受例程)
//#define TEST_RX_DIAG									//(11.Rx接受信号强度例程)
//#define TEST_RX_SNIFF									//(12.Rx接受Sniff模式例程)
//#define TEST_RX_TRIM									//(13.Rx接受晶体调整例程)
//#define TEST_SIMPLE_RX_PDOA							//(15.简单PDoA Rx接受例程)

//#define TEST_SIMPLE_TX_STS_SDC						//(07.简单Tx发送STS例程)
//#define TEST_SIMPLE_RX_STS_SDC						//(14.简单Rx接受STS例程)

//#define TEST_SIMPLE_TX_AES							//(09.简单Tx发送AES例程)
//#define TEST_SIMPLE_RX_AES							//(16.简单Rx接受AES例程)

//#define TEST_TX_WAIT_RESP								//(17.Tx发送等待Response例程)
//#define TEST_TX_WAIT_RESP_INT							//(19.Tx发送等待Response例程)
//#define TEST_RX_SEND_RESP								//(18.Rx接受发送Response例程)

//#define TEST_CONTINUOUS_WAVE							//(20.连续波模式例程)
//#define TEST_CONTINUOUS_FRAME							//(21.连续帧模式例程)

//#define TEST_DS_TWR_INITIATOR_STS						//(24.双边测距(DS TWR STS)发起者例程)
//#define TEST_DS_TWR_RESPONDER_STS						//(25.双边测距(DS TWR STS)回复者例程)

//#define TEST_DS_TWR_INITIATOR							//(22.双边测距(DS TWR)发起者例程)
//#define TEST_DS_TWR_RESPONDER							//(23.双边测距(DS TWR)回复者例程)

//#define TEST_DS_TWR_STS_SDC_INITIATOR					//(24b.双边测距(DS TWR STS SDC)发起者例程)
//#define TEST_DS_TWR_STS_SDC_RESPONDER					//(25b.双边测距(DS TWR STS SDC)回复者例程)

//#define TEST_SS_TWR_INITIATOR							//(26.单边测距(SS TWR)发起者例程)
//#define TEST_SS_TWR_RESPONDER							//(27.单边测距(SS TWR)回复者例程)

//#define TEST_SS_TWR_INITIATOR_STS
//#define TEST_SS_TWR_RESPONDER_STS

//#define TEST_SS_TWR_INITIATOR_STS_NO_DATA
//#define TEST_SS_TWR_RESPONDER_STS_NO_DATA

//#define TEST_AES_SS_TWR_INITIATOR						//(28.单边测距(SS TWR AES)发起者例程)
//#define TEST_AES_SS_TWR_RESPONDER						//(29.单边测距(SS TWR AES)回复者例程)

//#define TEST_ACK_DATA_TX								//(30.自动确认数据Tx例程)
//#define TEST_ACK_DATA_RX								//(31.自动确认数据Rx例程)

//#define TEST_ACK_DATA_RX_DBL_BUFF

//#define TEST_SPI_CRC									//(32.SPI CRC错误例程)
//#define TEST_GPIO										//(33.GPIO使用例程)

//#define TEST_OTP_WRITE								//(34.OTP写入例程)

//#define TEST_LE_PEND_TX								//
//#define TEST_LE_PEND_RX

//#define TEST_FRAME_FILTERING_TX
//#define TEST_FRAME_FILTERING_RX

#ifdef __cplusplus
}
#endif


#endif
