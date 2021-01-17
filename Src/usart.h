/*
 *   ******************************************************************************
 *  Copyright (c) 2019-2020
 *  All rights reserved.
 *
 *  Created on: 05/19/2019
 *  Author: Cui Lijun
 *  mail: cljun08@163.com
 *   ******************************************************************************
 */

#ifndef USART_H_
#define USART_H_

#define PAYLOAD_POS   0x4u
//#define UART_MSG_LENGTH   0x06

#define IPC_HEADER   0xa55a
#define	SHORT_PRESS    0x01
#define	MIDDLE_PRESS   0x02
#define LONG_PRESS	   0x03
#define IS_CHARGING    0x04
#define IS_NOT_CHARGING 0x05

#define	SHORT_PRESS_ACK    0xF1
#define	MIDDLE_PRESS_ACK   0xF2
#define	LONG_PRESS_ACK     0xF3
#define	LONG_PRESS_CANCEL_ACK   0xFF
#define IS_CHARGING_ACK     0xF4
#define IS_NOT_CHARGING_ACK 0xF5

enum {
	OFF = 0,
	WAITING_OFF,
	READY_OFF,
	ON
}POWER_STATE;
#endif /* USART_H_ */
