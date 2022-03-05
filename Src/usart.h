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

#define PAYLOAD_POS   0x5u //CM payload position,fixed lenth (MIN_IPC_MSG_LEN+1 = 7)
//#define UART_MSG_LENGTH   0x06

#define IPC_HEADER   0xa55a
#define	SHORT_PRESS    0x01
#define	SHUTDOWN_PRESS   0x03
#define IS_CHARGING    0x04
#define IS_NOT_CHARGING 0x05

#define	SHORT_PRESS_ACK    0xF1
#define	SHUTDOWN_PRESS_CONFIRM_ACK   0xF3
#define	SHUTDOWN_PRESS_CANCEL_ACK   0xFC
#define IS_CHARGING_ACK     0xF4
#define IS_NOT_CHARGING_ACK 0xF5
#define MCU_VERSION_GET    0xF6
#define POWEROFF_CMD       0xF7


/*system power status*/
#define OFF 0
#define WAITING_OFF 1
#define READY_OFF 2
#define ON  3

#endif /* USART_H_ */
