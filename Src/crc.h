/*
 *   ******************************************************************************
 *  Copyright (c) 2019-2020 Lijun.Cui
 *  All rights reserved.
 *
 *  Created on: 05/19/2019
 *  Author: Lijun.Cui
 *  mail: cljun08@163.com
 *   ******************************************************************************
 */

#ifndef CRC_H_
#define CRC_H_

#define MIN_IPC_MSG_LEN  0x6u
#define UART_MSG_LENGTH   0x6u

#define KEY_MSG_PAYLOAD_LEN 0x1
#define SOC_MSG_PAYLOAD_LEN 0X2
#define CHARGE_MSG_PAYLOAD_LEN 0x1 


uint8_t crc8_calculate(uint8_t *psrc, uint8_t len);

#endif /* CRC_H_ */
