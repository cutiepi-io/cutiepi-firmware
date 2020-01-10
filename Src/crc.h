/*
 * crc.h
 *
 *  Created on: Jul 20, 2019
 *      Author: cljun
 */

#ifndef CRC_H_
#define CRC_H_

#define MIN_IPC_MSG_LEN  0x5u
#define UART_MSG_LENGTH   0x6u

uint8_t crc8_calculate(uint8_t *psrc, uint8_t len);

#endif /* CRC_H_ */
