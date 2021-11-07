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

#include <stdint.h>

uint8_t checksum_calculate(uint8_t *psrc, uint8_t len);

#endif /* CRC_H_ */
