/*
 *   ******************************************************************************
 *  Copyright (c) 2019-2021 Lijun.Cui
 *  All rights reserved.
 *
 *  Created on: 06/29/2021
 *  Author: Lijun.Cui
 *  mail: cljun08@163.com
 *   ******************************************************************************
 */

#ifndef VERSION_H_
#define VERSION_H_

#include <stdint.h>

#define LOGISTIC_DATA_START_ADDRESS   (uint8_t *)((void*)(0x8003FE0))
#define LOGISTIC_DATA_MAX_LENGTH   (32)
#define LOGISTIC_DATA_LENGTH       (6)


uint32_t read_ver(uint8_t* address, uint8_t data[32], uint32_t len);

#endif /* VERSION_H_ */
