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

#include "checksum.h"

uint8_t checksum_calculate(uint8_t *psrc, uint8_t len)
{
  uint8_t crc8_val = 0;
  uint32_t  id;

  for(id = 0; id < len; id++)
  {
	  crc8_val += psrc[id];
  }

  return crc8_val;
}
