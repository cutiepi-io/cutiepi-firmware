/*
 * crc.c
 *
 *  Created on: Jul 20, 2019
 *      Author: cljun
 */

#include <stdint.h>

uint8_t crc8_calculate(uint8_t *psrc, uint8_t len)
{
  uint8_t crc8_val = 0;
  uint32_t  id;

  for(id = 0; id < len; id++)
  {
	  crc8_val += psrc[id];
  }

  return crc8_val;
}
