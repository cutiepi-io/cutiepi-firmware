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

#include "version.h"
#include <stddef.h>

const char def_version[32] = "Ver0.1 SN:123456";

/**
  * @brief  read mcu firmware version,
  * @retval  0: valid version is read;
  *         others: read failed or invalid version;
  */
uint32_t read_ver(const uint8_t* address, uint8_t data[], uint32_t *len)
{
	uint32_t i;
	uint32_t  ver_len = 0;
	uint32_t ret = 0xFF;
	uint8_t *ptr = NULL;

	if(!address)
		return ret;

	ptr = address;
	while(*ptr && ver_len <= LOGISTIC_DATA_MAX_LENGTH)
	{
		data[ver_len++] = *ptr++;
	}

	for(i = 0; i < ver_len; i++)
	{
		if (data[i] != 0xFF)
		{
			break;
		}
	}

	if(i == ver_len)/*all 0xFF, invalid version information*/
	{
		ret = 1;
	}
	else
	{
		*len = ver_len;
		ret = 0;
	}
	return ret;
}


