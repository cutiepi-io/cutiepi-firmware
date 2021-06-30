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

const uint8_t def_version[32] = {'V','e','r','0','.','1','\0'};

/**
  * @brief  read mcu firmware version,
  * @retval  0: valid version is read;
  *         others: read failed or invalid version;
  */
uint32_t read_ver(uint8_t* address, uint8_t data[], uint32_t len)
{
	uint32_t i;
	uint32_t ret = 0xFF;

	if(!address || len > LOGISTIC_DATA_MAX_LENGTH)
		return ret;

	memcpy(data, address, LOGISTIC_DATA_LENGTH);

	for(i = 1; i <= LOGISTIC_DATA_LENGTH; i++)
	{
		if (data[i] != 0xFF)
		{
			break;
		}
	}

	if(i == LOGISTIC_DATA_LENGTH)
	{
		ret = 1;
	}
	else
	{
		ret = 0;
	}
	return ret;
}


