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
#include <string.h>

/// max message size is 32, at maximum 27 bytes left for version characters
const char def_version[LOGISTIC_DATA_MAX_LENGTH] = "Ver:0.1 SN:123456";

/**
 * @brief read MCU firmware version
 *
 * @param data data to store version string characters
 * @param len length of data array
 * @return uint32_t size of version string read into data array.
 *   If data is not large enough, the 0 is returned and version characters
 *   are not read back.
 */
uint32_t read_ver(uint8_t data[], uint32_t len)
{
	uint32_t i;
	uint32_t  ver_len = 0;
	uint8_t *ptr = LOGISTIC_DATA_START_ADDRESS;

	if (len < LOGISTIC_DATA_MAX_LENGTH) {
		return 0;
	}

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

	/*all 0xFF, invalid version information*/
	if(i == ver_len)
	{
		/// use default version
		ver_len = strlen(def_version);
		for(i = 0; i < ver_len; ++i){
			data[i] = def_version[i];
		}
	}
	return ver_len;
}


