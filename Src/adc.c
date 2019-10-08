/*
 * adc.c
 *
 *  Created on: 2019年5月19日
 *      Author: cljun
 */

#include "stm32f0xx.h"
#include "crc.h"

extern UART_HandleTypeDef huart1;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint32_t Adc_val = 0;
	uint32_t ResultVoltage = 0;
	uint32_t batt_vol = 0;
	uint8_t tx_data[10] = {0};

	if( __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOS))
	{
		Adc_val = HAL_ADC_GetValue(hadc);
		ResultVoltage = (Adc_val * 3300) >> 12;
		batt_vol = ResultVoltage *673 / 523; /*15k+52.3=67.3*/

		tx_data[0] = 0x5a;
		tx_data[1] = 0xa5;
		tx_data[2] = 2;
		tx_data[3] = 0;
		tx_data[4] = (uint8_t)(batt_vol&0xFF);
		tx_data[5] = (uint8_t)((batt_vol >> 8) &0xFF);
		tx_data[6] = crc8_calculate(tx_data, MIN_IPC_MSG_LEN + 1);
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)tx_data, MIN_IPC_MSG_LEN + 2);
	}

	return;
}
