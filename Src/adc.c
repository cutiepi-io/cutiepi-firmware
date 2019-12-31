/*
 * adc.c
 *
 *  Created on: 2019年5月19日
 *      Author: cljun
 */

#include "stm32f0xx.h"
#include "crc.h"
#include "usart.h"
#include "main.h"

#define USB_CHARGE_THRESHOLD_VAL     3000  /*3.0V*/

extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc;

void ADC_Check_And_Send(void)
{
	uint32_t batt_adc = 0;
	uint32_t usb_char_adc = 0;
	uint32_t ResultVoltage = 0;
	uint32_t batt_vol = 0;
	uint8_t tx_data[6] = {0};

	tx_data[0] = 0x5a;
	tx_data[1] = 0xa5;
	tx_data[2] = 2; // msg length

	if(Get_CurrentPowState() != OFF)
	{
		//HAL_ADC_Start_IT(&hadc);
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc, 10);
		usb_char_adc = HAL_ADC_GetValue(&hadc);
		if(USB_CHARGE_THRESHOLD_VAL <= usb_char_adc)
		{
			tx_data[3] = 0; // data_msb
			tx_data[4] = IS_CHARGING; // data_lsb
			tx_data[5] = crc8_calculate(tx_data, UART_MSG_LENGTH - 1);
			HAL_UART_Transmit_IT(&huart1, (uint8_t *)tx_data, UART_MSG_LENGTH);
			LL_GPIO_ResetOutputPin(BOOST_EN_GPIO_Port, BOOST_EN_Pin);
		}
		else
		{
			tx_data[3] = 0; // data_msb
			tx_data[4] = IS_NOT_CHARGING; // data_lsb
			tx_data[5] = crc8_calculate(tx_data, UART_MSG_LENGTH - 1);
			HAL_UART_Transmit_IT(&huart1, (uint8_t *)tx_data, UART_MSG_LENGTH);
			LL_GPIO_ResetOutputPin(CHARGE_EN_GPIO_Port, CHARGE_EN_Pin);
		}

		HAL_ADC_PollForConversion(&hadc, 10);
		batt_adc = HAL_ADC_GetValue(&hadc);
		ResultVoltage = (batt_adc * 3300) >> 12;
		batt_vol = ResultVoltage *673 / 523; /*15k+52.3=67.3*/
		tx_data[3] = (uint8_t)((batt_vol >> 8) &0xFF); // data_msb
		tx_data[4] = (uint8_t)(batt_vol & 0xFF); // data_lsb
		tx_data[5] = crc8_calculate(tx_data, UART_MSG_LENGTH - 1);
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)tx_data, UART_MSG_LENGTH);

		HAL_ADC_Stop(&hadc);
	}
	else
	{
//		HAL_ADC_Start(&hadc);
//		HAL_ADC_PollForConversion(&hadc, 10);
//		usb_char_adc = HAL_ADC_GetValue(&hadc);
//		if(USB_CHARGE_THRESHOLD_VAL <= usb_char_adc)
//		{ /*power CM when charging at off mode*/
//		  /**/
//		  LL_GPIO_SetOutputPin(BOOST_EN_GPIO_Port, BOOST_EN_Pin);
//
//		  /**/
//		  LL_GPIO_SetOutputPin(CHARGE_EN_GPIO_Port, CHARGE_EN_Pin);
//
//		  /**/
//		  LL_GPIO_SetOutputPin(IN2SYS_EN_GPIO_Port, IN2SYS_EN_Pin);
//		  Set_CurrentPowState(ON);
//		}
//		HAL_ADC_PollForConversion(&hadc, 10);
//		batt_adc = HAL_ADC_GetValue(&hadc);
//
//		HAL_ADC_Stop(&hadc);

	}
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	uint32_t Adc_val = 0;
//	uint32_t ResultVoltage = 0;
//	uint32_t batt_vol = 0;
//	uint8_t tx_data[6] = {0};
//
//	if( __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOS))
//	{
//		Adc_val = HAL_ADC_GetValue(hadc);
//		ResultVoltage = (Adc_val * 3300) >> 12;
//		batt_vol = ResultVoltage *673 / 523; /*15k+52.3=67.3*/
//
//		tx_data[0] = 0x5a;
//		tx_data[1] = 0xa5;
//		tx_data[2] = 2; //battery msg
//		tx_data[3] = (uint8_t)((batt_vol >> 8) &0xFF); // data_msb
//		tx_data[4] = (uint8_t)(batt_vol & 0xFF); // data_lsb
//		tx_data[5] = crc8_calculate(tx_data, UART_MSG_LENGTH - 1);
//		HAL_UART_Transmit_IT(&huart1, (uint8_t *)tx_data, UART_MSG_LENGTH);
//	}
//
//	return;
//}
