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

#include "stm32f0xx.h"
#include "main.h"
#include <stdint.h>

#define USB_CHARGE_THRESHOLD_VAL     3000  /*3.0V*/

extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc;

static uint32_t adc_result_usb = 0;
static uint32_t adc_result_battery = 0;

void ADC_update()
{
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 10);
	adc_result_usb = HAL_ADC_GetValue(&hadc);
	HAL_ADC_PollForConversion(&hadc, 10);
	adc_result_battery = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
}

uint16_t get_battery_voltage(){
	/// 3300/4096 ; 15k + 52.3k = 67.3k
	return (((adc_result_battery * 3300) >> 12) * 673 / 523);
}

uint16_t get_usb_voltage(){
	return (adc_result_usb * 3300) >> 12;
}

