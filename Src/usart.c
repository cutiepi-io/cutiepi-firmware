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
#include "crc.h"
#include "usart.h"
#include "version.h"
#include <string.h>


uint8_t version[32] = {0xFF};
extern uint8_t def_version[32];
extern UART_HandleTypeDef huart1;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}

void uart_transmit(uint32_t ch, uint8_t *data, uint32_t data_len)
{
	uint8_t total_data[48];

	if(ch != 1) /* fixed 1 at current*/
		return;

	total_data[0] = 0x5a;
	total_data[1] = 0xa5;
	total_data[2] = 0x4; //mcu version msg type
	total_data[3] = data_len & 0xff;
	total_data[4] = (data_len >> 8) && 0xff;

	memcpy(&total_data[5], data, data_len);
	total_data[5 + data_len] = crc8_calculate(total_data, MIN_IPC_MSG_LEN + data_len - 1 );

	HAL_UART_Transmit_IT(&huart1, total_data, MIN_IPC_MSG_LEN + data_len);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	huart->pRxBuffPtr -= UART_MSG_LENGTH + 1; //set offset to buffer head
	
	switch(huart->pRxBuffPtr[PAYLOAD_POS])
	{

		case SHORT_PRESS_ACK:
		case IS_CHARGING_ACK:
		case IS_NOT_CHARGING_ACK:
			HAL_UART_Receive_IT(huart, huart->pRxBuffPtr, MIN_IPC_MSG_LEN + 1);
		break;

		case SHUTDOWN_PRESS_CONFIRM_ACK:
			if( WAITING_OFF == Get_CurrentPowState())
			{
				Set_CurrentPowState(READY_OFF);
			}
			HAL_UART_Receive_IT(huart, huart->pRxBuffPtr, MIN_IPC_MSG_LEN + 1);
		break;

		case SHUTDOWN_PRESS_CANCEL_ACK:
			Set_CurrentPowState(ON);
			HAL_UART_Receive_IT(huart, huart->pRxBuffPtr, MIN_IPC_MSG_LEN + 1);
		break;

		case MCU_VERSION_GET:
			if( 0 == read_ver(LOGISTIC_DATA_START_ADDRESS, version, LOGISTIC_DATA_LENGTH))
			{
				uart_transmit(1, version, LOGISTIC_DATA_LENGTH);
			}
			else
			{
				uart_transmit(1, def_version, LOGISTIC_DATA_LENGTH);
			}
			HAL_UART_Receive_IT(huart, huart->pRxBuffPtr, MIN_IPC_MSG_LEN + 1);
		break;

		default:
			break;
	}
}
