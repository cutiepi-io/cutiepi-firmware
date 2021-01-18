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


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->pRxBuffPtr[PAYLOAD_POS] == SHORT_PRESS_ACK)
	{

	}
	else if(huart->pRxBuffPtr[PAYLOAD_POS] == SHUTDOWN_PRESS_CONFIRM_ACK)
	{
		if( WAITING_OFF == Get_CurrentPowState())
		{
			Set_CurrentPowState(READY_OFF);
		}
	}
	else if(huart->pRxBuffPtr[PAYLOAD_POS] == SHUTDOWN_PRESS_CANCEL_ACK)
	{
		Set_CurrentPowState(ON);
	}
	else
	{

	}
}
