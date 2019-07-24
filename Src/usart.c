/*
 * usart.c
 *
 *  Created on: May 14, 2019
 *      Author: cljun
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
	else if(huart->pRxBuffPtr[PAYLOAD_POS] == MIDDLE_PRESS_ACK)
	{
		Set_CurrentPowState(READY_OFF);
	}
	else if(huart->pRxBuffPtr[PAYLOAD_POS] == MIDDLE_PRESS_CANCEL_ACK)
	{
		Set_CurrentPowState(ON);
	}
	else
	{

	}
}
