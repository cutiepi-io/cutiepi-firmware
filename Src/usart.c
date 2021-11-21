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

#include "checksum.h"
#include "main.h"
#include "stm32f0xx.h"
#include "version.h"
#include <string.h>

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t sum_checksum = 0;

    if (huart->RxXferSize != 6) /* wrong message format */
    {
        HAL_UART_Receive_IT(huart, huart->pRxBuffPtr, 6);
        return;
    }

    huart->pRxBuffPtr -= 6; // set offset to buffer head
    sum_checksum = checksum_calculate(huart->pRxBuffPtr, 5);

    if (huart->pRxBuffPtr[0] == 0x5a && huart->pRxBuffPtr[1] == 0xa5 && sum_checksum == huart->pRxBuffPtr[5])
    {
        switch (huart->pRxBuffPtr[2])
        {
        case MSG_TYPE_KEY:
        case MSG_TYPE_BATTERY:
        case MSG_TYPE_CHARGING:
        case MSG_TYPE_VERSION:
        case MSG_TYPE_POWEROFF:
            break;
        default:
            HAL_UART_Receive_IT(huart, huart->pRxBuffPtr, 6);
            return;
        }

        if(huart->pRxBuffPtr[3] != 1)
        {
            HAL_UART_Receive_IT(huart, huart->pRxBuffPtr, 6);
            return;
        }

        switch (huart->pRxBuffPtr[4])
        {
        case PI_ACK_SHORT_CLICKED:
        case PI_ACK_SHUTDOWN_PRESS_CONFIRM:
        case PI_ACK_SHUTDOWN_PRESS_CANCEL:
        case PI_ACK_IS_CHARGING:
        case PI_ACK_IS_NOT_CHARGING:
        case PI_MSG_MCU_VERSION_GET:
        case PI_MSG_POWEROFF_CMD:
            Set_MsgFromPi(huart->pRxBuffPtr[4]);
            break;
        default:
            break;
        }
    }

    HAL_UART_Receive_IT(huart, huart->pRxBuffPtr, 6);
}
