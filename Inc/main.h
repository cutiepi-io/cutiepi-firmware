/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

enum {
	OFF = 0,
	WAITING_OFF,
	READY_OFF,
	ON
}POWER_STATE;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define FLAG_1OMS      0x01
#define FLAG_1000MS    0x02

#define MSG_TYPE_KEY 0x01
#define MSG_TYPE_BATTERY 0x02
#define MSG_TYPE_CHARGING 0x03
#define MSG_TYPE_VERSION 0x04
#define MSG_TYPE_POWEROFF 0x05

#define	MCU_MSG_SHORT_CLICKED 0x01
#define	MCU_MSG_SHUTDOWN_HOLD_PRESSED   0x03
#define MCU_MSG_IS_CHARGING    0x04
#define MCU_MSG_IS_NOT_CHARGING 0x05

#define	PI_ACK_SHORT_CLICKED    0xF1
#define	PI_ACK_SHUTDOWN_PRESS_CONFIRM   0xF3
#define	PI_ACK_SHUTDOWN_PRESS_CANCEL   0xFC
#define PI_ACK_IS_CHARGING     0xF4
#define PI_ACK_IS_NOT_CHARGING 0xF5
#define PI_MSG_NULL 0x00
#define PI_MSG_MCU_VERSION_GET    0xF6
#define PI_MSG_POWEROFF_CMD       0xF7

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint32_t Get_CurrentPowState(void);
void Set_MsgFromPi(uint8_t cmd);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCU_REV1_Pin LL_GPIO_PIN_0
#define MCU_REV1_GPIO_Port GPIOA
#define MCU_REV2_Pin LL_GPIO_PIN_1
#define MCU_REV2_GPIO_Port GPIOA
#define USB5V_AD_Pin LL_GPIO_PIN_2
#define USB5V_AD_GPIO_Port GPIOA
#define MCU_IND_Pin LL_GPIO_PIN_3
#define MCU_IND_GPIO_Port GPIOA
#define switch_input_Pin LL_GPIO_PIN_4
#define switch_input_GPIO_Port GPIOA
#define BOOST_EN_Pin LL_GPIO_PIN_5
#define BOOST_EN_GPIO_Port GPIOA
#define CHARGE_EN_Pin LL_GPIO_PIN_6
#define CHARGE_EN_GPIO_Port GPIOA
#define IN2SYS_EN_Pin LL_GPIO_PIN_7
#define IN2SYS_EN_GPIO_Port GPIOA
#define BAT_AD_Pin LL_GPIO_PIN_1
#define BAT_AD_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
