/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "adc.h"
#include "crc.h"
#include "stm32f0xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SHORT_PRESS_DURATION	1000/10
#define MIDDLE_PRESS_DURATION	4000/10
#define LONG_PRESS_DURATION		6000/10
#define BATT_VOL_SEND_PERIOD    1000 //ms
#define POWKEY_ACTIVE 	    0
#define POWKEY_DEACTIVE 	1




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TRUE  1
#define FALSE 0
#define KEY_FIRST_ON        0x01
#define KEY_DEBOUNCE_START  0x02
#define KEY_DEBOUNCE_END    0x04
#define KEY_RELEASED        0x08
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t flag_key = 0;
static uint8_t current_powerState = OFF;
static uint8_t Uart_TxData[10] = {0};
static uint8_t Uart_RxData[10] = {0};
uint32_t u32KeyTimerCnt;
extern uint32_t flag_time;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t Get_Switch_Key_Val(GPIO_TypeDef *GPIOx, uint32_t key)
{
	return (LL_GPIO_IsInputPinSet(GPIOx, key) == TRUE) ? POWKEY_DEACTIVE :POWKEY_ACTIVE;
}

uint8_t Get_CurrentPowState(void)
{
	return current_powerState;
}

void Set_CurrentPowState(uint32_t state)
{
	 current_powerState = state;
}

void BatteryVol_Check_And_Send(void)
{
	if(Get_CurrentPowState() != OFF)
	{
		HAL_ADC_Start_IT(&hadc);
	}
}

void scan_key(void)
{
	static uint8_t debounce_time = 1; //10ms

	  if((flag_key & KEY_FIRST_ON) == FALSE)
	  {
		  if(POWKEY_ACTIVE == Get_Switch_Key_Val(switch_input_GPIO_Port, switch_input_Pin))
		  {
			  flag_key |= KEY_FIRST_ON;
			  flag_key |= KEY_DEBOUNCE_START;
		  }
	  }
	  else if(flag_key & KEY_DEBOUNCE_START)
	  {
		  debounce_time--;
		  if(debounce_time == 0)
		  {
			  debounce_time = 1;
			  flag_key &= ~KEY_DEBOUNCE_START;
			  flag_key |= KEY_DEBOUNCE_END;
		  }
	  }
	  else if(flag_key & KEY_DEBOUNCE_END)
	  {

		  if(POWKEY_ACTIVE == Get_Switch_Key_Val(switch_input_GPIO_Port, switch_input_Pin))
		  {
			  u32KeyTimerCnt++;
		  }
		  else
		  {
			  flag_key &= ~KEY_DEBOUNCE_END;
			  flag_key |= KEY_RELEASED;
		  }

	  }
	  else
	  {

	  }

	if((flag_key & KEY_RELEASED) != 0)
	{
		  if(0 < u32KeyTimerCnt && u32KeyTimerCnt <= SHORT_PRESS_DURATION
				  && ON == current_powerState)
		  {
			  flag_key = 0;
			  u32KeyTimerCnt = 0;
			  Uart_TxData[2] = 1; //key msg
			  Uart_TxData[3] = 1;
			  Uart_TxData[4] = 0; //length
			  Uart_TxData[5] = SHORT_PRESS;
			  Uart_TxData[6] = crc8_calculate(Uart_TxData, MIN_IPC_MSG_LEN + 1);
			  HAL_UART_Transmit_IT(&huart1, Uart_TxData, MIN_IPC_MSG_LEN + 1 + 1);
			  HAL_UART_Receive_IT(&huart1, Uart_RxData, MIN_IPC_MSG_LEN + 1 + 1);
		  }
		  else if(SHORT_PRESS_DURATION < u32KeyTimerCnt && u32KeyTimerCnt <= MIDDLE_PRESS_DURATION)
		  {
			  u32KeyTimerCnt = 0;
			  flag_key = 0;
	//			  if(/*LL_GPIO_IsOutputPinSet(BOOST_EN_GPIO_Port,BOOST_EN_Pin) &&*/
	//				 LL_GPIO_IsOutputPinSet(CHARGE_EN_GPIO_Port,CHARGE_EN_Pin) &&
	//				 LL_GPIO_IsOutputPinSet(IN2SYS_EN_GPIO_Port,IN2SYS_EN_Pin))
			  if(ON == current_powerState)
			  {
				  Uart_TxData[2] = 1;
				  Uart_TxData[3] = 1;
			    Uart_TxData[4] = 0; //length
				  Uart_TxData[5] = MIDDLE_PRESS;
				  Uart_TxData[6] = crc8_calculate(Uart_TxData, MIN_IPC_MSG_LEN + 1);
				  HAL_UART_Transmit_IT(&huart1, Uart_TxData, MIN_IPC_MSG_LEN + 1 + 1);
				  HAL_UART_Receive_IT(&huart1, Uart_RxData, MIN_IPC_MSG_LEN + 1 + 1);
				  Set_CurrentPowState(WAITING_OFF);
			  }
			  else
			  {
				  /**/
				  LL_GPIO_SetOutputPin(BOOST_EN_GPIO_Port, BOOST_EN_Pin);

				  /**/
				  LL_GPIO_SetOutputPin(CHARGE_EN_GPIO_Port, CHARGE_EN_Pin);

				  /**/
				  LL_GPIO_SetOutputPin(IN2SYS_EN_GPIO_Port, IN2SYS_EN_Pin);

				  Set_CurrentPowState(ON);
			  }
		  }
		  else
		  {
			  u32KeyTimerCnt = 0;
			  flag_key = 0;
		  }
	 }
	  if(u32KeyTimerCnt >= LONG_PRESS_DURATION)
	  {
		  /**/
		  LL_GPIO_ResetOutputPin(BOOST_EN_GPIO_Port, BOOST_EN_Pin);

		  /**/
		  LL_GPIO_ResetOutputPin(CHARGE_EN_GPIO_Port, CHARGE_EN_Pin);

		  /**/
		  LL_GPIO_ResetOutputPin(IN2SYS_EN_GPIO_Port, IN2SYS_EN_Pin);

		  Set_CurrentPowState(OFF);

		  u32KeyTimerCnt = 0;
		  flag_key = 0;

	  }
}

void time_10ms_proc(void)
{
	scan_key();
	  /*After pi excute power off sequence, it sends out ready off state to mcu,which will set
	   *  current_powerState to READY_OFF*/
	  if(current_powerState == READY_OFF)
	  {
		  /**/
		  LL_GPIO_ResetOutputPin(BOOST_EN_GPIO_Port, BOOST_EN_Pin);

		  /**/
		  LL_GPIO_ResetOutputPin(CHARGE_EN_GPIO_Port, CHARGE_EN_Pin);

		  /**/
		  LL_GPIO_ResetOutputPin(IN2SYS_EN_GPIO_Port, IN2SYS_EN_Pin);

		  Set_CurrentPowState(OFF);
	  }
}

void time_1000ms_proc(void)
{
	  BatteryVol_Check_And_Send();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	Uart_TxData[0] = 0x5a;
	Uart_TxData[1] = 0xa5;  //ipc header
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  NVIC_EnableIRQ(SysTick_IRQn);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  /**/
  LL_GPIO_ResetOutputPin(BOOST_EN_GPIO_Port, BOOST_EN_Pin);

  /**/
  LL_GPIO_ResetOutputPin(CHARGE_EN_GPIO_Port, CHARGE_EN_Pin);

  /**/
  LL_GPIO_ResetOutputPin(IN2SYS_EN_GPIO_Port, IN2SYS_EN_Pin);

  //crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)buf, sizeof(buf));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  time_proc();

	  LL_mDelay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = ENABLE;
  hadc.Init.LowPowerAutoPowerOff = ENABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */
  //LL_ADC_Enable(ADC1);
  //while(!LL_ADC_IsActiveFlag_ADRDY(ADC1)); //wait for ready
  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(MCU_REV1_GPIO_Port, MCU_REV1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(MCU_REV2_GPIO_Port, MCU_REV2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(MCU_REV3_GPIO_Port, MCU_REV3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(MCU_IND_GPIO_Port, MCU_IND_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BOOST_EN_GPIO_Port, BOOST_EN_Pin);

  /**/
  LL_GPIO_ResetOutputPin(CHARGE_EN_GPIO_Port, CHARGE_EN_Pin);

  /**/
  LL_GPIO_ResetOutputPin(IN2SYS_EN_GPIO_Port, IN2SYS_EN_Pin);

  /**/
  GPIO_InitStruct.Pin = MCU_REV1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(MCU_REV1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = MCU_REV2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(MCU_REV2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = MCU_REV3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(MCU_REV3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = MCU_IND_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(MCU_IND_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = switch_input_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(switch_input_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BOOST_EN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BOOST_EN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = CHARGE_EN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(CHARGE_EN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = IN2SYS_EN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(IN2SYS_EN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
