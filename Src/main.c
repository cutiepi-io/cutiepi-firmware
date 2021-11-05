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
#include "adc.h"
#include "version.h"
#include "checksum.h"
#include "stm32f0xx_it.h"
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BATT_VOL_SEND_PERIOD 1000 // ms
#define POWKEY_ACTIVE 0
#define POWKEY_DEACTIVE 1
#define DURATION_S(sec) ((sec * 1000) / 10)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TRUE 1
#define FALSE 0

#define KEY_EVT_NULL 0x00
#define KEY_EVT_SHORT_CLICKED 0x01
#define KEY_EVT_SHORT_PRESSED 0x02
#define KEY_EVT_MIDDLE_PRESSED 0x04
#define KEY_EVT_LONG_PRESSED 0x08

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

static uint32_t min_limit_key_short_clicked = 100;
static uint32_t max_limit_key_short_clicked = 500;
static uint32_t min_limit_key_short_pressed = 1500;
static uint32_t min_limit_key_middle_pressed = 2500;
static uint32_t min_limit_key_long_pressed = 8000;
static uint32_t current_key_evt = KEY_EVT_NULL;

/// 5000 ms time delay before power off
static const uint32_t power_off_delay = 12000;

static uint8_t current_powerState = OFF;
static uint8_t Uart_RxData[16] = {0};

static uint8_t current_cmd_from_pi = PI_MSG_NULL;

extern uint32_t flag_time;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t Get_Switch_Key_Val(GPIO_TypeDef *GPIOx, uint32_t key)
{
    return (LL_GPIO_IsInputPinSet(GPIOx, key) == TRUE) ? POWKEY_DEACTIVE
                                                       : POWKEY_ACTIVE;
}

static uint32_t Get_CurrentKeyEvent(void) { return current_key_evt; }

uint32_t Get_CurrentPowerState(void) { return current_powerState; }

static void toggle_indicator_led(void){
	static uint8_t indicator = 0;
	if(indicator){
		LL_GPIO_SetOutputPin(MCU_IND_GPIO_Port, MCU_IND_Pin);
	}else{
		LL_GPIO_ResetOutputPin(MCU_IND_GPIO_Port, MCU_IND_Pin);
	}
	indicator = !indicator;
}

static void Set_CurrentPowerState(uint32_t state)
{
    current_powerState = state;
}

static void Set_PowerGPIOInput(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = IN2SYS_EN_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(IN2SYS_EN_GPIO_Port, &GPIO_InitStruct);
}

static void Set_PowerGPIOOutput(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = BOOST_EN_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(BOOST_EN_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CHARGE_EN_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(CHARGE_EN_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = IN2SYS_EN_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(IN2SYS_EN_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief Get latest command from Pi
 *
 * @return uint8_t latest command need to response
 */
uint8_t Get_MsgFromPi(void)
{
    uint8_t ret_cmd = current_cmd_from_pi;
    return ret_cmd;
}

void Clear_MsgFromPi(void) { current_cmd_from_pi = PI_MSG_NULL; }

void Set_MsgFromPi(uint8_t cmd) { current_cmd_from_pi = cmd; }

static int32_t RPi_boot_pin_down_ms = 0;
static uint32_t RPi_alive_status = 0;

static uint32_t CheckRPiChipAlive(void) {

	if (LL_GPIO_IsInputPinSet(MCU_REV2_GPIO_Port, MCU_REV2_Pin)) {
		RPi_boot_pin_down_ms = 0;
		RPi_alive_status = 1;
	} else {
		RPi_boot_pin_down_ms += 10;
	}

	if (RPi_boot_pin_down_ms >= 3000) {
		RPi_boot_pin_down_ms = 3000;
		RPi_alive_status = 0;
	}

	return RPi_alive_status;
}

static void PowStateMichne(void)
{
    static uint32_t current_state_counter_ms = 0;

	uint32_t RPi_is_alive = CheckRPiChipAlive();

    uint8_t cmd_from_pi = Get_MsgFromPi();
    Clear_MsgFromPi();

    current_state_counter_ms += 10;
    if (current_state_counter_ms >= 100000)
    {
        current_state_counter_ms = 100000;
    }

    switch (Get_CurrentPowerState())
    {
    case OFF:
		Set_PowerGPIOOutput();
        LL_GPIO_ResetOutputPin(BOOST_EN_GPIO_Port, BOOST_EN_Pin);
        LL_GPIO_ResetOutputPin(CHARGE_EN_GPIO_Port, CHARGE_EN_Pin);
        LL_GPIO_ResetOutputPin(IN2SYS_EN_GPIO_Port, IN2SYS_EN_Pin);
        if (Get_CurrentKeyEvent() == KEY_EVT_SHORT_PRESSED)
        {
            Set_CurrentPowerState(ON);
            current_state_counter_ms = 0;
        }
        break;
    case WAITING_OFF:
        if (Get_CurrentKeyEvent() == KEY_EVT_LONG_PRESSED)
        {
            current_state_counter_ms = 0;
            Set_CurrentPowerState(READY_OFF);
        }
        else if (cmd_from_pi == PI_MSG_POWEROFF_CMD ||
                 cmd_from_pi == PI_ACK_SHUTDOWN_PRESS_CONFIRM)
        {
            Set_CurrentPowerState(READY_OFF);
            current_state_counter_ms = 0;
        }
        else if (cmd_from_pi == PI_ACK_SHUTDOWN_PRESS_CANCEL ||
                 current_state_counter_ms > 5000)
        {
            /// concel cmd from Pi or 5 seconds timeout waiting for ack from Pi
            Set_CurrentPowerState(ON);
            current_state_counter_ms = 0;
        }
        else
        {
            /// nothing to do for now
        }
        break;
    case READY_OFF:
		if ((current_state_counter_ms >= power_off_delay) || (!RPi_is_alive)) {
            Set_CurrentPowerState(OFF);
            current_state_counter_ms = 0;
        }
        break;
    case ON:
		Set_PowerGPIOInput();
		// LL_GPIO_SetOutputPin(IN2SYS_EN_GPIO_Port, IN2SYS_EN_Pin);
		LL_GPIO_SetOutputPin(BOOST_EN_GPIO_Port, BOOST_EN_Pin);
		LL_GPIO_SetOutputPin(CHARGE_EN_GPIO_Port, CHARGE_EN_Pin);

        if (Get_CurrentKeyEvent() == KEY_EVT_LONG_PRESSED)
        {
            current_state_counter_ms = 0;
            Set_CurrentPowerState(READY_OFF);
        }
        else if (cmd_from_pi == PI_MSG_POWEROFF_CMD ||
                 cmd_from_pi == PI_ACK_SHUTDOWN_PRESS_CONFIRM)
        {
            Set_CurrentPowerState(READY_OFF);
            current_state_counter_ms = 0;
        }
        else if (Get_CurrentKeyEvent() == KEY_EVT_MIDDLE_PRESSED)
        {
            Set_CurrentPowerState(WAITING_OFF);
            current_state_counter_ms = 0;
		} else if (!RPi_is_alive && current_state_counter_ms >= 2000) {
			Set_CurrentPowerState(OFF);
			current_state_counter_ms = 0;
        }
        else
        {
            /// nothing to do
        }
        break;
    default:
        break;
    }
}

/**
 * @brief scan key every 10 ms
 *
 * @return uint32_t the time in milliseconds the key is hold pressed.
 */
uint32_t scan_key(void)
{
    static uint32_t time_hold_pressed = 0;
    uint32_t key_event = KEY_EVT_NULL;

    if (POWKEY_ACTIVE ==
        Get_Switch_Key_Val(switch_input_GPIO_Port, switch_input_Pin))
    {
        time_hold_pressed += 10;
    }
    else
    {
        if (time_hold_pressed < min_limit_key_short_clicked)
        {
            /// nothing to do but wait
        }
        else if (time_hold_pressed < max_limit_key_short_clicked) {
			key_event = KEY_EVT_SHORT_CLICKED;
		}
        else if (time_hold_pressed < min_limit_key_short_pressed)
        {
			/// nothing to do but wait
        }
        else if (time_hold_pressed < min_limit_key_middle_pressed)
        {
            /// nothing to do here
        }
        else if (time_hold_pressed < min_limit_key_long_pressed)
        {
            /// nothing to do here
        }
        else
        {
            /// nothing to do here
        }
        /// clear static counter
        time_hold_pressed = 0;
    }

    if(time_hold_pressed == min_limit_key_short_pressed){
        key_event = KEY_EVT_SHORT_PRESSED;
    }else if(time_hold_pressed == min_limit_key_middle_pressed){
        key_event = KEY_EVT_MIDDLE_PRESSED;
    }else if(time_hold_pressed == min_limit_key_long_pressed){
        key_event = KEY_EVT_LONG_PRESSED;
    }

    return key_event;
}

void key_evt_comm_process(uint32_t key_evt)
{
    uint8_t tx_buff[6];
    tx_buff[0] = 0x5a;
    tx_buff[1] = 0xa5;
    tx_buff[2] = MSG_TYPE_KEY;
    tx_buff[3] = 1; // length of payload

    switch (key_evt)
    {
    case KEY_EVT_SHORT_CLICKED:
        tx_buff[4] = MCU_MSG_SHORT_CLICKED;
        break;
    case KEY_EVT_MIDDLE_PRESSED:
        tx_buff[4] = MCU_MSG_SHUTDOWN_HOLD_PRESSED;
        break;
    default:
        /// exit this function here.
        return;
    }

    tx_buff[5] = checksum_calculate(tx_buff, 5);
	HAL_UART_Transmit(&huart1, tx_buff, 6, 100);
}

void key_evt_process()
{
    current_key_evt = scan_key();
    key_evt_comm_process(current_key_evt);
    /// TODO: add more functional code here
}

void pi_msg_process()
{
    uint8_t tx[32] = {0x5a, 0xa5};
	uint8_t ver_len;

    uint8_t cmd_from_pi = Get_MsgFromPi();
    switch (cmd_from_pi)
    {
    case PI_MSG_MCU_VERSION_GET:
		tx[2] = MSG_TYPE_VERSION;
		ver_len = read_ver(tx + 4, LOGISTIC_DATA_MAX_LENGTH);
		if (ver_len > 0 && ver_len <= LOGISTIC_DATA_MAX_LENGTH) {
			tx[3] = ver_len;
			tx[4 + ver_len] = checksum_calculate(tx, 4 + ver_len);
			HAL_UART_Transmit(&huart1, tx, 5 + ver_len, 100);
		}
        break;
    default:
        break;
    }
}

void adc_comm_process() {
	uint8_t tx[8] = { 0x5a, 0xa5 };
	uint16_t adc_result = get_battery_voltage();

    tx[2] = MSG_TYPE_BATTERY;
    tx[3] = 0x02;
    tx[4] = adc_result >> 8;
    tx[5] = adc_result & 0xFF;
    tx[6] = checksum_calculate(tx, 6);
    HAL_UART_Transmit(&huart1, tx, 7, 100);

    if(get_usb_voltage() > USB_CHARGE_THRESHOLD_VAL){
        tx[2] = MSG_TYPE_CHARGING;
        tx[3] = 1;
        tx[4] = MCU_MSG_IS_CHARGING;
        tx[5] = checksum_calculate(tx, 5);
    }else{
        tx[2] = MSG_TYPE_CHARGING;
        tx[3] = 1;
        tx[4] = MCU_MSG_IS_NOT_CHARGING;
        tx[5] = checksum_calculate(tx, 5);
    }
    HAL_UART_Transmit(&huart1, tx, 6, 100);
}

void adc_process() {
	ADC_update();
    adc_comm_process();
}

void time_10ms_proc(void)
{
    key_evt_process();
    pi_msg_process();
    PowStateMichne();
    HAL_IWDG_Refresh(&hiwdg);
}

void time_1000ms_proc(void) {
	adc_process();
	toggle_indicator_led();
}

void time_proc(void)
{
    if (flag_time & FLAG_1OMS)
    {
        flag_time &= ~FLAG_1OMS;
        time_10ms_proc();
    }

    if (flag_time & FLAG_1000MS)
    {
        flag_time &= ~FLAG_1000MS;
        time_1000ms_proc();
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
    LL_GPIO_ResetOutputPin(MCU_IND_GPIO_Port, MCU_IND_Pin);
    LL_GPIO_ResetOutputPin(BOOST_EN_GPIO_Port, BOOST_EN_Pin);
    LL_GPIO_ResetOutputPin(CHARGE_EN_GPIO_Port, CHARGE_EN_Pin);
    LL_GPIO_ResetOutputPin(IN2SYS_EN_GPIO_Port, IN2SYS_EN_Pin);

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
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
    /*enable uart rx interrupt after init*/
    HAL_UART_Receive_IT(&huart1, Uart_RxData, 6);
    /**/
    LL_GPIO_ResetOutputPin(BOOST_EN_GPIO_Port, BOOST_EN_Pin);

    /**/
    LL_GPIO_ResetOutputPin(CHARGE_EN_GPIO_Port, CHARGE_EN_Pin);

    /**/
    LL_GPIO_ResetOutputPin(IN2SYS_EN_GPIO_Port, IN2SYS_EN_Pin);

    // crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)buf, sizeof(buf));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        time_proc();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  hadc.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */
    // LL_ADC_Enable(ADC1);
    // while(!LL_ADC_IsActiveFlag_ADRDY(ADC1)); //wait for ready
  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = 1250;
  hiwdg.Init.Reload = 1250;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  LL_GPIO_ResetOutputPin(MCU_IND_GPIO_Port, MCU_IND_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BOOST_EN_GPIO_Port, BOOST_EN_Pin);

  /**/
  LL_GPIO_ResetOutputPin(CHARGE_EN_GPIO_Port, CHARGE_EN_Pin);

  /**/
  GPIO_InitStruct.Pin = MCU_REV1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(MCU_REV1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = MCU_REV2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(MCU_REV2_GPIO_Port, &GPIO_InitStruct);

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
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BOOST_EN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = CHARGE_EN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(CHARGE_EN_GPIO_Port, &GPIO_InitStruct);

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
    /* User can add his own implementation to report the file name and line
 number, tex: printf("Wrong parameters value: file %s on line %d\r\n", file,
 line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
