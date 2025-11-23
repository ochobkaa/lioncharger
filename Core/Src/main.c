/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	CHARGE_OFF = 0u,
	CHARGE_CONST_I,
	CHARGE_CONST_V,
	CHARGE_FINISHED,
	CHARGE_UNDERVOLT
} ChargeMode;

typedef struct {
	int32_t prop;
	int32_t diff;
	int32_t sum;
} PID;

typedef struct {
	uint32_t duty;
	uint32_t duty_incr;
} Duty;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define V_MIN 0xDF1u
#define V_MAX 0x293u
#define V_STABLE 0x740u
#define V_CONST 0x2F2u
#define V_DELTA 0x100u

#define I_MIN 0xF65u
#define I_MAX 0x2A3u
#define I_CONST 0x76Bu
#define I_DELTA 0x12Du

#define DUTY_MIN 0x008u
#define DUTY_MAX 0x1AEu

#define PID_DIFF_MUL 0
#define PID_SUM_MUL 0
#define PID_PROP_MUL 1
#define PID_MUL_SUM (PID_DIFF_MUL + PID_SUM_MUL + PID_PROP_MUL)

#define PID_MUL_SCALE 0x00000400

#define V_OFFSET 1
#define I_OFFSET 1

#define DUTY_DIVIDER 0x00100000
#define DUTY_INCR_MIN (DUTY_MIN * DUTY_DIVIDER)
#define DUTY_INCR_MAX (DUTY_MAX * DUTY_DIVIDER)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
GPIO_PinState hasVoltage;
GPIO_PinState hasCurrent;

uint32_t voltage;
uint32_t current;

Duty duty;

ChargeMode current_mode;

PID pid;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void mainLoop(uint32_t* voltage, uint32_t* current,
		GPIO_PinState* has_voltage, GPIO_PinState* has_current,
		ChargeMode* current_mode, PID* pid, Duty* duty);

void getVoltage(uint32_t* voltage);
void getVoltageAndCurrent(uint32_t* voltage, uint32_t* current);

void setChargeMode(ChargeMode* current_mode, ChargeMode new_mode);
void setPWMDuty(PID* pid, Duty* current_duty, ChargeMode* mode, uint32_t* voltage, uint32_t* current);
void startPWM(Duty* duty);
void stopPWM(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  duty.duty = DUTY_MIN;
  duty.duty_incr = DUTY_MIN * DUTY_DIVIDER;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  mainLoop(&voltage, &current, &hasVoltage, &hasCurrent, &current_mode, &pid, &duty);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 511;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void mainLoop(uint32_t* voltage, uint32_t* current,
		GPIO_PinState* has_voltage, GPIO_PinState* has_current,
		ChargeMode* current_mode, PID* pid, Duty* duty) {
	*has_voltage = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	*has_current = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

	if (*has_voltage == GPIO_PIN_SET && *has_current == GPIO_PIN_RESET && *current_mode == CHARGE_OFF) {
		getVoltage(voltage);

		if (*voltage > V_MIN) {
			setChargeMode(current_mode, CHARGE_UNDERVOLT);
			return;
		}
		else if (*voltage < V_MIN && *voltage > V_CONST) {
			setChargeMode(current_mode, CHARGE_CONST_I);
		}
		else if (*voltage < V_CONST && *voltage > V_MAX) {
			setChargeMode(current_mode, CHARGE_CONST_V);
		}
		else if (*voltage < V_MAX) {
			setChargeMode(current_mode, CHARGE_FINISHED);
			return;
		}

		startPWM(duty);

		pid->prop = DUTY_MIN;
		pid->sum = DUTY_MIN;
	}
	else if (*has_voltage == GPIO_PIN_SET && *has_current == GPIO_PIN_RESET && *current_mode == CHARGE_CONST_I) {
		getVoltage(voltage);
		current = 0;

		if (*voltage > V_MIN) {
			setChargeMode(current_mode, CHARGE_UNDERVOLT);
			stopPWM();
		}
		else if (*voltage < V_CONST && *voltage > V_MAX) {
			setChargeMode(current_mode, CHARGE_CONST_V);
		}
		else if (*voltage < V_MAX) {
			setChargeMode(current_mode, CHARGE_FINISHED);
			stopPWM();
		}
	}
	else if (*has_voltage == GPIO_PIN_SET && *has_current == GPIO_PIN_RESET && *current_mode == CHARGE_CONST_V) {
		getVoltage(voltage);
		current = 0;

		if (*voltage > V_MIN) {
			setChargeMode(current_mode, CHARGE_UNDERVOLT);
			stopPWM();
		}
		else if (*voltage < V_MAX) {
			setChargeMode(current_mode, CHARGE_FINISHED);
			stopPWM();
		}
	}
	else if (*has_voltage == GPIO_PIN_SET && *has_current == GPIO_PIN_SET && *current_mode == CHARGE_OFF) {
		getVoltageAndCurrent(voltage, current);

		if (*voltage > V_MIN) {
			setChargeMode(current_mode, CHARGE_UNDERVOLT);
			return;
		}
		else if (*voltage < V_MIN && *voltage > V_CONST) {
			setChargeMode(current_mode, CHARGE_CONST_I);
		}
		else if (*voltage < V_CONST && *voltage > V_MAX) {
			setChargeMode(current_mode, CHARGE_CONST_V);
		}
		else if (*voltage < V_MAX) {
			setChargeMode(current_mode, CHARGE_FINISHED);
			return;
		}

		startPWM(duty);

		pid->prop = DUTY_MIN;
		pid->sum = DUTY_MIN;
	}
	else if (*has_voltage == GPIO_PIN_SET && *has_current == GPIO_PIN_SET && *current_mode == CHARGE_CONST_I) {
		getVoltageAndCurrent(voltage, current);

		if (*voltage < V_CONST && *voltage > V_MAX) {
			setChargeMode(current_mode, CHARGE_CONST_V);
		}
		else if (*voltage < V_MAX) {
			setChargeMode(current_mode, CHARGE_FINISHED);

			stopPWM();
			return;
		}
	}
	else if (*has_voltage == GPIO_PIN_SET && *has_current == GPIO_PIN_SET && *current_mode == CHARGE_CONST_V) {
		getVoltageAndCurrent(voltage, current);

		if (*voltage < V_MAX) {
			setChargeMode(current_mode, CHARGE_FINISHED);

			stopPWM();
			return;
		}
	}
	else if (*has_voltage == GPIO_PIN_SET && *has_current == GPIO_PIN_RESET && *current_mode == CHARGE_CONST_V) {
		setChargeMode(current_mode, CHARGE_FINISHED);

		stopPWM();
	}
	else if (*has_voltage == GPIO_PIN_RESET && *has_current == GPIO_PIN_RESET && *current_mode != CHARGE_OFF) {
		setChargeMode(current_mode, CHARGE_OFF);

		stopPWM();
	}
}

void getVoltage(uint32_t* voltage) {
	HAL_ADC_Start(&hadc1);

	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
		*voltage = HAL_ADC_GetValue(&hadc1);
}

void getVoltageAndCurrent(uint32_t* voltage, uint32_t* current) {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);

	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
		*voltage = HAL_ADC_GetValue(&hadc1);

	if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK)
		*current = HAL_ADC_GetValue(&hadc2);
}

void setChargeMode(ChargeMode* current_mode, ChargeMode new_mode) {
	*current_mode = new_mode;

	if (new_mode == CHARGE_OFF || new_mode == CHARGE_FINISHED || new_mode == CHARGE_UNDERVOLT) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	}
	else if (new_mode == CHARGE_CONST_I) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	}
	else if (new_mode == CHARGE_CONST_V) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	}
}

void startPWM(Duty* duty) {
	if (HAL_TIM_GetChannelState(&htim4, TIM_CHANNEL_1) == HAL_TIM_CHANNEL_STATE_READY) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty->duty);

		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		HAL_TIM_Base_Start_IT(&htim4);
	}
}

void stopPWM(void) {
	if (HAL_TIM_GetChannelState(&htim4, TIM_CHANNEL_1) == HAL_TIM_CHANNEL_STATE_BUSY) {
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	}
}

void setPWMDuty(PID* pid, Duty* current_duty, ChargeMode* mode, uint32_t* voltage, uint32_t* current) {
	int32_t i_err;
	if (*mode == CHARGE_CONST_I) {
		i_err = *current - I_CONST;
	}
	else if (*mode == CHARGE_CONST_V) {
		i_err = *voltage - V_CONST;
	}

	pid->diff = i_err - pid->prop;
	pid->sum += i_err;
	pid->prop = i_err;

	int32_t d_duty_incr = PID_MUL_SCALE * (PID_DIFF_MUL * pid->diff + PID_SUM_MUL * pid->sum + PID_PROP_MUL * pid->prop) / PID_MUL_SUM;

	uint32_t new_duty_incr = current_duty->duty_incr + d_duty_incr;
	uint32_t new_duty = new_duty_incr / DUTY_DIVIDER;

	if ((d_duty_incr > 0 && new_duty_incr > current_duty->duty_incr) || (d_duty_incr < 0 && new_duty_incr < current_duty->duty_incr)) {
		if (new_duty_incr <= DUTY_INCR_MAX && new_duty_incr >= DUTY_INCR_MIN) {
			current_duty->duty_incr = new_duty_incr;
			current_duty->duty = new_duty;

			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, current_duty->duty);
		}
		else if (new_duty_incr > DUTY_INCR_MAX) {
			current_duty->duty_incr = DUTY_INCR_MAX;
			current_duty->duty = DUTY_MAX;

			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, DUTY_MAX);
		}

		else if (new_duty_incr < DUTY_INCR_MIN) {
			current_duty->duty = 0;

			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
		}
	}
	else if (d_duty_incr > 0 && new_duty_incr < current_duty->duty_incr) {
		current_duty->duty_incr = DUTY_INCR_MAX;
		current_duty->duty = DUTY_MAX;

		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, DUTY_MAX);
	}
	else if (d_duty_incr < 0 && new_duty_incr > current_duty->duty_incr) {
		current_duty->duty_incr = 0;
		current_duty->duty = 0;

		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
    	if (current_mode == CHARGE_CONST_I || current_mode == CHARGE_CONST_V) {
    		setPWMDuty(&pid, &duty, &current_mode, &voltage, &current);
    	}
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  stopPWM();

  __disable_irq();

  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
