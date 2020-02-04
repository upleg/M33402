/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : M33402 Power detector
  ******************************************************************************
  * Date:            10.01.2020
  * Auhtor:          Oleg Borisenko
  ******************************************************************************
  * Part of this program code was generated using STM32Cube initialization
  * code generator
  * 
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
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
#include <math.h>
#include "7seg_led.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Экспоненциальное усреднение

/**	Yn = (Yn-1 * K) + Xn * (1-K),
 	где Yn - отфильтрованное значение
 		Yn-1 - отфильтрованное на предыдущем пересчёте значение
 		Xn - фильтруемые значения
 		K - коэффициент фильтрации (0..1)
 **/

void average(uint16_t *input, uint16_t *output, const float coeff)
{
		
	*output = (*output * coeff) + *input * (1 - coeff);
}


// Сортировка массива целочисленных значений вставками

/**	@*arrayPtr - указатель на массив
 **	@lenght - кол-во элементов массива
 **/

void sort_array(int *arrayPtr, int length)
{
	int temp;
	int item;
	
	for (int counter = 1; counter < length; counter++)
	{
		temp = arrayPtr[counter];
		item = counter - 1;
		
		while (item >= 0 && arrayPtr[item] > temp)
		{
			arrayPtr[item + 1] = arrayPtr[item];
			arrayPtr[item] = temp;
			item--;
		}
	}
}


const float v_amp = 3;	// Коэффициент передачи напряжения детектора

uint8_t atten = 0;

int adc_value = 0;
int adc_avg = 0;
int adc_buf[100] = { 0 };
int tmp;

float U = 0.0;
float P = 0.0;

// Математическая модель детектора М33402-16

/* General model Fourier1:
     f(x) =  a0 + a1*cos(x*w) + b1*sin(x*w)
Coefficients (with 95% confidence bounds):
       a0 =       45.53  (32.1, 58.97)
       a1 =      -45.53  (-58.97, -32.09)
       b1 =       2.255  (1.749, 2.76)
       w =   0.0006955  (0.0005855, 0.0008055)

Goodness of fit:
  SSE: 0.0223
  R-square: 1
  Adjusted R-square: 1
  RMSE: 0.01736
*/

float a0 = 45.53;
float a1 = -45.53;
float b1 = 2.255;
float w = 0.0006955;

/*	Выходное напряжение детектора, мВ
*/
float get_voltage(unsigned int adc_value) {

	return (adc_value * 3275 / 4095) / v_amp;
}


/*	Мощность на детекторе, мВт
*/
float calculate_power(float voltage) {

	return a0 + a1 * cos(voltage * w) + b1 * sin(voltage * w);
}


float convert_to_uw(float power) {

	float value;

	value = power * 1000;

	if (value < 100) {

		value = round(value * 10) / 10;
	}
	else
		value = round(value);
	
	HAL_GPIO_WritePin(LED_W_GPIO_Port, LED_W_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_mW_GPIO_Port, LED_mW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_uW_GPIO_Port, LED_uW_Pin, GPIO_PIN_SET);

	return value;
}


float convert_to_mw(float power) {

	float value;

	value = power;

	if (value < 10) {

		value = round(value * 100) / 100;
	}
	else if (value > 100) {
		
		value = round(value);
	}
	else {

		value = round(value * 10) / 10;
	}
	
	HAL_GPIO_WritePin(LED_W_GPIO_Port, LED_W_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_uW_GPIO_Port, LED_uW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_mW_GPIO_Port, LED_mW_Pin, GPIO_PIN_SET);

	return value;
}


float convert_to_w(float power) {

	float value;

	value = power / 1000;

	if (value < 10) {

		value = round(value * 100) / 100;
	}
	else {

		value = round(value * 10) / 10;
	}
	
	HAL_GPIO_WritePin(LED_uW_GPIO_Port, LED_uW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_mW_GPIO_Port, LED_mW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_W_GPIO_Port, LED_W_Pin, GPIO_PIN_SET);

	return value;
}


/*	Мощность на детекторе превышает допустимое значение
 */
void is_ovld() {

	if (U >= 950)
	{
		HAL_GPIO_WritePin(LED_OVLD_GPIO_Port, LED_OVLD_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_OVLD_GPIO_Port, LED_OVLD_Pin, GPIO_PIN_RESET);
	}
}


/*	Пересчёт мощности с учетом подключенного к детектору аттенюатора */
void set_atten() 
{	
	switch (atten) {
		
	case 0:
		HAL_GPIO_WritePin(LED_0dB_GPIO_Port, LED_0dB_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_10dB_GPIO_Port, LED_10dB_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_30dB_GPIO_Port, LED_30dB_Pin, GPIO_PIN_RESET);
		break;

	case 1:
		P = P * 10;
		HAL_GPIO_WritePin(LED_10dB_GPIO_Port, LED_10dB_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_0dB_GPIO_Port, LED_0dB_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_30dB_GPIO_Port, LED_30dB_Pin, GPIO_PIN_RESET);
		break;

	case 2:
		P = P * 1000;
		HAL_GPIO_WritePin(LED_30dB_GPIO_Port, LED_30dB_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_0dB_GPIO_Port, LED_0dB_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_10dB_GPIO_Port, LED_10dB_Pin, GPIO_PIN_RESET);
		break;
	}
}


void get_range() {

	if (P < 1) {

		P = convert_to_uw(P);
	}
	else
		if (P < 1000) {

		P = convert_to_mw(P);
	}
	else {

		P = convert_to_w(P);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
//	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_ADCEx_Calibration_Start(&hadc1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  	  
	  
	  for(int i = 0 ; i < 100 ; i++)
	  {
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, 100);	
		  adc_buf[i] = HAL_ADC_GetValue(&hadc1);
		  HAL_ADC_Stop(&hadc1);
	  }
	  
	  sort_array(adc_buf, 100);
	  adc_avg = (adc_buf[48] + adc_buf[49] + adc_buf[50]) / 3;
	  
	  if (adc_avg <= 8) adc_avg = 0;
	  
	  U = get_voltage(adc_avg);
	  P = calculate_power(U);
	  
	  is_ovld();
	  set_atten();
	  get_range();
	  
	  led_print(P);
	  
	  HAL_Delay(200);
	  
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
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
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D_Pin|E_Pin|DP_Pin|C_Pin 
                          |B_Pin|A_Pin|F_Pin|G_Pin 
                          |HG1_Pin|HG2_Pin|HG3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_30dB_Pin|LED_10dB_Pin|LED_0dB_Pin|LED_W_Pin 
                          |LED_OVLD_Pin|LED_uW_Pin|LED_mW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D_Pin E_Pin DP_Pin C_Pin 
                           B_Pin A_Pin F_Pin G_Pin 
                           HG1_Pin HG2_Pin HG3_Pin */
  GPIO_InitStruct.Pin = D_Pin|E_Pin|DP_Pin|C_Pin 
                          |B_Pin|A_Pin|F_Pin|G_Pin 
                          |HG1_Pin|HG2_Pin|HG3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_EXTI_Pin */
  GPIO_InitStruct.Pin = BTN_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BTN_EXTI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_30dB_Pin LED_10dB_Pin LED_0dB_Pin LED_W_Pin 
                           LED_OVLD_Pin LED_uW_Pin LED_mW_Pin */
  GPIO_InitStruct.Pin = LED_30dB_Pin|LED_10dB_Pin|LED_0dB_Pin|LED_W_Pin 
                          |LED_OVLD_Pin|LED_uW_Pin|LED_mW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
