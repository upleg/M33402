/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define D_Pin GPIO_PIN_0
#define D_GPIO_Port GPIOA
#define E_Pin GPIO_PIN_1
#define E_GPIO_Port GPIOA
#define DP_Pin GPIO_PIN_2
#define DP_GPIO_Port GPIOA
#define C_Pin GPIO_PIN_3
#define C_GPIO_Port GPIOA
#define B_Pin GPIO_PIN_4
#define B_GPIO_Port GPIOA
#define A_Pin GPIO_PIN_5
#define A_GPIO_Port GPIOA
#define F_Pin GPIO_PIN_6
#define F_GPIO_Port GPIOA
#define G_Pin GPIO_PIN_7
#define G_GPIO_Port GPIOA
#define BTN_EXTI_Pin GPIO_PIN_0
#define BTN_EXTI_GPIO_Port GPIOB
#define BTN_EXTI_EXTI_IRQn EXTI0_IRQn
#define LED_30dB_Pin GPIO_PIN_10
#define LED_30dB_GPIO_Port GPIOB
#define LED_10dB_Pin GPIO_PIN_11
#define LED_10dB_GPIO_Port GPIOB
#define HG1_Pin GPIO_PIN_8
#define HG1_GPIO_Port GPIOA
#define HG2_Pin GPIO_PIN_9
#define HG2_GPIO_Port GPIOA
#define HG3_Pin GPIO_PIN_10
#define HG3_GPIO_Port GPIOA
#define LED_0dB_Pin GPIO_PIN_5
#define LED_0dB_GPIO_Port GPIOB
#define LED_W_Pin GPIO_PIN_6
#define LED_W_GPIO_Port GPIOB
#define LED_OVLD_Pin GPIO_PIN_7
#define LED_OVLD_GPIO_Port GPIOB
#define LED_uW_Pin GPIO_PIN_8
#define LED_uW_GPIO_Port GPIOB
#define LED_mW_Pin GPIO_PIN_9
#define LED_mW_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
