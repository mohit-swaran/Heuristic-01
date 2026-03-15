/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LEDONB_Pin GPIO_PIN_13
#define LEDONB_GPIO_Port GPIOC
#define TOF3XSHUT_Pin GPIO_PIN_14
#define TOF3XSHUT_GPIO_Port GPIOC
#define FRONT_TOF_INT_Pin GPIO_PIN_2
#define FRONT_TOF_INT_GPIO_Port GPIOA
#define FRONT_TOF_INT_EXTI_IRQn EXTI2_IRQn
#define LEFT_TOF_INT_Pin GPIO_PIN_3
#define LEFT_TOF_INT_GPIO_Port GPIOA
#define LEFT_TOF_INT_EXTI_IRQn EXTI3_IRQn
#define ACS_Pin GPIO_PIN_4
#define ACS_GPIO_Port GPIOA
#define PWMA_Pin GPIO_PIN_0
#define PWMA_GPIO_Port GPIOB
#define PWMB_Pin GPIO_PIN_1
#define PWMB_GPIO_Port GPIOB
#define MCS_Pin GPIO_PIN_2
#define MCS_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_12
#define BIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_13
#define BIN1_GPIO_Port GPIOB
#define STBY_Pin GPIO_PIN_14
#define STBY_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_15
#define AIN2_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_8
#define AIN1_GPIO_Port GPIOA
#define RIGHT_TOF_INT_Pin GPIO_PIN_5
#define RIGHT_TOF_INT_GPIO_Port GPIOB
#define RIGHT_TOF_INT_EXTI_IRQn EXTI9_5_IRQn
#define TOF1XSHUT_Pin GPIO_PIN_6
#define TOF1XSHUT_GPIO_Port GPIOB
#define TOF2XSHUT_Pin GPIO_PIN_7
#define TOF2XSHUT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
