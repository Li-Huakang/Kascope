/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define LED_Pin GPIO_PIN_14
#define LED_GPIO_Port GPIOC
#define MENU_Pin GPIO_PIN_4
#define MENU_GPIO_Port GPIOA
#define MENU_EXTI_IRQn EXTI4_15_IRQn
#define BACK_Pin GPIO_PIN_5
#define BACK_GPIO_Port GPIOA
#define BACK_EXTI_IRQn EXTI4_15_IRQn
#define ENCODER_A_Pin GPIO_PIN_15
#define ENCODER_A_GPIO_Port GPIOA
#define ENCODER_A_EXTI_IRQn EXTI4_15_IRQn
#define ENCODER_Z_Pin GPIO_PIN_3
#define ENCODER_Z_GPIO_Port GPIOB
#define ENCODER_Z_EXTI_IRQn EXTI2_3_IRQn
#define ENCODER_B_Pin GPIO_PIN_4
#define ENCODER_B_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_5
#define OLED_DC_GPIO_Port GPIOB
#define OLED_RES_Pin GPIO_PIN_6
#define OLED_RES_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
