/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "UserDrivers.h"
#include "a_appl.h"
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
#define SMPS_POLA2_Pin GPIO_PIN_0
#define SMPS_POLA2_GPIO_Port GPIOC
#define SMPS_POLA1_Pin GPIO_PIN_1
#define SMPS_POLA1_GPIO_Port GPIOC
#define FLT_RLY_Pin GPIO_PIN_3
#define FLT_RLY_GPIO_Port GPIOC
#define USART2_RE_Pin GPIO_PIN_0
#define USART2_RE_GPIO_Port GPIOA
#define USART2_DE_Pin GPIO_PIN_1
#define USART2_DE_GPIO_Port GPIOA
#define DAC_NSS_Pin GPIO_PIN_4
#define DAC_NSS_GPIO_Port GPIOA
#define DAC_CLK_Pin GPIO_PIN_5
#define DAC_CLK_GPIO_Port GPIOA
#define DAC_MISO_Pin GPIO_PIN_6
#define DAC_MISO_GPIO_Port GPIOA
#define DAC_MOSI_Pin GPIO_PIN_7
#define DAC_MOSI_GPIO_Port GPIOA
#define DAC_NCLR_Pin GPIO_PIN_4
#define DAC_NCLR_GPIO_Port GPIOC
#define DAC_ALARM_Pin GPIO_PIN_5
#define DAC_ALARM_GPIO_Port GPIOC
#define USART3_DE_Pin GPIO_PIN_1
#define USART3_DE_GPIO_Port GPIOB
#define USART3_RE_Pin GPIO_PIN_2
#define USART3_RE_GPIO_Port GPIOB
#define ADC12_NSS_Pin GPIO_PIN_12
#define ADC12_NSS_GPIO_Port GPIOB
#define ADC_SCK_Pin GPIO_PIN_13
#define ADC_SCK_GPIO_Port GPIOB
#define ADC_MISO_Pin GPIO_PIN_14
#define ADC_MISO_GPIO_Port GPIOB
#define ADC_MOSI_Pin GPIO_PIN_15
#define ADC_MOSI_GPIO_Port GPIOB
#define ADC_NSYNC_Pin GPIO_PIN_6
#define ADC_NSYNC_GPIO_Port GPIOC
#define ADC34_NSS_Pin GPIO_PIN_7
#define ADC34_NSS_GPIO_Port GPIOC
#define BDID3_Pin GPIO_PIN_9
#define BDID3_GPIO_Port GPIOC
#define BDID2_Pin GPIO_PIN_8
#define BDID2_GPIO_Port GPIOA
#define BDID1_Pin GPIO_PIN_9
#define BDID1_GPIO_Port GPIOA
#define BDID0_Pin GPIO_PIN_10
#define BDID0_GPIO_Port GPIOA
#define BD_CHK_Pin GPIO_PIN_11
#define BD_CHK_GPIO_Port GPIOC
#define LED_ERR_Pin GPIO_PIN_12
#define LED_ERR_GPIO_Port GPIOC
#define LED_COM_Pin GPIO_PIN_2
#define LED_COM_GPIO_Port GPIOD
#define LED_RUN_Pin GPIO_PIN_3
#define LED_RUN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
