/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PM1_ZEROU_Pin GPIO_PIN_0
#define PM1_ZEROU_GPIO_Port GPIOA
#define PM1_ZEROV_Pin GPIO_PIN_1
#define PM1_ZEROV_GPIO_Port GPIOA
#define PM1_ZEROW_Pin GPIO_PIN_2
#define PM1_ZEROW_GPIO_Port GPIOA
#define AMP_IW_Pin GPIO_PIN_3
#define AMP_IW_GPIO_Port GPIOA
#define ADC_EMFW_W_Pin GPIO_PIN_4
#define ADC_EMFW_W_GPIO_Port GPIOA
#define ADC_EMFW_U_Pin GPIO_PIN_5
#define ADC_EMFW_U_GPIO_Port GPIOA
#define AMP_IV_Pin GPIO_PIN_6
#define AMP_IV_GPIO_Port GPIOA
#define ADC_EMFW_V_Pin GPIO_PIN_7
#define ADC_EMFW_V_GPIO_Port GPIOA
#define ADC_EMF_M_Pin GPIO_PIN_4
#define ADC_EMF_M_GPIO_Port GPIOC
#define ADC_12V_Pin GPIO_PIN_5
#define ADC_12V_GPIO_Port GPIOC
#define AMP_IU_Pin GPIO_PIN_0
#define AMP_IU_GPIO_Port GPIOB
#define VBUS_ADC_Pin GPIO_PIN_1
#define VBUS_ADC_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define PM1_PWM_UL_Pin GPIO_PIN_13
#define PM1_PWM_UL_GPIO_Port GPIOB
#define PM1_PWM_VL_Pin GPIO_PIN_14
#define PM1_PWM_VL_GPIO_Port GPIOB
#define PM1_PWM_WL_Pin GPIO_PIN_15
#define PM1_PWM_WL_GPIO_Port GPIOB
#define PWM_SD_Pin GPIO_PIN_8
#define PWM_SD_GPIO_Port GPIOD
#define PM1_PWM_UH_Pin GPIO_PIN_8
#define PM1_PWM_UH_GPIO_Port GPIOA
#define PM1_PWM_VH_Pin GPIO_PIN_9
#define PM1_PWM_VH_GPIO_Port GPIOA
#define PM1_PWM_WH_Pin GPIO_PIN_10
#define PM1_PWM_WH_GPIO_Port GPIOA
#define CTRL_SD_Pin GPIO_PIN_2
#define CTRL_SD_GPIO_Port GPIOD
#define OC_OUT_Pin GPIO_PIN_3
#define OC_OUT_GPIO_Port GPIOD
#define OC_OUT_EXTI_IRQn EXTI3_IRQn
#define PWM_SDD4_Pin GPIO_PIN_4
#define PWM_SDD4_GPIO_Port GPIOD
#define PWM_SDD4_EXTI_IRQn EXTI4_IRQn
#define EEPROM_EN_Pin GPIO_PIN_5
#define EEPROM_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
