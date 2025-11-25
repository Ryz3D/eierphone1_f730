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
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "core.h"
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
#define BAT_CHRG_Pin GPIO_PIN_3
#define BAT_CHRG_GPIO_Port GPIOE
#define KB_COL_1_Pin GPIO_PIN_0
#define KB_COL_1_GPIO_Port GPIOC
#define KB_COL_2_Pin GPIO_PIN_1
#define KB_COL_2_GPIO_Port GPIOC
#define KB_COL_3_Pin GPIO_PIN_2
#define KB_COL_3_GPIO_Port GPIOC
#define KB_COL_4_Pin GPIO_PIN_3
#define KB_COL_4_GPIO_Port GPIOC
#define KB_ROW_B_Pin GPIO_PIN_0
#define KB_ROW_B_GPIO_Port GPIOA
#define KB_ROW_A_Pin GPIO_PIN_1
#define KB_ROW_A_GPIO_Port GPIOA
#define KB_ROW_C_Pin GPIO_PIN_2
#define KB_ROW_C_GPIO_Port GPIOA
#define KB_ROW_D_Pin GPIO_PIN_3
#define KB_ROW_D_GPIO_Port GPIOA
#define KB_ROW_E_Pin GPIO_PIN_4
#define KB_ROW_E_GPIO_Port GPIOA
#define KB_ROW_F_Pin GPIO_PIN_5
#define KB_ROW_F_GPIO_Port GPIOA
#define KB_ROW_G_Pin GPIO_PIN_6
#define KB_ROW_G_GPIO_Port GPIOA
#define KB_ROW_H_Pin GPIO_PIN_7
#define KB_ROW_H_GPIO_Port GPIOA
#define KB_COL_5_Pin GPIO_PIN_4
#define KB_COL_5_GPIO_Port GPIOC
#define KB_COL_6_Pin GPIO_PIN_5
#define KB_COL_6_GPIO_Port GPIOC
#define LED_BLUE_Pin GPIO_PIN_0
#define LED_BLUE_GPIO_Port GPIOB
#define VBAT_SNS_Pin GPIO_PIN_1
#define VBAT_SNS_GPIO_Port GPIOB
#define LCD_TE_Pin GPIO_PIN_11
#define LCD_TE_GPIO_Port GPIOB
#define LCD_DAT_INS_Pin GPIO_PIN_11
#define LCD_DAT_INS_GPIO_Port GPIOD
#define LCD_POW_EN_Pin GPIO_PIN_8
#define LCD_POW_EN_GPIO_Port GPIOC
#define KB_ROW_I_Pin GPIO_PIN_8
#define KB_ROW_I_GPIO_Port GPIOA
#define LCD_EXTC_Pin GPIO_PIN_6
#define LCD_EXTC_GPIO_Port GPIOD
#define LED_RED_Pin GPIO_PIN_4
#define LED_RED_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOB
#define LCD_LIGHT_Pin GPIO_PIN_0
#define LCD_LIGHT_GPIO_Port GPIOE
#define LCD_RESET_Pin GPIO_PIN_1
#define LCD_RESET_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
