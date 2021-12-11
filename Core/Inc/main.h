/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define MD1_AM_Pin GPIO_PIN_0
#define MD1_AM_GPIO_Port GPIOA
#define ENC2_CHB_Pin GPIO_PIN_1
#define ENC2_CHB_GPIO_Port GPIOA
#define ENC2_CHA_Pin GPIO_PIN_5
#define ENC2_CHA_GPIO_Port GPIOA
#define MD2_PWM_Pin GPIO_PIN_7
#define MD2_PWM_GPIO_Port GPIOA
#define MD2_AM_Pin GPIO_PIN_0
#define MD2_AM_GPIO_Port GPIOB
#define LINK_LED_Pin GPIO_PIN_1
#define LINK_LED_GPIO_Port GPIOB
#define ENC1_CHA_Pin GPIO_PIN_8
#define ENC1_CHA_GPIO_Port GPIOA
#define ENC1_CHB_Pin GPIO_PIN_9
#define ENC1_CHB_GPIO_Port GPIOA
#define MD_EN_Pin GPIO_PIN_10
#define MD_EN_GPIO_Port GPIOA
#define MD1_DIR_Pin GPIO_PIN_11
#define MD1_DIR_GPIO_Port GPIOA
#define MD2_DIR_Pin GPIO_PIN_12
#define MD2_DIR_GPIO_Port GPIOA
#define MD1_PWM_Pin GPIO_PIN_4
#define MD1_PWM_GPIO_Port GPIOB
#define MD_STATE_Pin GPIO_PIN_7
#define MD_STATE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
