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
#define E22_DIO1_Pin GPIO_PIN_3
#define E22_DIO1_GPIO_Port GPIOA
#define E22_DIO1_EXTI_IRQn EXTI3_IRQn
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOA
#define E22_RESET_Pin GPIO_PIN_0
#define E22_RESET_GPIO_Port GPIOB
#define E22_BUSY_Pin GPIO_PIN_1
#define E22_BUSY_GPIO_Port GPIOB
#define E22_TXEN_Pin GPIO_PIN_12
#define E22_TXEN_GPIO_Port GPIOB
#define E22_RXEN_Pin GPIO_PIN_13
#define E22_RXEN_GPIO_Port GPIOB
#define LED_TX_Pin GPIO_PIN_15
#define LED_TX_GPIO_Port GPIOA
#define BUZZER_PWM_Pin GPIO_PIN_3
#define BUZZER_PWM_GPIO_Port GPIOB
#define KEY_UP_Pin GPIO_PIN_4
#define KEY_UP_GPIO_Port GPIOB
#define USB_CTRL_Pin GPIO_PIN_5
#define USB_CTRL_GPIO_Port GPIOB
#define LED_RX_Pin GPIO_PIN_6
#define LED_RX_GPIO_Port GPIOB
#define KEY_ENTER_Pin GPIO_PIN_7
#define KEY_ENTER_GPIO_Port GPIOB
#define KEY_DOWN_Pin GPIO_PIN_9
#define KEY_DOWN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
