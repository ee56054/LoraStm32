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
#include "stm32wlxx_hal.h"

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
void radio_reconfigure(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define EC11_A_PIN              GPIO_PIN_5
#define EC11_A_PORT             GPIOA
#define EC11_B_PIN              GPIO_PIN_6
#define EC11_B_PORT             GPIOA
#define EC11_CONFIRM_PIN        GPIO_PIN_7
#define EC11_CONFIRM_PORT       GPIOA
#define EC11_BACK_PIN           GPIO_PIN_9
#define EC11_BACK_PORT          GPIOA

/* XKC-Y26 Non-Contact Liquid Level Sensors (Optocoupler-Isolated Active-Low Inputs) */
#define XKC_SENSOR1_PIN         GPIO_PIN_3
#define XKC_SENSOR1_PORT        GPIOB
#define XKC_SENSOR2_PIN         GPIO_PIN_4
#define XKC_SENSOR2_PORT        GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
