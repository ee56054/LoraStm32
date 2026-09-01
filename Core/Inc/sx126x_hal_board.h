/**
 * @file      sx126x_hal_board.h
 *
 * @brief     Board-specific SX126x HAL initialization
 *
 * The Clear BSD License
 * Copyright 2026. All rights reserved.
 */

#ifndef SX126X_HAL_BOARD_H
#define SX126X_HAL_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "sx126x_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef struct
{
    SPI_HandleTypeDef* spi;
    GPIO_TypeDef*      nss_port;
    uint16_t           nss_pin;
    GPIO_TypeDef*      rst_port;
    uint16_t           rst_pin;
    GPIO_TypeDef*      dio1_port;
    uint16_t           dio1_pin;
    GPIO_TypeDef*      busy_port;
    uint16_t           busy_pin;
} sx126x_hal_board_t;

/*
 * Pin definitions for STM32F103C8T6
 */
#define SX126X_NSS_PORT      GPIOA
#define SX126X_NSS_PIN       GPIO_PIN_4

#define SX126X_RST_PORT      GPIOA
#define SX126X_RST_PIN       GPIO_PIN_3

#define SX126X_DIO1_PORT     GPIOC
#define SX126X_DIO1_PIN      GPIO_PIN_15

#define SX126X_BUSY_PORT     GPIOA
#define SX126X_BUSY_PIN      GPIO_PIN_2

/*
 * SPI interface
 */
#define SX126X_SPI           hspi1

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Initialize SX126x HAL board context
 *
 * @param [out] ctx      HAL board context structure to initialize
 * @param [in]  hspi     SPI handle
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_board_init( sx126x_hal_board_t* ctx, SPI_HandleTypeDef* hspi );

#ifdef __cplusplus
}
#endif

#endif  // SX126X_HAL_BOARD_H

/* --- EOF ------------------------------------------------------------------ */
