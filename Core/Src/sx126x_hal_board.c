/**
 * @file      sx126x_hal_board.c
 *
 * @brief     Board-specific SX126x HAL initialization for STM32F103C8T6
 *
 * The Clear BSD License
 * Copyright 2026. All rights reserved.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "sx126x_hal_board.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS IMPLEMENTATION ----------------------------------------
 */

sx126x_hal_status_t sx126x_hal_board_init( sx126x_hal_board_t* ctx, SPI_HandleTypeDef* hspi )
{
    if( ( ctx == NULL ) || ( hspi == NULL ) )
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    ctx->spi      = hspi;
    ctx->nss_port = SX126X_NSS_PORT;
    ctx->nss_pin  = SX126X_NSS_PIN;
    ctx->rst_port = SX126X_RST_PORT;
    ctx->rst_pin  = SX126X_RST_PIN;
    ctx->dio1_port = SX126X_DIO1_PORT;
    ctx->dio1_pin  = SX126X_DIO1_PIN;
    ctx->busy_port = SX126X_BUSY_PORT;
    ctx->busy_pin  = SX126X_BUSY_PIN;

    /* Validate GPIO ports */
    if( ( ctx->nss_port == NULL ) || ( ctx->rst_port == NULL ) || ( ctx->dio1_port == NULL ) || ( ctx->busy_port == NULL ) )
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    /* Set NSS high (inactive state) */
    HAL_GPIO_WritePin( ctx->nss_port, ctx->nss_pin, GPIO_PIN_SET );

    /* Set RST high (inactive state) */
    HAL_GPIO_WritePin( ctx->rst_port, ctx->rst_pin, GPIO_PIN_SET );

    return SX126X_HAL_STATUS_OK;
}

/* --- EOF ------------------------------------------------------------------ */
