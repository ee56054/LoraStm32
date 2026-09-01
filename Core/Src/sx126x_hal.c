/**
 * @file      sx126x_hal.c
 *
 * @brief     Hardware Abstraction Layer for SX126x on STM32F1
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include "sx126x_hal.h"
#include "main.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------
 */

#define SX126X_RESET_DURATION_MS    10
#define SX126X_WAKEUP_DURATION_MS   5

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
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
} sx126x_hal_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS PROTOTYPES -------------------------------------------
 */

static void sx126x_hal_cs_enable( const sx126x_hal_context_t* ctx );
static void sx126x_hal_cs_disable( const sx126x_hal_context_t* ctx );
static sx126x_hal_status_t sx126x_hal_wait_busy( const sx126x_hal_context_t* ctx );

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS IMPLEMENTATION ----------------------------------------
 */

static void sx126x_hal_cs_enable( const sx126x_hal_context_t* ctx )
{
    HAL_GPIO_WritePin( ctx->nss_port, ctx->nss_pin, GPIO_PIN_RESET );
}

static void sx126x_hal_cs_disable( const sx126x_hal_context_t* ctx )
{
    HAL_GPIO_WritePin( ctx->nss_port, ctx->nss_pin, GPIO_PIN_SET );
}

static sx126x_hal_status_t sx126x_hal_wait_busy( const sx126x_hal_context_t* ctx )
{
    uint32_t timeout = 0x100000;
    while( ( HAL_GPIO_ReadPin( ctx->busy_port, ctx->busy_pin ) == GPIO_PIN_SET ) && ( --timeout > 0 ) )
    {
        __NOP();
    }
    /* Return error if timeout occurred */
    return ( timeout == 0 ) ? SX126X_HAL_STATUS_ERROR : SX126X_HAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS IMPLEMENTATION ----------------------------------------
 */

sx126x_hal_status_t sx126x_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
    sx126x_hal_status_t status = SX126X_HAL_STATUS_OK;
    const sx126x_hal_context_t* ctx = ( const sx126x_hal_context_t* ) context;

    if( ( ctx == NULL ) || ( command == NULL ) || ( command_length == 0 ) )
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    if( sx126x_hal_wait_busy( ctx ) != SX126X_HAL_STATUS_OK )
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    sx126x_hal_cs_enable( ctx );

    /* Send command */
    if( HAL_SPI_Transmit( ctx->spi, (uint8_t*) command, command_length, HAL_MAX_DELAY ) != HAL_OK )
    {
        status = SX126X_HAL_STATUS_ERROR;
    }

    /* Send data if provided */
    if( ( status == SX126X_HAL_STATUS_OK ) && ( data != NULL ) && ( data_length > 0 ) )
    {
        if( HAL_SPI_Transmit( ctx->spi, (uint8_t*) data, data_length, HAL_MAX_DELAY ) != HAL_OK )
        {
            status = SX126X_HAL_STATUS_ERROR;
        }
    }

    sx126x_hal_cs_disable( ctx );

    return status;
}

sx126x_hal_status_t sx126x_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
    sx126x_hal_status_t status = SX126X_HAL_STATUS_OK;
    const sx126x_hal_context_t* ctx = ( const sx126x_hal_context_t* ) context;

    if( ( ctx == NULL ) || ( command == NULL ) || ( command_length == 0 ) )
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    if( sx126x_hal_wait_busy( ctx ) != SX126X_HAL_STATUS_OK )
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    sx126x_hal_cs_enable( ctx );

    /* Send command */
    if( HAL_SPI_Transmit( ctx->spi, (uint8_t*) command, command_length, HAL_MAX_DELAY ) != HAL_OK )
    {
        status = SX126X_HAL_STATUS_ERROR;
    }

    /* Receive data if provided */
    if( ( status == SX126X_HAL_STATUS_OK ) && ( data != NULL ) && ( data_length > 0 ) )
    {
        if( HAL_SPI_Receive( ctx->spi, data, data_length, HAL_MAX_DELAY ) != HAL_OK )
        {
            status = SX126X_HAL_STATUS_ERROR;
        }
    }

    sx126x_hal_cs_disable( ctx );

    return status;
}

sx126x_hal_status_t sx126x_hal_reset( const void* context )
{
    const sx126x_hal_context_t* ctx = ( const sx126x_hal_context_t* ) context;

    if( ctx == NULL )
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    /* Assert reset line */
    HAL_GPIO_WritePin( ctx->rst_port, ctx->rst_pin, GPIO_PIN_RESET );
    HAL_Delay( SX126X_RESET_DURATION_MS );

    /* Release reset line */
    HAL_GPIO_WritePin( ctx->rst_port, ctx->rst_pin, GPIO_PIN_SET );
    HAL_Delay( SX126X_RESET_DURATION_MS );

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup( const void* context )
{
    sx126x_hal_status_t status = SX126X_HAL_STATUS_OK;
    const sx126x_hal_context_t* ctx = ( const sx126x_hal_context_t* ) context;
    uint8_t dummy_command[] = {0xC0, 0x00};  /* GET_STATUS command */

    if( ctx == NULL )
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    /* Perform a GET_STATUS SPI transaction to wake up the radio */
    sx126x_hal_cs_enable( ctx );

    if( HAL_SPI_Transmit( ctx->spi, dummy_command, 2, HAL_MAX_DELAY ) != HAL_OK )
    {
        status = SX126X_HAL_STATUS_ERROR;
    }

    sx126x_hal_cs_disable( ctx );
    HAL_Delay( SX126X_WAKEUP_DURATION_MS );

    return status;
}

/* --- EOF ------------------------------------------------------------------ */
