#include "sx126x_hal.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

/**
 * @brief Wait until the BUSY pin goes low (active high busy state)
 */
static void sx126x_hal_wait_on_busy( void )
{
    while( HAL_GPIO_ReadPin( BUSY_GPIO_Port, BUSY_Pin ) == GPIO_PIN_SET )
    {
        // Block until BUSY is low
    }
}

sx126x_hal_status_t sx126x_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
    sx126x_hal_wait_on_busy();

    // NSS low
    HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET );

    // Transmit command
    if( command_length > 0 && command != NULL )
    {
        if( HAL_SPI_Transmit( &hspi1, ( uint8_t* ) command, command_length, HAL_MAX_DELAY ) != HAL_OK )
        {
            HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET );
            return SX126X_HAL_STATUS_ERROR;
        }
    }

    // Transmit data
    if( data_length > 0 && data != NULL )
    {
        if( HAL_SPI_Transmit( &hspi1, ( uint8_t* ) data, data_length, HAL_MAX_DELAY ) != HAL_OK )
        {
            HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET );
            return SX126X_HAL_STATUS_ERROR;
        }
    }

    // NSS high
    HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET );

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
    sx126x_hal_wait_on_busy();

    // NSS low
    HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET );

    // Transmit command
    if( command_length > 0 && command != NULL )
    {
        if( HAL_SPI_Transmit( &hspi1, ( uint8_t* ) command, command_length, HAL_MAX_DELAY ) != HAL_OK )
        {
            HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET );
            return SX126X_HAL_STATUS_ERROR;
        }
    }

    // Receive data
    if( data_length > 0 && data != NULL )
    {
        if( HAL_SPI_Receive( &hspi1, data, data_length, HAL_MAX_DELAY ) != HAL_OK )
        {
            HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET );
            return SX126X_HAL_STATUS_ERROR;
        }
    }

    // NSS high
    HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET );

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset( const void* context )
{
    // RESET is active low
    HAL_GPIO_WritePin( RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET );
    HAL_Delay( 20 );
    HAL_GPIO_WritePin( RST_GPIO_Port, RST_Pin, GPIO_PIN_SET );
    HAL_Delay( 10 );

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup( const void* context )
{
    // Pull NSS low and transmit a dummy byte to trigger wakeup
    HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET );
    uint8_t dummy = SX126X_NOP;
    HAL_SPI_Transmit( &hspi1, &dummy, 1, HAL_MAX_DELAY );
    HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET );

    // Wait for the BUSY pin to go low, indicating the chip is ready
    sx126x_hal_wait_on_busy();

    return SX126X_HAL_STATUS_OK;
}
