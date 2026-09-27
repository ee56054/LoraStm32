/**
 * @file      sx126x_hal.c
 *
 * @brief     Hardware Abstraction Layer for STM32WL LoRa SoC internal SUBGHZ radio
 *
 * Connects the Semtech SX126x driver directly to the STM32WL SUBGHZSPI peripheral
 * and internal radio control registers (PWR NSS, RFBUSY, and RCC reset).
 */

#include "sx126x_hal.h"
#include "sx126x_hal_board.h"
#include "stm32wlxx_ll_pwr.h"
#include "stm32wlxx_ll_rcc.h"
#include "stm32wlxx_ll_bus.h"
#include "stm32wlxx_ll_exti.h"
#include "stm32wlxx_ll_spi.h"

#define SX126X_DEFAULT_TIMEOUT_CYCLES  0x200000
#define SX126X_WAKEUP_TIMEOUT_MS       5

/* Radio IRQ callback prototype */
extern void sx126x_radio_irq_callback(void);

static sx126x_hal_status_t sx126x_hal_wait_busy(void)
{
    uint32_t timeout = SX126X_DEFAULT_TIMEOUT_CYCLES;
    while (LL_PWR_IsActiveFlag_RFBUSYS() && (--timeout > 0))
    {
        __NOP();
    }
    return (timeout == 0) ? SX126X_HAL_STATUS_ERROR : SX126X_HAL_STATUS_OK;
}

static inline void sx126x_hal_cs_enable(void)
{
    LL_PWR_SelectSUBGHZSPI_NSS();
}

static inline void sx126x_hal_cs_disable(void)
{
    LL_PWR_UnselectSUBGHZSPI_NSS();
}

static uint8_t sx126x_spi_transfer_byte(uint8_t byte)
{
    uint32_t timeout = SX126X_DEFAULT_TIMEOUT_CYCLES;
    while (!LL_SPI_IsActiveFlag_TXE(SUBGHZSPI) && (--timeout > 0))
    {
        __NOP();
    }

    *(__IO uint8_t *)&SUBGHZSPI->DR = byte;

    timeout = SX126X_DEFAULT_TIMEOUT_CYCLES;
    while (!LL_SPI_IsActiveFlag_RXNE(SUBGHZSPI) && (--timeout > 0))
    {
        __NOP();
    }

    return *(__IO uint8_t *)&SUBGHZSPI->DR;
}

sx126x_hal_status_t sx126x_hal_board_init(sx126x_hal_board_t* ctx)
{
    /* 0. Enable SUBGHZSPI bus clock */
    LL_APB3_GRP1_EnableClock(LL_APB3_GRP1_PERIPH_SUBGHZSPI);

    /* 1. Configure Radio reset line */
    LL_RCC_RF_DisableReset();
    uint32_t timeout = 100000;
    while (LL_RCC_IsRFUnderReset() && (--timeout > 0))
    {
        __NOP();
    }

    /* 3. NSS software control unselected */
    LL_PWR_UnselectSUBGHZSPI_NSS();
    LL_PWR_ClearFlag_RFBUSY();

    /* 4. Configure SUBGHZSPI */
    CLEAR_BIT(SUBGHZSPI->CR1, SPI_CR1_SPE);
    WRITE_REG(SUBGHZSPI->CR1, (SPI_CR1_MSTR | SPI_CR1_SSI | LL_SPI_BAUDRATEPRESCALER_DIV4 | SPI_CR1_SSM));
    WRITE_REG(SUBGHZSPI->CR2, (SPI_CR2_FRXTH | SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2));
    SET_BIT(SUBGHZSPI->CR1, SPI_CR1_SPE);

    /* 5. Enable Radio IRQ through EXTI line 44 */
    LL_EXTI_EnableIT_32_63(LL_EXTI_LINE_44);
    HAL_NVIC_SetPriority(SUBGHZ_Radio_IRQn, 0x03, 0);
    HAL_NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);

    if (ctx != NULL)
    {
        ctx->initialized = 1;
    }

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_write(const void* context, const uint8_t* command, const uint16_t command_length,
                                     const uint8_t* data, const uint16_t data_length)
{
    (void)context;
    if ((command == NULL) || (command_length == 0))
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    if (sx126x_hal_wait_busy() != SX126X_HAL_STATUS_OK)
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    sx126x_hal_cs_enable();

    for (uint16_t i = 0; i < command_length; i++)
    {
        sx126x_spi_transfer_byte(command[i]);
    }

    if ((data != NULL) && (data_length > 0))
    {
        for (uint16_t i = 0; i < data_length; i++)
        {
            sx126x_spi_transfer_byte(data[i]);
        }
    }

    sx126x_hal_cs_disable();

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read(const void* context, const uint8_t* command, const uint16_t command_length,
                                    uint8_t* data, const uint16_t data_length)
{
    (void)context;
    if ((command == NULL) || (command_length == 0))
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    if (sx126x_hal_wait_busy() != SX126X_HAL_STATUS_OK)
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    sx126x_hal_cs_enable();

    for (uint16_t i = 0; i < command_length; i++)
    {
        sx126x_spi_transfer_byte(command[i]);
    }

    if ((data != NULL) && (data_length > 0))
    {
        for (uint16_t i = 0; i < data_length; i++)
        {
            data[i] = sx126x_spi_transfer_byte(0x00);
        }
    }

    sx126x_hal_cs_disable();

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset(const void* context)
{
    (void)context;

    LL_RCC_RF_EnableReset();
    HAL_Delay(10);
    LL_RCC_RF_DisableReset();
    HAL_Delay(10);

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void* context)
{
    (void)context;
    const uint8_t dummy_cmd[] = {0xC0, 0x00}; /* GET_STATUS */

    sx126x_hal_cs_enable();
    for (uint16_t i = 0; i < sizeof(dummy_cmd); i++)
    {
        sx126x_spi_transfer_byte(dummy_cmd[i]);
    }
    sx126x_hal_cs_disable();

    HAL_Delay(SX126X_WAKEUP_TIMEOUT_MS);
    return SX126X_HAL_STATUS_OK;
}

/* Weak default RF switch controls - can be overridden by user board file if dedicated GPIOs exist */
__weak void sx126x_hal_rf_switch_tx(void)
{
    /* Default: DIO2 handles RF switch internally if sx126x_set_dio2_as_rf_sw_ctrl() is enabled */
}

__weak void sx126x_hal_rf_switch_rx(void)
{
    /* Default: DIO2 handles RF switch internally if sx126x_set_dio2_as_rf_sw_ctrl() is enabled */
}

__weak void sx126x_hal_rf_switch_off(void)
{
}

__weak void sx126x_radio_irq_callback(void)
{
}

/**
 * @brief SUBGHZ Radio Interrupt Handler (EXTI Line 44)
 */
void SUBGHZ_Radio_IRQHandler(void)
{
    if (LL_EXTI_IsActiveFlag_32_63(LL_EXTI_LINE_44))
    {
        LL_EXTI_ClearFlag_32_63(LL_EXTI_LINE_44);
    }

    sx126x_radio_irq_callback();
}
