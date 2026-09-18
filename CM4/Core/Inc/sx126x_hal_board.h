/**
 * @file      sx126x_hal_board.h
 *
 * @brief     STM32WL LoRa SoC internal SUBGHZ Hardware Abstraction Layer
 *
 * Replaces external SPI LoRa module with internal STM32WL SUBGHZ radio IP.
 */

#ifndef SX126X_HAL_BOARD_H
#define SX126X_HAL_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32wlxx_hal.h"
#include "sx126x_hal.h"

typedef struct
{
    uint8_t initialized;
} sx126x_hal_board_t;

/**
 * @brief Initialize STM32WL SUBGHZ internal SPI & radio interface
 */
sx126x_hal_status_t sx126x_hal_board_init(sx126x_hal_board_t* ctx);

/**
 * @brief Enable RF switch for TX mode
 */
void sx126x_hal_rf_switch_tx(void);

/**
 * @brief Enable RF switch for RX mode
 */
void sx126x_hal_rf_switch_rx(void);

/**
 * @brief Disable RF switch
 */
void sx126x_hal_rf_switch_off(void);

/**
 * @brief Process radio IRQ (called from SUBGHZ_Radio_IRQHandler)
 */
void sx126x_radio_irq_process(void);

#ifdef __cplusplus
}
#endif

#endif  // SX126X_HAL_BOARD_H

