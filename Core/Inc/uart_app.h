#ifndef UART_APP_H
#define UART_APP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize Serial / USB communication module
 */
void uart_app_init(void);

/**
 * @brief Transmit raw buffer over UART
 * @param data Pointer to data bytes
 * @param len Number of bytes to send
 * @return HAL_StatusTypeDef result of transmission
 */
HAL_StatusTypeDef uart_send(const uint8_t *data, uint16_t len);

/**
 * @brief Print null-terminated string over UART
 * @param str Null-terminated string
 * @return HAL_StatusTypeDef result of transmission
 */
HAL_StatusTypeDef uart_print(const char *str);

/**
 * @brief Formatted print over UART (similar to printf)
 * @param format Format string
 * @param ... Arguments
 * @return Number of characters printed
 */
int uart_printf(const char *format, ...);

/**
 * @brief Print a byte buffer in hexadecimal format with optional prefix and trailing newline
 * @param prefix Optional prefix string (e.g. "RX HEX: " or "MODBUS TX: "), can be NULL
 * @param data Pointer to data buffer
 * @param len Number of bytes to print
 */
void uart_print_hex(const char *prefix, const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* UART_APP_H */

