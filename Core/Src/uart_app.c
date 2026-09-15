#include "uart_app.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
static UART_HandleTypeDef *g_huart = &huart1;

void uart_app_init(UART_HandleTypeDef *huart) {
  if (huart != NULL) {
    g_huart = huart;
  }
}

HAL_StatusTypeDef uart_send(const uint8_t *data, uint16_t len) {
  if (g_huart == NULL || data == NULL || len == 0) {
    return HAL_ERROR;
  }
  return HAL_UART_Transmit(g_huart, (uint8_t *)data, len, 100);
}

HAL_StatusTypeDef uart_print(const char *str) {
  if (str == NULL) {
    return HAL_ERROR;
  }
  return uart_send((const uint8_t *)str, (uint16_t)strlen(str));
}

int uart_printf(const char *format, ...) {
  char buffer[128];
  va_list args;
  va_start(args, format);
  int len = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (len > 0) {
    uint16_t to_send = (len < (int)sizeof(buffer)) ? (uint16_t)len : (uint16_t)(sizeof(buffer) - 1);
    uart_send((const uint8_t *)buffer, to_send);
  }
  return len;
}

void uart_print_hex(const char *prefix, const uint8_t *data, uint16_t len) {
  if (prefix != NULL) {
    uart_print(prefix);
  }
  if (data != NULL && len > 0) {
    char hex_chunk[64];
    int chunk_idx = 0;
    for (uint16_t i = 0; i < len; i++) {
      chunk_idx += snprintf(&hex_chunk[chunk_idx], sizeof(hex_chunk) - chunk_idx, "%02X ", data[i]);
      if (chunk_idx >= (int)sizeof(hex_chunk) - 4 || i == len - 1) {
        uart_send((const uint8_t *)hex_chunk, (uint16_t)chunk_idx);
        chunk_idx = 0;
      }
    }
  }
  uart_print("\r\n");
}

int __io_putchar(int ch) {
  uint8_t c = (uint8_t)ch;
  uart_send(&c, 1);
  return ch;
}

