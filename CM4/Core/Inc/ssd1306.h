#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "main.h"
#include "ssd1306_fonts.h"

#ifdef __cplusplus
extern "C" {
#endif

// I2C address (7-bit 0x3C shifted left by 1 for STM32 HAL -> 0x78)
#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR        0x78
#endif

#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH           128
#endif

#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT          64
#endif

#define SSD1306_BUFFER_SIZE     ((SSD1306_WIDTH * SSD1306_HEIGHT) / 8)

typedef enum {
    SSD1306_COLOR_BLACK = 0x00, // Black, pixel OFF
    SSD1306_COLOR_WHITE = 0x01  // White, pixel ON
} SSD1306_COLOR_t;

typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Initialized;
} SSD1306_t;

/**
 * @brief Initialize SSD1306 OLED display using specified I2C handle
 * @param hi2c Pointer to I2C_HandleTypeDef (e.g. &hi2c2)
 * @return 0 on success, non-zero on failure
 */
uint8_t ssd1306_init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Fill entire screen buffer with specified color
 * @param color SSD1306_COLOR_BLACK or SSD1306_COLOR_WHITE
 */
void ssd1306_fill(SSD1306_COLOR_t color);

/**
 * @brief Flush display buffer to SSD1306 over I2C
 */
void ssd1306_update_screen(void);

/**
 * @brief Draw a single pixel in the screen buffer
 * @param x X coordinate (0..127)
 * @param y Y coordinate (0..63)
 * @param color Pixel color
 */
void ssd1306_draw_pixel(uint8_t x, uint8_t y, SSD1306_COLOR_t color);

/**
 * @brief Set text cursor position
 * @param x X coordinate (0..127)
 * @param y Y coordinate (0..63)
 */
void ssd1306_set_cursor(uint8_t x, uint8_t y);

/**
 * @brief Write a single character at current cursor position
 * @param ch Character to write
 * @param Font Font descriptor (e.g. Font_7x10, Font_11x18)
 * @param color Text color
 * @return Character written, or 0 on error
 */
char ssd1306_write_char(char ch, SSD1306_Font_t Font, SSD1306_COLOR_t color);

/**
 * @brief Write a string at current cursor position
 * @param str String to write
 * @param Font Font descriptor
 * @param color Text color
 * @return Character written, or 0 on error
 */
char ssd1306_write_string(const char *str, SSD1306_Font_t Font, SSD1306_COLOR_t color);

/**
 * @brief Draw a line using Bresenham algorithm
 */
void ssd1306_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, SSD1306_COLOR_t color);

/**
 * @brief Draw a rectangle outline
 */
void ssd1306_draw_rectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, SSD1306_COLOR_t color);

/**
 * @brief Draw a filled rectangle
 */
void ssd1306_draw_filled_rectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, SSD1306_COLOR_t color);

/**
 * @brief Formatted print at current cursor position
 */
int ssd1306_printf(SSD1306_Font_t Font, SSD1306_COLOR_t color, const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* SSD1306_H */

