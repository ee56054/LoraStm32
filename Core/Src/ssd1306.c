#include "ssd1306.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

static I2C_HandleTypeDef *g_hi2c = NULL;
static uint8_t g_i2c_addr = SSD1306_I2C_ADDR;

// Screen buffer: 128x64 pixels = 1024 bytes
static uint8_t SSD1306_Buffer[SSD1306_BUFFER_SIZE];
static SSD1306_t SSD1306;

static void ssd1306_write_command(uint8_t cmd) {
    if (g_hi2c == NULL) return;
    uint8_t payload[2] = {0x00, cmd};
    HAL_I2C_Master_Transmit(g_hi2c, g_i2c_addr, payload, 2, 10);
}

uint8_t ssd1306_init(I2C_HandleTypeDef *hi2c) {
    if (hi2c == NULL) {
        return 1;
    }
    g_hi2c = hi2c;

    // Detect I2C address (0x78 or 0x7A)
    if (HAL_I2C_IsDeviceReady(g_hi2c, SSD1306_I2C_ADDR, 2, 10) == HAL_OK) {
        g_i2c_addr = SSD1306_I2C_ADDR;
    } else if (HAL_I2C_IsDeviceReady(g_hi2c, 0x7A, 2, 10) == HAL_OK) {
        g_i2c_addr = 0x7A;
    } else {
        // Device not responding on either address
        return 2;
    }

    HAL_Delay(10); // Wait for screen to boot up

    // Initialization sequence
    ssd1306_write_command(0xAE); // Display OFF
    ssd1306_write_command(0x20); // Set Memory Addressing Mode
    ssd1306_write_command(0x00); // 00b: Horizontal Addressing Mode
    ssd1306_write_command(0xB0); // Set Page Start Address for Page Addressing Mode
    ssd1306_write_command(0xC8); // Set COM Output Scan Direction (remapped)
    ssd1306_write_command(0x00); // Set low column address
    ssd1306_write_command(0x10); // Set high column address
    ssd1306_write_command(0x40); // Set start line address
    ssd1306_write_command(0x81); // Set contrast control register
    ssd1306_write_command(0xCF);
    ssd1306_write_command(0xA1); // Set segment re-map 0 to 127
    ssd1306_write_command(0xA6); // Set normal display
    ssd1306_write_command(0xA8); // Set multiplex ratio(1 to 64)
    ssd1306_write_command(0x3F); // 64MUX
    ssd1306_write_command(0xA4); // 0xA4: Output follows RAM content; 0xA5: Output ignores RAM content
    ssd1306_write_command(0xD3); // Set display offset
    ssd1306_write_command(0x00); // No offset
    ssd1306_write_command(0xD5); // Set display clock divide ratio/oscillator frequency
    ssd1306_write_command(0x80); // Set divide ratio
    ssd1306_write_command(0xD9); // Set pre-charge period
    ssd1306_write_command(0xF1);
    ssd1306_write_command(0xDA); // Set com pins hardware configuration
    ssd1306_write_command(0x12);
    ssd1306_write_command(0xDB); // Set vcomh
    ssd1306_write_command(0x40);
    ssd1306_write_command(0x8D); // Set DC-DC enable
    ssd1306_write_command(0x14); // Enable Charge Pump
    ssd1306_write_command(0xAF); // Display ON

    // Clear screen
    ssd1306_fill(SSD1306_COLOR_BLACK);
    ssd1306_update_screen();

    SSD1306.CurrentX = 0;
    SSD1306.CurrentY = 0;
    SSD1306.Initialized = 1;

    return 0;
}

void ssd1306_fill(SSD1306_COLOR_t color) {
    memset(SSD1306_Buffer, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer));
}

void ssd1306_update_screen(void) {
    if (g_hi2c == NULL) return;

    for (uint8_t m = 0; m < 8; m++) {
        ssd1306_write_command(0xB0 + m);
        ssd1306_write_command(0x00);
        ssd1306_write_command(0x10);

        HAL_I2C_Mem_Write(g_hi2c, g_i2c_addr, 0x40, 1,
                          &SSD1306_Buffer[SSD1306_WIDTH * m], SSD1306_WIDTH, 25);
    }
}

void ssd1306_draw_pixel(uint8_t x, uint8_t y, SSD1306_COLOR_t color) {
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return;
    }

    if (color == SSD1306_COLOR_WHITE) {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y % 8));
    } else {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

void ssd1306_set_cursor(uint8_t x, uint8_t y) {
    SSD1306.CurrentX = x;
    SSD1306.CurrentY = y;
}

char ssd1306_write_char(char ch, SSD1306_Font_t Font, SSD1306_COLOR_t color) {
    if (ch < 32 || ch > 126) {
        return 0;
    }

    // Check remaining horizontal space
    if (SSD1306_WIDTH < (SSD1306.CurrentX + Font.width) ||
        SSD1306_HEIGHT < (SSD1306.CurrentY + Font.height)) {
        return 0;
    }

    for (uint32_t i = 0; i < Font.height; i++) {
        uint16_t b = Font.data[(ch - 32) * Font.height + i];
        for (uint32_t j = 0; j < Font.width; j++) {
            if ((b << j) & 0x8000) {
                ssd1306_draw_pixel(SSD1306.CurrentX + j, SSD1306.CurrentY + i, color);
            } else {
                ssd1306_draw_pixel(SSD1306.CurrentX + j, SSD1306.CurrentY + i,
                                   (color == SSD1306_COLOR_WHITE) ? SSD1306_COLOR_BLACK : SSD1306_COLOR_WHITE);
            }
        }
    }

    SSD1306.CurrentX += Font.width;
    return ch;
}

char ssd1306_write_string(const char *str, SSD1306_Font_t Font, SSD1306_COLOR_t color) {
    if (str == NULL) return 0;
    while (*str) {
        if (ssd1306_write_char(*str, Font, color) != *str) {
            return *str;
        }
        str++;
    }
    return *str;
}

void ssd1306_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, SSD1306_COLOR_t color) {
    int16_t dx = abs((int16_t)x1 - (int16_t)x0);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t dy = -abs((int16_t)y1 - (int16_t)y0);
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx + dy;

    while (1) {
        ssd1306_draw_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int16_t e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void ssd1306_draw_rectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, SSD1306_COLOR_t color) {
    ssd1306_draw_line(x, y, x + w - 1, y, color);
    ssd1306_draw_line(x, y + h - 1, x + w - 1, y + h - 1, color);
    ssd1306_draw_line(x, y, x, y + h - 1, color);
    ssd1306_draw_line(x + w - 1, y, x + w - 1, y + h - 1, color);
}

void ssd1306_draw_filled_rectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, SSD1306_COLOR_t color) {
    for (uint8_t i = 0; i < h; i++) {
        ssd1306_draw_line(x, y + i, x + w - 1, y + i, color);
    }
}

int ssd1306_printf(SSD1306_Font_t Font, SSD1306_COLOR_t color, const char *format, ...) {
    char buffer[64];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len > 0) {
        ssd1306_write_string(buffer, Font, color);
    }
    return len;
}

