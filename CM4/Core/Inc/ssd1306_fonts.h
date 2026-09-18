#ifndef SSD1306_FONTS_H
#define SSD1306_FONTS_H

#include <stdint.h>
#include <stddef.h>

#ifndef SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_7x10
#endif

#ifndef SSD1306_INCLUDE_FONT_11x18
#define SSD1306_INCLUDE_FONT_11x18
#endif

typedef struct {
    const uint8_t width;
    const uint8_t height;
    const uint16_t *data;
    const uint8_t *char_width;
} SSD1306_Font_t;

#ifdef SSD1306_INCLUDE_FONT_7x10
extern const SSD1306_Font_t Font_7x10;
#endif

#ifdef SSD1306_INCLUDE_FONT_11x18
extern const SSD1306_Font_t Font_11x18;
#endif

#endif /* SSD1306_FONTS_H */

