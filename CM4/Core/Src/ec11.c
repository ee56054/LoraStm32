#include "ec11.h"

// State transition table for Gray-code quadrature decoding
// Valid transition table: returns -1 (CCW), +1 (CW), or 0 (invalid/noise)
static const int8_t quad_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static uint8_t s_last_state = 0x03;
static int8_t s_sub_count = 0;
static uint32_t s_btn_press_tick = 0;
static bool s_btn_pressed = false;
static bool s_long_press_reported = false;

static uint32_t s_back_press_tick = 0;
static bool s_back_pressed = false;

void ec11_init(void) {
    if (EC11_A_PORT == GPIOA || EC11_B_PORT == GPIOA || EC11_CONFIRM_PORT == GPIOA || EC11_BACK_PORT == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    if (EC11_A_PORT == GPIOB || EC11_B_PORT == GPIOB || EC11_CONFIRM_PORT == GPIOB || EC11_BACK_PORT == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    if (EC11_A_PORT == GPIOC || EC11_B_PORT == GPIOC || EC11_CONFIRM_PORT == GPIOC || EC11_BACK_PORT == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = EC11_A_PIN;
    HAL_GPIO_Init(EC11_A_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = EC11_B_PIN;
    HAL_GPIO_Init(EC11_B_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = EC11_CONFIRM_PIN;
    HAL_GPIO_Init(EC11_CONFIRM_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = EC11_BACK_PIN;
    HAL_GPIO_Init(EC11_BACK_PORT, &GPIO_InitStruct);

    // Initial state read
    uint8_t a = (HAL_GPIO_ReadPin(EC11_A_PORT, EC11_A_PIN) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t b = (HAL_GPIO_ReadPin(EC11_B_PORT, EC11_B_PIN) == GPIO_PIN_SET) ? 1 : 0;
    s_last_state = (a << 1) | b;
    s_sub_count = 0;
}

ec11_event_t ec11_update(void) {
    ec11_event_t event = EC11_EVENT_NONE;
    uint32_t now = HAL_GetTick();

    // 1. Read quadrature inputs
    uint8_t a = (HAL_GPIO_ReadPin(EC11_A_PORT, EC11_A_PIN) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t b = (HAL_GPIO_ReadPin(EC11_B_PORT, EC11_B_PIN) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t current_state = (a << 1) | b;

    if (current_state != s_last_state) {
        uint8_t idx = (s_last_state << 2) | current_state;
        int8_t step = quad_table[idx];
        s_last_state = current_state;

        if (step != 0) {
            s_sub_count += step;
            // Standard EC11 encoders give 2 state transitions per detent click
            if (s_sub_count >= 2) {
                event = EC11_EVENT_CW;
                s_sub_count = 0;
            } else if (s_sub_count <= -2) {
                event = EC11_EVENT_CCW;
                s_sub_count = 0;
            }
        }
    }

    // 2. Read Confirm Push-Button (PA7, Active LOW with pullup)
    bool btn_down = (HAL_GPIO_ReadPin(EC11_CONFIRM_PORT, EC11_CONFIRM_PIN) == GPIO_PIN_RESET);

    if (btn_down) {
        if (!s_btn_pressed) {
            // Button pressed down
            s_btn_pressed = true;
            s_btn_press_tick = now;
            s_long_press_reported = false;
        } else {
            // Button held down
            if (!s_long_press_reported && (now - s_btn_press_tick >= 800)) {
                s_long_press_reported = true;
                if (event == EC11_EVENT_NONE) {
                    event = EC11_EVENT_LONG_PRESS;
                }
            }
        }
    } else {
        if (s_btn_pressed) {
            // Button released
            uint32_t duration = now - s_btn_press_tick;
            s_btn_pressed = false;

            if (!s_long_press_reported && duration >= 30 && duration < 800) {
                if (event == EC11_EVENT_NONE) {
                    event = EC11_EVENT_CLICK;
                }
            }
        }
    }

    // 3. Read Back Push-Button (PA9, Active LOW with pullup)
    bool back_down = (HAL_GPIO_ReadPin(EC11_BACK_PORT, EC11_BACK_PIN) == GPIO_PIN_RESET);

    if (back_down) {
        if (!s_back_pressed) {
            s_back_pressed = true;
            s_back_press_tick = now;
        }
    } else {
        if (s_back_pressed) {
            uint32_t back_duration = now - s_back_press_tick;
            s_back_pressed = false;

            if (back_duration >= 30) {
                if (event == EC11_EVENT_NONE) {
                    event = EC11_EVENT_BACK;
                }
            }
        }
    }

    return event;
}

