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
static volatile int8_t s_pending_steps = 0;
static volatile bool s_pending_confirm = false;
static volatile bool s_pending_back = false;
static bool s_initialized = false;

static uint8_t s_confirm_debounce_cnt = 0;
static bool s_confirm_pressed = false;

static uint8_t s_back_debounce_cnt = 0;
static bool s_back_pressed = false;

static volatile uint32_t s_last_isr_tick = 0;

#define BTN_DEBOUNCE_MS 20

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

    // Allow internal pull-ups to charge any external RC debounce capacitors
    HAL_Delay(10);

    // Initial state read
    uint8_t a = (HAL_GPIO_ReadPin(EC11_A_PORT, EC11_A_PIN) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t b = (HAL_GPIO_ReadPin(EC11_B_PORT, EC11_B_PIN) == GPIO_PIN_SET) ? 1 : 0;
    s_last_state = (a << 1) | b;
    s_sub_count = 0;
    s_pending_steps = 0;
    s_pending_confirm = false;
    s_pending_back = false;

    s_confirm_debounce_cnt = 0;
    s_confirm_pressed = (HAL_GPIO_ReadPin(EC11_CONFIRM_PORT, EC11_CONFIRM_PIN) == GPIO_PIN_RESET);

    s_back_debounce_cnt = 0;
    s_back_pressed = (HAL_GPIO_ReadPin(EC11_BACK_PORT, EC11_BACK_PIN) == GPIO_PIN_RESET);

    s_last_isr_tick = HAL_GetTick();
    s_initialized = true;
}

void ec11_tick_1ms(void) {
    if (!s_initialized) {
        return;
    }

    // 1. Sample quadrature encoder inputs (PA5, PA6)
    uint8_t a = (HAL_GPIO_ReadPin(EC11_A_PORT, EC11_A_PIN) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t b = (HAL_GPIO_ReadPin(EC11_B_PORT, EC11_B_PIN) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t current_state = (a << 1) | b;

    if (current_state != s_last_state) {
        uint8_t idx = (s_last_state << 2) | current_state;
        int8_t step = quad_table[idx];
        s_last_state = current_state;

        if (step != 0) {
            s_sub_count += step;

            // Full-cycle EC11 encoders rest at 0x03 (both A & B pulled HIGH at detent)
            // and complete 4 quadrature transitions (11->10->00->01->11) per click.
            // Emit strictly 1 step event per physical detent when returning to 0x03.
            if (current_state == 0x03) {
                if (s_sub_count >= 2) {
                    if (s_pending_steps < 16) s_pending_steps++;
                } else if (s_sub_count <= -2) {
                    if (s_pending_steps > -16) s_pending_steps--;
                }
                s_sub_count = 0;
            } else if (s_sub_count >= 4) {
                if (s_pending_steps < 16) s_pending_steps++;
                s_sub_count = 0;
            } else if (s_sub_count <= -4) {
                if (s_pending_steps > -16) s_pending_steps--;
                s_sub_count = 0;
            }
        }
    }

    s_last_isr_tick = HAL_GetTick();

    // 2. Debounce Confirm Push-Button (PA7, Active LOW with pull-up)
    bool confirm_down = (HAL_GPIO_ReadPin(EC11_CONFIRM_PORT, EC11_CONFIRM_PIN) == GPIO_PIN_RESET);
    if (confirm_down != s_confirm_pressed) {
        s_confirm_debounce_cnt++;
        if (s_confirm_debounce_cnt >= BTN_DEBOUNCE_MS) {
            s_confirm_pressed = confirm_down;
            s_confirm_debounce_cnt = 0;
            if (s_confirm_pressed) {
                // Trigger immediately on debounced press-down edge
                s_pending_confirm = true;
            }
        }
    } else {
        s_confirm_debounce_cnt = 0;
    }

    // 3. Debounce Back Push-Button (PA9, Active LOW with pull-up)
    bool back_down = (HAL_GPIO_ReadPin(EC11_BACK_PORT, EC11_BACK_PIN) == GPIO_PIN_RESET);
    if (back_down != s_back_pressed) {
        s_back_debounce_cnt++;
        if (s_back_debounce_cnt >= BTN_DEBOUNCE_MS) {
            s_back_pressed = back_down;
            s_back_debounce_cnt = 0;
            if (s_back_pressed) {
                // Trigger immediately on debounced press-down edge
                s_pending_back = true;
            }
        }
    } else {
        s_back_debounce_cnt = 0;
    }
}

ec11_event_t ec11_update(void) {
    // Fallback if SysTick_Handler has not invoked ec11_tick_1ms() in this millisecond
    if (HAL_GetTick() != s_last_isr_tick) {
        ec11_tick_1ms();
    }

    ec11_event_t event = EC11_EVENT_NONE;

    __disable_irq();
    if (s_pending_confirm) {
        s_pending_confirm = false;
        s_pending_steps = 0;
        event = EC11_EVENT_CLICK;
    } else if (s_pending_back) {
        s_pending_back = false;
        s_pending_steps = 0;
        event = EC11_EVENT_BACK;
    } else if (s_pending_steps > 0) {
        s_pending_steps--;
        event = EC11_EVENT_CW;
    } else if (s_pending_steps < 0) {
        s_pending_steps++;
        event = EC11_EVENT_CCW;
    }
    __enable_irq();

    return event;
}

