#ifndef EC11_H
#define EC11_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// Pin assignments for Samirob EC11 encoder & buttons on STM32WL
// PA5 = EC11 Phase A (CLK)
// PA6 = EC11 Phase B (DT)
// PA7 = Confirm Button (Push button / SW)
// PA9 = Back Button
#ifndef EC11_A_PORT
#define EC11_A_PORT         GPIOA
#define EC11_A_PIN          GPIO_PIN_5
#endif

#ifndef EC11_B_PORT
#define EC11_B_PORT         GPIOA
#define EC11_B_PIN          GPIO_PIN_6
#endif

#ifndef EC11_CONFIRM_PORT
#define EC11_CONFIRM_PORT   GPIOA
#define EC11_CONFIRM_PIN    GPIO_PIN_7
#endif

// Backward compatibility alias for EC11_SW
#ifndef EC11_SW_PORT
#define EC11_SW_PORT        EC11_CONFIRM_PORT
#define EC11_SW_PIN         EC11_CONFIRM_PIN
#endif

#ifndef EC11_BACK_PORT
#define EC11_BACK_PORT      GPIOA
#define EC11_BACK_PIN       GPIO_PIN_9
#endif

typedef enum {
    EC11_EVENT_NONE = 0,
    EC11_EVENT_CW,          // Rotated Clockwise (Next / Increment)
    EC11_EVENT_CCW,         // Rotated Counter-Clockwise (Prev / Decrement)
    EC11_EVENT_CLICK,       // Confirm Button (PA7) Short Press (< 800ms)
    EC11_EVENT_LONG_PRESS,  // Confirm Button (PA7) Long Press (> 800ms)
    EC11_EVENT_BACK         // Back Button (PA9) Press
} ec11_event_t;

#define EC11_EVENT_CONFIRM  EC11_EVENT_CLICK

/**
 * @brief Initialize GPIO pins (inputs with pull-up) for EC11 encoder
 */
void ec11_init(void);

/**
 * @brief Scan the EC11 encoder and return pending events
 * Call periodically (e.g. every 5-10ms in main loop)
 * @return Detected event (EC11_EVENT_NONE if no action)
 */
ec11_event_t ec11_update(void);

#ifdef __cplusplus
}
#endif

#endif // EC11_H

