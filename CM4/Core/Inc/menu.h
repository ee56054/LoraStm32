#ifndef MENU_H
#define MENU_H

#include <stdint.h>
#include <stdbool.h>
#include "ec11.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UI_MODE_DASHBOARD = 0, // Live status overview (Tx, HW, Valves, Sec, RX status)
    UI_MODE_MAIN_MENU,     // Main navigation menu
    UI_MODE_LORA_MENU,     // LoRa Radio settings sub-menu
    UI_MODE_KEY_MENU,      // Security Key sub-menu
    UI_MODE_KEY_VIEW,      // Full Hex Key display screen
    UI_MODE_KEY_EDIT_BYTE, // Byte-by-byte hex editor
    UI_MODE_KEY_PRESET,    // Key preset selector
    UI_MODE_EDIT_MAIN,     // Editing a main menu parameter
    UI_MODE_EDIT_LORA,     // Editing a LoRa parameter
    UI_MODE_INFO           // Node & Radio hardware info
} ui_mode_t;

/**
 * @brief Initialize the menu system and draw default dashboard
 * @param hw_id Device hardware ID
 */
void menu_init(uint32_t hw_id);

/**
 * @brief Process EC11 encoder event and update display
 * @param event Event from ec11_update()
 */
void menu_process(ec11_event_t event);

/**
 * @brief Notify menu system that radio telemetry/status has changed
 * @param status_msg Status message string (e.g. "Listening...", "Modbus Replied")
 * @param tx_count Current transmission counter
 */
void menu_update_status(const char *status_msg, uint32_t tx_count);

/**
 * @brief Get current UI mode
 */
ui_mode_t menu_get_mode(void);

#ifdef __cplusplus
}
#endif

#endif // MENU_H
