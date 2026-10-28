#include "menu.h"
#include "ssd1306.h"
#include "config.h"
#include "modbus_app.h"
#include "lora_security.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#define MENU_TIMEOUT_MS         12000

// Main Menu Items (Exiting to Dashboard is handled strictly by PA9 Back button)
typedef enum {
    MAIN_ITEM_VALVE1 = 0,
    MAIN_ITEM_VALVE2,
    MAIN_ITEM_SECURITY,
    MAIN_ITEM_SEC_KEY,
    MAIN_ITEM_SLAVE_ID,
    MAIN_ITEM_LORA_CONFIG,
    MAIN_ITEM_SCREEN_TYPE,
    MAIN_ITEM_NODE_INFO,
    MAIN_ITEM_COUNT
} main_item_id_t;

// LoRa Sub-Menu Items (Returning to Main Menu is handled strictly by PA9 Back button)
typedef enum {
    LORA_ITEM_FREQ = 0,
    LORA_ITEM_SF,
    LORA_ITEM_POWER,
    LORA_ITEM_BW,
    LORA_ITEM_CR,
    LORA_ITEM_COUNT
} lora_item_id_t;

// Security Key Sub-Menu Items (Returning to Main Menu is handled strictly by PA9 Back button)
typedef enum {
    KEY_ITEM_VIEW = 0,
    KEY_ITEM_PRESET,
    KEY_ITEM_EDIT_BYTE,
    KEY_ITEM_GEN_RANDOM,
    KEY_ITEM_RESET_DEFAULT,
    KEY_ITEM_COUNT
} key_menu_item_id_t;

// LoRa Parameter Presets
static const uint32_t s_freq_options[] = {
    433000000,
    868000000,
    915000000,
    920000000,
    923000000,
    925000000
};
#define FREQ_OPT_COUNT 6
static const char *s_freq_names[] = {
    "433.0 MHz",
    "868.0 MHz",
    "915.0 MHz",
    "920.0 MHz",
    "923.0 MHz",
    "925.0 MHz"
};

static void format_freq_short(char *buf, size_t max_len, uint32_t freq) {
    uint32_t mhz = freq / 1000000;
    uint32_t dec = (freq % 1000000) / 100000;
    snprintf(buf, max_len, "%lu.%luM", (unsigned long)mhz, (unsigned long)dec);
}

static const sx126x_lora_sf_t s_sf_options[] = {
    SX126X_LORA_SF7,
    SX126X_LORA_SF8,
    SX126X_LORA_SF9,
    SX126X_LORA_SF10,
    SX126X_LORA_SF11,
    SX126X_LORA_SF12
};
#define SF_OPT_COUNT 6
static const char *s_sf_names[] = {"SF7", "SF8", "SF9", "SF10", "SF11", "SF12"};

static const int8_t s_power_options[] = { 10, 14, 17, 20, 22 };
#define POWER_OPT_COUNT 5

static const sx126x_lora_bw_t s_bw_options[] = {
    SX126X_LORA_BW_125,
    SX126X_LORA_BW_250,
    SX126X_LORA_BW_500
};
#define BW_OPT_COUNT 3
static const char *s_bw_names[] = {"125 kHz", "250 kHz", "500 kHz"};

static const sx126x_lora_cr_t s_cr_options[] = {
    SX126X_LORA_CR_4_5,
    SX126X_LORA_CR_4_6,
    SX126X_LORA_CR_4_7,
    SX126X_LORA_CR_4_8
};
#define CR_OPT_COUNT 4
static const char *s_cr_names[] = {"4/5", "4/6", "4/7", "4/8"};

// AES Key Presets
#define KEY_PRESET_COUNT 4
static const char *s_key_preset_names[KEY_PRESET_COUNT] = {
    "1:Default FIPS",
    "2:Test (00..0F)",
    "3:Test (A5..5A)",
    "4:Test (All 01)"
};
static const uint8_t s_key_preset_values[KEY_PRESET_COUNT][16] = {
    { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
      0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C },
    { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
      0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F },
    { 0xA5, 0x5A, 0xA5, 0x5A, 0xA5, 0x5A, 0xA5, 0x5A,
      0xA5, 0x5A, 0xA5, 0x5A, 0xA5, 0x5A, 0xA5, 0x5A },
    { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
      0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 }
};

// State variables
static ui_mode_t s_mode = UI_MODE_DASHBOARD;
static uint32_t s_hw_id = 0;
static uint32_t s_tx_count = 0;
static char s_status_msg[32] = "Listening...";
static uint32_t s_last_action_tick = 0;

static int8_t s_cursor_idx = 0;
static int8_t s_scroll_top = 0;
static bool s_is_1_3_inch = false;

#define DASHBOARD_ITEM_COUNT 9
static int8_t s_dash_scroll_top = 0;

#define INFO_ITEM_COUNT 10
static int8_t s_info_scroll_top = 0;

// Edit state variables
static bool s_edit_valve1 = false;
static bool s_edit_valve2 = false;
static uint8_t s_edit_security = 0;
static uint8_t s_edit_slave_id = 1;
static int8_t s_edit_lora_opt_idx = 0;

static int8_t s_edit_key_preset_idx = 0;
static int8_t s_edit_key_byte_idx = 0;
static uint8_t s_temp_key[16];

// Function prototypes
static void render_dashboard(void);
static void render_main_menu(void);
static void render_lora_menu(void);
static void render_key_menu(void);
static void render_key_view(void);
static void render_key_preset(void);
static void render_key_edit_byte(void);
static void render_edit_main(void);
static void render_edit_lora(void);
static void render_info(void);

static const char *get_sf_name(sx126x_lora_sf_t sf);
static const char *get_bw_name(sx126x_lora_bw_t bw);
static const char *get_cr_name(sx126x_lora_cr_t cr);
static int find_freq_idx(uint32_t freq);
static int find_sf_idx(sx126x_lora_sf_t sf);
static int find_power_idx(int8_t power);
static int find_bw_idx(sx126x_lora_bw_t bw);
static int find_cr_idx(sx126x_lora_cr_t cr);

void menu_init(uint32_t hw_id) {
    s_hw_id = hw_id;
    s_tx_count = 0;
    s_mode = UI_MODE_DASHBOARD;
    s_dash_scroll_top = 0;
    s_info_scroll_top = 0;
    s_last_action_tick = HAL_GetTick();
    render_dashboard();
}

void menu_update_status(const char *status_msg, uint32_t tx_count) {
    s_tx_count = tx_count;
    if (status_msg != NULL) {
        strncpy(s_status_msg, status_msg, sizeof(s_status_msg) - 1);
        s_status_msg[sizeof(s_status_msg) - 1] = '\0';
    }

    if (s_mode == UI_MODE_DASHBOARD) {
        render_dashboard();
    }
}

ui_mode_t menu_get_mode(void) {
    return s_mode;
}

static void get_dashboard_line(int idx, char *out, size_t out_sz) {
    switch (idx) {
        case 0:
            snprintf(out, out_sz, "Stat:  %.11s", s_status_msg);
            break;
        case 1:
            snprintf(out, out_sz, "Slave: ID %u", (unsigned int)g_lora_config.slave_id);
            break;
        case 2:
            snprintf(out, out_sz, "Tx:    %lu pkts", (unsigned long)s_tx_count);
            break;
        case 3: {
            bool v1 = modbus_app_get_valve_state(0);
            snprintf(out, out_sz, "Valve1: %s", v1 ? "OPEN" : "CLOSED");
            break;
        }
        case 4: {
            bool v2 = modbus_app_get_valve_state(1);
            snprintf(out, out_sz, "Valve2: %s", v2 ? "OPEN" : "CLOSED");
            break;
        }
        case 5: {
            bool s1 = (HAL_GPIO_ReadPin(XKC_SENSOR1_PORT, XKC_SENSOR1_PIN) == GPIO_PIN_RESET);
            snprintf(out, out_sz, "Sens1:  %s", s1 ? "LIQUID" : "DRY");
            break;
        }
        case 6: {
            bool s2 = (HAL_GPIO_ReadPin(XKC_SENSOR2_PORT, XKC_SENSOR2_PIN) == GPIO_PIN_RESET);
            snprintf(out, out_sz, "Sens2:  %s", s2 ? "LIQUID" : "DRY");
            break;
        }
        case 7:
            snprintf(out, out_sz, "Sec:   %s", g_lora_config.security_enabled ? "AES-128" : "RAW");
            break;
        case 8:
            snprintf(out, out_sz, "HW ID: 0x%08lX", (unsigned long)s_hw_id);
            break;
        default:
            out[0] = '\0';
            break;
    }
}

static void render_dashboard(void) {
    ssd1306_fill(SSD1306_COLOR_BLACK);

    // Header banner with current item range
    ssd1306_draw_filled_rectangle(0, 0, 128, 12, SSD1306_COLOR_WHITE);
    char hdr[24];
    snprintf(hdr, sizeof(hdr), "STATUS [%d-%d/%d]", s_dash_scroll_top + 1, s_dash_scroll_top + 4, DASHBOARD_ITEM_COUNT);
    ssd1306_set_cursor(11, 1);
    ssd1306_write_string(hdr, Font_7x10, SSD1306_COLOR_BLACK);

    // Render 4 lines, exactly 1 value per line
    uint8_t y_coords[4] = {15, 27, 39, 51};
    char buf[24];

    for (int i = 0; i < 4; i++) {
        int item_idx = s_dash_scroll_top + i;
        if (item_idx >= DASHBOARD_ITEM_COUNT) break;

        get_dashboard_line(item_idx, buf, sizeof(buf));
        ssd1306_set_cursor(2, y_coords[i]);
        ssd1306_write_string(buf, Font_7x10, SSD1306_COLOR_WHITE);
    }

    ssd1306_update_screen();
}

static void render_main_menu(void) {
    ssd1306_fill(SSD1306_COLOR_BLACK);

    // Header banner
    ssd1306_draw_filled_rectangle(0, 0, 128, 12, SSD1306_COLOR_WHITE);
    ssd1306_set_cursor(18, 1);
    ssd1306_write_string("EC11 MAIN MENU", Font_7x10, SSD1306_COLOR_BLACK);

    // Maintain 3-item scrolling window
    if (s_cursor_idx < s_scroll_top) {
        s_scroll_top = s_cursor_idx;
    } else if (s_cursor_idx >= s_scroll_top + 3) {
        s_scroll_top = s_cursor_idx - 2;
    }

    char line[24];
    uint8_t y_coords[3] = {16, 32, 48};

    for (int i = 0; i < 3; i++) {
        int item_idx = s_scroll_top + i;
        if (item_idx >= MAIN_ITEM_COUNT) break;

        char cursor_char = (item_idx == s_cursor_idx) ? '>' : ' ';

        switch (item_idx) {
            case MAIN_ITEM_VALVE1: {
                bool v1 = modbus_app_get_valve_state(0);
                snprintf(line, sizeof(line), "%c Valve 1: [%s]", cursor_char, v1 ? "ON" : "OFF");
                break;
            }
            case MAIN_ITEM_VALVE2: {
                bool v2 = modbus_app_get_valve_state(1);
                snprintf(line, sizeof(line), "%c Valve 2: [%s]", cursor_char, v2 ? "ON" : "OFF");
                break;
            }
            case MAIN_ITEM_SECURITY:
                snprintf(line, sizeof(line), "%c Sec: [%s]", cursor_char,
                         g_lora_config.security_enabled ? "AES-128" : "RAW");
                break;
            case MAIN_ITEM_SEC_KEY:
                snprintf(line, sizeof(line), "%c Sec Key >", cursor_char);
                break;
            case MAIN_ITEM_SLAVE_ID:
                snprintf(line, sizeof(line), "%c Slave ID: [%u]", cursor_char, (unsigned int)g_lora_config.slave_id);
                break;
            case MAIN_ITEM_LORA_CONFIG:
                snprintf(line, sizeof(line), "%c LoRa Config >", cursor_char);
                break;
            case MAIN_ITEM_SCREEN_TYPE:
                snprintf(line, sizeof(line), "%c OLED: [%s]", cursor_char, s_is_1_3_inch ? "1.3\"" : "0.96\"");
                break;
            case MAIN_ITEM_NODE_INFO:
                snprintf(line, sizeof(line), "%c Node Info...", cursor_char);
                break;
            default:
                line[0] = '\0';
                break;
        }

        ssd1306_set_cursor(0, y_coords[i]);
        ssd1306_write_string(line, Font_7x10, SSD1306_COLOR_WHITE);
    }

    ssd1306_update_screen();
}

static void render_lora_menu(void) {
    ssd1306_fill(SSD1306_COLOR_BLACK);

    // Header banner
    ssd1306_draw_filled_rectangle(0, 0, 128, 12, SSD1306_COLOR_WHITE);
    ssd1306_set_cursor(16, 1);
    ssd1306_write_string("LORA RADIO CFG", Font_7x10, SSD1306_COLOR_BLACK);

    // Maintain 3-item scrolling window
    if (s_cursor_idx < s_scroll_top) {
        s_scroll_top = s_cursor_idx;
    } else if (s_cursor_idx >= s_scroll_top + 3) {
        s_scroll_top = s_cursor_idx - 2;
    }

    char line[24];
    uint8_t y_coords[3] = {16, 32, 48};

    for (int i = 0; i < 3; i++) {
        int item_idx = s_scroll_top + i;
        if (item_idx >= LORA_ITEM_COUNT) break;

        char cursor_char = (item_idx == s_cursor_idx) ? '>' : ' ';

        switch (item_idx) {
            case LORA_ITEM_FREQ: {
                char f_str[16];
                format_freq_short(f_str, sizeof(f_str), g_lora_config.frequency);
                snprintf(line, sizeof(line), "%c Freq:[%s]", cursor_char, f_str);
                break;
            }
            case LORA_ITEM_SF:
                snprintf(line, sizeof(line), "%c SF:   [%s]", cursor_char,
                         get_sf_name(g_lora_config.spreading_factor));
                break;
            case LORA_ITEM_POWER:
                snprintf(line, sizeof(line), "%c Pwr:  [%d dBm]", cursor_char,
                         (int)g_lora_config.tx_power);
                break;
            case LORA_ITEM_BW:
                snprintf(line, sizeof(line), "%c BW:   [%s]", cursor_char,
                         get_bw_name(g_lora_config.bandwidth));
                break;
            case LORA_ITEM_CR:
                snprintf(line, sizeof(line), "%c CR:   [%s]", cursor_char,
                         get_cr_name(g_lora_config.coding_rate));
                break;
            default:
                line[0] = '\0';
                break;
        }

        ssd1306_set_cursor(0, y_coords[i]);
        ssd1306_write_string(line, Font_7x10, SSD1306_COLOR_WHITE);
    }

    ssd1306_update_screen();
}

static void render_key_menu(void) {
    ssd1306_fill(SSD1306_COLOR_BLACK);

    // Header banner
    ssd1306_draw_filled_rectangle(0, 0, 128, 12, SSD1306_COLOR_WHITE);
    ssd1306_set_cursor(16, 1);
    ssd1306_write_string("AES KEY MENU", Font_7x10, SSD1306_COLOR_BLACK);

    if (s_cursor_idx < s_scroll_top) {
        s_scroll_top = s_cursor_idx;
    } else if (s_cursor_idx >= s_scroll_top + 3) {
        s_scroll_top = s_cursor_idx - 2;
    }

    char line[24];
    uint8_t y_coords[3] = {16, 32, 48};

    for (int i = 0; i < 3; i++) {
        int item_idx = s_scroll_top + i;
        if (item_idx >= KEY_ITEM_COUNT) break;

        char cursor_char = (item_idx == s_cursor_idx) ? '>' : ' ';

        switch (item_idx) {
            case KEY_ITEM_VIEW:
                snprintf(line, sizeof(line), "%c View Key", cursor_char);
                break;
            case KEY_ITEM_PRESET:
                snprintf(line, sizeof(line), "%c Key Presets >", cursor_char);
                break;
            case KEY_ITEM_EDIT_BYTE:
                snprintf(line, sizeof(line), "%c Edit Bytes...", cursor_char);
                break;
            case KEY_ITEM_GEN_RANDOM:
                snprintf(line, sizeof(line), "%c Gen Random Key", cursor_char);
                break;
            case KEY_ITEM_RESET_DEFAULT:
                snprintf(line, sizeof(line), "%c Reset Default", cursor_char);
                break;
            default:
                line[0] = '\0';
                break;
        }

        ssd1306_set_cursor(0, y_coords[i]);
        ssd1306_write_string(line, Font_7x10, SSD1306_COLOR_WHITE);
    }

    ssd1306_update_screen();
}

static void render_key_view(void) {
    ssd1306_fill(SSD1306_COLOR_BLACK);

    // Header banner
    ssd1306_draw_filled_rectangle(0, 0, 128, 12, SSD1306_COLOR_WHITE);
    ssd1306_set_cursor(10, 1);
    ssd1306_write_string("AES-128 KEY (HEX)", Font_7x10, SSD1306_COLOR_BLACK);

    char l1[24];
    char l2[24];
    snprintf(l1, sizeof(l1), "0-7:%02X%02X%02X%02X%02X%02X%02X%02X",
             g_lora_config.aes_key[0], g_lora_config.aes_key[1], g_lora_config.aes_key[2], g_lora_config.aes_key[3],
             g_lora_config.aes_key[4], g_lora_config.aes_key[5], g_lora_config.aes_key[6], g_lora_config.aes_key[7]);
    snprintf(l2, sizeof(l2), "8-F:%02X%02X%02X%02X%02X%02X%02X%02X",
             g_lora_config.aes_key[8], g_lora_config.aes_key[9], g_lora_config.aes_key[10], g_lora_config.aes_key[11],
             g_lora_config.aes_key[12], g_lora_config.aes_key[13], g_lora_config.aes_key[14], g_lora_config.aes_key[15]);

    ssd1306_set_cursor(0, 18);
    ssd1306_write_string(l1, Font_7x10, SSD1306_COLOR_WHITE);

    ssd1306_set_cursor(0, 32);
    ssd1306_write_string(l2, Font_7x10, SSD1306_COLOR_WHITE);

    ssd1306_set_cursor(0, 50);
    ssd1306_write_string("Press PA9: Back", Font_7x10, SSD1306_COLOR_WHITE);

    ssd1306_update_screen();
}

static void render_key_preset(void) {
    ssd1306_fill(SSD1306_COLOR_BLACK);

    // Header banner
    ssd1306_draw_filled_rectangle(0, 0, 128, 12, SSD1306_COLOR_WHITE);
    ssd1306_set_cursor(10, 1);
    ssd1306_write_string("SELECT KEY PRESET", Font_7x10, SSD1306_COLOR_BLACK);

    ssd1306_set_cursor(2, 16);
    ssd1306_write_string(s_key_preset_names[s_edit_key_preset_idx], Font_7x10, SSD1306_COLOR_WHITE);

    char preview[24];
    const uint8_t *kp = s_key_preset_values[s_edit_key_preset_idx];
    snprintf(preview, sizeof(preview), "%02X %02X %02X %02X..%02X",
             kp[0], kp[1], kp[2], kp[3], kp[15]);
    ssd1306_set_cursor(2, 32);
    ssd1306_write_string(preview, Font_7x10, SSD1306_COLOR_WHITE);

    ssd1306_set_cursor(0, 50);
    ssd1306_write_string("PA7:Save PA9:Back", Font_7x10, SSD1306_COLOR_WHITE);

    ssd1306_update_screen();
}

static void render_key_edit_byte(void) {
    ssd1306_fill(SSD1306_COLOR_BLACK);

    // Header banner
    ssd1306_draw_filled_rectangle(0, 0, 128, 12, SSD1306_COLOR_WHITE);
    ssd1306_set_cursor(14, 1);
    ssd1306_write_string("EDIT KEY BYTE", Font_7x10, SSD1306_COLOR_BLACK);

    char title[24];
    snprintf(title, sizeof(title), "Byte [%02d/16]", s_edit_key_byte_idx + 1);
    ssd1306_set_cursor(4, 16);
    ssd1306_write_string(title, Font_7x10, SSD1306_COLOR_WHITE);

    char val[24];
    snprintf(val, sizeof(val), "Value: [ 0x%02X ]", s_temp_key[s_edit_key_byte_idx]);
    ssd1306_set_cursor(4, 30);
    ssd1306_write_string(val, Font_7x10, SSD1306_COLOR_WHITE);

    ssd1306_set_cursor(0, 43);
    ssd1306_write_string("Turn:Val PA7:Next", Font_7x10, SSD1306_COLOR_WHITE);
    ssd1306_set_cursor(0, 54);
    ssd1306_write_string("PA7:Hold PA9:Back", Font_7x10, SSD1306_COLOR_WHITE);

    ssd1306_update_screen();
}

static void render_edit_main(void) {
    ssd1306_fill(SSD1306_COLOR_BLACK);

    // Header banner
    ssd1306_draw_filled_rectangle(0, 0, 128, 12, SSD1306_COLOR_WHITE);
    ssd1306_set_cursor(14, 1);
    ssd1306_write_string("EDIT SETTING", Font_7x10, SSD1306_COLOR_BLACK);

    char val_str[24] = "";
    const char *title = "";

    switch (s_cursor_idx) {
        case MAIN_ITEM_VALVE1:
            title = "Valve 1 State";
            snprintf(val_str, sizeof(val_str), "[ %s ]", s_edit_valve1 ? "OPEN (ON)" : "CLOSED (OFF)");
            break;
        case MAIN_ITEM_VALVE2:
            title = "Valve 2 State";
            snprintf(val_str, sizeof(val_str), "[ %s ]", s_edit_valve2 ? "OPEN (ON)" : "CLOSED (OFF)");
            break;
        case MAIN_ITEM_SECURITY:
            title = "Payload Security";
            snprintf(val_str, sizeof(val_str), "[ %s ]", s_edit_security ? "AES-128" : "PLAIN (RAW)");
            break;
        case MAIN_ITEM_SLAVE_ID:
            title = "Modbus Slave ID";
            snprintf(val_str, sizeof(val_str), "[ ID: %u ]", (unsigned int)s_edit_slave_id);
            break;
        default:
            break;
    }

    ssd1306_set_cursor(4, 18);
    ssd1306_write_string(title, Font_7x10, SSD1306_COLOR_WHITE);

    ssd1306_set_cursor(8, 34);
    ssd1306_write_string(val_str, Font_7x10, SSD1306_COLOR_WHITE);

    ssd1306_set_cursor(0, 52);
    ssd1306_write_string("PA7:Save PA9:Back", Font_7x10, SSD1306_COLOR_WHITE);

    ssd1306_update_screen();
}

static void render_edit_lora(void) {
    ssd1306_fill(SSD1306_COLOR_BLACK);

    // Header banner
    ssd1306_draw_filled_rectangle(0, 0, 128, 12, SSD1306_COLOR_WHITE);
    ssd1306_set_cursor(12, 1);
    ssd1306_write_string("EDIT LORA PARAM", Font_7x10, SSD1306_COLOR_BLACK);

    char val_str[24] = "";
    const char *title = "";

    switch (s_cursor_idx) {
        case LORA_ITEM_FREQ: {
            title = "RF Frequency";
            if (s_edit_lora_opt_idx >= 0 && s_edit_lora_opt_idx < FREQ_OPT_COUNT) {
                snprintf(val_str, sizeof(val_str), "[ %s ]", s_freq_names[s_edit_lora_opt_idx]);
            } else {
                char f_str[16];
                format_freq_short(f_str, sizeof(f_str), g_lora_config.frequency);
                snprintf(val_str, sizeof(val_str), "[ %s ]", f_str);
            }
            break;
        }
        case LORA_ITEM_SF:
            title = "Spreading Factor";
            snprintf(val_str, sizeof(val_str), "[ %s ]", s_sf_names[s_edit_lora_opt_idx]);
            break;
        case LORA_ITEM_POWER:
            title = "TX Output Power";
            snprintf(val_str, sizeof(val_str), "[ %d dBm ]", (int)s_power_options[s_edit_lora_opt_idx]);
            break;
        case LORA_ITEM_BW:
            title = "Bandwidth";
            snprintf(val_str, sizeof(val_str), "[ %s ]", s_bw_names[s_edit_lora_opt_idx]);
            break;
        case LORA_ITEM_CR:
            title = "Coding Rate";
            snprintf(val_str, sizeof(val_str), "[ %s ]", s_cr_names[s_edit_lora_opt_idx]);
            break;
        default:
            break;
    }

    ssd1306_set_cursor(4, 18);
    ssd1306_write_string(title, Font_7x10, SSD1306_COLOR_WHITE);

    ssd1306_set_cursor(8, 34);
    ssd1306_write_string(val_str, Font_7x10, SSD1306_COLOR_WHITE);

    ssd1306_set_cursor(0, 52);
    ssd1306_write_string("PA7:Save PA9:Back", Font_7x10, SSD1306_COLOR_WHITE);

    ssd1306_update_screen();
}

static void get_info_line(int idx, char *out, size_t out_sz) {
    char f_str[16];
    switch (idx) {
        case 0:
            format_freq_short(f_str, sizeof(f_str), g_lora_config.frequency);
            snprintf(out, out_sz, "Freq:  %s", f_str);
            break;
        case 1:
            snprintf(out, out_sz, "SF:    %s", get_sf_name(g_lora_config.spreading_factor));
            break;
        case 2:
            snprintf(out, out_sz, "BW:    %s", get_bw_name(g_lora_config.bandwidth));
            break;
        case 3:
            snprintf(out, out_sz, "CR:    %s", get_cr_name(g_lora_config.coding_rate));
            break;
        case 4:
            snprintf(out, out_sz, "Power: %d dBm", (int)g_lora_config.tx_power);
            break;
        case 5:
            snprintf(out, out_sz, "Slave: ID %u", (unsigned int)g_lora_config.slave_id);
            break;
        case 6:
            snprintf(out, out_sz, "Sec:   %s", g_lora_config.security_enabled ? "AES-128" : "PLAIN (RAW)");
            break;
        case 7:
            snprintf(out, out_sz, "HW ID: 0x%08lX", (unsigned long)s_hw_id);
            break;
        case 8:
            snprintf(out, out_sz, "Tx:    %lu pkts", (unsigned long)s_tx_count);
            break;
        case 9:
            snprintf(out, out_sz, "OLED:  %s", s_is_1_3_inch ? "1.3\" SH1106" : "0.96\" SSD1306");
            break;
        default:
            out[0] = '\0';
            break;
    }
}

static void render_info(void) {
    ssd1306_fill(SSD1306_COLOR_BLACK);

    // Header banner with current item range
    ssd1306_draw_filled_rectangle(0, 0, 128, 12, SSD1306_COLOR_WHITE);
    char hdr[24];
    snprintf(hdr, sizeof(hdr), "INFO [%d-%d/%d]", s_info_scroll_top + 1, s_info_scroll_top + 3, INFO_ITEM_COUNT);
    ssd1306_set_cursor(18, 1);
    ssd1306_write_string(hdr, Font_7x10, SSD1306_COLOR_BLACK);

    // Render 3 lines, exactly 1 value per line
    uint8_t y_coords[3] = {15, 27, 39};
    char buf[24];

    for (int i = 0; i < 3; i++) {
        int item_idx = s_info_scroll_top + i;
        if (item_idx >= INFO_ITEM_COUNT) break;

        get_info_line(item_idx, buf, sizeof(buf));
        ssd1306_set_cursor(2, y_coords[i]);
        ssd1306_write_string(buf, Font_7x10, SSD1306_COLOR_WHITE);
    }

    // Footer banner
    ssd1306_set_cursor(0, 51);
    ssd1306_write_string("Press PA9: Back", Font_7x10, SSD1306_COLOR_WHITE);

    ssd1306_update_screen();
}

void menu_process(ec11_event_t event) {
    uint32_t now = HAL_GetTick();

    if (event != EC11_EVENT_NONE) {
        s_last_action_tick = now;
    }

    // Auto-return to dashboard on timeout is disabled: going back is handled strictly by PA9 Back button
    /*
    if (s_mode != UI_MODE_DASHBOARD && (now - s_last_action_tick >= MENU_TIMEOUT_MS)) {
        s_mode = UI_MODE_DASHBOARD;
        render_dashboard();
        return;
    }
    */

    switch (s_mode) {
        case UI_MODE_DASHBOARD:
            if (event == EC11_EVENT_CW) {
                if (s_dash_scroll_top < DASHBOARD_ITEM_COUNT - 4) {
                    s_dash_scroll_top++;
                    render_dashboard();
                }
            } else if (event == EC11_EVENT_CCW) {
                if (s_dash_scroll_top > 0) {
                    s_dash_scroll_top--;
                    render_dashboard();
                }
            } else if (event == EC11_EVENT_CLICK) {
                s_mode = UI_MODE_MAIN_MENU;
                s_cursor_idx = 0;
                s_scroll_top = 0;
                render_main_menu();
            }
            break;

        case UI_MODE_MAIN_MENU:
            if (event == EC11_EVENT_CW) {
                if (s_cursor_idx < MAIN_ITEM_COUNT - 1) {
                    s_cursor_idx++;
                    render_main_menu();
                }
            } else if (event == EC11_EVENT_CCW) {
                if (s_cursor_idx > 0) {
                    s_cursor_idx--;
                    render_main_menu();
                }
            } else if (event == EC11_EVENT_BACK) {
                s_mode = UI_MODE_DASHBOARD;
                render_dashboard();
            } else if (event == EC11_EVENT_CLICK) {
                switch (s_cursor_idx) {
                    case MAIN_ITEM_VALVE1:
                        s_edit_valve1 = modbus_app_get_valve_state(0);
                        s_mode = UI_MODE_EDIT_MAIN;
                        render_edit_main();
                        break;
                    case MAIN_ITEM_VALVE2:
                        s_edit_valve2 = modbus_app_get_valve_state(1);
                        s_mode = UI_MODE_EDIT_MAIN;
                        render_edit_main();
                        break;
                    case MAIN_ITEM_SECURITY:
                        s_edit_security = g_lora_config.security_enabled;
                        s_mode = UI_MODE_EDIT_MAIN;
                        render_edit_main();
                        break;
                    case MAIN_ITEM_SEC_KEY:
                        s_mode = UI_MODE_KEY_MENU;
                        s_cursor_idx = 0;
                        s_scroll_top = 0;
                        render_key_menu();
                        break;
                    case MAIN_ITEM_SLAVE_ID:
                        s_edit_slave_id = g_lora_config.slave_id;
                        s_mode = UI_MODE_EDIT_MAIN;
                        render_edit_main();
                        break;
                    case MAIN_ITEM_LORA_CONFIG:
                        s_mode = UI_MODE_LORA_MENU;
                        s_cursor_idx = 0;
                        s_scroll_top = 0;
                        render_lora_menu();
                        break;
                    case MAIN_ITEM_SCREEN_TYPE:
                        s_is_1_3_inch = !s_is_1_3_inch;
                        ssd1306_set_sh1106_mode(s_is_1_3_inch);
                        render_main_menu();
                        break;
                    case MAIN_ITEM_NODE_INFO:
                        s_info_scroll_top = 0;
                        s_mode = UI_MODE_INFO;
                        render_info();
                        break;
                    default:
                        break;
                }
            }
            break;

        case UI_MODE_LORA_MENU:
            if (event == EC11_EVENT_CW) {
                if (s_cursor_idx < LORA_ITEM_COUNT - 1) {
                    s_cursor_idx++;
                    render_lora_menu();
                }
            } else if (event == EC11_EVENT_CCW) {
                if (s_cursor_idx > 0) {
                    s_cursor_idx--;
                    render_lora_menu();
                }
            } else if (event == EC11_EVENT_BACK) {
                s_mode = UI_MODE_MAIN_MENU;
                s_cursor_idx = MAIN_ITEM_LORA_CONFIG;
                s_scroll_top = (s_cursor_idx > 2) ? (s_cursor_idx - 2) : 0;
                render_main_menu();
            } else if (event == EC11_EVENT_CLICK) {
                switch (s_cursor_idx) {
                    case LORA_ITEM_FREQ:
                        s_edit_lora_opt_idx = find_freq_idx(g_lora_config.frequency);
                        break;
                    case LORA_ITEM_SF:
                        s_edit_lora_opt_idx = find_sf_idx(g_lora_config.spreading_factor);
                        break;
                    case LORA_ITEM_POWER:
                        s_edit_lora_opt_idx = find_power_idx(g_lora_config.tx_power);
                        break;
                    case LORA_ITEM_BW:
                        s_edit_lora_opt_idx = find_bw_idx(g_lora_config.bandwidth);
                        break;
                    case LORA_ITEM_CR:
                        s_edit_lora_opt_idx = find_cr_idx(g_lora_config.coding_rate);
                        break;
                    default:
                        break;
                }
                s_mode = UI_MODE_EDIT_LORA;
                render_edit_lora();
            }
            break;

        case UI_MODE_KEY_MENU:
            if (event == EC11_EVENT_CW) {
                if (s_cursor_idx < KEY_ITEM_COUNT - 1) {
                    s_cursor_idx++;
                    render_key_menu();
                }
            } else if (event == EC11_EVENT_CCW) {
                if (s_cursor_idx > 0) {
                    s_cursor_idx--;
                    render_key_menu();
                }
            } else if (event == EC11_EVENT_BACK) {
                s_mode = UI_MODE_MAIN_MENU;
                s_cursor_idx = MAIN_ITEM_SEC_KEY;
                s_scroll_top = (s_cursor_idx > 2) ? (s_cursor_idx - 2) : 0;
                render_main_menu();
            } else if (event == EC11_EVENT_CLICK) {
                switch (s_cursor_idx) {
                    case KEY_ITEM_VIEW:
                        s_mode = UI_MODE_KEY_VIEW;
                        render_key_view();
                        break;
                    case KEY_ITEM_PRESET:
                        s_edit_key_preset_idx = 0;
                        s_mode = UI_MODE_KEY_PRESET;
                        render_key_preset();
                        break;
                    case KEY_ITEM_EDIT_BYTE:
                        memcpy(s_temp_key, g_lora_config.aes_key, 16);
                        s_edit_key_byte_idx = 0;
                        s_mode = UI_MODE_KEY_EDIT_BYTE;
                        render_key_edit_byte();
                        break;
                    case KEY_ITEM_GEN_RANDOM: {
                        uint8_t rand_key[16];
                        for (int i = 0; i < 16; i += 4) {
                            uint32_t rnd = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetTick() ^ (HAL_GetUIDw2() + (uint32_t)i * 1103515245U);
                            rand_key[i]   = (uint8_t)(rnd & 0xFF);
                            rand_key[i+1] = (uint8_t)((rnd >> 8) & 0xFF);
                            rand_key[i+2] = (uint8_t)((rnd >> 16) & 0xFF);
                            rand_key[i+3] = (uint8_t)((rnd >> 24) & 0xFF);
                        }
                        lora_security_set_key(rand_key);
                        memcpy(g_lora_config.aes_key, rand_key, 16);
                        config_save_to_flash();
                        s_mode = UI_MODE_KEY_VIEW;
                        render_key_view();
                        break;
                    }
                    case KEY_ITEM_RESET_DEFAULT: {
                        static const uint8_t factory_key[16] = {
                            0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
                            0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
                        };
                        lora_security_set_key(factory_key);
                        memcpy(g_lora_config.aes_key, factory_key, 16);
                        config_save_to_flash();
                        s_mode = UI_MODE_KEY_VIEW;
                        render_key_view();
                        break;
                    }
                    default:
                        break;
                }
            }
            break;

        case UI_MODE_KEY_VIEW:
            if (event == EC11_EVENT_BACK) {
                s_mode = UI_MODE_KEY_MENU;
                render_key_menu();
            }
            break;

        case UI_MODE_KEY_PRESET:
            if (event == EC11_EVENT_CW) {
                if (s_edit_key_preset_idx < KEY_PRESET_COUNT - 1) {
                    s_edit_key_preset_idx++;
                    render_key_preset();
                }
            } else if (event == EC11_EVENT_CCW) {
                if (s_edit_key_preset_idx > 0) {
                    s_edit_key_preset_idx--;
                    render_key_preset();
                }
            } else if (event == EC11_EVENT_BACK) {
                s_mode = UI_MODE_KEY_MENU;
                render_key_menu();
            } else if (event == EC11_EVENT_CLICK) {
                lora_security_set_key(s_key_preset_values[s_edit_key_preset_idx]);
                memcpy(g_lora_config.aes_key, s_key_preset_values[s_edit_key_preset_idx], 16);
                config_save_to_flash();
                s_mode = UI_MODE_KEY_MENU;
                render_key_menu();
            }
            break;

        case UI_MODE_KEY_EDIT_BYTE:
            if (event == EC11_EVENT_CW) {
                s_temp_key[s_edit_key_byte_idx]++;
                render_key_edit_byte();
            } else if (event == EC11_EVENT_CCW) {
                s_temp_key[s_edit_key_byte_idx]--;
                render_key_edit_byte();
            } else if (event == EC11_EVENT_BACK) {
                s_mode = UI_MODE_KEY_MENU;
                render_key_menu();
            } else if (event == EC11_EVENT_CLICK) {
                // Advance to next byte (0..15)
                s_edit_key_byte_idx = (s_edit_key_byte_idx + 1) % 16;
                render_key_edit_byte();
            } else if (event == EC11_EVENT_LONG_PRESS) {
                // Long press commits all 16 bytes
                lora_security_set_key(s_temp_key);
                memcpy(g_lora_config.aes_key, s_temp_key, 16);
                config_save_to_flash();
                s_mode = UI_MODE_KEY_VIEW;
                render_key_view();
            }
            break;

        case UI_MODE_EDIT_MAIN:
            if (event == EC11_EVENT_CW) {
                if (s_cursor_idx == MAIN_ITEM_VALVE1) s_edit_valve1 = !s_edit_valve1;
                else if (s_cursor_idx == MAIN_ITEM_VALVE2) s_edit_valve2 = !s_edit_valve2;
                else if (s_cursor_idx == MAIN_ITEM_SECURITY) s_edit_security = (s_edit_security) ? 0 : 1;
                else if (s_cursor_idx == MAIN_ITEM_SLAVE_ID) {
                    if (s_edit_slave_id < 247) s_edit_slave_id++;
                }
                render_edit_main();
            } else if (event == EC11_EVENT_CCW) {
                if (s_cursor_idx == MAIN_ITEM_VALVE1) s_edit_valve1 = !s_edit_valve1;
                else if (s_cursor_idx == MAIN_ITEM_VALVE2) s_edit_valve2 = !s_edit_valve2;
                else if (s_cursor_idx == MAIN_ITEM_SECURITY) s_edit_security = (s_edit_security) ? 0 : 1;
                else if (s_cursor_idx == MAIN_ITEM_SLAVE_ID) {
                    if (s_edit_slave_id > 1) s_edit_slave_id--;
                }
                render_edit_main();
            } else if (event == EC11_EVENT_BACK) {
                s_mode = UI_MODE_MAIN_MENU;
                render_main_menu();
            } else if (event == EC11_EVENT_CLICK) {
                if (s_cursor_idx == MAIN_ITEM_VALVE1) {
                    modbus_app_set_valve_state(0, s_edit_valve1);
                } else if (s_cursor_idx == MAIN_ITEM_VALVE2) {
                    modbus_app_set_valve_state(1, s_edit_valve2);
                } else if (s_cursor_idx == MAIN_ITEM_SECURITY) {
                    g_lora_config.security_enabled = s_edit_security;
                    config_save_to_flash();
                } else if (s_cursor_idx == MAIN_ITEM_SLAVE_ID) {
                    modbus_app_set_slave_id(s_edit_slave_id);
                    g_lora_config.slave_id = s_edit_slave_id;
                    config_save_to_flash();
                }
                s_mode = UI_MODE_MAIN_MENU;
                render_main_menu();
            }
            break;

        case UI_MODE_EDIT_LORA:
            if (event == EC11_EVENT_CW) {
                switch (s_cursor_idx) {
                    case LORA_ITEM_FREQ:
                        if (s_edit_lora_opt_idx < FREQ_OPT_COUNT - 1) s_edit_lora_opt_idx++;
                        break;
                    case LORA_ITEM_SF:
                        if (s_edit_lora_opt_idx < SF_OPT_COUNT - 1) s_edit_lora_opt_idx++;
                        break;
                    case LORA_ITEM_POWER:
                        if (s_edit_lora_opt_idx < POWER_OPT_COUNT - 1) s_edit_lora_opt_idx++;
                        break;
                    case LORA_ITEM_BW:
                        if (s_edit_lora_opt_idx < BW_OPT_COUNT - 1) s_edit_lora_opt_idx++;
                        break;
                    case LORA_ITEM_CR:
                        if (s_edit_lora_opt_idx < CR_OPT_COUNT - 1) s_edit_lora_opt_idx++;
                        break;
                    default:
                        break;
                }
                render_edit_lora();
            } else if (event == EC11_EVENT_CCW) {
                if (s_edit_lora_opt_idx > 0) {
                    s_edit_lora_opt_idx--;
                }
                render_edit_lora();
            } else if (event == EC11_EVENT_BACK) {
                s_mode = UI_MODE_LORA_MENU;
                render_lora_menu();
            } else if (event == EC11_EVENT_CLICK) {
                switch (s_cursor_idx) {
                    case LORA_ITEM_FREQ:
                        g_lora_config.frequency = s_freq_options[s_edit_lora_opt_idx];
                        break;
                    case LORA_ITEM_SF:
                        g_lora_config.spreading_factor = s_sf_options[s_edit_lora_opt_idx];
                        break;
                    case LORA_ITEM_POWER:
                        g_lora_config.tx_power = s_power_options[s_edit_lora_opt_idx];
                        break;
                    case LORA_ITEM_BW:
                        g_lora_config.bandwidth = s_bw_options[s_edit_lora_opt_idx];
                        break;
                    case LORA_ITEM_CR:
                        g_lora_config.coding_rate = s_cr_options[s_edit_lora_opt_idx];
                        break;
                    default:
                        break;
                }

                config_save_to_flash();
                radio_reconfigure();

                s_mode = UI_MODE_LORA_MENU;
                render_lora_menu();
            }
            break;

        case UI_MODE_INFO:
            if (event == EC11_EVENT_CW) {
                if (s_info_scroll_top < INFO_ITEM_COUNT - 3) {
                    s_info_scroll_top++;
                    render_info();
                }
            } else if (event == EC11_EVENT_CCW) {
                if (s_info_scroll_top > 0) {
                    s_info_scroll_top--;
                    render_info();
                }
            } else if (event == EC11_EVENT_BACK) {
                s_mode = UI_MODE_MAIN_MENU;
                render_main_menu();
            }
            break;
    }
}

static const char *get_sf_name(sx126x_lora_sf_t sf) {
    int idx = find_sf_idx(sf);
    return (idx >= 0 && idx < SF_OPT_COUNT) ? s_sf_names[idx] : "SF7";
}

static const char *get_bw_name(sx126x_lora_bw_t bw) {
    int idx = find_bw_idx(bw);
    return (idx >= 0 && idx < BW_OPT_COUNT) ? s_bw_names[idx] : "125kHz";
}

static const char *get_cr_name(sx126x_lora_cr_t cr) {
    int idx = find_cr_idx(cr);
    return (idx >= 0 && idx < CR_OPT_COUNT) ? s_cr_names[idx] : "4/5";
}

static int find_freq_idx(uint32_t freq) {
    for (int i = 0; i < FREQ_OPT_COUNT; i++) {
        if (s_freq_options[i] == freq) return i;
    }
    return 2; // Default to 915 MHz
}

static int find_sf_idx(sx126x_lora_sf_t sf) {
    for (int i = 0; i < SF_OPT_COUNT; i++) {
        if (s_sf_options[i] == sf) return i;
    }
    return 0; // Default to SF7
}

static int find_power_idx(int8_t power) {
    for (int i = 0; i < POWER_OPT_COUNT; i++) {
        if (s_power_options[i] == power) return i;
    }
    return 1; // Default to 14 dBm
}

static int find_bw_idx(sx126x_lora_bw_t bw) {
    for (int i = 0; i < BW_OPT_COUNT; i++) {
        if (s_bw_options[i] == bw) return i;
    }
    return 0; // Default to 125 kHz
}

static int find_cr_idx(sx126x_lora_cr_t cr) {
    for (int i = 0; i < CR_OPT_COUNT; i++) {
        if (s_cr_options[i] == cr) return i;
    }
    return 0; // Default to 4/5
}
