#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "sx126x.h"

typedef struct {
    uint32_t frequency;
    int8_t tx_power;
    sx126x_lora_sf_t spreading_factor;
    sx126x_lora_bw_t bandwidth;
    sx126x_lora_cr_t coding_rate;
    uint16_t preamble_length;
    uint32_t rx_timeout;
} lora_config_t;

extern lora_config_t g_lora_config;

void config_init(void);
bool config_load_from_flash(void);
bool config_save_to_flash(void);

#endif // CONFIG_H
