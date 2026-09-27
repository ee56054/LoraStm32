#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "sx126x.h"

// Flash Page 63 for STM32WL54 CM4 (2KB per page: 63 * 2048 = 0x0801F800)
#define CONFIG_FLASH_PAGE       63
#define CONFIG_FLASH_ADDRESS    0x0801F800
#define CONFIG_FLASH_MAGIC      0x4C524346U  // 'LRCF' (LoRa Config Flash)
#define CONFIG_FLASH_VERSION    2U

typedef struct {
    uint32_t frequency;
    int8_t tx_power;
    sx126x_lora_sf_t spreading_factor;
    sx126x_lora_bw_t bandwidth;
    sx126x_lora_cr_t coding_rate;
    uint16_t preamble_length;
    uint32_t rx_timeout;
    uint8_t slave_id;
    uint8_t security_enabled; // 0 = Plaintext, 1 = AES-128 Encryption Enabled
    uint8_t aes_key[16];      // 128-bit pre-shared AES encryption key
} lora_config_t;

// Binary flash record structure aligned to 64-bit doubleword for STM32WL
typedef struct __attribute__((packed, aligned(8))) {
    uint32_t magic;           // CONFIG_FLASH_MAGIC
    uint16_t version;         // CONFIG_FLASH_VERSION
    uint16_t payload_len;     // sizeof(lora_config_t)
    lora_config_t config;     // The configuration data
    uint32_t checksum;        // Checksum over config bytes
    uint32_t reserved;        // Padding to maintain 64-bit doubleword alignment
} flash_storage_record_t;

extern lora_config_t g_lora_config;

void config_init(void);
bool config_load_from_flash(void);
bool config_save_to_flash(void);

#endif // CONFIG_H
