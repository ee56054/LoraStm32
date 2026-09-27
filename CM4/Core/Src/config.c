#include "config.h"
#include "main.h"
#include <string.h>

lora_config_t g_lora_config;

static uint32_t calculate_checksum(const uint8_t *data, size_t len) {
    uint32_t crc = 0x55AA55AAU;
    for (size_t i = 0; i < len; ++i) {
        crc = (crc << 1) ^ data[i];
    }
    return crc;
}

void config_init(void) {
    // 1. Factory defaults for STM32WL LoRa node
    g_lora_config.frequency = 915000000;
    g_lora_config.tx_power = 14;
    g_lora_config.spreading_factor = SX126X_LORA_SF7;
    g_lora_config.bandwidth = SX126X_LORA_BW_125;
    g_lora_config.coding_rate = SX126X_LORA_CR_4_5;
    g_lora_config.preamble_length = 8;
    g_lora_config.rx_timeout = 0;
    g_lora_config.slave_id = 1;

    // 2. Load persistent config from Flash if valid
    config_load_from_flash();
}

bool config_load_from_flash(void) {
    const flash_storage_record_t *flash_record = (const flash_storage_record_t *)CONFIG_FLASH_ADDRESS;

    // Validate magic word
    if (flash_record->magic != CONFIG_FLASH_MAGIC) {
        return false;
    }

    // Validate version compatibility
    if (flash_record->version != CONFIG_FLASH_VERSION) {
        return false;
    }

    // Validate payload length
    if (flash_record->payload_len != sizeof(lora_config_t)) {
        return false;
    }

    // Verify data integrity
    uint32_t expected_checksum = calculate_checksum((const uint8_t *)&flash_record->config, sizeof(lora_config_t));
    if (flash_record->checksum != expected_checksum) {
        return false;
    }

    // Sanity check parameters
    if (flash_record->config.slave_id < 1 || flash_record->config.slave_id > 247) {
        return false;
    }

    // Successfully loaded
    memcpy(&g_lora_config, &flash_record->config, sizeof(lora_config_t));
    return true;
}

bool config_save_to_flash(void) {
    // 1. Prepare record in local RAM buffer aligned to 64-bit doubleword
    flash_storage_record_t record;
    memset(&record, 0, sizeof(record));

    record.magic = CONFIG_FLASH_MAGIC;
    record.version = CONFIG_FLASH_VERSION;
    record.payload_len = sizeof(lora_config_t);
    memcpy(&record.config, &g_lora_config, sizeof(lora_config_t));
    record.checksum = calculate_checksum((const uint8_t *)&g_lora_config, sizeof(lora_config_t));

    // Pad buffer to multiple of 8 bytes (64-bit doublewords)
    uint64_t dword_buf[(sizeof(flash_storage_record_t) + 7) / 8];
    memset(dword_buf, 0xFF, sizeof(dword_buf));
    memcpy(dword_buf, &record, sizeof(record));
    size_t num_dwords = sizeof(dword_buf) / sizeof(uint64_t);

    // 2. Unlock Flash control registers
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return false;
    }

    // Clear any previous flash error flags
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    // 3. Erase Page 63
    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t pageError = 0;

    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.Page = CONFIG_FLASH_PAGE;
    eraseInitStruct.NbPages = 1;

    if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    // 4. Program doubleword by doubleword (64-bit writes required on STM32WL)
    uint32_t addr = CONFIG_FLASH_ADDRESS;
    for (size_t i = 0; i < num_dwords; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, dword_buf[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        addr += 8;
    }

    // 5. Lock Flash control registers
    HAL_FLASH_Lock();

    // 6. Verify written data matches
    if (memcmp((const void *)CONFIG_FLASH_ADDRESS, &record, sizeof(record)) != 0) {
        return false;
    }

    return true;
}
