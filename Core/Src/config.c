#include "config.h"
#include "main.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

lora_config_t g_lora_config;

// Flash page 63 for STM32F103C8 (64KB total, 1KB per page)
#define CONFIG_FLASH_ADDRESS 0x0800FC00

static const char* sf_to_str(sx126x_lora_sf_t sf) {
    switch (sf) {
        case SX126X_LORA_SF5: return "SF5";
        case SX126X_LORA_SF6: return "SF6";
        case SX126X_LORA_SF7: return "SF7";
        case SX126X_LORA_SF8: return "SF8";
        case SX126X_LORA_SF9: return "SF9";
        case SX126X_LORA_SF10: return "SF10";
        case SX126X_LORA_SF11: return "SF11";
        case SX126X_LORA_SF12: return "SF12";
        default: return "SF9";
    }
}

static sx126x_lora_sf_t str_to_sf(const char* s) {
    if (!s) return SX126X_LORA_SF9;
    if (strcmp(s, "SF5") == 0) return SX126X_LORA_SF5;
    if (strcmp(s, "SF6") == 0) return SX126X_LORA_SF6;
    if (strcmp(s, "SF7") == 0) return SX126X_LORA_SF7;
    if (strcmp(s, "SF8") == 0) return SX126X_LORA_SF8;
    if (strcmp(s, "SF9") == 0) return SX126X_LORA_SF9;
    if (strcmp(s, "SF10") == 0) return SX126X_LORA_SF10;
    if (strcmp(s, "SF11") == 0) return SX126X_LORA_SF11;
    if (strcmp(s, "SF12") == 0) return SX126X_LORA_SF12;
    return SX126X_LORA_SF9;
}

static const char* bw_to_str(sx126x_lora_bw_t bw) {
    switch (bw) {
        case SX126X_LORA_BW_007: return "7.81";
        case SX126X_LORA_BW_010: return "10.42";
        case SX126X_LORA_BW_015: return "15.63";
        case SX126X_LORA_BW_020: return "20.83";
        case SX126X_LORA_BW_031: return "31.25";
        case SX126X_LORA_BW_041: return "41.67";
        case SX126X_LORA_BW_062: return "62.5";
        case SX126X_LORA_BW_125: return "125";
        case SX126X_LORA_BW_250: return "250";
        case SX126X_LORA_BW_500: return "500";
        default: return "125";
    }
}

static sx126x_lora_bw_t str_to_bw(const char* s) {
    if (!s) return SX126X_LORA_BW_125;
    if (strcmp(s, "7.81") == 0) return SX126X_LORA_BW_007;
    if (strcmp(s, "10.42") == 0) return SX126X_LORA_BW_010;
    if (strcmp(s, "15.63") == 0) return SX126X_LORA_BW_015;
    if (strcmp(s, "20.83") == 0) return SX126X_LORA_BW_020;
    if (strcmp(s, "31.25") == 0) return SX126X_LORA_BW_031;
    if (strcmp(s, "41.67") == 0) return SX126X_LORA_BW_041;
    if (strcmp(s, "62.5") == 0) return SX126X_LORA_BW_062;
    if (strcmp(s, "125") == 0) return SX126X_LORA_BW_125;
    if (strcmp(s, "250") == 0) return SX126X_LORA_BW_250;
    if (strcmp(s, "500") == 0) return SX126X_LORA_BW_500;
    return SX126X_LORA_BW_125;
}

static const char* cr_to_str(sx126x_lora_cr_t cr) {
    switch (cr) {
        case SX126X_LORA_CR_4_5: return "4/5";
        case SX126X_LORA_CR_4_6: return "4/6";
        case SX126X_LORA_CR_4_7: return "4/7";
        case SX126X_LORA_CR_4_8: return "4/8";
        default: return "4/6";
    }
}

static sx126x_lora_cr_t str_to_cr(const char* s) {
    if (!s) return SX126X_LORA_CR_4_6;
    if (strcmp(s, "4/5") == 0) return SX126X_LORA_CR_4_5;
    if (strcmp(s, "4/6") == 0) return SX126X_LORA_CR_4_6;
    if (strcmp(s, "4/7") == 0) return SX126X_LORA_CR_4_7;
    if (strcmp(s, "4/8") == 0) return SX126X_LORA_CR_4_8;
    return SX126X_LORA_CR_4_6;
}

void config_init(void) {
    // Default values
    g_lora_config.frequency = 915000000;
    g_lora_config.tx_power = 22;
    g_lora_config.spreading_factor = SX126X_LORA_SF9;
    g_lora_config.bandwidth = SX126X_LORA_BW_125;
    g_lora_config.coding_rate = SX126X_LORA_CR_4_6;
    g_lora_config.preamble_length = 8;
    g_lora_config.rx_timeout = 5000;
    
    // Try to load from flash
    config_load_from_flash();
}

bool config_load_from_flash(void) {
    const char* flash_data = (const char*)CONFIG_FLASH_ADDRESS;
    
    // Check if the flash is empty (all 0xFF)
    if (flash_data[0] == (char)0xFF) {
        return false;
    }

    cJSON *doc = cJSON_Parse(flash_data);
    if (doc == NULL) {
        return false;
    }

    cJSON *item;
    
    item = cJSON_GetObjectItem(doc, "frequency");
    if (cJSON_IsNumber(item)) g_lora_config.frequency = item->valueint;
    
    item = cJSON_GetObjectItem(doc, "tx_power");
    if (cJSON_IsNumber(item)) g_lora_config.tx_power = item->valueint;
    
    item = cJSON_GetObjectItem(doc, "spreading_factor");
    if (cJSON_IsString(item)) g_lora_config.spreading_factor = str_to_sf(item->valuestring);
    
    item = cJSON_GetObjectItem(doc, "bandwidth");
    if (cJSON_IsString(item)) g_lora_config.bandwidth = str_to_bw(item->valuestring);
    
    item = cJSON_GetObjectItem(doc, "coding_rate");
    if (cJSON_IsString(item)) g_lora_config.coding_rate = str_to_cr(item->valuestring);
    
    item = cJSON_GetObjectItem(doc, "preamble_length");
    if (cJSON_IsNumber(item)) g_lora_config.preamble_length = item->valueint;
    
    item = cJSON_GetObjectItem(doc, "rx_timeout");
    if (cJSON_IsNumber(item)) g_lora_config.rx_timeout = item->valueint;

    cJSON_Delete(doc);
    return true;
}

bool config_save_to_flash(void) {
    cJSON *doc = cJSON_CreateObject();
    if (doc == NULL) {
        return false;
    }

    cJSON_AddNumberToObject(doc, "frequency", g_lora_config.frequency);
    cJSON_AddNumberToObject(doc, "tx_power", g_lora_config.tx_power);
    cJSON_AddStringToObject(doc, "spreading_factor", sf_to_str(g_lora_config.spreading_factor));
    cJSON_AddStringToObject(doc, "bandwidth", bw_to_str(g_lora_config.bandwidth));
    cJSON_AddStringToObject(doc, "coding_rate", cr_to_str(g_lora_config.coding_rate));
    cJSON_AddNumberToObject(doc, "preamble_length", g_lora_config.preamble_length);
    cJSON_AddNumberToObject(doc, "rx_timeout", g_lora_config.rx_timeout);

    char *json_str = cJSON_PrintUnformatted(doc);
    cJSON_Delete(doc);
    
    if (json_str == NULL) {
        return false;
    }

    size_t len = strlen(json_str);
    if (len >= 1024) { // Don't exceed 1KB page size
        free(json_str);
        return false;
    }

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t pageError = 0;
    
    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.PageAddress = CONFIG_FLASH_ADDRESS;
    eraseInitStruct.NbPages = 1;

    if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError) != HAL_OK) {
        HAL_FLASH_Lock();
        free(json_str);
        return false;
    }

    uint32_t addr = CONFIG_FLASH_ADDRESS;
    // Pad to multiple of 2 (half-word) since STM32F1 writes in half-words
    for (size_t i = 0; i <= len; i += 2) {
        uint16_t data = json_str[i];
        if (i + 1 <= len) {
            data |= (json_str[i+1] << 8);
        } else {
            data |= (0xFF << 8); // pad with 0xFF
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, data) != HAL_OK) {
            HAL_FLASH_Lock();
            free(json_str);
            return false;
        }
        addr += 2;
    }

    HAL_FLASH_Lock();
    free(json_str);
    return true;
}
