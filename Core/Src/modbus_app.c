#include "modbus_app.h"
#include "modbus.h"
#include "modbus-private.h"
#include "config.h"
#include "uart_app.h"
#include <stdio.h>
#include <string.h>

static modbus_t mb_ctx;
static modbus_mapping_t *mb_mapping = NULL;
static modbus_tx_callback_t g_tx_callback = NULL;
static modbus_valve_callback_t g_valve_callback = NULL;

static uint16_t modbus_crc16(const uint8_t *buffer, uint16_t buffer_length) {
  static const uint8_t table_crc_hi[] = {
      0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
      0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
      0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
      0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
      0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
      0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
      0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
      0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
      0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
      0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
      0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
      0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
      0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
      0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
      0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
      0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
      0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
      0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
      0x00, 0xC1, 0x81, 0x40
  };
  static const uint8_t table_crc_lo[] = {
      0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5,
      0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B,
      0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE,
      0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6,
      0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
      0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
      0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8,
      0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
      0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21,
      0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
      0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A,
      0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
      0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7,
      0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91, 0x51,
      0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
      0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
      0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D,
      0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
      0x41, 0x81, 0x80, 0x40
  };
  uint8_t crc_hi = 0xFF;
  uint8_t crc_lo = 0xFF;
  while (buffer_length--) {
    unsigned int i = crc_lo ^ *buffer++;
    crc_lo = crc_hi ^ table_crc_hi[i];
    crc_hi = table_crc_lo[i];
  }
  return (crc_hi << 8) | crc_lo;
}

static int _modbus_embedded_set_slave(modbus_t *ctx, int slave) {
  ctx->slave = slave;
  return 0;
}

static int _modbus_embedded_build_request_basis(modbus_t *ctx, int function, int addr, int nb, uint8_t *req) {
  req[0] = ctx->slave;
  req[1] = function;
  req[2] = addr >> 8;
  req[3] = addr & 0xFF;
  req[4] = nb >> 8;
  req[5] = nb & 0xFF;
  return 6;
}

static int _modbus_embedded_build_response_basis(sft_t *sft, uint8_t *rsp) {
  rsp[0] = sft->slave;
  rsp[1] = sft->function;
  return 2;
}

static int _modbus_embedded_get_response_tid(const uint8_t *req) {
  (void)req;
  return 0;
}

static int _modbus_embedded_send_msg_pre(uint8_t *req, int req_length) {
  uint16_t crc = modbus_crc16(req, req_length);
  req[req_length++] = crc & 0xFF;
  req[req_length++] = (crc >> 8) & 0xFF;
  return req_length;
}

static ssize_t _modbus_embedded_send(modbus_t *ctx, const uint8_t *req, int req_length) {
  (void)ctx;
  uart_print_hex("MODBUS TX: ", req, (uint16_t)req_length);

  if (g_tx_callback != NULL) {
    g_tx_callback(req, (uint8_t)req_length);
  }
  return req_length;
}

static int _modbus_embedded_check_integrity(modbus_t *ctx, uint8_t *msg, const int msg_length) {
  if (msg_length < 4) {
    return -1;
  }
  uint16_t crc_calc = modbus_crc16(msg, msg_length - 2);
  uint16_t crc_recv = msg[msg_length - 2] | (msg[msg_length - 1] << 8);
  if (crc_calc != crc_recv) {
    return -1;
  }

  int slave = msg[0];
  if (ctx != NULL && slave != ctx->slave && slave != MODBUS_BROADCAST_ADDRESS) {
    return 0;
  }

  return msg_length;
}

static const modbus_backend_t modbus_embedded_backend = {
    .backend_type = _MODBUS_BACKEND_TYPE_RTU,
    .header_length = 1,
    .checksum_length = 2,
    .max_adu_length = 256,
    .set_slave = _modbus_embedded_set_slave,
    .build_request_basis = _modbus_embedded_build_request_basis,
    .build_response_basis = _modbus_embedded_build_response_basis,
    .get_response_tid = _modbus_embedded_get_response_tid,
    .send_msg_pre = _modbus_embedded_send_msg_pre,
    .send = _modbus_embedded_send,
    .receive = NULL,
    .recv = NULL,
    .check_integrity = _modbus_embedded_check_integrity,
    .pre_check_confirmation = NULL,
    .connect = NULL,
    .is_connected = NULL,
    .close = NULL,
    .flush = NULL,
    .select = NULL,
    .free = NULL
};

void modbus_app_init(uint8_t slave_id, uint32_t hardware_id, modbus_tx_callback_t tx_cb) {
  g_tx_callback = tx_cb;

  _modbus_init_common(&mb_ctx);
  mb_ctx.backend = &modbus_embedded_backend;
  
  uint8_t id = (slave_id >= 1 && slave_id <= 247) ? slave_id : 1;
  modbus_set_slave(&mb_ctx, id);

  // Allocate 16 Coils, 16 Discrete Inputs, 16 Holding Registers, 16 Input Registers
  mb_mapping = modbus_mapping_new(16, 16, 16, 16);
  if (mb_mapping != NULL) {
    // Hardware ID in Holding Registers 0 and 1
    mb_mapping->tab_registers[MODBUS_HOLD_REG_HW_ID_HI] = (uint16_t)(hardware_id >> 16);
    mb_mapping->tab_registers[MODBUS_HOLD_REG_HW_ID_LO] = (uint16_t)(hardware_id & 0xFFFF);
    mb_mapping->tab_registers[MODBUS_HOLD_REG_TX_COUNT_HI] = 0;
    mb_mapping->tab_registers[MODBUS_HOLD_REG_TX_COUNT_LO] = 0;

    // Valves 1 and 2 initially Closed / 0
    mb_mapping->tab_bits[MODBUS_COIL_VALVE_1] = 0;
    mb_mapping->tab_bits[MODBUS_COIL_VALVE_2] = 0;
    mb_mapping->tab_input_bits[MODBUS_DISC_INPUT_VALVE_1] = 0;
    mb_mapping->tab_input_bits[MODBUS_DISC_INPUT_VALVE_2] = 0;
    mb_mapping->tab_registers[MODBUS_HOLD_REG_VALVE_1] = 0;
    mb_mapping->tab_registers[MODBUS_HOLD_REG_VALVE_2] = 0;

    // Sensors 1 and 2 initially 0
    mb_mapping->tab_input_registers[MODBUS_INPUT_REG_SENSOR_1] = 0;
    mb_mapping->tab_input_registers[MODBUS_INPUT_REG_SENSOR_2] = 0;
    mb_mapping->tab_registers[MODBUS_HOLD_REG_SENSOR_1] = 0;
    mb_mapping->tab_registers[MODBUS_HOLD_REG_SENSOR_2] = 0;

    // Slave ID in Holding Register 8
    mb_mapping->tab_registers[MODBUS_HOLD_REG_SLAVE_ID] = id;
  }
}

void modbus_app_set_slave_id(uint8_t slave_id) {
  if (slave_id >= 1 && slave_id <= 247) {
    modbus_set_slave(&mb_ctx, slave_id);
    if (mb_mapping != NULL) {
      mb_mapping->tab_registers[MODBUS_HOLD_REG_SLAVE_ID] = slave_id;
    }
  }
}

uint8_t modbus_app_get_slave_id(void) {
  return (uint8_t)mb_ctx.slave;
}

void modbus_app_set_valve_callback(modbus_valve_callback_t cb) {
  g_valve_callback = cb;
}

void modbus_app_set_valve_state(uint8_t valve_idx, bool state) {
  if (mb_mapping == NULL || valve_idx > 1) return;

  uint8_t val = state ? 1 : 0;
  mb_mapping->tab_bits[valve_idx] = val;
  mb_mapping->tab_input_bits[valve_idx] = val;
  mb_mapping->tab_registers[MODBUS_HOLD_REG_VALVE_1 + valve_idx] = val;

  if (g_valve_callback != NULL) {
    g_valve_callback(valve_idx, state);
  }
}

bool modbus_app_get_valve_state(uint8_t valve_idx) {
  if (mb_mapping == NULL || valve_idx > 1) return false;
  return mb_mapping->tab_bits[valve_idx] != 0;
}

void modbus_app_set_sensor_values(uint16_t sensor1, uint16_t sensor2) {
  if (mb_mapping == NULL) return;

  mb_mapping->tab_input_registers[MODBUS_INPUT_REG_SENSOR_1] = sensor1;
  mb_mapping->tab_registers[MODBUS_HOLD_REG_SENSOR_1] = sensor1;

  mb_mapping->tab_input_registers[MODBUS_INPUT_REG_SENSOR_2] = sensor2;
  mb_mapping->tab_registers[MODBUS_HOLD_REG_SENSOR_2] = sensor2;
}

void modbus_app_update_telemetry(uint32_t tx_count) {
  if (mb_mapping == NULL) return;

  mb_mapping->tab_registers[MODBUS_HOLD_REG_TX_COUNT_HI] = (uint16_t)(tx_count >> 16);
  mb_mapping->tab_registers[MODBUS_HOLD_REG_TX_COUNT_LO] = (uint16_t)(tx_count & 0xFFFF);
}

int modbus_app_process_packet(uint8_t *rx_payload, uint16_t rx_len) {
  if (mb_mapping == NULL || rx_payload == NULL || rx_len < 4) {
    return -1;
  }

  // Verify CRC16 frame integrity and filter by slave ID
  int integrity = _modbus_embedded_check_integrity(&mb_ctx, rx_payload, rx_len);
  if (integrity < 0) {
    return -1;
  }

  if (integrity == 0) {
    // Valid Modbus frame, but addressed to another slave
    uart_printf("[MODBUS] Ignored packet for Slave %u (current Slave is %u)\r\n",
                (unsigned int)rx_payload[0], (unsigned int)mb_ctx.slave);
    return 0;
  }

  // Save previous valve states before processing reply
  uint8_t old_coil[2] = {
      mb_mapping->tab_bits[MODBUS_COIL_VALVE_1],
      mb_mapping->tab_bits[MODBUS_COIL_VALVE_2]
  };
  uint16_t old_reg[2] = {
      mb_mapping->tab_registers[MODBUS_HOLD_REG_VALVE_1],
      mb_mapping->tab_registers[MODBUS_HOLD_REG_VALVE_2]
  };

  // Process Modbus RTU message and formulate response
  int mb_res = modbus_reply(&mb_ctx, rx_payload, rx_len, mb_mapping);
  if (mb_res > 0) {
    uart_printf("MODBUS Reply Processed [Bytes: %d]\r\n", mb_res);
  } else if (mb_res == 0 && rx_payload[0] == MODBUS_BROADCAST_ADDRESS) {
    uart_printf("[MODBUS] Broadcast command executed (no reply sent)\r\n");
  }

  if (mb_res >= 0) {
    // Detect if coil write (FC 05, 15) or holding register write (FC 06, 16) changed valve states
    for (uint8_t i = 0; i < 2; i++) {
      bool changed = false;
      bool new_state = false;

      if (mb_mapping->tab_bits[i] != old_coil[i]) {
        new_state = (mb_mapping->tab_bits[i] != 0);
        changed = true;
      } else if (mb_mapping->tab_registers[MODBUS_HOLD_REG_VALVE_1 + i] != old_reg[i]) {
        new_state = (mb_mapping->tab_registers[MODBUS_HOLD_REG_VALVE_1 + i] != 0);
        changed = true;
      }

      if (changed) {
        uint8_t val = new_state ? 1 : 0;
        mb_mapping->tab_bits[i] = val;
        mb_mapping->tab_input_bits[i] = val;
        mb_mapping->tab_registers[MODBUS_HOLD_REG_VALVE_1 + i] = val;

        uart_printf("[MODBUS] Valve %u -> %s\r\n", i + 1, new_state ? "OPEN" : "CLOSED");

        if (g_valve_callback != NULL) {
          g_valve_callback(i, new_state);
        }
      }
    }

    // Detect if Slave ID was modified via Holding Register 8
    uint16_t requested_id = mb_mapping->tab_registers[MODBUS_HOLD_REG_SLAVE_ID];
    if (requested_id != mb_ctx.slave) {
      if (requested_id >= 1 && requested_id <= 247) {
        uint8_t new_id = (uint8_t)requested_id;
        modbus_set_slave(&mb_ctx, new_id);
        g_lora_config.slave_id = new_id;
        config_save_to_flash();

        uart_printf("[MODBUS] Slave ID changed to %u & saved to Flash\r\n", new_id);
      } else {
        // Invalid slave ID: revert holding register to current active slave ID
        mb_mapping->tab_registers[MODBUS_HOLD_REG_SLAVE_ID] = mb_ctx.slave;
      }
    }
  }

  return mb_res;
}

