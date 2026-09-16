#ifndef MODBUS_APP_H
#define MODBUS_APP_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Modbus Table Addresses */
#define MODBUS_COIL_VALVE_1              0
#define MODBUS_COIL_VALVE_2              1

#define MODBUS_DISC_INPUT_VALVE_1        0
#define MODBUS_DISC_INPUT_VALVE_2        1

#define MODBUS_INPUT_REG_SENSOR_1        0
#define MODBUS_INPUT_REG_SENSOR_2        1

#define MODBUS_HOLD_REG_HW_ID_HI         0
#define MODBUS_HOLD_REG_HW_ID_LO         1
#define MODBUS_HOLD_REG_TX_COUNT_HI      2
#define MODBUS_HOLD_REG_TX_COUNT_LO      3
#define MODBUS_HOLD_REG_VALVE_1          4
#define MODBUS_HOLD_REG_VALVE_2          5
#define MODBUS_HOLD_REG_SENSOR_1         6
#define MODBUS_HOLD_REG_SENSOR_2         7
#define MODBUS_HOLD_REG_SLAVE_ID         8

/* Transmit callback type (e.g. transmit packet over LoRa) */
typedef void (*modbus_tx_callback_t)(const uint8_t *payload, uint8_t payload_len);

/* Valve state change callback type */
typedef void (*modbus_valve_callback_t)(uint8_t valve_index, bool is_open);

/**
 * @brief Initialize Modbus RTU stack, mapping registers, and set slave ID
 * @param slave_id Modbus Slave Address (1..247)
 * @param hardware_id Hardware ID to store in holding registers 0 and 1
 * @param tx_cb Callback function to transmit Modbus responses
 */
void modbus_app_init(uint8_t slave_id, uint32_t hardware_id, modbus_tx_callback_t tx_cb);

/**
 * @brief Set the Modbus Slave ID (1..247)
 * @param slave_id New slave address
 */
void modbus_app_set_slave_id(uint8_t slave_id);

/**
 * @brief Get the current Modbus Slave ID
 * @return Current slave address
 */
uint8_t modbus_app_get_slave_id(void);

/**
 * @brief Register a callback to be called whenever valve 1 or valve 2 state changes
 * @param cb Callback function
 */
void modbus_app_set_valve_callback(modbus_valve_callback_t cb);

/**
 * @brief Manually set the state of a valve
 * @param valve_idx 0 for Valve 1, 1 for Valve 2
 * @param state true for Open / ON, false for Closed / OFF
 */
void modbus_app_set_valve_state(uint8_t valve_idx, bool state);

/**
 * @brief Get the current state of a valve
 * @param valve_idx 0 for Valve 1, 1 for Valve 2
 * @return true if Open / ON, false if Closed / OFF
 */
bool modbus_app_get_valve_state(uint8_t valve_idx);

/**
 * @brief Update 16-bit sensor readings in input registers and mirrored holding registers
 * @param sensor1 Sensor 1 reading
 * @param sensor2 Sensor 2 reading
 */
void modbus_app_set_sensor_values(uint16_t sensor1, uint16_t sensor2);

/**
 * @brief Update Modbus holding registers with runtime telemetry (e.g. TX counter)
 * @param tx_count Transmission counter to store in holding registers 2 and 3
 */
void modbus_app_update_telemetry(uint32_t tx_count);

/**
 * @brief Check if incoming packet is a Modbus RTU frame and process it
 * @param rx_payload Pointer to received payload buffer
 * @param rx_len Length of received payload
 * @return Positive integer (response byte count) if frame was handled by Modbus;
 *         <= 0 if frame is not a valid Modbus request for this slave.
 * @return Positive integer (> 0, response byte count) if frame was handled and replied;
 *         0 if frame is valid Modbus but ignored (addressed to another slave) or broadcast;
 *         Negative (< 0) if frame is not a valid Modbus frame (CRC error or invalid format).
 */
int modbus_app_process_packet(uint8_t *rx_payload, uint16_t rx_len);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_APP_H */

