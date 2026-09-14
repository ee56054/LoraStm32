/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "sx126x.h"
#include "sx126x_hal_board.h"
#include "modbus.h"
#include "modbus-private.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
typedef struct {
  void *context;
} sx126x_device_t;

static sx126x_hal_board_t sx126x_board_ctx;
static sx126x_device_t sx126x_device;
static uint32_t hardware_id = 0;
static uint32_t tx_count = 0;
static volatile bool rx_packet_received = false;
static volatile bool tx_done_flag = false;
static uint8_t rx_payload_buf[256];
static uint16_t rx_payload_len = 0;

static modbus_t mb_ctx;
static modbus_mapping_t *mb_mapping = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void sx126x_transmit_packet(const uint8_t *payload, const uint8_t payload_len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void RxEn(void) {
  HAL_GPIO_WritePin(GPIOA, RXEN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, TXEN_Pin, GPIO_PIN_RESET);
}

static void TxEn(void) {
  HAL_GPIO_WritePin(GPIOA, RXEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, TXEN_Pin, GPIO_PIN_SET);
}

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
  const char mb_tx_hdr[] = "MODBUS TX: ";
  HAL_UART_Transmit(&huart1, (uint8_t *)mb_tx_hdr, sizeof(mb_tx_hdr) - 1, 100);
  for (int i = 0; i < req_length; i++) {
    char hex[4];
    snprintf(hex, sizeof(hex), "%02X ", req[i]);
    HAL_UART_Transmit(&huart1, (uint8_t *)hex, strlen(hex), 100);
  }
  HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 100);

  // Transmit Modbus response over LoRa
  sx126x_transmit_packet(req, (uint8_t)req_length);
  return req_length;
}

static int _modbus_embedded_check_integrity(modbus_t *ctx, uint8_t *msg, const int msg_length) {
  (void)ctx;
  if (msg_length < 4) {
    return -1;
  }
  uint16_t crc_calc = modbus_crc16(msg, msg_length - 2);
  uint16_t crc_recv = msg[msg_length - 2] | (msg[msg_length - 1] << 8);
  return (crc_calc == crc_recv) ? msg_length : -1;
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
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // Initialize SX126x HAL board context
  sx126x_hal_board_init(&sx126x_board_ctx, &hspi1);

  // Initialize SX126x device
  sx126x_device.context = &sx126x_board_ctx;
  sx126x_reset(sx126x_device.context);
  sx126x_wakeup(sx126x_device.context);

  // Configure radio for LoRa
  sx126x_set_standby(sx126x_device.context, SX126X_STANDBY_CFG_RC);
  sx126x_set_standby(sx126x_device.context, SX126X_STANDBY_CFG_XOSC);
  sx126x_set_reg_mode(sx126x_device.context, SX126X_REG_MODE_DCDC);
  sx126x_set_buffer_base_address(sx126x_device.context, 0x00, 0x00);
  
  sx126x_set_pkt_type(sx126x_device.context, SX126X_PKT_TYPE_LORA);
  sx126x_set_trimming_capacitor_values(sx126x_device.context, 0x4, 0x2f);

  // Set PA Config
  sx126x_pa_cfg_params_t pa_params = {
      .pa_duty_cycle = 0x04,
      .hp_max = 0x07,
      .device_sel = 0x00,
      .pa_lut = 0x01
  };
  sx126x_set_pa_cfg(sx126x_device.context, &pa_params);

  // Configure DIO1 interrupts for RX, TX Done and Timeout
  sx126x_set_dio_irq_params(sx126x_device.context, 
                            SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
                            SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
                            SX126X_IRQ_NONE, 
                            SX126X_IRQ_NONE);
  sx126x_clear_irq_status(sx126x_device.context, SX126X_IRQ_ALL);

  // Initialize configuration from flash or use defaults
  config_init();

  // Set RF frequency
  sx126x_set_rf_freq(sx126x_device.context, g_lora_config.frequency);

  // Set modulation parameters for LoRa
  sx126x_mod_params_lora_t mod_params = {.sf = g_lora_config.spreading_factor,
                                         .bw = g_lora_config.bandwidth,
                                         .cr = g_lora_config.coding_rate,
                                         .ldro = 0};
  sx126x_set_lora_mod_params(sx126x_device.context, &mod_params);

  // Set packet parameters for LoRa
  sx126x_pkt_params_lora_t pkt_params = {
      .preamble_len_in_symb = g_lora_config.preamble_length,
      .header_type = SX126X_LORA_PKT_EXPLICIT,
      .pld_len_in_bytes = 255,
      .crc_is_on = false,
      .invert_iq_is_on = false};
  sx126x_set_lora_pkt_params(sx126x_device.context, &pkt_params);

  // Set TX parameters
  sx126x_set_tx_params(sx126x_device.context, g_lora_config.tx_power,
                       SX126X_RAMP_3400_US);

  // Generate random hardware ID from SX126x internal RNG
  sx126x_get_random_numbers(sx126x_device.context, &hardware_id, 1);
  if (hardware_id == 0) {
    hardware_id = HAL_GetUIDw0() ^ HAL_GetTick();
  }

  // Initialize Modbus RTU / LoRa Context
  _modbus_init_common(&mb_ctx);
  mb_ctx.backend = &modbus_embedded_backend;
  modbus_set_slave(&mb_ctx, 1); // Default slave address 1

  // Initialize Modbus mapping (16 coils, 16 discrete inputs, 16 holding registers, 16 input registers)
  mb_mapping = modbus_mapping_new(16, 16, 16, 16);
  if (mb_mapping != NULL) {
    mb_mapping->tab_registers[0] = (uint16_t)(hardware_id >> 16);
    mb_mapping->tab_registers[1] = (uint16_t)(hardware_id & 0xFFFF);
    mb_mapping->tab_registers[2] = 0;
    mb_mapping->tab_registers[3] = 0;
  }

  // Transmit hardware ID over UART
  char init_msg[64];
  int init_len = snprintf(init_msg, sizeof(init_msg), "System Init - Hardware ID: 0x%08lX\r\n", (unsigned long)hardware_id);
  HAL_UART_Transmit(&huart1, (uint8_t *)init_msg, init_len, 100);

  // Start continuous receive
  RxEn();
  sx126x_set_rx(sx126x_device.context, 0); // Continuous RX
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char tx_payload[256];
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (rx_packet_received)
    {
      rx_packet_received = false;

      // Trim trailing newline from received message for clean reply formatting
      while (rx_payload_len > 0 && (rx_payload_buf[rx_payload_len - 1] == '\r' || rx_payload_buf[rx_payload_len - 1] == '\n'))
      {
        rx_payload_len--;
      // Update telemetry registers in Modbus mapping
      if (mb_mapping != NULL) {
        mb_mapping->tab_registers[2] = (uint16_t)(tx_count >> 16);
        mb_mapping->tab_registers[3] = (uint16_t)(tx_count & 0xFFFF);
      }

      tx_count++;
      // Formulate direct reply containing received message, Hardware ID, and count
      int pld_len = snprintf(tx_payload, sizeof(tx_payload), "REPLY [HW_ID: 0x%08lX | Count: %lu] -> %.*s\r\n",
                             (unsigned long)hardware_id, (unsigned long)tx_count,
                             (int)rx_payload_len, rx_payload_buf);
      // Check if received message is a Modbus RTU frame and pass it to libmodbus
      int mb_res = -1;
      if (rx_payload_len >= 4 && _modbus_embedded_check_integrity(&mb_ctx, rx_payload_buf, rx_payload_len) > 0) {
        // Send message to libmodbus for processing and replying
        mb_res = modbus_reply(&mb_ctx, rx_payload_buf, rx_payload_len, mb_mapping);
      }

      // Send reply payload directly to UART Transmit
      HAL_UART_Transmit(&huart1, (uint8_t *)tx_payload, pld_len, 100);
      if (mb_res > 0) {
        tx_count++;
        char mb_done_msg[64];
        int len = snprintf(mb_done_msg, sizeof(mb_done_msg), "MODBUS Reply Processed [Bytes: %d | Count: %lu]\r\n", mb_res, (unsigned long)tx_count);
        HAL_UART_Transmit(&huart1, (uint8_t *)mb_done_msg, len, 100);
      }
      else {
        // Fallback for non-Modbus message: text reply
        while (rx_payload_len > 0 && (rx_payload_buf[rx_payload_len - 1] == '\r' || rx_payload_buf[rx_payload_len - 1] == '\n'))
        {
          rx_payload_len--;
        }

      // Transmit reply packet back over LoRa (and automatically return to continuous RX)
      sx126x_transmit_packet((const uint8_t *)tx_payload, (uint8_t)pld_len);
        tx_count++;
        // Formulate direct reply containing received message, Hardware ID, and count
        int pld_len = snprintf(tx_payload, sizeof(tx_payload), "REPLY [HW_ID: 0x%08lX | Count: %lu] -> %.*s\r\n",
                               (unsigned long)hardware_id, (unsigned long)tx_count,
                               (int)rx_payload_len, rx_payload_buf);

        // Send reply payload directly to UART Transmit
        HAL_UART_Transmit(&huart1, (uint8_t *)tx_payload, pld_len, 100);

        // Transmit reply packet back over LoRa (and automatically return to continuous RX)
        sx126x_transmit_packet((const uint8_t *)tx_payload, (uint8_t)pld_len);
      }
    }
    HAL_Delay(5);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TXEN_Pin|RXEN_Pin|RST_Pin|NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DIO1_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TXEN_Pin RXEN_Pin RST_Pin NSS_Pin */
  GPIO_InitStruct.Pin = TXEN_Pin|RXEN_Pin|RST_Pin|NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUSY_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // DIO1 pin (PC15) - Input with interrupt
  GPIO_InitStruct.Pin = DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DIO1_GPIO_Port, &GPIO_InitStruct);

  // Enable EXTI interrupt for DIO1 (PC15 -> EXTI15_10_IRQn)
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == DIO1_Pin) // DIO1 interrupt (PC15)
  {
    sx126x_irq_mask_t irq_mask = SX126X_IRQ_NONE;

    // Read and clear all pending IRQs
    sx126x_get_and_clear_irq_status(sx126x_device.context, &irq_mask);

    if (irq_mask & SX126X_IRQ_TX_DONE) {
      tx_done_flag = true;
      // TX completed from task, optionally signal success
      char tx_done_msg[64];
      int len = snprintf(tx_done_msg, sizeof(tx_done_msg), "TX_DONE [HW_ID: 0x%08lX] | Count: %lu\r\n", (unsigned long)hardware_id, (unsigned long)tx_count);
      HAL_UART_Transmit(&huart1, (uint8_t *)tx_done_msg, len, 100);

      // Return to continuous RX after transmission with full RX capacity
      sx126x_pkt_params_lora_t rx_pkt_params = {
          .preamble_len_in_symb = g_lora_config.preamble_length,
          .header_type = SX126X_LORA_PKT_EXPLICIT,
          .pld_len_in_bytes = 255,
          .crc_is_on = false,
          .invert_iq_is_on = false};
      sx126x_set_lora_pkt_params(sx126x_device.context, &rx_pkt_params);

      RxEn();
      sx126x_set_rx(sx126x_device.context, 0);
    } else if (irq_mask & SX126X_IRQ_RX_DONE) {
      sx126x_rx_buffer_status_t rx_buffer_status;
      sx126x_pkt_status_lora_t pkt_status;

      // Get RX buffer status
      sx126x_get_rx_buffer_status(sx126x_device.context, &rx_buffer_status);

      // Get packet status
      sx126x_get_lora_pkt_status(sx126x_device.context, &pkt_status);

      uint16_t len_to_read = rx_buffer_status.pld_len_in_bytes;
      if (len_to_read > sizeof(rx_payload_buf)) {
        len_to_read = sizeof(rx_payload_buf);
      }
      rx_payload_len = len_to_read;

      // Read received data into buffer
      sx126x_read_buffer(sx126x_device.context,
                         rx_buffer_status.buffer_start_pointer, rx_payload_buf,
                         (uint8_t)len_to_read);

      // Echo received payload as ASCII over UART for debug
      const char rx_ascii_hdr[] = "RX ASCII: ";
      HAL_UART_Transmit(&huart1, (uint8_t *)rx_ascii_hdr, sizeof(rx_ascii_hdr) - 1, 100);
      HAL_UART_Transmit(&huart1, rx_payload_buf, len_to_read, 100);

      // Print received payload in HEX format
      const char rx_hex_hdr[] = "\r\nRX HEX: ";
      HAL_UART_Transmit(&huart1, (uint8_t *)rx_hex_hdr, sizeof(rx_hex_hdr) - 1, 100);
      char hex_chunk[64];
      int chunk_idx = 0;
      for (uint16_t i = 0; i < len_to_read; i++) {
        chunk_idx += snprintf(&hex_chunk[chunk_idx], sizeof(hex_chunk) - chunk_idx, "%02X ", rx_payload_buf[i]);
        if (chunk_idx >= (int)sizeof(hex_chunk) - 4 || i == len_to_read - 1) {
          HAL_UART_Transmit(&huart1, (uint8_t *)hex_chunk, chunk_idx, 100);
          chunk_idx = 0;
        }
      }

      char rx_done_msg[64];
      int len = snprintf(rx_done_msg, sizeof(rx_done_msg), "\r\nRX_DONE [HW_ID: 0x%08lX | Len: %u]\r\n", (unsigned long)hardware_id, (unsigned int)len_to_read);
      HAL_UART_Transmit(&huart1, (uint8_t *)rx_done_msg, len, 100);

      // Signal main loop that a packet was received and a reply transmission can occur
      rx_packet_received = true;

      // Return to continuous RX
      RxEn();
      sx126x_set_rx(sx126x_device.context, 0);
    }
  }
}

static void sx126x_transmit_packet(const uint8_t *payload,
                                   const uint8_t payload_len) {
  // Force radio to standby before changing buffer/state
  sx126x_set_standby(sx126x_device.context, SX126X_STANDBY_CFG_RC);

  // Clear pending interrupts and reset TX flag
  sx126x_clear_irq_status(sx126x_device.context, SX126X_IRQ_ALL);
  tx_done_flag = false;

  // Set TX packet parameters matching actual payload length
  sx126x_pkt_params_lora_t tx_pkt_params = {
      .preamble_len_in_symb = g_lora_config.preamble_length,
      .header_type = SX126X_LORA_PKT_EXPLICIT,
      .pld_len_in_bytes = payload_len,
      .crc_is_on = false,
      .invert_iq_is_on = false};
  sx126x_set_lora_pkt_params(sx126x_device.context, &tx_pkt_params);

  // Write payload into TX buffer
  sx126x_write_buffer(sx126x_device.context, 0x00, payload, payload_len);

  // Start TX with timeout (ms)
  TxEn();
  sx126x_set_tx(sx126x_device.context, 5000);

  // Wait until TX is completed via interrupt or fallback timeout
  for (int i = 0; i < 500 && !tx_done_flag; ++i) {
    HAL_Delay(5);
  }

  // Restore RX packet parameters and return to continuous RX mode
  sx126x_pkt_params_lora_t rx_pkt_params = {
      .preamble_len_in_symb = g_lora_config.preamble_length,
      .header_type = SX126X_LORA_PKT_EXPLICIT,
      .pld_len_in_bytes = 255,
      .crc_is_on = false,
      .invert_iq_is_on = false};
  sx126x_set_lora_pkt_params(sx126x_device.context, &rx_pkt_params);

  RxEn();
  sx126x_set_rx(sx126x_device.context, 0);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
