/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - STM32WL LoRa SoC Modbus RTU Node
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "config.h"
#include "sx126x.h"
#include "sx126x_hal.h"
#include "sx126x_hal_board.h"
#include "modbus_app.h"
#include "uart_app.h"
#include "ssd1306.h"
#include "lora_security.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  void *context;
} sx126x_device_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
static sx126x_hal_board_t sx126x_board_ctx;
static sx126x_device_t sx126x_device;
static uint32_t hardware_id = 0;
static uint32_t tx_count = 0;
static volatile bool rx_packet_received = false;
static volatile bool tx_done_flag = false;
static uint8_t rx_payload_buf[256];
static uint16_t rx_payload_len = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
static void sx126x_transmit_packet(const uint8_t *payload, const uint8_t payload_len);
static void update_oled_display(const char *status_msg);
static void on_valve_state_changed(uint8_t valve_idx, bool is_open);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void update_oled_display(const char *status_msg) {
  ssd1306_fill(SSD1306_COLOR_BLACK);

  // Header banner
  ssd1306_draw_filled_rectangle(0, 0, 128, 12, SSD1306_COLOR_WHITE);
  ssd1306_set_cursor(6, 1);
  ssd1306_write_string("STM32WL LoRa SoC", Font_7x10, SSD1306_COLOR_BLACK);

  char buf[32];
  // Slave ID and Tx count
  ssd1306_set_cursor(0, 15);
  snprintf(buf, sizeof(buf), "Slave:%u  Tx:%lu", (unsigned int)g_lora_config.slave_id, (unsigned long)tx_count);
  ssd1306_write_string(buf, Font_7x10, SSD1306_COLOR_WHITE);

  // Hardware ID
  ssd1306_set_cursor(0, 27);
  snprintf(buf, sizeof(buf), "HW:0x%08lX", (unsigned long)hardware_id);
  ssd1306_write_string(buf, Font_7x10, SSD1306_COLOR_WHITE);

  // Valve status and Security state
  bool v1 = modbus_app_get_valve_state(0);
  bool v2 = modbus_app_get_valve_state(1);
  ssd1306_set_cursor(0, 39);
  snprintf(buf, sizeof(buf), "V1:%-3s V2:%-3s %s",
           v1 ? "ON" : "OFF", v2 ? "ON" : "OFF",
           g_lora_config.security_enabled ? "SEC" : "RAW");
  ssd1306_write_string(buf, Font_7x10, SSD1306_COLOR_WHITE);

  // Status message
  ssd1306_set_cursor(0, 51);
  snprintf(buf, sizeof(buf), "%-18s", (status_msg != NULL) ? status_msg : "Listening...");
  ssd1306_write_string(buf, Font_7x10, SSD1306_COLOR_WHITE);

  ssd1306_update_screen();
}

static void on_valve_state_changed(uint8_t valve_idx, bool is_open) {
  uart_printf("[VALVE ACTION] Valve %u is now %s\r\n",
              valve_idx + 1, is_open ? "OPEN (ON)" : "CLOSED (OFF)");

  char status[24];
  snprintf(status, sizeof(status), "V%u -> %s", valve_idx + 1, is_open ? "ON" : "OFF");
  update_oled_display(status);
}

void sx126x_radio_irq_callback(void) {
  sx126x_irq_mask_t irq_mask = SX126X_IRQ_NONE;

  // Read and clear all pending IRQs
  sx126x_get_and_clear_irq_status(sx126x_device.context, &irq_mask);

  if (irq_mask & SX126X_IRQ_TX_DONE) {
    tx_done_flag = true;
    uart_printf("TX_DONE [HW_ID: 0x%08lX] | Count: %lu\r\n", (unsigned long)hardware_id, (unsigned long)tx_count);

    // Return to continuous RX after transmission with full RX capacity
    sx126x_pkt_params_lora_t rx_pkt_params = {
        .preamble_len_in_symb = g_lora_config.preamble_length,
        .header_type = SX126X_LORA_PKT_EXPLICIT,
        .pld_len_in_bytes = 255,
        .crc_is_on = false,
        .invert_iq_is_on = false};
    sx126x_set_lora_pkt_params(sx126x_device.context, &rx_pkt_params);

    sx126x_hal_rf_switch_rx();
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

    // Print received payload in HEX format
    uart_print_hex("RX HEX: ", rx_payload_buf, len_to_read);
    uart_printf("RX_DONE [HW_ID: 0x%08lX | Len: %u | RSSI: %d dBm | SNR: %d dB]\r\n",
                (unsigned long)hardware_id, (unsigned int)len_to_read,
                (int)pkt_status.rssi_pkt_in_dbm, (int)pkt_status.snr_pkt_in_db);

    // Signal main loop that a packet was received
    rx_packet_received = true;

    // Return to continuous RX
    sx126x_hal_rf_switch_rx();
    sx126x_set_rx(sx126x_device.context, 0);
  }
}

static void sx126x_transmit_packet(const uint8_t *payload,
                                   const uint8_t payload_len) {
  const uint8_t *tx_data = payload;
  uint8_t tx_len = payload_len;
  uint8_t sec_buf[256];
  uint16_t sec_len = 0;

  if (g_lora_config.security_enabled) {
    if (lora_security_encrypt(payload, payload_len, sec_buf, &sec_len)) {
      tx_data = sec_buf;
      tx_len = (uint8_t)sec_len;
      uart_printf("[SECURITY] Encrypted TX frame (%u -> %u bytes)\r\n", payload_len, tx_len);
    }
  }

  // Force radio to standby before changing buffer/state
  sx126x_set_standby(sx126x_device.context, SX126X_STANDBY_CFG_RC);

  // Clear pending interrupts and reset TX flag
  sx126x_clear_irq_status(sx126x_device.context, SX126X_IRQ_ALL);
  tx_done_flag = false;

  // Set TX packet parameters matching actual payload length
  sx126x_pkt_params_lora_t tx_pkt_params = {
      .preamble_len_in_symb = g_lora_config.preamble_length,
      .header_type = SX126X_LORA_PKT_EXPLICIT,
      .pld_len_in_bytes = tx_len,
      .crc_is_on = false,
      .invert_iq_is_on = false};
  sx126x_set_lora_pkt_params(sx126x_device.context, &tx_pkt_params);

  // Write payload into TX buffer
  sx126x_write_buffer(sx126x_device.context, 0x00, tx_data, tx_len);

  // Enable RF switch for TX and start transmission
  sx126x_hal_rf_switch_tx();
  sx126x_set_tx(sx126x_device.context, 5000);

  // Wait until TX is completed via interrupt or timeout
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

  sx126x_hal_rf_switch_rx();
  sx126x_set_rx(sx126x_device.context, 0);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */
  uart_app_init(&huart2);
  ssd1306_init(&hi2c2);
  update_oled_display("Booting...");

  // Initialize SX126x HAL board context for STM32WL SUBGHZ
  sx126x_hal_board_init(&sx126x_board_ctx);

  // Initialize SX126x device context
  sx126x_device.context = &sx126x_board_ctx;
  sx126x_reset(sx126x_device.context);
  sx126x_wakeup(sx126x_device.context);

  // Configure radio for LoRa
  sx126x_set_standby(sx126x_device.context, SX126X_STANDBY_CFG_RC);
  sx126x_set_reg_mode(sx126x_device.context, SX126X_REG_MODE_DCDC);
  sx126x_set_buffer_base_address(sx126x_device.context, 0x00, 0x00);

  // Enable internal DIO2 as RF switch control
  sx126x_set_dio2_as_rf_sw_ctrl(sx126x_device.context, true);

  sx126x_set_pkt_type(sx126x_device.context, SX126X_PKT_TYPE_LORA);

  // Set PA Config for STM32WL
  sx126x_pa_cfg_params_t pa_params = {
      .pa_duty_cycle = 0x04,
      .hp_max = 0x07,
      .device_sel = 0x00,
      .pa_lut = 0x01
  };
  sx126x_set_pa_cfg(sx126x_device.context, &pa_params);

  // Configure interrupts for RX, TX Done and Timeout
  sx126x_set_dio_irq_params(sx126x_device.context, 
                            SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
                            SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
                            SX126X_IRQ_NONE, 
                            SX126X_IRQ_NONE);
  sx126x_clear_irq_status(sx126x_device.context, SX126X_IRQ_ALL);

  // Initialize configuration from flash or use defaults
  config_init();

  // Initialize LoRa Security (AES-128 Payload Encryption)
  lora_security_init(g_lora_config.aes_key);
  uart_printf("LoRa Security (AES-128): %s\r\n",
              g_lora_config.security_enabled ? "ENABLED" : "DISABLED");

  // Set RF frequency
  sx126x_set_rf_freq(sx126x_device.context, g_lora_config.frequency);

  // Set modulation parameters for LoRa
  sx126x_mod_params_lora_t mod_params = {
      .sf = g_lora_config.spreading_factor,
      .bw = g_lora_config.bandwidth,
      .cr = g_lora_config.coding_rate,
      .ldro = 0
  };
  sx126x_set_lora_mod_params(sx126x_device.context, &mod_params);

  // Set packet parameters for LoRa
  sx126x_pkt_params_lora_t pkt_params = {
      .preamble_len_in_symb = g_lora_config.preamble_length,
      .header_type = SX126X_LORA_PKT_EXPLICIT,
      .pld_len_in_bytes = 255,
      .crc_is_on = false,
      .invert_iq_is_on = false
  };
  sx126x_set_lora_pkt_params(sx126x_device.context, &pkt_params);

  // Set TX parameters
  sx126x_set_tx_params(sx126x_device.context, g_lora_config.tx_power,
                       SX126X_RAMP_3400_US);

  // Generate hardware ID from internal RNG / UID
  sx126x_get_random_numbers(sx126x_device.context, &hardware_id, 1);
  if (hardware_id == 0) {
    hardware_id = HAL_GetUIDw0() ^ HAL_GetTick();
  }

  // Initialize Modbus RTU module with LoRa transmission callback and configured Slave ID
  modbus_app_init(g_lora_config.slave_id, hardware_id, sx126x_transmit_packet);
  modbus_app_set_valve_callback(on_valve_state_changed);

  // Default status of valves are 0 (Closed) and sensors are 0
  modbus_app_set_valve_state(0, false);
  modbus_app_set_valve_state(1, false);
  modbus_app_set_sensor_values(0, 0);

  // Transmit hardware ID and Modbus Slave ID over UART
  uart_printf("STM32WL LoRa SoC Init - Hardware ID: 0x%08lX | Modbus Slave ID: %u\r\n",
              (unsigned long)hardware_id, (unsigned int)g_lora_config.slave_id);

  /* Boot CPU2 (CM0PLUS) if dual-core active */
  HAL_PWREx_ReleaseCore(PWR_CORE_CPU2);

  // Start continuous receive
  sx126x_hal_rf_switch_rx();
  sx126x_set_rx(sx126x_device.context, 0); // Continuous RX
  update_oled_display("Listening (RX)");
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

      uint8_t plain_buf[256];
      uint16_t plain_len = 0;
      const uint8_t *proc_buf = rx_payload_buf;
      uint16_t proc_len = rx_payload_len;

      if (lora_security_is_encrypted(rx_payload_buf, rx_payload_len))
      {
        if (lora_security_decrypt(rx_payload_buf, rx_payload_len, plain_buf, &plain_len))
        {
          proc_buf = plain_buf;
          proc_len = plain_len;
          uart_printf("[SECURITY] Decrypted %u bytes (MIC verified)\r\n", plain_len);
          uart_print_hex("DECRYPTED: ", proc_buf, proc_len);
        }
        else
        {
          uart_printf("[SECURITY] Decryption FAILED (corrupted packet or invalid key)\r\n");
          update_oled_display("SEC Auth Fail");
          continue;
        }
      }
      else if (g_lora_config.security_enabled)
      {
        uart_printf("[SECURITY] Warning: Plaintext packet received while security is enabled\r\n");
      }

      // Update runtime telemetry in Modbus holding registers
      modbus_app_update_telemetry(tx_count);

      // Process received packet via Modbus handler
      int mb_res = modbus_app_process_packet((uint8_t *)proc_buf, proc_len);
      if (mb_res > 0)
      {
        tx_count++;
        update_oled_display("Modbus Replied");
      }
      else if (mb_res == 0)
      {
        // Valid Modbus packet addressed to another slave or broadcast: do not transmit reply
        update_oled_display("Modbus Ignored");
      }
      else
      {
        // Fallback for non-Modbus message: text reply
        while (proc_len > 0 && (proc_buf[proc_len - 1] == '\r' || proc_buf[proc_len - 1] == '\n'))
        {
          proc_len--;
        }

        tx_count++;
        int pld_len = snprintf(tx_payload, sizeof(tx_payload), "REPLY [HW_ID: 0x%08lX | Count: %lu] -> %.*s\r\n",
                               (unsigned long)hardware_id, (unsigned long)tx_count,
                               (int)proc_len, proc_buf);

        uart_send((const uint8_t *)tx_payload, pld_len);
        sx126x_transmit_packet((const uint8_t *)tx_payload, (uint8_t)pld_len);
        update_oled_display("Text Replied");
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00707CBB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
