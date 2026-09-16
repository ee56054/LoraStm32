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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "gpio.h"
#include "spi.h"
#include "i2c.h"
#include "usb_device.h"
#include "config.h"
#include "sx126x.h"
#include "sx126x_hal_board.h"
#include "modbus_app.h"
#include "uart_app.h"
#include "ssd1306.h"
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void sx126x_transmit_packet(const uint8_t *payload, const uint8_t payload_len);
static void update_oled_display(const char *status_msg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void RxEn(void) {
  HAL_GPIO_WritePin(E22_TXEN_GPIO_Port, E22_TXEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(E22_RXEN_GPIO_Port, E22_RXEN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_TX_GPIO_Port, LED_TX_Pin, GPIO_PIN_SET);
}

static void TxEn(void) {
  HAL_GPIO_WritePin(E22_RXEN_GPIO_Port, E22_RXEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(E22_TXEN_GPIO_Port, E22_TXEN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_TX_GPIO_Port, LED_TX_Pin, GPIO_PIN_RESET);
}

static void update_oled_display(const char *status_msg) {
  ssd1306_fill(SSD1306_COLOR_BLACK);

  // Header banner
  ssd1306_draw_filled_rectangle(0, 0, 128, 12, SSD1306_COLOR_WHITE);
  ssd1306_set_cursor(10, 1);
  ssd1306_write_string("LoRa STM32 Node", Font_7x10, SSD1306_COLOR_BLACK);

  char buf[32];
  // Slave ID and Tx count
  ssd1306_set_cursor(0, 15);
  snprintf(buf, sizeof(buf), "Slave:%u  Tx:%lu", (unsigned int)g_lora_config.slave_id, (unsigned long)tx_count);
  ssd1306_write_string(buf, Font_7x10, SSD1306_COLOR_WHITE);

  // Hardware ID
  ssd1306_set_cursor(0, 27);
  snprintf(buf, sizeof(buf), "HW:0x%08lX", (unsigned long)hardware_id);
  ssd1306_write_string(buf, Font_7x10, SSD1306_COLOR_WHITE);

  // Valve status
  bool v1 = modbus_app_get_valve_state(0);
  bool v2 = modbus_app_get_valve_state(1);
  ssd1306_set_cursor(0, 39);
  snprintf(buf, sizeof(buf), "V1:%-3s  V2:%-3s", v1 ? "ON" : "OFF", v2 ? "ON" : "OFF");
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
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  uart_app_init();
  ssd1306_init(&hi2c2);
  update_oled_display("Booting...");

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

  // Initialize Modbus RTU module with LoRa transmission callback and configured Slave ID
  modbus_app_init(g_lora_config.slave_id, hardware_id, sx126x_transmit_packet);
  modbus_app_set_valve_callback(on_valve_state_changed);

  // Default status of valves are 0 (Closed) and sensors are 0
  modbus_app_set_valve_state(0, false);
  modbus_app_set_valve_state(1, false);
  modbus_app_set_sensor_values(0, 0);

  // Transmit hardware ID and Modbus Slave ID over UART
  uart_printf("System Init - Hardware ID: 0x%08lX | Modbus Slave ID: %u\r\n",
              (unsigned long)hardware_id, (unsigned int)g_lora_config.slave_id);

  // Start continuous receive
  RxEn();
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

      // Update runtime telemetry in Modbus holding registers
      modbus_app_update_telemetry(tx_count);

      // Process received packet via Modbus handler
      int mb_res = modbus_app_process_packet(rx_payload_buf, rx_payload_len);
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
        while (rx_payload_len > 0 && (rx_payload_buf[rx_payload_len - 1] == '\r' || rx_payload_buf[rx_payload_len - 1] == '\n'))
        {
          rx_payload_len--;
        }

        tx_count++;
        // Formulate direct reply containing received message, Hardware ID, and count
        int pld_len = snprintf(tx_payload, sizeof(tx_payload), "REPLY [HW_ID: 0x%08lX | Count: %lu] -> %.*s\r\n",
                               (unsigned long)hardware_id, (unsigned long)tx_count,
                               (int)rx_payload_len, rx_payload_buf);

        // Send reply payload directly to UART Transmit
        uart_send((const uint8_t *)tx_payload, pld_len);

        // Transmit reply packet back over LoRa (and automatically return to continuous RX)
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == E22_DIO1_Pin) // E22 DIO1 interrupt (PA3 / EXTI3)
  {
    sx126x_irq_mask_t irq_mask = SX126X_IRQ_NONE;

    // Read and clear all pending IRQs
    sx126x_get_and_clear_irq_status(sx126x_device.context, &irq_mask);

    if (irq_mask & SX126X_IRQ_TX_DONE) {
      tx_done_flag = true;
      // TX completed from task, optionally signal success
      uart_printf("TX_DONE [HW_ID: 0x%08lX] | Count: %lu\r\n", (unsigned long)hardware_id, (unsigned long)tx_count);

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

      // Print received payload in HEX format
      uart_print_hex("RX HEX: ", rx_payload_buf, len_to_read);
      uart_printf("RX_DONE [HW_ID: 0x%08lX | Len: %u]\r\n", (unsigned long)hardware_id, (unsigned int)len_to_read);

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
