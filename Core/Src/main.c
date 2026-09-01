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
#include "config.h"
#include "sx126x.h"
#include "sx126x_hal_board.h"
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
      .pld_len_in_bytes = 16,
      .crc_is_on = false,
      .invert_iq_is_on = false};
  sx126x_set_lora_pkt_params(sx126x_device.context, &pkt_params);

  // Set TX parameters
  sx126x_set_tx_params(sx126x_device.context, g_lora_config.tx_power,
                       SX126X_RAMP_3400_US);

  // Start continuous receive
  RxEn();
  sx126x_set_rx(sx126x_device.context, 0); // Continuous RX
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  const uint8_t tx_payload[] = "Hello from STM32 LoRa\r\n";
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    sx126x_transmit_packet(tx_payload, sizeof(tx_payload) - 1);
    HAL_Delay(1000);
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
      // TX completed from task, optionally signal success
      const uint8_t tx_done_msg[] = "TX_DONE\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t *)tx_done_msg,
                        sizeof(tx_done_msg) - 1, 100);

      // Return to continuous RX after transmission
      RxEn();
      sx126x_set_rx(sx126x_device.context, 0);
    } else if (irq_mask & SX126X_IRQ_RX_DONE) {
      sx126x_rx_buffer_status_t rx_buffer_status;
      sx126x_pkt_status_lora_t pkt_status;
      uint8_t rx_data[16]; // Buffer for received data

      // Get RX buffer status
      sx126x_get_rx_buffer_status(sx126x_device.context, &rx_buffer_status);

      // Get packet status
      sx126x_get_lora_pkt_status(sx126x_device.context, &pkt_status);

      // Read received data
      sx126x_read_buffer(sx126x_device.context,
                         rx_buffer_status.buffer_start_pointer, rx_data,
                         rx_buffer_status.pld_len_in_bytes);

      // Optional: echo received payload over UART for debug
      HAL_UART_Transmit(&huart1, rx_data, rx_buffer_status.pld_len_in_bytes,
                        100);
      const uint8_t rx_done_msg[] = "\r\nRX_DONE\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t *)rx_done_msg,
                        sizeof(rx_done_msg) - 1, 100);

      // Return to continuous RX
      RxEn();
      sx126x_set_rx(sx126x_device.context, 0);
    }
  }
}

static void sx126x_transmit_packet(const uint8_t *payload,
                                   const uint8_t payload_len) {
  sx126x_irq_mask_t irq_mask = SX126X_IRQ_NONE;

  // Force radio to standby before changing buffer/state
  sx126x_set_standby(sx126x_device.context, SX126X_STANDBY_CFG_RC);

  // Clear pending interrupts
  sx126x_clear_irq_status(sx126x_device.context, SX126X_IRQ_ALL);

  // Write payload into TX buffer
  sx126x_write_buffer(sx126x_device.context, 0x00, payload, payload_len);

  // Start TX with a generous timeout (ms)
  TxEn();
  sx126x_set_tx(sx126x_device.context, 5000);

  // Block until TX_DONE or timeout interrupt
  for (int i = 0; i < 500; ++i) {
    sx126x_get_irq_status(sx126x_device.context, &irq_mask);

    if (irq_mask & SX126X_IRQ_TX_DONE) {
      sx126x_clear_irq_status(sx126x_device.context, SX126X_IRQ_TX_DONE);
      break;
    }
    if (irq_mask & SX126X_IRQ_TIMEOUT) {
      sx126x_clear_irq_status(sx126x_device.context, SX126X_IRQ_TIMEOUT);
      break;
    }

    HAL_Delay(10);
  }

  // Go back to RX mode after transmit
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
