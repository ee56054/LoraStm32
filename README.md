# STM32WL LoRa SoC Modbus RTU Node

An embedded firmware project for **STM32WL54CCU7 / STM32WL55** that implements a **Modbus RTU Slave over LoRa** using the **integrated on-chip Sub-GHz radio SoC** (`SUBGHZSPI`) and an embedded-tailored **libmodbus** stack.

---

## Features

- **Integrated LoRa SoC (No External Module)**:
  - Drives the internal Semtech SX126x radio core via the dedicated on-chip `SUBGHZSPI` peripheral.
  - Internal hardware NSS and RFBUSYS control via Power Controller (`PWR`) registers.
  - Internal hardware reset via Reset & Clock Controller (`RCC`) registers.
  - Internal interrupt handling via **EXTI Line 44** (`SUBGHZ_Radio_IRQHandler`).
  - Automatic RF switch control via internal `DIO2`.
  - Zero external SPI or radio wiring needed!
- **Modbus RTU over LoRa**:
  - Embedded [libmodbus](https://github.com/stephane/libmodbus) stack running bare-metal on Cortex-M4.
  - Complete CRC16 validation and Slave ID filtering.
- **2 Valves Control & Feedback**:
  - Controlled via Modbus Coils (FC 01, 05, 15) and Holding Registers (FC 03, 06, 16).
  - Status feedback via Discrete Inputs (FC 02).
  - Callback hook (`modbus_valve_callback_t`) to drive physical outputs/relays.
- **2 Sensors Support**:
  - Measurement readings via Input Registers (FC 04) and mirrored Holding Registers (FC 03).
- **Zero-Heap Binary Flash Storage (No cJSON)**:
  - Configurable Slave ID (1–247) via Holding Register 8.
  - Automatically saved to MCU Flash memory (Page 63, `0x0801F800`) using 64-bit doubleword programming.
  - Pure binary struct with magic word (`0x4C524346`) and checksum — zero dynamic heap allocation (`malloc`/`free`) and ~19 KB smaller than JSON parsing.
- **OLED Display (SSD1306)**:
  - 128x64 display over I2C2 showing live Slave ID, Tx count, HW ID, valve states, and radio status.
- **UART Debug Logger**:
  - Formatted logging, hex dumps, and event reporting via USART2 (115200 baud).

---

## Modbus Address Map

**Default Slave ID:** `1` (Configurable 1–247 via Holding Register 8)

### Coils (0x / Read-Write)
| Address | Name | Function Codes | Description |
| :---: | :--- | :---: | :--- |
| `0x0000` (0) | **Valve 1** | `01`, `05`, `15` | `0` = Closed / OFF, `1` = Open / ON |
| `0x0001` (1) | **Valve 2** | `01`, `05`, `15` | `0` = Closed / OFF, `1` = Open / ON |

### Discrete Inputs (1x / Read-Only)
| Address | Name | Function Codes | Description |
| :---: | :--- | :---: | :--- |
| `0x0000` (0) | **Valve 1 Status** | `02` | `0` = Closed, `1` = Open |
| `0x0001` (1) | **Valve 2 Status** | `02` | `0` = Closed, `1` = Open |

### Input Registers (3x / Read-Only)
| Address | Name | Function Codes | Description |
| :---: | :--- | :---: | :--- |
| `0x0000` (0) | **Sensor 1** | `04` | 16-bit sensor reading (Default: `0`) |
| `0x0001` (1) | **Sensor 2** | `04` | 16-bit sensor reading (Default: `0`) |

### Holding Registers (4x / Read-Write)
| Address | Name | Access | Function Codes | Description |
| :---: | :--- | :---: | :---: | :--- |
| `0x0000` (0) | **HW ID High** | RO | `03` | Device Hardware ID bits [31..16] |
| `0x0001` (1) | **HW ID Low** | RO | `03` | Device Hardware ID bits [15..0] |
| `0x0002` (2) | **TX Count High** | RO | `03` | LoRa packet counter bits [31..16] |
| `0x0003` (3) | **TX Count Low** | RO | `03` | LoRa packet counter bits [15..0] |
| `0x0004` (4) | **Valve 1 Register** | RW | `03`, `06`, `16` | Mirrored Valve 1 (`0` = Closed, `1` = Open) |
| `0x0005` (5) | **Valve 2 Register** | RW | `03`, `06`, `16` | Mirrored Valve 2 (`0` = Closed, `1` = Open) |
| `0x0006` (6) | **Sensor 1 Register**| RO | `03` | Mirrored Sensor 1 (Default: `0`) |
| `0x0007` (7) | **Sensor 2 Register**| RO | `03` | Mirrored Sensor 2 (Default: `0`) |
| `0x0008` (8) | **Slave ID** | RW | `03`, `06`, `16` | **Modbus Slave Address (1..247)**; writing updates ID and persists to Flash |

---

## Hardware Architecture & Pinout

### On-Chip Sub-GHz Radio SoC
All radio communication is routed internally inside the STM32WL package:

| Internal Signal | Peripheral / Register | Description |
| :--- | :--- | :--- |
| **SPI Interface** | Dedicated `SUBGHZSPI` | On-chip high-speed SPI bus |
| **NSS / Chip Select** | `PWR->CR3` (`SUBGHZSPINSS`) | Managed by `LL_PWR_SelectSUBGHZSPI_NSS()` |
| **Busy Indicator** | `PWR->SR2` (`RFBUSYS`) | Read by `LL_PWR_IsActiveFlag_RFBUSYS()` |
| **Reset** | `RCC->CSR` (`RFRST`) | Toggled by `LL_RCC_RF_EnableReset()` / `DisableReset()` |
| **Radio IRQ** | `EXTI Line 44` / NVIC Vector 80 | Handled by `SUBGHZ_Radio_IRQHandler()` |
| **RF Switch** | Internal DIO2 control | Driven by `sx126x_set_dio2_as_rf_sw_ctrl()` |

### External Board Peripherals
| Pin | Peripheral | Signal | Description |
| :--- | :--- | :--- | :--- |
| **PA2** | USART2 | TX | UART Debug Logger (115200 8N1) |
| **PA3** | USART2 | RX | UART RX |
| **PA11** | I2C2 | SDA | SSD1306 0.96" OLED Display Data |
| **PA12** | I2C2 | SCL | SSD1306 0.96" OLED Display Clock |

---

## Project Structure

```
LoraStm32/
├── CMakeLists.txt              # Root multi-context CMake orchestration
├── mx-generated.cmake          # Dual-core build definitions (ExternalProject)
├── LORA.ioc                    # STM32CubeMX dual-core project file
├── cmake/
│   └── gcc-arm-none-eabi.cmake # Toolchain configuration (-Os -g3)
├── CM4/                        # Cortex-M4 Application Core
│   ├── CMakeLists.txt          # CM4 CMake build configuration
│   ├── STM32WL54XX_FLASH_CM4.ld# Linker script for CM4 (128 KB Flash)
│   └── Core/
│       ├── Inc/
│       │   ├── config.h        # Binary Flash storage definitions (Magic 0x4C524346)
│       │   ├── modbus_app.h    # Modbus RTU slave API & table addresses
│       │   ├── ssd1306.h       # SSD1306 OLED I2C driver
│       │   ├── ssd1306_fonts.h # Font tables (7x10, 11x18)
│       │   ├── sx126x_hal_board.h # STM32WL internal SUBGHZ HAL header
│       │   ├── uart_app.h      # Serial print, logging & hex dumps
│       │   └── main.h          # Hardware defines and prototypes
│       └── Src/
│           ├── config.c        # Direct 64-bit doubleword Flash load/save
│           ├── main.c          # Application main loop, LoRa task & display
│           ├── modbus_app.c    # Modbus RTU protocol engine & CRC16 table
│           ├── ssd1306.c       # OLED display implementation
│           ├── ssd1306_fonts.c # Font bitmap tables
│           ├── sx126x_hal.c    # Internal SUBGHZSPI & register HAL
│           ├── uart_app.c      # UART logger implementation
│           └── stm32wlxx_hal_msp.c # MSP init for USART2 and I2C2
├── CM0PLUS/                    # Cortex-M0+ Secondary Core
│   ├── CMakeLists.txt
│   └── Core/
├── Common/                     # Shared System Files
│   └── System/
│       └── system_stm32wlxx.c
└── Drivers/
    ├── STM32WLxx_HAL_Driver/   # STM32WL HAL & LL drivers (subghz, pwr, rcc...)
    ├── CMSIS/                  # ARM CMSIS Cortex-M libraries
    ├── sx126x_driver/          # Semtech SX126x radio driver
    └── libmodbus/              # Bare-metal embedded libmodbus library
```

---

## Building the Project

### Prerequisites
- **STM32Cube for VS Code Extension** (or standard CMake 3.22+ & Ninja)
- **Arm GNU Toolchain** (`arm-none-eabi-gcc` 14.3+)

### 1. Build via `cube-cmake` (Recommended / VS Code)
```bash
# Configure
cube-cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -S . -B build/Debug -G Ninja

# Build
cube-cmake --build build/Debug
```

### 2. Build Outputs
Binaries are generated inside each core's build directory:
- **CM4**:
  - `CM4/build/Lora_CM4.elf`
  - `CM4/build/Lora_CM4.hex`
  - `CM4/build/Lora_CM4.bin`
- **CM0PLUS**:
  - `CM0PLUS/build/Lora_CM0PLUS.elf`

### 3. Flash to Target
Use **STM32CubeProgrammer** or **STM32_Programmer_CLI**:
```bash
STM32_Programmer_CLI -c port=SWD -w CM4/build/Lora_CM4.bin 0x08000000 -v -rst
```

---

## Flash Memory Storage

Configuration parameters are stored in non-volatile Flash memory on **Page 63 (`0x0801F800`)**:
- **Format**: Packed binary structure (`flash_storage_record_t`) aligned to 64-bit doublewords.
- **Validation**: 32-bit Magic Word (`0x4C524346`), Version (`1`), and 32-bit integrity checksum.
- **Factory Reset**: If Page 63 is unwritten or erased (`0xFFFFFFFF`), factory defaults are loaded automatically (Frequency: 915 MHz, SF7, BW 125 kHz, CR 4/5, Slave ID: 1).

---

## License

- **Semtech SX126x Driver**: Clear BSD License
- **libmodbus**: LGPL-2.1-or-later (Embedded adaptation)
- **STMicroelectronics HAL**: BSD-3-Clause License
