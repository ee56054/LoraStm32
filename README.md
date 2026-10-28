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
| `0x0009` (9) | **Security Enable** | RW | `03`, `06`, `16` | **Payload Encryption (0 = Raw Plaintext, 1 = AES-128)**; persists to Flash |
| `0x000A`–`0x0011` (10–17) | **AES-128 Key** | WO | `16` | **128-bit Encryption Key (8 registers / 16 bytes)**; Write-only (returns 0 on read for security); updates active key and persists to Flash |

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
| **PA11** | I2C2 | SDA | SSD1306 / SH1106 OLED Data |
| **PA12** | I2C2 | SCL | SSD1306 / SH1106 OLED Clock |
| **PA5** | GPIO (Pull-Up) | Input | EC11 Encoder Phase A (CLK) |
| **PA6** | GPIO (Pull-Up) | Input | EC11 Encoder Phase B (DT) |
| **PA7** | GPIO (Pull-Up) | Input | **CONFIRM Button** (EC11 Knob Push / Select / Save) |
| **PA9** | GPIO (Pull-Up) | Input | **BACK Button** (Cancel / Return to Previous Menu or Dashboard) |

---

## EC11 Interactive On-Screen Menu

The node features an interactive UI on the SSD1306/SH1106 OLED display using the EC11 rotary knob and dedicated navigation buttons (`PA5`, `PA6`, `PA7`, `PA9`):

### Navigation & Control
- **Rotate Knob (`PA5` / `PA6`)**:
  - Status Dashboard & Node Info: Scroll status lines smoothly (1 value per line).
  - Menus: Move cursor `>` up and down across options.
  - Edit Mode: Adjust parameter values.
- **Confirm Button (`PA7`)**:
  - Status Dashboard: Enter Main Menu.
  - Main Menu: Select highlighted item / enter submenu.
  - Edit Mode: Save modifications immediately to hardware and commit to **Flash Page 63**.
  - Hex Byte Editor: Click advances to next byte; Hold (> 800ms) commits full 16-byte key.
- **Back Button (`PA9`)**:
  - **Exclusively controls all "Go Back" navigation**:
    - From Submenus (*LoRa Config*, *Sec Key*, *Node Info*): Return to Main Menu.
    - From Main Menu: Return to live Status Dashboard.
    - From Info / Key View screens: Return to parent menu.
    - From Edit Modes: Cancel and discard modifications without saving.

### Menu Structure
```
[ Status Dashboard ] (Live RX/TX, valves, telemetry)
       |
       +---> [ Main Menu ] (Press PA9 to return to Dashboard)
                ├── Valve 1: [OPEN / CLOSED]
                ├── Valve 2: [OPEN / CLOSED]
                ├── Sec: [AES-128 / RAW]
                ├── Sec Key > (Press PA9 to return to Main Menu)
                │      ├── View Key (Displays 32-digit Hex key; PA9 to exit)
                │      ├── Key Presets > (Factory, Test 00..0F, A5..5A, All 01)
                │      ├── Edit Bytes... (Byte-by-byte hex editor)
                │      ├── Gen Random Key (Hardware TRNG key generator)
                │      └── Reset Default (Restores factory default key)
                ├── Slave ID: [1..247]
                ├── LoRa Config > (Press PA9 to return to Main Menu)
                │      ├── Frequency: [433.0, 868.0, 915.0, 920.0, 923.0, 925.0 MHz]
                │      ├── Spreading Factor: [SF7, SF8, SF9, SF10, SF11, SF12]
                │      ├── TX Power: [10, 14, 17, 20, 22 dBm]
                │      ├── Bandwidth: [125, 250, 500 kHz]
                │      └── Coding Rate: [4/5, 4/6, 4/7, 4/8]
                ├── OLED Type: [0.96" SSD1306 / 1.3" SH1106]
                └── Node Info... (Hardware & radio overview; PA9 to exit)
```

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
│       │   ├── lora_security.h # AES-128 payload encryption & authentication API
│       │   ├── modbus_app.h    # Modbus RTU slave API & table addresses
│       │   ├── ssd1306.h       # SSD1306 OLED I2C driver
│       │   ├── ssd1306_fonts.h # Font tables (7x10, 11x18)
│       │   ├── sx126x_hal_board.h # STM32WL internal SUBGHZ HAL header
│       │   ├── uart_app.h      # Serial print, logging & hex dumps
│       │   └── main.h          # Hardware defines and prototypes
│       └── Src/
│           ├── config.c        # Direct 64-bit doubleword Flash load/save
│           ├── lora_security.c # AES-128-CTR stream cipher & MIC verification
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

## LoRa Payload Security (SSL / AES-128-CTR)

When payload security is enabled (`security_enabled = 1`), all Modbus RTU frames and replies transmitted over LoRa are encrypted and authenticated using **AES-128-CTR**:

### Secure Frame Layout
```
+---------------+------------------+-----------------------------+-------------------+
| Magic (1B)    | Nonce / IV (4B)  | Encrypted Ciphertext (N B)  | MIC / Auth (2B)   |
| 0x53 ('S')    | 32-bit counter   | Modbus RTU Frame (AES-CTR)  | CRC16 on Payload  |
+---------------+------------------+-----------------------------+-------------------+
```

- **Zero Payload Expansion**: AES in Counter mode (CTR) produces ciphertext of the exact same length as plaintext, avoiding LoRa airtime and duty cycle bloat.
- **Anti-Replay Protection**: Each packet uses an incrementing 32-bit transmission Nonce seed.
- **Message Integrity Code (MIC)**: Ensures received packets are authentic and immediately discards tampered packets or packets with an invalid key before reaching the Modbus engine.
- **Dynamic Configuration**: Can be enabled or disabled at runtime via **Modbus Holding Register 9** and persists across reboots in Flash Page 63.
- **Key Updating via Modbus**: A new 16-byte key can be provisioned at runtime by writing to **Holding Registers 10–17** using Function Code 16 (`0x10`). The registers are write-only to prevent unauthorized reading over the air.
- **Compile-Time Default Key**: Defined in [`CM4/Core/Src/config.c`](CM4/Core/Src/config.c) in `default_key[16]`.

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
