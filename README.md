# STM32 LoRa Modbus RTU Node (SX1262 + libmodbus)

An embedded firmware project for **STM32F103C8T6** that implements a **Modbus RTU Slave over LoRa** using the Semtech **SX1262** transceiver and an embedded-tailored **libmodbus** stack.

---

## Features

- **Semtech SX126x LoRa Driver**: Official driver integration from [Lora-net/sx126x_driver](https://github.com/Lora-net/sx126x_driver) running via SPI1 with interrupt-driven RX/TX (DIO1).
- **Modbus RTU over LoRa**: Custom embedded backend using [libmodbus](https://github.com/stephane/libmodbus) adapted for bare-metal microcontrollers (no POSIX sockets or termios required).
- **2 Valves Support**:
  - Controlled via Modbus Coils (FC 01, 05, 15) and Holding Registers (FC 03, 06, 16).
  - Status feedback via Discrete Inputs (FC 02).
  - Callback hook (`modbus_valve_callback_t`) for driving physical GPIOs/relays.
- **2 Sensors Support**:
  - Measurement readings via Input Registers (FC 04) and mirrored Holding Registers (FC 03).
- **Configurable Slave ID**:
  - Default Slave ID: `1` (range 1–247).
  - Configurable via Modbus Holding Register 8.
  - Automatically saved to MCU Flash memory (Page 63) in JSON format and persisted across reboots.
- **Hardware Telemetry**:
  - Internal RNG / Unique Hardware ID stored in Holding Registers 0–1.
  - LoRa packet counter stored in Holding Registers 2–3.
- **Modular Design**:
  - `Core/Src/modbus_app.c` completely isolates protocol parsing, register mapping, and CRC16 checks from `main.c`.
  - `Core/Src/uart_app.c` encapsulates all UART transmission, logging, formatted output, and hex dump routines.
  - Fallback mechanism responds with plain text echo if received packet is not a Modbus frame.

---

## Modbus Address Map

**Default Slave ID:** `1` (Configurable 1–247 via Register 8)

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
| `0x0008` (8) | **Slave ID** | RW | `03`, `06`, `16` | **Modbus Slave Address (1..247)**; writing here updates ID and persists to Flash |

---

## Hardware Pinout (STM32F103C8T6)

### SX1262 LoRa Transceiver (SPI1 + GPIO)
| STM32 Pin | Signal | Direction | Description |
| :--- | :--- | :---: | :--- |
| **PA5** | SPI1_SCK | Output | SPI Clock |
| **PA6** | SPI1_MISO | Input | SPI Master-In-Slave-Out |
| **PA7** | SPI1_MOSI | Output | SPI Master-Out-Slave-In |
| **PA4** | NSS | Output | SPI Chip Select (Active Low) |
| **PA2** | BUSY | Input | SX1262 Busy Indicator |
| **PA3** | RST | Output | SX1262 Hardware Reset (Active Low) |
| **PC15** | DIO1 | Input (EXTI) | Interrupt on Rising Edge |
| **PA0** | TXEN | Output | RF Switch TX Enable |
| **PA1** | RXEN | Output | RF Switch RX Enable |

### UART1 (Debug & Log)
| STM32 Pin | Signal | Direction | Description |
| :--- | :--- | :---: | :--- |
| **PA9** | USART1_TX | Output | 115200 baud, 8N1 |
| **PA10** | USART1_RX | Input | 115200 baud, 8N1 |

---

## Project Structure

```
LoraStm32/
├── CMakeLists.txt              # CMake build configuration
├── CMakePresets.json           # Presets for Debug and Release builds
├── STM32F103xx_FLASH.ld        # Linker script (64KB Flash, 20KB RAM)
├── LORA.ioc                    # STM32CubeMX project file
├── Core/
│   ├── Inc/
│   │   ├── config.h            # LoRa and Modbus configuration struct
│   │   ├── modbus_app.h        # Modbus application API & address definitions
│   │   ├── uart_app.h          # UART transmit and debug logging interface
│   │   ├── main.h              # Pin and peripheral definitions
│   │   └── sx126x_hal_board.h  # Board-specific SX126x definitions
│   └── Src/
│       ├── config.c            # Flash load/save routines (JSON via cJSON)
│       ├── modbus_app.c        # Modbus embedded backend, tables & handlers
│       ├── uart_app.c          # UART wrapper, printf retarget, and hex printing
│       ├── sx126x_hal.c        # SX126x HAL implementation (SPI + GPIO)
│       └── main.c              # Application entry, LoRa task & callbacks
└── Drivers/
    ├── CMSIS/                  # ARM CMSIS libraries
    ├── STM32F1xx_HAL_Driver/   # ST HAL driver
    ├── sx126x_driver/          # Semtech SX126x driver (Git submodule)
    └── libmodbus/              # Embedded libmodbus static library
```

---

## Building the Project

### Prerequisites
- **CMake** 3.22 or higher
- **Ninja** build system
- **GNU Arm Embedded Toolchain** (`arm-none-eabi-gcc`)
- **Git**

### 1. Clone & Initialize Submodules
```bash
git clone https://github.com/ee56054/LoraStm32.git
cd LoraStm32
git submodule update --init --recursive
```

### 2. Configure & Build
Using CMake Presets:
```bash
# Configure
cmake --preset Debug

# Build
cmake --build --preset Debug
```

Build outputs will be generated in `build/Debug/`:
- `LORA.elf`
- `LORA.hex`
- `LORA.bin`

### 3. Flash to Target
Flash using **STM32CubeProgrammer**, **ST-Link CLI**, or **pyocd**:
```bash
# Example with STM32_Programmer_CLI
STM32_Programmer_CLI -c port=SWD -w build/Debug/LORA.bin 0x08000000 -v -rst
```

---

## Flash Memory Configuration

The last 1KB Flash page of STM32F103C8 (Page 63: `0x0800FC00`) is reserved for configuration storage:
```json
{
  "frequency": 915000000,
  "tx_power": 22,
  "spreading_factor": "SF9",
  "bandwidth": "125",
  "coding_rate": "4/6",
  "preamble_length": 8,
  "rx_timeout": 5000,
  "slave_id": 1
}
```
If the page is unwritten (0xFF), the firmware initializes with default values automatically.

---

## License

This project incorporates:
- **Semtech SX126x Driver**: BSD-3-Clause License
- **libmodbus**: LGPL-2.1-or-later License (Embedded adaptation)
- **STMicroelectronics HAL**: BSD-3-Clause License

