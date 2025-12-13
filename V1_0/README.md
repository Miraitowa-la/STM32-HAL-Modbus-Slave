# STM32-HAL-Modbus-Slave

English | [简体中文](README-zh_CN.md)

A lightweight Modbus RTU slave protocol stack based on STM32 HAL library. Supports automatic RS485/TTL switching, uses internal Flash to emulate EEPROM for storing slave address and baud rate, and supports custom function code 0x64 for online modification of communication parameters.

## ✨ Features

### Standard Modbus Protocol Support
*   **0x01**: Read Coils
*   **0x02**: Read Discrete Inputs
*   **0x03**: Read Holding Registers
*   **0x04**: Read Input Registers
*   **0x05**: Write Single Coil
*   **0x06**: Write Single Register
*   **0x0F**: Write Multiple Coils
*   **0x10**: Write Multiple Registers

### Exclusive Features
*   **0x64 Custom Command**: Supports online modification of slave address and baud rate (format compatible with standard 0x06 command)
*   **Flash Non-volatile Storage**: Configuration parameters stored in STM32 internal Flash specified page, no data loss on power failure
*   **Auto Reboot to Apply**: Automatically soft reset after parameter modification, new configuration takes effect immediately
*   **Broadcast Recovery Mode**: Supports using broadcast address 0xFF to force modify device address

### Hardware Compatibility
*   Supports **RS485** (automatic DE/RE pin control) and **RS232/TTL** (transparent transmission), switch via macro definition
*   Based on `HAL_UARTEx_ReceiveToIdle_IT` idle interrupt reception, handles variable-length data frames without DMA
*   Supports broadcast address 0xFF for emergency device recovery
*   Compatible with all STM32 series (F0/F1/F4/L0/G0, etc.)

## 📂 File Structure

```
.
├── modbus_slave.c      # Protocol stack core implementation, Flash read/write logic
├── modbus_slave.h      # External interface and data structure definitions
├── modbus_config.h     # User configuration file (UART, Flash address, RS485 pins, etc.)
├── README.md           # English documentation
└── README-zh_CN.md     # Chinese documentation
```

## 🚀 Quick Start

### 1. CubeMX Hardware Configuration

#### UART Configuration
*   Enable the corresponding USART (e.g., USART1)
*   Baud rate: 9600 (default, modifiable via 0x64 command)
*   Data bits: 8
*   Stop bits: 1
*   Parity: None

#### NVIC Interrupt Configuration
⚠️ **Must enable** `USARTx global interrupt` (Enabled)

#### GPIO Configuration (RS485 mode only)
*   Configure DE/RE control pin as `GPIO_Output`
*   Default level: `Low` (receive mode)

### 2. Add Source Files

Add the following files to your STM32 project:
*   `modbus_slave.c` → Src directory
*   `modbus_slave.h` → Inc directory  
*   `modbus_config.h` → Inc directory

### 3. Configure modbus_config.h

Open `modbus_config.h` and modify the following parameters according to your actual hardware:

```c
/* ============= Hardware Configuration ============= */
// 1. Specify UART handle (must be declared in main.h)
extern UART_HandleTypeDef huart2;
#define MODBUS_UART_HANDLE      &huart2

/* ============= Flash Storage Configuration ============= */
// 2. Specify Flash storage address (⚠️ Very important!)
// Please refer to chip datasheet to ensure the address is an unused page
// STM32F103C8T6 (64KB Flash) last page: 0x0800FC00
// STM32F103RCT6 (256KB Flash) last page: 0x0803FC00
// STM32F407VET6 (512KB Flash) Sector 11: 0x080E0000
#define MODBUS_FLASH_ADDR       0x0800F800

/* ============= Physical Layer Interface Configuration ============= */
// 3. RS485 mode switch
#define MODBUS_USE_RS485        1    // 1=Enable, 0=Disable (use RS232/TTL)

// 4. RS485 control pins (only valid when MODBUS_USE_RS485 is 1)
#define RS485_PORT              GPIOD
#define RS485_PIN               GPIO_PIN_7
```

> **💡 Flash Address Selection Recommendations**
> - Choose the last page or sector of the chip to avoid conflicts with program code
> - Confirm the address is unused in the linker script
> - F1 series: divided by pages (1KB or 2KB per page depending on chip)
> - F4 series: divided by sectors (16KB~128KB per sector)
>
> **⚠️ Flash Lifespan Warning**
> - Flash memory has limited write/erase cycles (typically 10K-100K cycles)
> - Avoid frequent use of function code 0x64 to prevent premature Flash wear-out
> - The implementation includes data comparison to minimize unnecessary writes

### 4. Write Application Code

Integrate Modbus protocol stack in `main.c`:

```c
/* USER CODE BEGIN Includes */
#include "modbus_slave.h"
/* USER CODE END Includes */

int main(void)
{
    /* MCU Initialization */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */
    // Initialize Modbus (auto load Flash config and start reception)
    Modbus_Init();
    /* USER CODE END 2 */

    /* Main Loop */
    while (1)
    {
        /* USER CODE BEGIN 3 */
        // Modbus protocol processing (must be called in loop)
        Modbus_Process();

        // ========== Application Logic Example ==========
        // Example 1: Sync holding register 0 value to input register 0
        mb_input_regs[0] = mb_holding_regs[0];

        // Example 2: Use coil 0 to control LED (PC13)
        if (mb_coils[0] & 0x01) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED ON
        } else {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // LED OFF
        }
        /* USER CODE END 3 */
    }
}

/* USER CODE BEGIN 4 */
/**
 * @brief  UART idle interrupt callback function
 * @note   Must add this callback for receiving Modbus data frames
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    Modbus_RxCpltCallback(huart, Size);
}
/* USER CODE END 4 */
```

## 🛠 Function Code 0x64 Usage

Custom function code **0x64** (100) is used to modify device configuration parameters online. Command format is fully compatible with standard Modbus function code **0x06**.

### Command Format

```
[Device Address] [0x64] [Param Addr High] [Param Addr Low] [Value High] [Value Low] [CRC Low] [CRC High]
```

### 1️⃣ Modify Slave Address

| Item | Description |
|------|------|
| **Parameter Address** | `0x0000` |
| **Value Range** | `0x0001` ~ `0x00F7` (1 ~ 247) |
| **Example** | Change address from 0x01 to 0x02 |
| **Send (Hex)** | `01 64 00 00 00 02 51 F0` |

### 2️⃣ Modify Baud Rate

| Item | Description |
|------|------|
| **Parameter Address** | `0x0001` |
| **Value Range** | `0x0001` ~ `0x0008` (see table below) |

#### Baud Rate Lookup Table

| Index | Baud Rate | Index | Baud Rate |
|:----:|:------:|:----:|:------:|
| 0x01 | 1200   | 0x05 | 19200  |
| 0x02 | 2400   | 0x06 | 38400  |
| 0x03 | 4800   | 0x07 | 57600  |
| 0x04 | **9600** (default) | 0x08 | 115200 |

**Example**: Change baud rate to 115200 (Index 0x08)  
**Send (Hex)**: `01 64 00 01 00 08 50 0C`

### 3️⃣ Broadcast Recovery Mode

When forgetting current device address or baud rate, you can use broadcast address **0xFF** to force modify.

**Example**: Force change any device address to 0x01  
**Send (Hex)**: `FF 64 00 00 00 01 90 14`

> **⚠️ Notes**
> - After successful modification, STM32 will echo the command
> - After a brief delay, automatically save to Flash and soft reset
> - New configuration takes effect immediately after reboot

## 📊 Data Mapping

The following global arrays are defined in `modbus_slave.h`, directly operate them to exchange data with host:

```c
// Coils - 0xxxx area
// Bit-packed storage, read/write
extern uint8_t mb_coils[(MB_COIL_COUNT + 7) / 8];

// Discrete Inputs - 1xxxx area
// Bit-packed storage, read-only
extern uint8_t mb_discrete_inputs[(MB_DISCRETE_COUNT + 7) / 8];

// Holding Registers - 4xxxx area
// 16-bit registers, read/write
extern uint16_t mb_holding_regs[MB_HOLDING_REG_COUNT];

// Input Registers - 3xxxx area  
// 16-bit registers, read-only
extern uint16_t mb_input_regs[MB_INPUT_REG_COUNT];
```

### Usage Example

```c
// Read holding register 0 value
mb_input_regs[0] = mb_holding_regs[0];

// Set coil 0 (bit operation)
mb_coils[0] |= 0x01;  // Set to 1
mb_coils[0] &= ~0x01; // Clear

// Check coil 0 status
if (mb_coils[0] & 0x01) {
    // Coil 0 is high level
}
```

## ⚠️ Important Notes

### Flash Address Configuration
*   **Must ensure** `MODBUS_FLASH_ADDR` points to an address without program code
*   Recommended to use **last page/sector** of Flash
*   Refer to chip datasheet to confirm page/sector addresses

### Flash Operation Optimization
*   **Data Comparison**: Before writing to Flash, the system compares current values to prevent unnecessary writes and extend Flash lifespan
*   **Compatibility**: Enhanced Flash erase/program operations support more STM32 series (F1, F4, G0, G4, L4, L5, H7)
*   **Performance Note**: Flash operations stall the CPU during execution. Frequent use of function code 0x64 can quickly consume Flash write/erase cycles (typically 10K-100K cycles)

### Ping-Pong Buffer (Double Buffer) Mechanism
*   **Problem Solved**: Race condition where new data overwrites `rx_buf` while `Modbus_Process()` is still processing previous frame
*   **Solution**: Dual receive buffers (`rx_buf_a` and `rx_buf_b`) with automatic switching in interrupt callback
*   **How It Works**:
    - Interrupt receives data into buffer A, sets ready flag
    - Main loop processes buffer A while interrupt receives into buffer B
    - No data loss even with high-frequency communication
*   **Benefit**: Eliminates buffer corruption during back-to-back Modbus requests

### Dynamic Transmission Timeout
*   **Problem Solved**: Hardcoded 100ms timeout causes transmission failures at low baud rates
*   **Issue**: At 1200 baud, sending 256 bytes requires ~2.1 seconds, exceeding the fixed 100ms timeout
*   **Solution**: Timeout is dynamically calculated based on baud rate and data length
*   **Formula**: `timeout_ms = (bytes * 10 bits * 1000) / baud_rate + safety_margin`
*   **Safety Margin**: Max of 50ms or 10% of transmission time, minimum timeout is 100ms
*   **Benefit**: Reliable transmission at all supported baud rates (1200~115200)

### CRC16 Algorithm Selection
*   **Configuration**: Set `MODBUS_CRC_USE_TABLE` in `modbus_config.h`
*   **Options**:
    - `0` (Default): **Bit-shift method** - Minimal code size, slower computation
    - `1`: **Lookup table method** - ~10x faster, requires 512 bytes ROM for table
*   **Recommendations**:
    - ROM-constrained devices: Use bit-shift method (0)
    - High baud rate (115200+) or heavy traffic: Use lookup table (1)
*   **Performance Comparison** (STM32F103 @ 72MHz, 256 bytes):
    - Bit-shift: ~1.5ms
    - Lookup table: ~0.15ms

### Transmission Mode Selection (Blocking vs DMA)
*   **Configuration**: Set `MODBUS_USE_DMA_TX` in `modbus_config.h`
*   **Options**:
    - `0` (Default): **Blocking mode** - Simple and reliable, CPU waits during transmission
    - `1`: **DMA mode** - Frees CPU, allows other tasks during transmission
*   **Recommendations**:
    - Simple applications or debugging: Use blocking mode (0)
    - Real-time requirements: Use DMA mode (1)
*   **DMA Mode Requirements**:
    - Configure USART TX DMA in CubeMX
    - Add `HAL_UART_TxCpltCallback` in main.c (see example below)

#### main.c Example - Blocking Mode (Default)
```c
/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    Modbus_RxCpltCallback(huart, Size);
}
/* USER CODE END 4 */
```

#### main.c Example - DMA Mode
```c
/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    Modbus_RxCpltCallback(huart, Size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    Modbus_TxCpltCallback(huart);  // Required for RS485 direction control
}
/* USER CODE END 4 */
```

### Interrupt Priority
*   Modbus communication depends on UART interrupt
*   Ensure UART interrupt priority is set reasonably to avoid being blocked by other high-priority interrupts

### RS485 Direction Control
*   Code automatically handles RS485 DE/RE pin control
*   **Key point**: Must wait for TC flag after sending before switching to receive mode
*   If RS485 communication is unstable, check hardware circuit and DE/RE pin definitions

### Multi-Slave Networking
*   Each slave address must be unique (1-247)
*   Baud rate and protocol parameters must be consistent
*   RS485 bus requires termination resistors (120Ω) to avoid signal reflection

## 📝 License

MIT License - Suitable for any commercial or personal project

## 🤝 Contributing

Welcome to submit Issues and Pull Requests!

## 💬 Contact & Support

If you have any questions or suggestions, please open an issue on GitHub.

---

**⭐ If this project helps you, please give it a Star!**
