# STM32-HAL-Modbus-Slave V2.0

English | [简体中文](README-zh_CN.md)

A lightweight, multi-instance Modbus RTU slave protocol stack based on STM32 HAL library. V2.0 features a completely redesigned architecture with runtime configuration, data source decoupling, and callback-based custom function codes.

## ✨ New Features in V2.0

### Architecture Improvements
*   **Multi-Instance Support**: Run multiple independent Modbus slaves on different UARTs simultaneously
*   **Runtime Configuration**: All settings passed through initialization functions, no static macros required
*   **Data Source Decoupling**: Library stores only pointers to external data, multiple instances can share one data source
*   **Callback Mechanism**: Custom function code 0x64 implemented via callbacks, Flash operations handled by application layer

### Standard Modbus Protocol Support
*   **0x01**: Read Coils
*   **0x02**: Read Discrete Inputs
*   **0x03**: Read Holding Registers
*   **0x04**: Read Input Registers
*   **0x05**: Write Single Coil
*   **0x06**: Write Single Register
*   **0x0F**: Write Multiple Coils
*   **0x10**: Write Multiple Registers
*   **0x64**: Custom Configuration Command (via callback)

### Preserved Features from V1.0
*   **Ping-Pong Buffer (Double Buffering)**: Eliminates buffer corruption during back-to-back requests
*   **Dynamic Transmission Timeout**: Reliable transmission at all baud rates (1200~115200)
*   **Configurable CRC Algorithm**: Choose between lookup table (fast) or bit-shift (compact) method
*   **DMA/Blocking Send**: Switch between DMA and blocking mode per instance
*   **RS485 Support**: Automatic DE/RE pin control with configurable polarity

## 📂 File Structure

```
.
├── modbus_slave.c      # Protocol stack implementation (no global variables)
├── modbus_slave.h      # API declarations and data structures
├── example_main.c      # Usage example with dual instances
├── README.md           # English documentation
└── README-zh_CN.md     # Chinese documentation
```

## 🚀 Quick Start

### 1. CubeMX Hardware Configuration

#### UART Configuration
*   Enable corresponding USART (e.g., USART1, USART2)
*   Baud rate: 9600 (configurable at runtime)
*   Data bits: 8, Stop bits: 1, Parity: None

#### NVIC Interrupt Configuration
⚠️ **Must enable** `USARTx global interrupt`

#### GPIO Configuration (RS485 mode only)
*   Configure DE/RE control pin as `GPIO_Output`
*   Default level: `Low` (receive mode)

### 2. Add Source Files

Add these files to your STM32 project:
*   `modbus_slave.c` → Src directory
*   `modbus_slave.h` → Inc directory

### 3. Define Data Source

```c
/* Define your data source - can be shared by multiple Modbus instances */
#define DATA_COIL_COUNT           16
#define DATA_DISCRETE_COUNT       16
#define DATA_HOLDING_REG_COUNT    32
#define DATA_INPUT_REG_COUNT      32

static uint8_t  g_coils[(DATA_COIL_COUNT + 7) / 8] = {0};
static uint8_t  g_discrete_inputs[(DATA_DISCRETE_COUNT + 7) / 8] = {0};
static uint16_t g_holding_regs[DATA_HOLDING_REG_COUNT] = {0};
static uint16_t g_input_regs[DATA_INPUT_REG_COUNT] = {0};
```

### 4. Define Buffers (Per Instance)

```c
/* Each instance needs independent buffers */
#define MODBUS_BUF_SIZE     256

static ModbusHandle_t hModbus1;
static uint8_t mb1_rx_buf_a[MODBUS_BUF_SIZE];
static uint8_t mb1_rx_buf_b[MODBUS_BUF_SIZE];
static uint8_t mb1_tx_buf[MODBUS_BUF_SIZE];
```

### 5. Initialize Modbus Instance

```c
bool Modbus_AppInit(void) {
    /* Configure data mapping - shared by multiple instances */
    Modbus_DataMap_t shared_data_map = {
        .coils = g_coils,
        .coil_count = DATA_COIL_COUNT,
        .discrete_inputs = g_discrete_inputs,
        .discrete_count = DATA_DISCRETE_COUNT,
        .holding_regs = g_holding_regs,
        .holding_reg_count = DATA_HOLDING_REG_COUNT,
        .input_regs = g_input_regs,
        .input_reg_count = DATA_INPUT_REG_COUNT
    };
    
    /* Initialize Modbus instance */
    Modbus_Config_t config = {
        .huart = &huart1,
        .slave_addr = 0x01,
        .baud_rate = 9600,
        
        .buffer = {
            .rx_buf_a = mb1_rx_buf_a,
            .rx_buf_b = mb1_rx_buf_b,
            .tx_buf = mb1_tx_buf,
            .rx_buf_size = MODBUS_BUF_SIZE,
            .tx_buf_size = MODBUS_BUF_SIZE
        },
        
        .data_map = shared_data_map,
        
        .rs485 = {
            .enabled = true,
            .de_port = GPIOA,
            .de_pin = GPIO_PIN_8,
            .de_polarity = true  /* High = transmit */
        },
        
        .use_dma_tx = false,
        .use_crc_table = true,
        
        .custom_config_cb = CustomConfigCallback,
        .write_cb = NULL
    };
    
    return Modbus_Init(&hModbus1, &config);
}
```

### 6. Implement Callbacks and Main Loop

```c
/* UART receive callback */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        Modbus_RxCallback(&hModbus1, Size);
    }
}

/* Main loop */
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    
    Modbus_AppInit();
    
    while (1) {
        Modbus_Process(&hModbus1);
        
        /* Your application logic */
        g_input_regs[0] = ReadADC();  /* Update sensor data */
    }
}
```

## 🔄 Dual Instance Example (Shared Data Source)

See `example_main.c` for a complete example of:
*   Two Modbus instances (UART1 and UART2)
*   Both instances sharing the same data source
*   Custom 0x64 callback implementation
*   Concurrent protection considerations

## ⚠️ Concurrent Protection

When multiple Modbus instances share the same data source, race conditions may occur.

### For STM32 Cortex-M:
*   Single 8/16-bit read/write operations are atomic
*   32-bit aligned read/write operations are atomic
*   Multi-register operations are NOT atomic

### Protection Strategies:

```c
/* Option 1: Disable interrupts (simple approach) */
__disable_irq();
// Access shared data
__enable_irq();

/* Option 2: RTOS mutex (for RTOS applications) */
osMutexAcquire(dataMutexHandle, osWaitForever);
// Access shared data
osMutexRelease(dataMutexHandle);
```

## 🛠 Custom Function Code 0x64

In V2.0, the 0x64 handler is a callback function. Application layer is responsible for:
*   Parameter validation
*   Flash storage (if needed)
*   System reset (if needed)

```c
static bool CustomConfigCallback(ModbusHandle_t *hmodbus, 
                                  uint16_t param_addr, 
                                  uint16_t param_val) {
    if (param_addr == 0x0000) {  /* Slave address */
        if (param_val >= 1 && param_val <= 247) {
            Modbus_SetSlaveAddr(hmodbus, (uint8_t)param_val);
            /* Schedule Flash save and reset in main loop */
            g_config_update_pending = true;
            return true;
        }
    }
    else if (param_addr == 0x0001) {  /* Baud rate */
        if (param_val >= 1 && param_val <= 8) {
            Modbus_SetBaudRate(hmodbus, BAUD_RATE_TABLE[param_val]);
            g_config_update_pending = true;
            return true;
        }
    }
    return false;
}
```

## 📊 API Reference

| Function | Description |
|----------|-------------|
| `Modbus_Init()` | Initialize a Modbus instance |
| `Modbus_Process()` | Process received frames (call in main loop) |
| `Modbus_RxCallback()` | UART receive callback (call from ISR) |
| `Modbus_TxCallback()` | UART transmit complete callback (DMA mode) |
| `Modbus_SetSlaveAddr()` | Update slave address at runtime |
| `Modbus_SetBaudRate()` | Update baud rate configuration |
| `Modbus_StartReceive()` | Restart UART reception |
| `Modbus_GetSlaveAddr()` | Get current slave address |
| `Modbus_GetBaudRate()` | Get current baud rate |

## 🔄 Migration from V1.0

| V1.0 | V2.0 |
|------|------|
| `modbus_config.h` macros | `Modbus_Config_t` structure |
| Global `hmodbus` | User-defined `ModbusHandle_t` |
| Global `mb_holding_regs[]` | User-defined arrays + pointers |
| Direct Flash save | Callback + application layer Flash |
| Single instance | Multiple instances supported |

## 📝 License

MIT License - Suitable for any commercial or personal project

## 🤝 Contributing

Welcome to submit Issues and Pull Requests!

---

**⭐ If this project helps you, please give it a Star!**
