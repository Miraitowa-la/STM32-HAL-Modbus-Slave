# STM32-HAL-Modbus-Slave V2.0

[English](README.md) | 简体中文

基于 STM32 HAL 库的轻量级多实例 Modbus RTU 从机协议栈。V2.0 版本采用全新架构设计，支持运行时配置、数据源解耦和基于回调的自定义功能码。

## ✨ V2.0 新特性

### 架构改进
*   **多实例支持**: 可在不同 UART 上同时运行多个独立的 Modbus 从站
*   **运行时配置**: 所有设置通过初始化函数传入，无需静态宏定义
*   **数据源解耦**: 库仅存储外部数据指针，多个实例可共享同一数据源
*   **回调机制**: 自定义功能码 0x64 通过回调实现，Flash 操作由应用层处理

### 标准 Modbus 协议支持
*   **0x01**: 读线圈 (Read Coils)
*   **0x02**: 读离散输入 (Read Discrete Inputs)
*   **0x03**: 读保持寄存器 (Read Holding Registers)
*   **0x04**: 读输入寄存器 (Read Input Registers)
*   **0x05**: 写单个线圈 (Write Single Coil)
*   **0x06**: 写单个寄存器 (Write Single Register)
*   **0x0F**: 写多个线圈 (Write Multiple Coils)
*   **0x10**: 写多个寄存器 (Write Multiple Registers)
*   **0x64**: 自定义配置命令 (通过回调实现)

### 保留的 V1.0 特性
*   **乒乓缓冲(双缓冲)机制**: 消除连续请求时的缓冲区数据损坏
*   **动态发送超时**: 在所有波特率(1200~115200)下可靠发送
*   **可配置 CRC 算法**: 查表法(快速)或移位法(紧凑)可选
*   **DMA/阻塞发送**: 每个实例可独立配置
*   **RS485 支持**: 自动 DE/RE 引脚控制，极性可配置

## 📂 文件结构

```
.
├── modbus_slave.c      # 协议栈实现 (无全局变量)
├── modbus_slave.h      # API 声明和数据结构
├── example_main.c      # 双实例使用示例
├── README.md           # 英文文档
└── README-zh_CN.md     # 中文文档
```

## 🚀 快速开始

### 1. CubeMX 硬件配置

#### UART 配置
*   开启对应的 USART (如 USART1, USART2)
*   波特率: 9600 (运行时可配置)
*   数据位: 8, 停止位: 1, 校验位: 无

#### NVIC 中断配置
⚠️ **必须勾选** `USARTx global interrupt`

#### GPIO 配置 (仅 RS485 模式)
*   配置 DE/RE 控制引脚为 `GPIO_Output`
*   默认电平: `Low` (接收模式)

### 2. 添加源文件

将以下文件添加到你的 STM32 工程:
*   `modbus_slave.c` → Src 目录
*   `modbus_slave.h` → Inc 目录

### 3. 定义数据源

```c
/* 定义数据源 - 可被多个 Modbus 实例共享 */
#define DATA_COIL_COUNT           16
#define DATA_DISCRETE_COUNT       16
#define DATA_HOLDING_REG_COUNT    32
#define DATA_INPUT_REG_COUNT      32

static uint8_t  g_coils[(DATA_COIL_COUNT + 7) / 8] = {0};
static uint8_t  g_discrete_inputs[(DATA_DISCRETE_COUNT + 7) / 8] = {0};
static uint16_t g_holding_regs[DATA_HOLDING_REG_COUNT] = {0};
static uint16_t g_input_regs[DATA_INPUT_REG_COUNT] = {0};
```

### 4. 定义缓冲区 (每个实例独立)

```c
/* 每个实例需要独立的缓冲区 */
#define MODBUS_BUF_SIZE     256

static ModbusHandle_t hModbus1;
static uint8_t mb1_rx_buf_a[MODBUS_BUF_SIZE];
static uint8_t mb1_rx_buf_b[MODBUS_BUF_SIZE];
static uint8_t mb1_tx_buf[MODBUS_BUF_SIZE];
```

### 5. 初始化 Modbus 实例

```c
bool Modbus_AppInit(void) {
    /* 配置数据映射 - 可被多个实例共享 */
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
    
    /* 初始化 Modbus 实例 */
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
            .de_polarity = true  /* 高电平 = 发送 */
        },
        
        .use_dma_tx = false,
        .use_crc_table = true,
        
        .custom_config_cb = CustomConfigCallback,
        .write_cb = NULL
    };
    
    return Modbus_Init(&hModbus1, &config);
}
```

### 6. 实现回调和主循环

```c
/* UART 接收回调 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        Modbus_RxCallback(&hModbus1, Size);
    }
}

/* 主循环 */
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    
    Modbus_AppInit();
    
    while (1) {
        Modbus_Process(&hModbus1);
        
        /* 你的应用逻辑 */
        g_input_regs[0] = ReadADC();  /* 更新传感器数据 */
    }
}
```

## 🔄 双实例示例 (共享数据源)

请参考 `example_main.c` 了解完整示例:
*   两个 Modbus 实例 (UART1 和 UART2)
*   两个实例共享同一份数据源
*   自定义 0x64 回调实现
*   并发保护考虑

## ⚠️ 并发保护

当多个 Modbus 实例共享同一数据源时，可能发生竞态条件。

### STM32 Cortex-M 内核特性:
*   单个 8/16 位读写操作是原子的
*   32 位对齐的读写操作是原子的
*   多寄存器操作不是原子的

### 保护策略:

```c
/* 方案 1: 关闭中断 (简单方法) */
__disable_irq();
// 访问共享数据
__enable_irq();

/* 方案 2: RTOS 互斥锁 (适用于 RTOS 应用) */
osMutexAcquire(dataMutexHandle, osWaitForever);
// 访问共享数据
osMutexRelease(dataMutexHandle);
```

## 🛠 自定义功能码 0x64

在 V2.0 中，0x64 处理器是一个回调函数。应用层负责:
*   参数验证
*   Flash 存储 (如需要)
*   系统复位 (如需要)

```c
static bool CustomConfigCallback(ModbusHandle_t *hmodbus, 
                                  uint16_t param_addr, 
                                  uint16_t param_val) {
    if (param_addr == 0x0000) {  /* 从站地址 */
        if (param_val >= 1 && param_val <= 247) {
            Modbus_SetSlaveAddr(hmodbus, (uint8_t)param_val);
            /* 在主循环中安排 Flash 保存和复位 */
            g_config_update_pending = true;
            return true;
        }
    }
    else if (param_addr == 0x0001) {  /* 波特率 */
        if (param_val >= 1 && param_val <= 8) {
            Modbus_SetBaudRate(hmodbus, BAUD_RATE_TABLE[param_val]);
            g_config_update_pending = true;
            return true;
        }
    }
    return false;
}
```

## 📊 API 参考

| 函数 | 描述 |
|------|------|
| `Modbus_Init()` | 初始化 Modbus 实例 |
| `Modbus_Process()` | 处理接收帧 (在主循环中调用) |
| `Modbus_RxCallback()` | UART 接收回调 (从 ISR 中调用) |
| `Modbus_TxCallback()` | UART 发送完成回调 (DMA 模式) |
| `Modbus_SetSlaveAddr()` | 运行时更新从站地址 |
| `Modbus_SetBaudRate()` | 更新波特率配置 |
| `Modbus_StartReceive()` | 重新启动 UART 接收 |
| `Modbus_GetSlaveAddr()` | 获取当前从站地址 |
| `Modbus_GetBaudRate()` | 获取当前波特率 |

## 🔄 从 V1.0 迁移

| V1.0 | V2.0 |
|------|------|
| `modbus_config.h` 宏定义 | `Modbus_Config_t` 结构体 |
| 全局 `hmodbus` | 用户定义的 `ModbusHandle_t` |
| 全局 `mb_holding_regs[]` | 用户定义的数组 + 指针 |
| 直接 Flash 保存 | 回调 + 应用层 Flash |
| 单实例 | 支持多实例 |

## 📝 许可证

MIT License - 适用于任何商业或个人项目

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

---

**⭐ 如果这个项目对你有帮助，请给一个 Star 支持一下！**
