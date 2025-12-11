# STM32-HAL-Modbus-Slave

[English](README.md) | 简体中文

一个轻量级的 STM32 HAL 库 Modbus RTU 从机协议栈。支持 RS485/TTL 自动切换，使用内部 Flash 模拟 EEPROM 存储从机地址和波特率，并支持自定义功能码 0x64 在线修改通讯参数。

## ✨ 功能特性

### 标准 Modbus 协议支持
*   **0x01**: 读线圈 (Read Coils)
*   **0x02**: 读离散输入 (Read Discrete Inputs)
*   **0x03**: 读保持寄存器 (Read Holding Registers)
*   **0x04**: 读输入寄存器 (Read Input Registers)
*   **0x05**: 写单个线圈 (Write Single Coil)
*   **0x06**: 写单个寄存器 (Write Single Register)
*   **0x0F**: 写多个线圈 (Write Multiple Coils)
*   **0x10**: 写多个寄存器 (Write Multiple Registers)

### 独家功能
*   **0x64 自定义指令**：支持在线修改从机地址和波特率（格式兼容标准 0x06 指令）
*   **Flash 掉电保存**：配置参数存储在 STM32 内部 Flash 指定页，断电不丢失
*   **自动重启生效**：修改参数后自动软复位，新配置立即生效
*   **广播救砖模式**：支持使用广播地址 0xFF 强制修改设备地址

### 硬件适配
*   支持 **RS485** (自动控制 DE/RE 引脚) 和 **RS232/TTL** (透明传输)，通过宏定义一键切换
*   基于 `HAL_UARTEx_ReceiveToIdle_IT` 空闲中断接收，无需 DMA 即可处理不定长数据帧
*   支持广播地址 0xFF 用于紧急设备恢复
*   兼容 STM32 全系列 (F0/F1/F4/L0/G0 等)

## 📂 文件结构

```
.
├── modbus_slave.c      # 协议栈核心实现、Flash 读写逻辑
├── modbus_slave.h      # 对外接口和数据结构定义
├── modbus_config.h     # 用户配置文件（串口、Flash地址、RS485引脚等）
├── README.md           # 英文说明文档
└── README-zh_CN.md     # 中文说明文档
```

## 🚀 快速开始

### 1. CubeMX 硬件配置

#### UART 配置
*   开启对应的 USART (如 USART1)
*   波特率：9600 (默认，可通过 0x64 指令修改)
*   数据位：8
*   停止位：1
*   校验位：无

#### NVIC 中断配置
⚠️ **必须勾选** `USARTx global interrupt` (Enabled)

#### GPIO 配置（仅 RS485 模式）
*   配置 DE/RE 控制引脚为 `GPIO_Output`
*   默认电平：`Low` (接收模式)

### 2. 添加源文件

将以下文件添加到你的 STM32 工程：
*   `modbus_slave.c` → Src 目录
*   `modbus_slave.h` → Inc 目录  
*   `modbus_config.h` → Inc 目录

### 3. 配置 modbus_config.h

打开 `modbus_config.h`，根据实际硬件修改以下参数：

```c
/* ============= 硬件配置 ============= */
// 1. 指定 UART 句柄（需在 main.h 中声明）
extern UART_HandleTypeDef huart2;
#define MODBUS_UART_HANDLE      &huart2

/* ============= Flash 存储配置 ============= */
// 2. 指定 Flash 存储地址（⚠️ 非常重要！）
// 请查阅芯片手册，确保该地址为空闲页，不会覆盖程序代码
// STM32F103C8T6 (64KB Flash) 最后一页: 0x0800FC00
// STM32F103RCT6 (256KB Flash) 最后一页: 0x0803FC00
// STM32F407VET6 (512KB Flash) 扇区11: 0x080E0000
#define MODBUS_FLASH_ADDR       0x0800F800

/* ============= 物理层接口配置 ============= */
// 3. RS485 模式开关
#define MODBUS_USE_RS485        1    // 1=开启, 0=关闭（使用RS232/TTL）

// 4. RS485 控制引脚（仅当 MODBUS_USE_RS485 为 1 时有效）
#define RS485_PORT              GPIOD
#define RS485_PIN               GPIO_PIN_7
```

> **💡 Flash 地址选择建议**
> - 选择芯片最后一页或扇区，避免与程序代码冲突
> - 确认该地址在链接脚本中未被使用
> - F1 系列：按页划分（每页 1KB 或 2KB，取决于具体芯片）
> - F4 系列：按扇区划分（每扇区 16KB~128KB）

### 4. 编写应用代码

在 `main.c` 中集成 Modbus 协议栈：

```c
/* USER CODE BEGIN Includes */
#include "modbus_slave.h"
/* USER CODE END Includes */

int main(void)
{
    /* MCU 初始化 */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */
    // 初始化 Modbus（自动加载 Flash 配置并启动接收）
    Modbus_Init();
    /* USER CODE END 2 */

    /* 主循环 */
    while (1)
    {
        /* USER CODE BEGIN 3 */
        // Modbus 协议处理（必须循环调用）
        Modbus_Process();

        // ========== 应用逻辑示例 ==========
        // 示例1：将保持寄存器0的值同步到输入寄存器0
        mb_input_regs[0] = mb_holding_regs[0];

        // 示例2：使用线圈0控制 LED (PC13)
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
 * @brief  UART 空闲中断回调函数
 * @note   必须添加此回调，用于接收 Modbus 数据帧
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    Modbus_RxCpltCallback(huart, Size);
}
/* USER CODE END 4 */
```

## 🛠 功能码 0x64 使用说明

自定义功能码 **0x64** (100) 用于在线修改设备配置参数。指令格式与标准 Modbus 功能码 **0x06** 完全一致。

### 指令格式

```
[设备地址] [0x64] [参数地址高] [参数地址低] [写入值高] [写入值低] [CRC低] [CRC高]
```

### 1️⃣ 修改从机地址

| 项目 | 说明 |
|------|------|
| **参数地址** | `0x0000` |
| **取值范围** | `0x0001` ~ `0x00F7` (1 ~ 247) |
| **示例** | 将地址从 0x01 改为 0x02 |
| **发送 (Hex)** | `01 64 00 00 00 02 51 F0` |

### 2️⃣ 修改波特率

| 项目 | 说明 |
|------|------|
| **参数地址** | `0x0001` |
| **取值范围** | `0x0001` ~ `0x0008` (对应下表) |

#### 波特率对应表

| 索引 | 波特率 | 索引 | 波特率 |
|:----:|:------:|:----:|:------:|
| 0x01 | 1200   | 0x05 | 19200  |
| 0x02 | 2400   | 0x06 | 38400  |
| 0x03 | 4800   | 0x07 | 57600  |
| 0x04 | **9600** (默认) | 0x08 | 115200 |

**示例**：将波特率修改为 115200 (Index 0x08)  
**发送 (Hex)**：`01 64 00 01 00 08 50 0C`

### 3️⃣ 广播救砖模式

当忘记当前设备地址或波特率时，可使用广播地址 **0xFF** 强制修改。

**示例**：强制将任意设备地址修改为 0x01  
**发送 (Hex)**：`FF 64 00 00 00 01 90 14`

> **⚠️ 注意事项**
> - 修改成功后，STM32 会原样返回指令
> - 稍作延时后自动保存到 Flash 并软复位
> - 重启后新配置立即生效

## 📊 数据映射

在 `modbus_slave.h` 中定义了以下全局数组，直接操作即可实现与上位机的数据交换：

```c
// 线圈 (Coils) - 0xxxx 区域
// 位压缩存储，可读写
extern uint8_t mb_coils[(MB_COIL_COUNT + 7) / 8];

// 离散输入 (Discrete Inputs) - 1xxxx 区域
// 位压缩存储，只读
extern uint8_t mb_discrete_inputs[(MB_DISCRETE_COUNT + 7) / 8];

// 保持寄存器 (Holding Registers) - 4xxxx 区域
// 16位寄存器，可读写
extern uint16_t mb_holding_regs[MB_HOLDING_REG_COUNT];

// 输入寄存器 (Input Registers) - 3xxxx 区域  
// 16位寄存器，只读
extern uint16_t mb_input_regs[MB_INPUT_REG_COUNT];
```

### 使用示例

```c
// 读取保持寄存器0的值
mb_input_regs[0] = mb_holding_regs[0];

// 设置线圈0（位操作）
mb_coils[0] |= 0x01;  // 设置为 1
mb_coils[0] &= ~0x01; // 清零

// 判断线圈0状态
if (mb_coils[0] & 0x01) {
    // 线圈0为高电平
}
```

## ⚠️ 注意事项

### Flash 地址配置
*   **必须确保** `MODBUS_FLASH_ADDR` 指向的地址没有存放程序代码
*   建议使用 Flash 的**最后一页/扇区**
*   参考芯片数据手册确认分页/扇区地址

### 中断优先级
*   Modbus 通信依赖 UART 中断
*   请确保 UART 中断优先级设置合理，避免被其他高优先级中断阻塞

### RS485 方向控制
*   代码已自动处理 RS485 的 DE/RE 引脚控制
*   **关键点**：发送后必须等待 TC 标志位再切换为接收模式
*   如果 RS485 通信不稳定，检查硬件电路和 DE/RE 引脚定义

### 多从机组网
*   每个从机地址必须唯一 (1-247)
*   波特率和协议参数必须一致
*   RS485 总线需要端接电阻 (120Ω) 避免信号反射

## 📝 许可证

MIT License - 适用于任何商业或个人项目

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 💬 联系与支持

如有问题或建议，请在 GitHub Issues 中提出。

---

**⭐ 如果这个项目对你有帮助，请给一个 Star 支持一下！**