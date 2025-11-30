/*
 * modbus_slave.c
 * Modbus从机实现 / Modbus Slave Implementation
 * 
 * 说明: 实现完整的Modbus RTU从机协议栈,包括标准功能码处理、自定义配置功能和Flash存储
 * Note: Implements complete Modbus RTU slave protocol stack, including standard function codes, 
 *       custom configuration functions, and Flash storage
 */

#include "modbus_slave.h"
#include <string.h>

// 魔数,用于校验Flash内容是否有效 / Magic key for validating Flash content
#define FLASH_MAGIC_KEY 0xDEADBEEF

// 波特率查找表 / Baud rate lookup table
// 索引值对应功能码0x64中的波特率参数 / Index corresponds to baud rate parameter in function code 0x64
static const uint32_t BAUD_RATE_TABLE[] = {
    0,      // 0: Invalid / 无效
    1200,   // 1
    2400,   // 2
    4800,   // 3
    9600,   // 4 (默认 / Default)
    19200,  // 5
    38400,  // 6
    57600,  // 7
    115200  // 8
};

// 全局变量定义 / Global variable definitions
// 这些数组是Modbus数据映射表,用户可以直接读写 / These arrays are Modbus data mapping tables for direct user access
uint8_t  mb_coils[MB_COIL_COUNT / 8 + 1] = {0};           // 线圈状态(位压缩) / Coil status (bit-packed)
uint8_t  mb_discrete_inputs[MB_DISCRETE_COUNT / 8 + 1] = {0}; // 离散输入(位压缩) / Discrete inputs (bit-packed)
uint16_t mb_holding_regs[MB_HOLDING_REG_COUNT] = {0};     // 保持寄存器 / Holding registers
uint16_t mb_input_regs[MB_INPUT_REG_COUNT] = {0};         // 输入寄存器 / Input registers

ModbusHandle_t hmodbus;  // Modbus句柄实例 / Modbus handle instance

// 内部函数声明 / Internal function declarations
static uint16_t CRC16(uint8_t *buffer, uint16_t buffer_length);
static void Modbus_SendResponse(uint16_t len);
static void Modbus_SendException(uint8_t func_code, uint8_t exception_code);
static void Flash_LoadConfig(void);
static void Flash_SaveConfig(uint8_t addr, uint32_t baud);

// ================= 初始化函数 / Initialization Function =================
/**
 * @brief  初始化Modbus从机 / Initialize Modbus slave
 * @note   此函数完成以下操作 / This function performs the following:
 *         1. 初始化RS485控制引脚(如启用) / Initialize RS485 control pin (if enabled)
 *         2. 从Flash加载配置参数 / Load configuration parameters from Flash
 *         3. 根据配置重新初始化UART / Re-initialize UART based on configuration
 *         4. 启动UART空闲中断接收 / Start UART idle interrupt reception
 */
void Modbus_Init(void) {
    hmodbus.huart = MODBUS_UART_HANDLE;

    // 1. 硬件层初始化适配 / Hardware layer initialization
    #if MODBUS_USE_RS485 == 1
        // RS485 默认为接收模式 (低电平) / RS485 defaults to receive mode (low level)
        // 注意:需确保 GPIO 时钟在 main.c 的 MX_GPIO_Init() 中已开启
        // Note: Ensure GPIO clock is enabled in MX_GPIO_Init() in main.c
        RS485_RX_ENABLE();
    #endif
    
    // 2. 读取Flash配置 / Read Flash configuration
    Flash_LoadConfig();

    // 3. 如果当前波特率与配置不符,重新初始化UART / Re-initialize UART if baud rate differs
    if (hmodbus.huart->Init.BaudRate != hmodbus.config.baud_rate) {
        hmodbus.huart->Init.BaudRate = hmodbus.config.baud_rate;
        if (HAL_UART_Init(hmodbus.huart) != HAL_OK) {
            // 初始化失败处理,这里死循环或者保持原状 / Handle initialization failure
            Error_Handler(); 
        }
    }

    // 4. 开启空闲中断接收 / Enable idle interrupt reception
    HAL_UARTEx_ReceiveToIdle_IT(hmodbus.huart, hmodbus.rx_buf, MB_RX_BUF_SIZE);
}

// ================= 中断回调接口 / Interrupt Callback Interface =================
/**
 * @brief  UART接收完成回调函数 / UART receive complete callback
 * @param  huart: UART句柄 / UART handle
 * @param  Size: 接收到的字节数 / Number of bytes received
 * @note   请在 stm32fxxx_it.c 或 main.c 的 HAL_UARTEx_RxEventCallback 中调用此函数
 *         Call this function in HAL_UARTEx_RxEventCallback in stm32fxxx_it.c or main.c
 *         接收到完整帧后,rx_len会被设置,由Modbus_Process()处理
 *         After receiving a complete frame, rx_len is set and processed by Modbus_Process()
 */
void Modbus_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == hmodbus.huart->Instance) {
        hmodbus.rx_len = Size;  // 记录接收长度 / Record received length
        // 标记接收完成,由主循环处理(这里可以加个标志位,但简单的应用直接利用rx_len非0判断即可,需注意原子性)
        // Mark reception complete, processed in main loop (can add a flag, but simple apps use rx_len != 0)
        // 重新开启接收放在 Process 处理完之后,或者这里开启双缓冲。
        // Restart reception after Process completes, or enable double buffering here
        // 为简单起见,我们在Process里处理完后再开启下一次接收。
        // For simplicity, we restart reception after processing in Process()
        // 注意:如果在处理期间有数据进来会丢失,工业级建议用DMA+环形缓冲。
        // Warning: Data arriving during processing will be lost; industrial applications should use DMA + ring buffer
    }
}

// ================= 主处理函数 / Main Processing Function =================
/**
 * @brief  Modbus协议处理主循环 / Modbus protocol processing main loop
 * @note   此函数应在主循环中持续调用,完成以下任务 / Call continuously in main loop, performs:
 *         1. 检查接收缓冲区是否有新数据 / Check if new data in receive buffer
 *         2. 验证帧格式(长度、地址、CRC) / Validate frame format (length, address, CRC)
 *         3. 解析并执行功能码 / Parse and execute function code
 *         4. 发送响应或异常码 / Send response or exception code
 *         5. 重新启动接收 / Restart reception
 */
void Modbus_Process(void) {
		uint8_t req_addr;
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint8_t func_code;
    uint16_t start_addr, quantity; // 删除未使用的 crc 变量 / Removed unused crc variable
    uint8_t byte_count;
    uint16_t i;
		
    if (hmodbus.rx_len == 0) return; // 无数据 / No data

    // 1. 检查长度 / Check frame length
    if (hmodbus.rx_len < 4) {
        goto restart_rx; // 帧太短 / Frame too short
    }

    // 2. 检查地址 / Check address
    // 支持本机地址,或者 0xFF (特殊广播,带返回) / Support local address or 0xFF (special broadcast with response)
    req_addr = hmodbus.rx_buf[0];
    if (req_addr != hmodbus.config.slave_addr && req_addr != 0xFF) {
        goto restart_rx; // 不是发给我的 / Not addressed to this slave
    }

    // 3. 检查CRC / Verify CRC
    received_crc = (hmodbus.rx_buf[hmodbus.rx_len - 1] << 8) | hmodbus.rx_buf[hmodbus.rx_len - 2];
    calculated_crc = CRC16(hmodbus.rx_buf, hmodbus.rx_len - 2);
    
    if (received_crc != calculated_crc) {
        goto restart_rx; // CRC 错误 / CRC error
    }

    // 4. 解析功能码 / Parse function code
    func_code = hmodbus.rx_buf[1];

    // 准备发送缓冲区头部:地址 + 功能码 / Prepare response header: address + function code
    hmodbus.tx_buf[0] = hmodbus.config.slave_addr; // 无论请求是0xFF还是本机,回复都用本机真实地址 / Always reply with real address
    hmodbus.tx_buf[1] = func_code;

    switch (func_code) {
        case MB_FUNC_READ_COILS: // 0x01 读线圈 / Read Coils
        case MB_FUNC_READ_DISCRETE: // 0x02 读离散输入 / Read Discrete Inputs
        {
            start_addr = (hmodbus.rx_buf[2] << 8) | hmodbus.rx_buf[3];
            quantity = (hmodbus.rx_buf[4] << 8) | hmodbus.rx_buf[5];
            
            // 简单边界检查 / Simple boundary check
            if (quantity > 2000) { Modbus_SendException(func_code, 0x03); break; }

            uint8_t *src_buf = (func_code == MB_FUNC_READ_COILS) ? mb_coils : mb_discrete_inputs;
            byte_count = (quantity + 7) / 8;
            hmodbus.tx_buf[2] = byte_count;

            // 读取位状态并打包 / Read bit status and pack
            memset(&hmodbus.tx_buf[3], 0, byte_count);
            for (i = 0; i < quantity; i++) {
                uint16_t bit_idx = start_addr + i;
                if (src_buf[bit_idx / 8] & (1 << (bit_idx % 8))) {
                    hmodbus.tx_buf[3 + i / 8] |= (1 << (i % 8));
                }
            }
            Modbus_SendResponse(3 + byte_count);
            break;
        }

        case MB_FUNC_READ_HOLDING: // 0x03 读保持寄存器 / Read Holding Registers
        case MB_FUNC_READ_INPUT:   // 0x04 读输入寄存器 / Read Input Registers
        {
            start_addr = (hmodbus.rx_buf[2] << 8) | hmodbus.rx_buf[3];
            quantity = (hmodbus.rx_buf[4] << 8) | hmodbus.rx_buf[5];

            if (quantity > 125) { Modbus_SendException(func_code, 0x03); break; }

            uint16_t *src_buf = (func_code == MB_FUNC_READ_HOLDING) ? mb_holding_regs : mb_input_regs;
            uint16_t max_count = (func_code == MB_FUNC_READ_HOLDING) ? MB_HOLDING_REG_COUNT : MB_INPUT_REG_COUNT;

            if (start_addr + quantity > max_count) { Modbus_SendException(func_code, 0x02); break; }

            hmodbus.tx_buf[2] = quantity * 2;
            for (i = 0; i < quantity; i++) {
                hmodbus.tx_buf[3 + i * 2] = (src_buf[start_addr + i] >> 8) & 0xFF;  // 高字节 / High byte
                hmodbus.tx_buf[4 + i * 2] = src_buf[start_addr + i] & 0xFF;         // 低字节 / Low byte
            }
            Modbus_SendResponse(3 + quantity * 2);
            break;
        }

        case MB_FUNC_WRITE_SINGLE_COIL: // 0x05 写单个线圈 / Write Single Coil
        {
            start_addr = (hmodbus.rx_buf[2] << 8) | hmodbus.rx_buf[3];
            uint16_t val = (hmodbus.rx_buf[4] << 8) | hmodbus.rx_buf[5];

            if (start_addr >= MB_COIL_COUNT) { Modbus_SendException(func_code, 0x02); break; }

            if (val == 0xFF00) { // ON / 开
                mb_coils[start_addr / 8] |= (1 << (start_addr % 8));
            } else if (val == 0x0000) { // OFF / 关
                mb_coils[start_addr / 8] &= ~(1 << (start_addr % 8));
            }

            // 原样返回请求帧 / Echo request frame
            memcpy(hmodbus.tx_buf, hmodbus.rx_buf, 6);
            Modbus_SendResponse(6);
            break;
        }

        case MB_FUNC_WRITE_SINGLE_REG: // 0x06 写单个寄存器 / Write Single Register
        {
            start_addr = (hmodbus.rx_buf[2] << 8) | hmodbus.rx_buf[3];
            uint16_t val = (hmodbus.rx_buf[4] << 8) | hmodbus.rx_buf[5];

            if (start_addr >= MB_HOLDING_REG_COUNT) { Modbus_SendException(func_code, 0x02); break; }

            mb_holding_regs[start_addr] = val;

            // 原样返回请求帧 / Echo request frame
            memcpy(hmodbus.tx_buf, hmodbus.rx_buf, 6);
            Modbus_SendResponse(6);
            break;
        }

        case MB_FUNC_WRITE_MULTI_COILS: // 0x0F 写多个线圈 / Write Multiple Coils
        {
            start_addr = (hmodbus.rx_buf[2] << 8) | hmodbus.rx_buf[3];
            quantity = (hmodbus.rx_buf[4] << 8) | hmodbus.rx_buf[5];
            byte_count = hmodbus.rx_buf[6];

            if (start_addr + quantity > MB_COIL_COUNT) { Modbus_SendException(func_code, 0x02); break; }

            for (i = 0; i < quantity; i++) {
                uint8_t val = (hmodbus.rx_buf[7 + i / 8] >> (i % 8)) & 0x01;
                if (val) mb_coils[(start_addr + i) / 8] |= (1 << ((start_addr + i) % 8));
                else     mb_coils[(start_addr + i) / 8] &= ~(1 << ((start_addr + i) % 8));
            }

            // 返回 起始地址(2) + 数量(2) / Return start address (2) + quantity (2)
            memcpy(&hmodbus.tx_buf[2], &hmodbus.rx_buf[2], 4);
            Modbus_SendResponse(6);
            break;
        }

        case MB_FUNC_WRITE_MULTI_REGS: // 0x10 写多个寄存器 / Write Multiple Registers
        {
            start_addr = (hmodbus.rx_buf[2] << 8) | hmodbus.rx_buf[3];
            quantity = (hmodbus.rx_buf[4] << 8) | hmodbus.rx_buf[5];
            byte_count = hmodbus.rx_buf[6];

            if (start_addr + quantity > MB_HOLDING_REG_COUNT) { Modbus_SendException(func_code, 0x02); break; }

            for (i = 0; i < quantity; i++) {
                mb_holding_regs[start_addr + i] = (hmodbus.rx_buf[7 + i * 2] << 8) | hmodbus.rx_buf[8 + i * 2];
            }

            // 返回 起始地址(2) + 数量(2) / Return start address (2) + quantity (2)
            memcpy(&hmodbus.tx_buf[2], &hmodbus.rx_buf[2], 4);
            Modbus_SendResponse(6);
            break;
        }

        case MB_FUNC_CUSTOM_CONFIG: // 0x64 自定义配置功能 / Custom Configuration Function
        {
            // 格式: [Addr][64][RegHi][RegLo][ValHi][ValLo][CRC][CRC]
            // Format: [Addr][64][RegHi][RegLo][ValHi][ValLo][CRC][CRC]
            // 长度必须为 8 字节 (和标准写寄存器 06 指令一致)
            // Length must be 8 bytes (same as standard write register 0x06 command)
            if (hmodbus.rx_len != 8) {
                Modbus_SendException(func_code, 0x03); 
                break; 
            }

            uint16_t param_addr = (hmodbus.rx_buf[2] << 8) | hmodbus.rx_buf[3];
            uint16_t param_val  = (hmodbus.rx_buf[4] << 8) | hmodbus.rx_buf[5];
            
            // 准备临时配置变量 (基于当前配置修改) / Prepare temporary config variables (modify based on current)
            uint8_t  new_slave_addr = hmodbus.config.slave_addr;
            uint32_t new_baud_rate  = hmodbus.config.baud_rate;
            bool need_save = false;

            // --- 寄存器地址 00 00 : 修改从机地址 / Register Address 0x0000: Modify Slave Address ---
            if (param_addr == 0x0000) {
                if (param_val >= 1 && param_val <= 247) {
                    new_slave_addr = (uint8_t)param_val;
                    need_save = true;
                } else {
                    Modbus_SendException(func_code, 0x03); // 非法数据 / Illegal data value
                    break;
                }
            }
            // --- 寄存器地址 00 01 : 修改波特率 / Register Address 0x0001: Modify Baud Rate ---
            else if (param_addr == 0x0001) {
                if (param_val >= 1 && param_val <= 8) {
                    new_baud_rate = BAUD_RATE_TABLE[param_val];
                    need_save = true;
                } else {
                    Modbus_SendException(func_code, 0x03); // 非法数据 / Illegal data value
                    break;
                }
            }
            // --- 后续扩展 00 02 等 / Future Extension 0x0002, etc. ---
            else {
                Modbus_SendException(func_code, 0x02); // 非法地址 / Illegal address
                break;
            }

            if (need_save) {
                // 1. 先回复确认包(原样返回),确保上位机知道指令已执行
                // 1. Reply with confirmation (echo request) to ensure host knows command executed
                memcpy(hmodbus.tx_buf, hmodbus.rx_buf, 8); 
                // 特殊处理:如果是广播修改地址,回复帧里的地址应该是它"原来的"地址
                // Special handling: If broadcast address modification, reply address should be the "original" address
                // 发送逻辑通用,直接调用即可 / Sending logic is generic, call directly
                Modbus_SendResponse(6); 
                
                // 2. 阻塞延时,等待数据发送完毕 (RS485模式下这很重要)
                // 2. Blocking delay, wait for data transmission to complete (critical for RS485 mode)
                HAL_Delay(50); 

                // 3. 保存配置到 Flash / Save configuration to Flash
                Flash_SaveConfig(new_slave_addr, new_baud_rate);

                // 4. 软复位系统,让新配置生效 / Software reset system to apply new configuration
                NVIC_SystemReset();
            }
            break;
        }

        default:
            Modbus_SendException(func_code, 0x01); // 非法功能码 / Illegal function code
            break;
    }

restart_rx:
    hmodbus.rx_len = 0;  // 清空接收长度 / Clear receive length
    // 重新开启接收 / Restart reception
    HAL_UARTEx_ReceiveToIdle_IT(hmodbus.huart, hmodbus.rx_buf, MB_RX_BUF_SIZE);
}

// ================= 底层辅助函数 / Low-level Helper Functions =================

/**
 * @brief  发送Modbus响应帧 / Send Modbus response frame
 * @param  len: 有效数据长度(不包括CRC) / Valid data length (excluding CRC)
 * @note   此函数自动计算CRC,控制RS485收发方向,并阻塞发送
 *         This function automatically calculates CRC, controls RS485 direction, and sends data blocking
 */
static void Modbus_SendResponse(uint16_t len) {
    // 1. 计算 CRC / Calculate CRC
    uint16_t crc = CRC16(hmodbus.tx_buf, len);
    hmodbus.tx_buf[len] = crc & 0xFF;         // CRC低字节 / CRC low byte
    hmodbus.tx_buf[len + 1] = (crc >> 8) & 0xFF;  // CRC高字节 / CRC high byte
    uint16_t total_len = len + 2;

    // 2. RS485 方向控制:切换为发送 / RS485 direction control: Switch to transmit
    #if MODBUS_USE_RS485 == 1
        RS485_TX_ENABLE();
        // 某些老旧光耦隔离电路可能需要极短延时等待电平稳定,通常不需要
        // Some old optocoupler isolation circuits may need a short delay for level stabilization, usually not needed
        // for(int k=0;k<100;k++); 
    #endif

    // 3. 执行发送 (阻塞式发送最安全,防止发送未完成就切回接收)
    // 3. Execute transmission (blocking transmission is safest, prevents switching back to receive before completion)
    HAL_UART_Transmit(hmodbus.huart, hmodbus.tx_buf, total_len, 100);

    // 4. RS485 方向控制:等待发送完成并切回接收 / RS485 direction control: Wait for completion and switch back to receive
    #if MODBUS_USE_RS485 == 1
        // ！！！关键步骤！！！ / !!!CRITICAL STEP!!!
        // HAL_UART_Transmit 返回只代表数据进入了缓冲区,不代表发完了。
        // HAL_UART_Transmit return only means data entered buffer, not that transmission completed
        // 对于RS485,必须等待 TC (Transmission Complete) 标志位置位。
        // For RS485, must wait for TC (Transmission Complete) flag to be set
        // 如果这里不等待直接拉低,最后一个字节会被切断。
        // If not waiting here and directly pulling low, the last byte will be cut off
        while(__HAL_UART_GET_FLAG(hmodbus.huart, UART_FLAG_TC) == RESET);
        
        RS485_RX_ENABLE();
    #endif
}

/**
 * @brief  发送Modbus异常响应 / Send Modbus exception response
 * @param  func_code: 功能码 / Function code
 * @param  exception_code: 异常码 (01:非法功能 02:非法地址 03:非法数据)
 *                         Exception code (01:Illegal function 02:Illegal address 03:Illegal data)
 */
static void Modbus_SendException(uint8_t func_code, uint8_t exception_code) {
    hmodbus.tx_buf[0] = hmodbus.config.slave_addr;
    hmodbus.tx_buf[1] = func_code | 0x80;  // 功能码最高位置1表示异常 / Set MSB of function code to indicate exception
    hmodbus.tx_buf[2] = exception_code;
    Modbus_SendResponse(3);
}

/**
 * @brief  标准 CRC16-Modbus 算法 / Standard CRC16-Modbus algorithm
 * @param  buffer: 数据缓冲区 / Data buffer
 * @param  buffer_length: 缓冲区长度 / Buffer length
 * @return CRC16校验值 / CRC16 checksum value
 */
static uint16_t CRC16(uint8_t *buffer, uint16_t buffer_length) {
    uint16_t crc = 0xFFFF;  // 初始值 / Initial value
    for (uint16_t i = 0; i < buffer_length; i++) {
        crc ^= buffer[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;  // 多项式 / Polynomial
            else         crc >>= 1;
        }
    }
    return crc;
}

// ================= Flash 操作逻辑 / Flash Operation Logic =================
// 注意:Flash写入操作会暂停CPU执行,实际项目中建议放在RAM中执行或关闭中断
// Warning: Flash write operations will pause CPU execution; in real projects, consider executing in RAM or disabling interrupts
// 此处代码为通用逻辑,可能需要根据具体 STM32 系列（F1/F4/L0 等）微调 Flash 解锁/擦除 API
// This code is generic logic; may need fine-tuning based on specific STM32 series (F1/F4/L0, etc.) for Flash unlock/erase APIs

/**
 * @brief  从Flash加载配置参数 / Load configuration parameters from Flash
 * @note   通过魔数判断是否为第一次启动,如是则使用默认值
 *         Uses magic key to determine if first boot; if so, uses default values
 */
static void Flash_LoadConfig(void) {
    ModbusConfig_t *pConfigInFlash = (ModbusConfig_t *)MODBUS_FLASH_ADDR;

    // 检查魔数 / Check magic key
    if (pConfigInFlash->magic_key == FLASH_MAGIC_KEY) {
        // 读取存储的配置 / Read stored configuration
        hmodbus.config = *pConfigInFlash;
    } else {
        // 第一次使用,加载默认值 / First time use, load default values
        hmodbus.config.magic_key = FLASH_MAGIC_KEY;
        hmodbus.config.slave_addr = DEFAULT_SLAVE_ADDR;
        hmodbus.config.baud_rate = DEFAULT_BAUD_RATE;
        // 注意:这里我们不在Init时自动写Flash,避免异常死循环写入,
        // Note: We don't automatically write Flash during Init to avoid abnormal loop writes
        // 只有用户调用0x64指令时才真正写入Flash
        // Only truly write to Flash when user calls 0x64 command
    }
}

/**
 * @brief  保存配置参数到Flash / Save configuration parameters to Flash
 * @param  addr: 从机地址 / Slave address
 * @param  baud: 波特率 / Baud rate
 * @note   此函数会擦除并重写Flash指定页/扇区
 *         This function will erase and rewrite the specified Flash page/sector
 */
static void Flash_SaveConfig(uint8_t addr, uint32_t baud) {
    ModbusConfig_t new_config;
    new_config.magic_key = FLASH_MAGIC_KEY;
    new_config.slave_addr = addr;
    new_config.baud_rate = baud;

    HAL_FLASH_Unlock();  // 解锁Flash / Unlock Flash

    // 1. 擦除页/扇区 / Erase page/sector
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    // --- STM32F1 系列示例 (按页擦除) / STM32F1 Series Example (Page Erase) ---
    /*
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = MODBUS_FLASH_ADDR;
    EraseInitStruct.NbPages = 1;
    */
   
    // --- STM32F4/Generic 示例 (通用性更强需要根据HAL库版本调整) ---
    // --- STM32F4/Generic Example (adjust based on HAL library version for better compatibility) ---
    // 为了代码通用性,这里假设是 F1 的页擦除方式,如果是 F4 需要改为 Sector 擦除
    // For code compatibility, assume F1 page erase; if F4, change to Sector erase
    // 请务必根据你的芯片型号修改此处代码！
    // IMPORTANT: Modify this code based on your chip model!
    #if defined(STM32F1)
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
        EraseInitStruct.PageAddress = MODBUS_FLASH_ADDR;
        EraseInitStruct.NbPages = 1;
    #elif defined(STM32F4)
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
        // 假设 MODBUS_FLASH_ADDR 对应 Sector 11 / Assume MODBUS_FLASH_ADDR corresponds to Sector 11
        EraseInitStruct.Sector = FLASH_SECTOR_11; 
        EraseInitStruct.NbSectors = 1;
        EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    #else
        // 默认为按页擦除 (通用 F0, G0, L0 等) / Default to page erase (generic F0, G0, L0, etc.)
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
        EraseInitStruct.PageAddress = MODBUS_FLASH_ADDR;
        EraseInitStruct.NbPages = 1;
    #endif

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
        HAL_FLASH_Lock();  // 擦除失败,上锁返回 / Erase failed, lock and return
        return;
    }

    // 2. 写入数据 (按字写入) / Write data (word by word)
    uint32_t *pSource = (uint32_t *)&new_config;
    uint32_t write_addr = MODBUS_FLASH_ADDR;
    
    for (int i = 0; i < sizeof(ModbusConfig_t) / 4; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, write_addr, *pSource) == HAL_OK) {
            write_addr += 4;  // 移动到下一个字 / Move to next word
            pSource++;
        } else {
            break;  // 写入失败 / Write failed
        }
    }

    HAL_FLASH_Lock();  // 上锁Flash / Lock Flash
}
