/*
 * modbus_slave.c
 * Modbus Slave Implementation
 */

#include "modbus_slave.h"
#include <string.h>

// 魔数，用于校验Flash内容是否有效
#define FLASH_MAGIC_KEY 0xDEADBEEF

// 波特率查找表
static const uint32_t BAUD_RATE_TABLE[] = {
    0,      // 0: Invalid
    1200,   // 1
    2400,   // 2
    4800,   // 3
    9600,   // 4 (默认)
    19200,  // 5
    38400,  // 6
    57600,  // 7
    115200  // 8
};

// 全局变量定义
uint8_t  mb_coils[MB_COIL_COUNT / 8 + 1] = {0};
uint8_t  mb_discrete_inputs[MB_DISCRETE_COUNT / 8 + 1] = {0};
uint16_t mb_holding_regs[MB_HOLDING_REG_COUNT] = {0};
uint16_t mb_input_regs[MB_INPUT_REG_COUNT] = {0};

ModbusHandle_t hmodbus;

// 内部函数声明
static uint16_t CRC16(uint8_t *buffer, uint16_t buffer_length);
static void Modbus_SendResponse(uint16_t len);
static void Modbus_SendException(uint8_t func_code, uint8_t exception_code);
static void Flash_LoadConfig(void);
static void Flash_SaveConfig(uint8_t addr, uint32_t baud);

// ================= 初始化 =================
void Modbus_Init(void) {
    hmodbus.huart = MODBUS_UART_HANDLE;

    // 1. 硬件层初始化适配
    #if MODBUS_USE_RS485 == 1
        // RS485 默认为接收模式 (低电平)
        // 注意：需确保 GPIO 时钟在 main.c 的 MX_GPIO_Init() 中已开启
        RS485_RX_ENABLE();
    #endif
    
    // 2. 读取Flash配置
    Flash_LoadConfig();

    // 3. 如果当前波特率与配置不符，重新初始化UART
    if (hmodbus.huart->Init.BaudRate != hmodbus.config.baud_rate) {
        hmodbus.huart->Init.BaudRate = hmodbus.config.baud_rate;
        if (HAL_UART_Init(hmodbus.huart) != HAL_OK) {
            // 初始化失败处理，这里死循环或者保持原状
            Error_Handler(); 
        }
    }

    // 4. 开启空闲中断接收
    HAL_UARTEx_ReceiveToIdle_IT(hmodbus.huart, hmodbus.rx_buf, MB_RX_BUF_SIZE);
}

// ================= 中断回调接口 =================
// 请在 stm32fxxx_it.c 或 main.c 的 HAL_UARTEx_RxEventCallback 中调用此函数
void Modbus_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == hmodbus.huart->Instance) {
        hmodbus.rx_len = Size;
        // 标记接收完成，由主循环处理（这里可以加个标志位，但简单的应用直接利用rx_len非0判断即可，需注意原子性）
        // 重新开启接收放在 Process 处理完之后，或者这里开启双缓冲。
        // 为简单起见，我们在Process里处理完后再开启下一次接收。
        // 注意：如果在处理期间有数据进来会丢失，工业级建议用DMA+环形缓冲。
    }
}

// ================= 主处理函数 =================
void Modbus_Process(void) {
		uint8_t req_addr;
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint8_t func_code;
    uint16_t start_addr, quantity; // 删除未使用的 crc 变量
    uint8_t byte_count;
    uint16_t i;
		
    if (hmodbus.rx_len == 0) return; // 无数据

    // 1. 检查长度
    if (hmodbus.rx_len < 4) {
        goto restart_rx; // 帧太短
    }

    // 2. 检查地址
    // 支持本机地址，或者 0xFF (特殊广播，带返回)
    req_addr = hmodbus.rx_buf[0];
    if (req_addr != hmodbus.config.slave_addr && req_addr != 0xFF) {
        goto restart_rx; // 不是发给我的
    }

    // 3. 检查CRC
    received_crc = (hmodbus.rx_buf[hmodbus.rx_len - 1] << 8) | hmodbus.rx_buf[hmodbus.rx_len - 2];
    calculated_crc = CRC16(hmodbus.rx_buf, hmodbus.rx_len - 2);
    
    if (received_crc != calculated_crc) {
        goto restart_rx; // CRC 错误
    }

    // 4. 解析功能码
    func_code = hmodbus.rx_buf[1];

    // 准备发送缓冲区头部：地址 + 功能码
    hmodbus.tx_buf[0] = hmodbus.config.slave_addr; // 无论请求是0xFF还是本机，回复都用本机真实地址
    hmodbus.tx_buf[1] = func_code;

    switch (func_code) {
        case MB_FUNC_READ_COILS: // 0x01
        case MB_FUNC_READ_DISCRETE: // 0x02
        {
            start_addr = (hmodbus.rx_buf[2] << 8) | hmodbus.rx_buf[3];
            quantity = (hmodbus.rx_buf[4] << 8) | hmodbus.rx_buf[5];
            
            // 简单边界检查
            if (quantity > 2000) { Modbus_SendException(func_code, 0x03); break; }

            uint8_t *src_buf = (func_code == MB_FUNC_READ_COILS) ? mb_coils : mb_discrete_inputs;
            byte_count = (quantity + 7) / 8;
            hmodbus.tx_buf[2] = byte_count;

            // 读取位状态并打包
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

        case MB_FUNC_READ_HOLDING: // 0x03
        case MB_FUNC_READ_INPUT:   // 0x04
        {
            start_addr = (hmodbus.rx_buf[2] << 8) | hmodbus.rx_buf[3];
            quantity = (hmodbus.rx_buf[4] << 8) | hmodbus.rx_buf[5];

            if (quantity > 125) { Modbus_SendException(func_code, 0x03); break; }

            uint16_t *src_buf = (func_code == MB_FUNC_READ_HOLDING) ? mb_holding_regs : mb_input_regs;
            uint16_t max_count = (func_code == MB_FUNC_READ_HOLDING) ? MB_HOLDING_REG_COUNT : MB_INPUT_REG_COUNT;

            if (start_addr + quantity > max_count) { Modbus_SendException(func_code, 0x02); break; }

            hmodbus.tx_buf[2] = quantity * 2;
            for (i = 0; i < quantity; i++) {
                hmodbus.tx_buf[3 + i * 2] = (src_buf[start_addr + i] >> 8) & 0xFF;
                hmodbus.tx_buf[4 + i * 2] = src_buf[start_addr + i] & 0xFF;
            }
            Modbus_SendResponse(3 + quantity * 2);
            break;
        }

        case MB_FUNC_WRITE_SINGLE_COIL: // 0x05
        {
            start_addr = (hmodbus.rx_buf[2] << 8) | hmodbus.rx_buf[3];
            uint16_t val = (hmodbus.rx_buf[4] << 8) | hmodbus.rx_buf[5];

            if (start_addr >= MB_COIL_COUNT) { Modbus_SendException(func_code, 0x02); break; }

            if (val == 0xFF00) { // ON
                mb_coils[start_addr / 8] |= (1 << (start_addr % 8));
            } else if (val == 0x0000) { // OFF
                mb_coils[start_addr / 8] &= ~(1 << (start_addr % 8));
            }

            // 原样返回请求帧
            memcpy(hmodbus.tx_buf, hmodbus.rx_buf, 6);
            Modbus_SendResponse(6);
            break;
        }

        case MB_FUNC_WRITE_SINGLE_REG: // 0x06
        {
            start_addr = (hmodbus.rx_buf[2] << 8) | hmodbus.rx_buf[3];
            uint16_t val = (hmodbus.rx_buf[4] << 8) | hmodbus.rx_buf[5];

            if (start_addr >= MB_HOLDING_REG_COUNT) { Modbus_SendException(func_code, 0x02); break; }

            mb_holding_regs[start_addr] = val;

            // 原样返回请求帧
            memcpy(hmodbus.tx_buf, hmodbus.rx_buf, 6);
            Modbus_SendResponse(6);
            break;
        }

        case MB_FUNC_WRITE_MULTI_COILS: // 0x0F
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

            // 返回 起始地址(2) + 数量(2)
            memcpy(&hmodbus.tx_buf[2], &hmodbus.rx_buf[2], 4);
            Modbus_SendResponse(6);
            break;
        }

        case MB_FUNC_WRITE_MULTI_REGS: // 0x10
        {
            start_addr = (hmodbus.rx_buf[2] << 8) | hmodbus.rx_buf[3];
            quantity = (hmodbus.rx_buf[4] << 8) | hmodbus.rx_buf[5];
            byte_count = hmodbus.rx_buf[6];

            if (start_addr + quantity > MB_HOLDING_REG_COUNT) { Modbus_SendException(func_code, 0x02); break; }

            for (i = 0; i < quantity; i++) {
                mb_holding_regs[start_addr + i] = (hmodbus.rx_buf[7 + i * 2] << 8) | hmodbus.rx_buf[8 + i * 2];
            }

            // 返回 起始地址(2) + 数量(2)
            memcpy(&hmodbus.tx_buf[2], &hmodbus.rx_buf[2], 4);
            Modbus_SendResponse(6);
            break;
        }

        case MB_FUNC_CUSTOM_CONFIG: // 0x64
        {
            // 格式: [Addr][64][RegHi][RegLo][ValHi][ValLo][CRC][CRC]
            // 长度必须为 8 字节 (和标准写寄存器 06 指令一致)
            if (hmodbus.rx_len != 8) {
                Modbus_SendException(func_code, 0x03); 
                break; 
            }

            uint16_t param_addr = (hmodbus.rx_buf[2] << 8) | hmodbus.rx_buf[3];
            uint16_t param_val  = (hmodbus.rx_buf[4] << 8) | hmodbus.rx_buf[5];
            
            // 准备临时配置变量 (基于当前配置修改)
            uint8_t  new_slave_addr = hmodbus.config.slave_addr;
            uint32_t new_baud_rate  = hmodbus.config.baud_rate;
            bool need_save = false;

            // --- 寄存器地址 00 00 : 修改从机地址 ---
            if (param_addr == 0x0000) {
                if (param_val >= 1 && param_val <= 247) {
                    new_slave_addr = (uint8_t)param_val;
                    need_save = true;
                } else {
                    Modbus_SendException(func_code, 0x03); // 非法数据
                    break;
                }
            }
            // --- 寄存器地址 00 01 : 修改波特率 ---
            else if (param_addr == 0x0001) {
                if (param_val >= 1 && param_val <= 8) {
                    new_baud_rate = BAUD_RATE_TABLE[param_val];
                    need_save = true;
                } else {
                    Modbus_SendException(func_code, 0x03); // 非法数据
                    break;
                }
            }
            // --- 后续扩展 00 02 等 ---
            else {
                Modbus_SendException(func_code, 0x02); // 非法地址
                break;
            }

            if (need_save) {
                // 1. 先回复确认包（原样返回），确保上位机知道指令已执行
                memcpy(hmodbus.tx_buf, hmodbus.rx_buf, 8); 
                // 特殊处理：如果是广播修改地址，回复帧里的地址应该是它“原来的”地址
                // 发送逻辑通用，直接调用即可
                Modbus_SendResponse(6); 
                
                // 2. 阻塞延时，等待数据发送完毕 (RS485模式下这很重要)
                HAL_Delay(50); 

                // 3. 保存配置到 Flash
                Flash_SaveConfig(new_slave_addr, new_baud_rate);

                // 4. 软复位系统，让新配置生效
                NVIC_SystemReset();
            }
            break;
        }

        default:
            Modbus_SendException(func_code, 0x01); // 非法功能码
            break;
    }

restart_rx:
    hmodbus.rx_len = 0;
    // 重新开启接收
    HAL_UARTEx_ReceiveToIdle_IT(hmodbus.huart, hmodbus.rx_buf, MB_RX_BUF_SIZE);
}

// ================= 底层辅助函数 =================

static void Modbus_SendResponse(uint16_t len) {
    // 1. 计算 CRC
    uint16_t crc = CRC16(hmodbus.tx_buf, len);
    hmodbus.tx_buf[len] = crc & 0xFF;
    hmodbus.tx_buf[len + 1] = (crc >> 8) & 0xFF;
    uint16_t total_len = len + 2;

    // 2. RS485 方向控制：切换为发送
    #if MODBUS_USE_RS485 == 1
        RS485_TX_ENABLE();
        // 某些老旧光耦隔离电路可能需要极短延时等待电平稳定，通常不需要
        // for(int k=0;k<100;k++); 
    #endif

    // 3. 执行发送 (阻塞式发送最安全，防止发送未完成就切回接收)
    HAL_UART_Transmit(hmodbus.huart, hmodbus.tx_buf, total_len, 100);

    // 4. RS485 方向控制：等待发送完成并切回接收
    #if MODBUS_USE_RS485 == 1
        // ！！！关键步骤！！！
        // HAL_UART_Transmit 返回只代表数据进入了缓冲区，不代表发完了。
        // 对于RS485，必须等待 TC (Transmission Complete) 标志位置位。
        // 如果这里不等待直接拉低，最后一个字节会被切断。
        while(__HAL_UART_GET_FLAG(hmodbus.huart, UART_FLAG_TC) == RESET);
        
        RS485_RX_ENABLE();
    #endif
}

static void Modbus_SendException(uint8_t func_code, uint8_t exception_code) {
    hmodbus.tx_buf[0] = hmodbus.config.slave_addr;
    hmodbus.tx_buf[1] = func_code | 0x80;
    hmodbus.tx_buf[2] = exception_code;
    Modbus_SendResponse(3);
}

// 标准 CRC16-Modbus 算法
static uint16_t CRC16(uint8_t *buffer, uint16_t buffer_length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < buffer_length; i++) {
        crc ^= buffer[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}

// ================= Flash 操作逻辑 =================
// 注意：Flash写入操作会暂停CPU执行，实际项目中建议放在RAM中执行或关闭中断
// 此处代码为通用逻辑，可能需要根据具体 STM32 系列（F1/F4/L0 等）微调 Flash 解锁/擦除 API

static void Flash_LoadConfig(void) {
    ModbusConfig_t *pConfigInFlash = (ModbusConfig_t *)MODBUS_FLASH_ADDR;

    // 检查魔数
    if (pConfigInFlash->magic_key == FLASH_MAGIC_KEY) {
        // 读取存储的配置
        hmodbus.config = *pConfigInFlash;
    } else {
        // 第一次使用，加载默认值
        hmodbus.config.magic_key = FLASH_MAGIC_KEY;
        hmodbus.config.slave_addr = DEFAULT_SLAVE_ADDR;
        hmodbus.config.baud_rate = DEFAULT_BAUD_RATE;
        // 注意：这里我们不在Init时自动写Flash，避免异常死循环写入，
        // 只有用户调用0x64指令时才真正写入Flash
    }
}

static void Flash_SaveConfig(uint8_t addr, uint32_t baud) {
    ModbusConfig_t new_config;
    new_config.magic_key = FLASH_MAGIC_KEY;
    new_config.slave_addr = addr;
    new_config.baud_rate = baud;

    HAL_FLASH_Unlock();

    // 1. 擦除页/扇区
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    // --- STM32F1 系列示例 (按页擦除) ---
    /*
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = MODBUS_FLASH_ADDR;
    EraseInitStruct.NbPages = 1;
    */
   
    // --- STM32F4/Generic 示例 (通用性更强需要根据HAL库版本调整) ---
    // 为了代码通用性，这里假设是 F1 的页擦除方式，如果是 F4 需要改为 Sector 擦除
    // 请务必根据你的芯片型号修改此处代码！
    #if defined(STM32F1)
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
        EraseInitStruct.PageAddress = MODBUS_FLASH_ADDR;
        EraseInitStruct.NbPages = 1;
    #elif defined(STM32F4)
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
        // 假设 MODBUS_FLASH_ADDR 对应 Sector 11
        EraseInitStruct.Sector = FLASH_SECTOR_11; 
        EraseInitStruct.NbSectors = 1;
        EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    #else
        // 默认为按页擦除 (通用 F0, G0, L0 等)
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
        EraseInitStruct.PageAddress = MODBUS_FLASH_ADDR;
        EraseInitStruct.NbPages = 1;
    #endif

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
        HAL_FLASH_Lock();
        return;
    }

    // 2. 写入数据 (按字写入)
    uint32_t *pSource = (uint32_t *)&new_config;
    uint32_t write_addr = MODBUS_FLASH_ADDR;
    
    for (int i = 0; i < sizeof(ModbusConfig_t) / 4; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, write_addr, *pSource) == HAL_OK) {
            write_addr += 4;
            pSource++;
        } else {
            break;
        }
    }

    HAL_FLASH_Lock();
}
