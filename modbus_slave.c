/**
 * @file    modbus_slave.c
 * @brief   Modbus从站协议栈实现
 * @details 实现Modbus RTU从站的核心功能，包括帧解析、功能码处理、Flash配置存储等
 */

#include "modbus_slave.h"
#include <string.h>

/* ============================================================================
 *                              内部常量定义
 * ============================================================================ */

/** @brief Flash数据有效性魔数标识 */
#define FLASH_MAGIC_KEY 0xDEADBEEF

/**
 * @brief   波特率索引映射表
 * @note    用于0x64自定义功能码，通过索引值设置波特率
 */
static const uint32_t BAUD_RATE_TABLE[] = {
    0,       /* 0: 无效 */
    1200,    /* 1 */
    2400,    /* 2 */
    4800,    /* 3 */
    9600,    /* 4: 默认值 */
    19200,   /* 5 */
    38400,   /* 6 */
    57600,   /* 7 */
    115200   /* 8 */
};

/* ============================================================================
 *                              全局变量定义
 * ============================================================================ */

/* Modbus数据区存储数组 */
#if MB_COIL_COUNT > 0
uint8_t  mb_coils[(MB_COIL_COUNT + 7) / 8] = {0};               /* 线圈存储区 */
#endif

#if MB_DISCRETE_COUNT > 0
uint8_t  mb_discrete_inputs[(MB_DISCRETE_COUNT + 7) / 8] = {0}; /* 离散输入存储区 */
#endif

#if MB_HOLDING_REG_COUNT > 0
uint16_t mb_holding_regs[MB_HOLDING_REG_COUNT] = {0};           /* 保持寄存器存储区 */
#endif

#if MB_INPUT_REG_COUNT > 0
uint16_t mb_input_regs[MB_INPUT_REG_COUNT] = {0};               /* 输入寄存器存储区 */
#endif

/** @brief Modbus从站全局句柄 */
ModbusHandle_t hmodbus;

/* ============================================================================
 *                              内部函数声明
 * ============================================================================ */

static uint16_t CRC16(uint8_t *buffer, uint16_t buffer_length);
static void Modbus_SendResponse(uint16_t len);
static void Modbus_SendException(uint8_t func_code, uint8_t exception_code);
static void Flash_LoadConfig(void);
static void Flash_SaveConfig(uint8_t addr, uint32_t baud);

/* ============================================================================
 *                              初始化函数
 * ============================================================================ */

/**
 * @brief   Modbus从站初始化
 */
void Modbus_Init(void) {
    hmodbus.huart = MODBUS_UART_HANDLE;

    /* 步骤1: 初始化乒乓缓冲区指针
     * 中断接收指向A，处理指向B(初始为空) */
    hmodbus.rx_active_buf = hmodbus.rx_buf_a;
    hmodbus.rx_process_buf = hmodbus.rx_buf_b;
    hmodbus.rx_len = 0;
    hmodbus.rx_ready = 0;

    /* 步骤2: RS485硬件层初始化 */
    #if MODBUS_USE_RS485 == 1
        /* RS485默认为接收模式
         * 注意: GPIO时钟需在main.c的MX_GPIO_Init()中已开启 */
        RS485_RX_ENABLE();
    #endif

    /* 步骤3: 从Flash加载配置参数 */
    Flash_LoadConfig();

    /* 步骤4: 若当前波特率与配置不符，重新初始化UART */
    if (hmodbus.huart->Init.BaudRate != hmodbus.config.baud_rate) {
        hmodbus.huart->Init.BaudRate = hmodbus.config.baud_rate;
        if (HAL_UART_Init(hmodbus.huart) != HAL_OK) {
            Error_Handler();  /* 初始化失败，进入错误处理 */
        }
    }

    /* 步骤5: 开启UART空闲中断接收 (使用乒乓缓冲A) */
    HAL_UARTEx_ReceiveToIdle_IT(hmodbus.huart, hmodbus.rx_active_buf, MB_RX_BUF_SIZE);
}

/* ============================================================================
 *                              中断回调接口
 * ============================================================================ */

/**
 * @brief   UART接收完成/空闲中断回调
 * @param   huart   UART句柄指针
 * @param   Size    接收到的数据长度
 * @note    需在HAL_UARTEx_RxEventCallback中调用此函数
 *          采用乒乓缓冲机制，中断中交换缓冲区指针
 */
void Modbus_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == hmodbus.huart->Instance) {
        /* 乒乓缓冲切换:
         * 1. 将当前接收缓冲区设为处理缓冲区
         * 2. 记录接收数据长度
         * 3. 切换中断接收目标到另一个缓冲区
         * 4. 立即重新启动接收，减少数据丢失窗口 */
        
        /* 交换缓冲区指针 */
        uint8_t* completed_buf = hmodbus.rx_active_buf;
        hmodbus.rx_active_buf = (completed_buf == hmodbus.rx_buf_a) 
                                ? hmodbus.rx_buf_b : hmodbus.rx_buf_a;
        
        /* 设置处理缓冲区和数据长度 */
        hmodbus.rx_process_buf = completed_buf;
        hmodbus.rx_len = Size;
        hmodbus.rx_ready = 1;
        
        /* 立即重新启动接收，指向新的缓冲区
         * 这样即使主循环处理较慢，也不会丢失新到达的数据 */
        HAL_UARTEx_ReceiveToIdle_IT(hmodbus.huart, hmodbus.rx_active_buf, MB_RX_BUF_SIZE);
    }
}

/* ============================================================================
 *                              主处理函数
 * ============================================================================ */

/**
 * @brief   Modbus帧解析与响应处理
 * @note    应在主循环中周期性调用
 *          采用乒乓缓冲，处理期间不会影响新数据接收
 */
void Modbus_Process(void) {
    uint8_t req_addr;
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint8_t func_code;
    uint16_t start_addr, quantity;
    uint8_t byte_count;
    uint16_t i;
    
    /* 乒乓缓冲机制: 使用本地指针指向处理缓冲区 */
    uint8_t* rx_buf;
    uint16_t rx_len;

    if (!hmodbus.rx_ready) return;  /* 无数据待处理 */
    
    /* 获取待处理数据的本地副本
     * 注意: 在处理过程中，中断可能会更新rx_ready和rx_len */
    rx_buf = hmodbus.rx_process_buf;
    rx_len = hmodbus.rx_len;
    
    /* 清除接收标志，允许中断更新新数据 */
    hmodbus.rx_ready = 0;
    hmodbus.rx_len = 0;

    /* 步骤1: 检查帧最小长度 */
    if (rx_len < 4) {
        return;  /* 帧长度不足 */
    }

    /* 步骤2: 校验从站地址
     * 支持本机地址和0xFF广播地址(带返回) */
    req_addr = rx_buf[0];
    if (req_addr != hmodbus.config.slave_addr && req_addr != 0xFF) {
        return;  /* 地址不匹配 */
    }

    /* 步骤3: CRC校验 */
    received_crc = (rx_buf[rx_len - 1] << 8) | rx_buf[rx_len - 2];
    calculated_crc = CRC16(rx_buf, rx_len - 2);

    if (received_crc != calculated_crc) {
        return;  /* CRC错误 */
    }

    /* 步骤4: 解析功能码 */
    func_code = rx_buf[1];

    /* 准备响应帧头部: 从站地址 + 功能码
     * 无论请求地址是0xFF还是本机地址，响应始终使用本机真实地址 */
    hmodbus.tx_buf[0] = hmodbus.config.slave_addr;
    hmodbus.tx_buf[1] = func_code;

    switch (func_code) {
        /* ================================================================
         *                      线圈处理 (Coils)
         *                  功能码: 0x01, 0x05, 0x0F
         * ================================================================ */
        #if MB_COIL_COUNT > 0
        case MB_FUNC_READ_COILS:  /* 0x01: 读线圈状态 */
        {
            start_addr = (rx_buf[2] << 8) | rx_buf[3];
            quantity = (rx_buf[4] << 8) | rx_buf[5];

            /* 参数校验 */
            if (quantity < 1 || quantity > 2000) { Modbus_SendException(func_code, 0x03); break; }
            if (start_addr + quantity > MB_COIL_COUNT) { Modbus_SendException(func_code, 0x02); break; }

            byte_count = (quantity + 7) / 8;
            hmodbus.tx_buf[2] = byte_count;
            memset(&hmodbus.tx_buf[3], 0, byte_count);

            /* 读取线圈状态并打包响应 */
            for (i = 0; i < quantity; i++) {
                uint16_t bit_idx = start_addr + i;
                if (mb_coils[bit_idx / 8] & (1 << (bit_idx % 8))) {
                    hmodbus.tx_buf[3 + i / 8] |= (1 << (i % 8));
                }
            }
            Modbus_SendResponse(3 + byte_count);
            break;
        }

        case MB_FUNC_WRITE_SINGLE_COIL:  /* 0x05: 写单个线圈 */
        {
            start_addr = (rx_buf[2] << 8) | rx_buf[3];
            uint16_t val = (rx_buf[4] << 8) | rx_buf[5];

            if (start_addr >= MB_COIL_COUNT) { Modbus_SendException(func_code, 0x02); break; }

            /* 0xFF00=置位, 0x0000=复位 */
            if (val == 0xFF00) {      mb_coils[start_addr / 8] |= (1 << (start_addr % 8)); }
            else if (val == 0x0000) { mb_coils[start_addr / 8] &= ~(1 << (start_addr % 8)); }

            /* 原样返回请求帧 */
            memcpy(hmodbus.tx_buf, rx_buf, 6);
            Modbus_SendResponse(6);
            break;
        }

        case MB_FUNC_WRITE_MULTI_COILS:  /* 0x0F: 写多个线圈 */
        {
            start_addr = (rx_buf[2] << 8) | rx_buf[3];
            quantity = (rx_buf[4] << 8) | rx_buf[5];
            byte_count = rx_buf[6];

            if (start_addr + quantity > MB_COIL_COUNT) { Modbus_SendException(func_code, 0x02); break; }

            /* 写入线圈状态 */
            for (i = 0; i < quantity; i++) {
                uint8_t val = (rx_buf[7 + i / 8] >> (i % 8)) & 0x01;
                if (val) mb_coils[(start_addr + i) / 8] |= (1 << ((start_addr + i) % 8));
                else     mb_coils[(start_addr + i) / 8] &= ~(1 << ((start_addr + i) % 8));
            }

            memcpy(&hmodbus.tx_buf[2], &rx_buf[2], 4);
            Modbus_SendResponse(6);
            break;
        }
        #endif

        /* ================================================================
         *                    离散输入处理 (Discrete Inputs)
         *                        功能码: 0x02
         * ================================================================ */
        #if MB_DISCRETE_COUNT > 0
        case MB_FUNC_READ_DISCRETE:  /* 0x02: 读离散输入 */
        {
            start_addr = (rx_buf[2] << 8) | rx_buf[3];
            quantity = (rx_buf[4] << 8) | rx_buf[5];

            /* 参数校验 */
            if (quantity < 1 || quantity > 2000) { Modbus_SendException(func_code, 0x03); break; }
            if (start_addr + quantity > MB_DISCRETE_COUNT) { Modbus_SendException(func_code, 0x02); break; }

            byte_count = (quantity + 7) / 8;
            hmodbus.tx_buf[2] = byte_count;
            memset(&hmodbus.tx_buf[3], 0, byte_count);

            /* 读取离散输入状态 */
            for (i = 0; i < quantity; i++) {
                uint16_t bit_idx = start_addr + i;
                if (mb_discrete_inputs[bit_idx / 8] & (1 << (bit_idx % 8))) {
                    hmodbus.tx_buf[3 + i / 8] |= (1 << (i % 8));
                }
            }
            Modbus_SendResponse(3 + byte_count);
            break;
        }
        #endif

        /* ================================================================
         *                   保持寄存器处理 (Holding Registers)
         *                   功能码: 0x03, 0x06, 0x10
         * ================================================================ */
        #if MB_HOLDING_REG_COUNT > 0
        case MB_FUNC_READ_HOLDING:  /* 0x03: 读保持寄存器 */
        {
            start_addr = (rx_buf[2] << 8) | rx_buf[3];
            quantity = (rx_buf[4] << 8) | rx_buf[5];

            /* 参数校验 */
            if (quantity < 1 || quantity > 125) { Modbus_SendException(func_code, 0x03); break; }
            if (start_addr + quantity > MB_HOLDING_REG_COUNT) { Modbus_SendException(func_code, 0x02); break; }

            hmodbus.tx_buf[2] = quantity * 2;
            /* 读取保持寄存器数据 */
            for (i = 0; i < quantity; i++) {
                hmodbus.tx_buf[3 + i * 2] = (mb_holding_regs[start_addr + i] >> 8) & 0xFF;
                hmodbus.tx_buf[4 + i * 2] = mb_holding_regs[start_addr + i] & 0xFF;
            }
            Modbus_SendResponse(3 + quantity * 2);
            break;
        }

        case MB_FUNC_WRITE_SINGLE_REG:  /* 0x06: 写单个寄存器 */
        {
            start_addr = (rx_buf[2] << 8) | rx_buf[3];
            uint16_t val = (rx_buf[4] << 8) | rx_buf[5];

            if (start_addr >= MB_HOLDING_REG_COUNT) { Modbus_SendException(func_code, 0x02); break; }

            mb_holding_regs[start_addr] = val;

            /* 原样返回请求帧 */
            memcpy(hmodbus.tx_buf, rx_buf, 6);
            Modbus_SendResponse(6);
            break;
        }

        case MB_FUNC_WRITE_MULTI_REGS:  /* 0x10: 写多个寄存器 */
        {
            start_addr = (rx_buf[2] << 8) | rx_buf[3];
            quantity = (rx_buf[4] << 8) | rx_buf[5];

            if (start_addr + quantity > MB_HOLDING_REG_COUNT) { Modbus_SendException(func_code, 0x02); break; }

            /* 写入保持寄存器数据 */
            for (i = 0; i < quantity; i++) {
                mb_holding_regs[start_addr + i] = (rx_buf[7 + i * 2] << 8) | rx_buf[8 + i * 2];
            }

            memcpy(&hmodbus.tx_buf[2], &rx_buf[2], 4);
            Modbus_SendResponse(6);
            break;
        }
        #endif

        /* ================================================================
         *                    输入寄存器处理 (Input Registers)
         *                         功能码: 0x04
         * ================================================================ */
        #if MB_INPUT_REG_COUNT > 0
        case MB_FUNC_READ_INPUT:  /* 0x04: 读输入寄存器 */
        {
            start_addr = (rx_buf[2] << 8) | rx_buf[3];
            quantity = (rx_buf[4] << 8) | rx_buf[5];

            /* 参数校验 */
            if (quantity < 1 || quantity > 125) { Modbus_SendException(func_code, 0x03); break; }
            if (start_addr + quantity > MB_INPUT_REG_COUNT) { Modbus_SendException(func_code, 0x02); break; }

            hmodbus.tx_buf[2] = quantity * 2;
            /* 读取输入寄存器数据 */
            for (i = 0; i < quantity; i++) {
                hmodbus.tx_buf[3 + i * 2] = (mb_input_regs[start_addr + i] >> 8) & 0xFF;
                hmodbus.tx_buf[4 + i * 2] = mb_input_regs[start_addr + i] & 0xFF;
            }
            Modbus_SendResponse(3 + quantity * 2);
            break;
        }
        #endif

        /* ================================================================
         *                    自定义配置功能码 (0x64)
         *             用于修改从站地址和波特率，并保存到Flash
         * ================================================================ */
        case MB_FUNC_CUSTOM_CONFIG:  /* 0x64: 自定义配置指令 */
        {
            /* 帧格式: [Addr][64][RegHi][RegLo][ValHi][ValLo][CRC][CRC]
             * 长度固定为8字节 */
            if (rx_len != 8) {
                Modbus_SendException(func_code, 0x03);
                break;
            }

            uint16_t param_addr = (rx_buf[2] << 8) | rx_buf[3];
            uint16_t param_val  = (rx_buf[4] << 8) | rx_buf[5];

            /* 基于当前配置创建临时配置变量 */
            uint8_t  new_slave_addr = hmodbus.config.slave_addr;
            uint32_t new_baud_rate  = hmodbus.config.baud_rate;
            bool need_save = false;

            /* 参数地址 0x0000: 修改从站地址 */
            if (param_addr == 0x0000) {
                if (param_val >= 1 && param_val <= 247) {
                    new_slave_addr = (uint8_t)param_val;
                    need_save = true;
                } else {
                    Modbus_SendException(func_code, 0x03);  /* 非法数据值 */
                    break;
                }
            }
            /* 参数地址 0x0001: 修改波特率 */
            else if (param_addr == 0x0001) {
                if (param_val >= 1 && param_val <= 8) {
                    new_baud_rate = BAUD_RATE_TABLE[param_val];
                    need_save = true;
                } else {
                    Modbus_SendException(func_code, 0x03);  /* 非法数据值 */
                    break;
                }
            }
            /* 其他地址: 预留扩展 */
            else {
                Modbus_SendException(func_code, 0x02);  /* 非法地址 */
                break;
            }

            if (need_save) {
                /* 步骤1: 先回复确认包，确保主站知道指令已执行 */
                memcpy(hmodbus.tx_buf, rx_buf, 8);
                Modbus_SendResponse(6);

                /* 步骤2: 等待数据发送完毕 (RS485模式关键步骤) */
                HAL_Delay(50);

                /* 步骤3: 保存配置到Flash */
                Flash_SaveConfig(new_slave_addr, new_baud_rate);

                /* 步骤4: 软复位系统，让新配置生效 */
                NVIC_SystemReset();
            }
            break;
        }

        default:
            /* 不支持的功能码，静默忽略 */
            break;
    }
    /* 乒乓缓冲机制: 无需重新启动接收，中断回调中已完成 */
}

/* ============================================================================
 *                              底层辅助函数
 * ============================================================================ */

/**
 * @brief   发送Modbus响应帧
 * @param   len   数据部分长度(不含CRC)
 */
static void Modbus_SendResponse(uint16_t len) {
    /* 计算并追加CRC校验码 */
    uint16_t crc = CRC16(hmodbus.tx_buf, len);
    hmodbus.tx_buf[len] = crc & 0xFF;
    hmodbus.tx_buf[len + 1] = (crc >> 8) & 0xFF;
    uint16_t total_len = len + 2;

    /* RS485方向控制: 切换为发送模式 */
    #if MODBUS_USE_RS485 == 1
        RS485_TX_ENABLE();
    #endif

    /* 阻塞式发送，确保数据完整发出 */
    HAL_UART_Transmit(hmodbus.huart, hmodbus.tx_buf, total_len, 100);

    /* RS485方向控制: 等待发送完成后切回接收模式 */
    #if MODBUS_USE_RS485 == 1
        /* 等待TC标志置位，确保最后一个字节发送完毕
         * 否则提前切换方向会切断最后一个字节 */
        while(__HAL_UART_GET_FLAG(hmodbus.huart, UART_FLAG_TC) == RESET);
        RS485_RX_ENABLE();
    #endif
}

/**
 * @brief   发送Modbus异常响应帧
 * @param   func_code       请求的功能码
 * @param   exception_code  异常码
 */
static void Modbus_SendException(uint8_t func_code, uint8_t exception_code) {
    hmodbus.tx_buf[0] = hmodbus.config.slave_addr;
    hmodbus.tx_buf[1] = func_code | 0x80;  /* 功能码最高位置1表示异常 */
    hmodbus.tx_buf[2] = exception_code;
    Modbus_SendResponse(3);
}

/**
 * @brief   标准CRC16-Modbus算法
 * @param   buffer          数据缓冲区指针
 * @param   buffer_length   数据长度
 * @return  16位CRC校验码
 */
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

/* ============================================================================
 *                              Flash配置存储函数
 * ============================================================================ */

/**
 * @brief   从Flash加载配置参数
 * @note    检查魔数标识判断数据有效性，无效则使用默认值
 */
static void Flash_LoadConfig(void) {
    ModbusConfig_t *pConfigInFlash = (ModbusConfig_t *)MODBUS_FLASH_ADDR;

    if (pConfigInFlash->magic_key == FLASH_MAGIC_KEY) {
        /* Flash数据有效，加载已存储的配置 */
        hmodbus.config = *pConfigInFlash;
    } else {
        /* Flash数据无效(首次使用)，加载默认值
         * 注意: 此处不写入Flash，仅在用户0x64指令时才写入 */
        hmodbus.config.magic_key = FLASH_MAGIC_KEY;
        hmodbus.config.slave_addr = DEFAULT_SLAVE_ADDR;
        hmodbus.config.baud_rate = DEFAULT_BAUD_RATE;
    }
}

/**
 * @brief   保存配置参数到Flash
 * @param   addr   新的从站地址
 * @param   baud   新的波特率
 * @note    为延长Flash寿命，只在数据确实改变时才执行写入操作
 * @warning Flash写入操作会暂停CPU执行，生产环境建议关闭中断或从RAM执行
 *          频繁调用此函数会迅速消耗Flash擦写寿命(通常1万-10万次)
 */
static void Flash_SaveConfig(uint8_t addr, uint32_t baud) {
    ModbusConfig_t new_config;
    new_config.magic_key = FLASH_MAGIC_KEY;
    new_config.slave_addr = addr;
    new_config.baud_rate = baud;

    /* 为延长Flash寿命，先读取当前Flash内容进行比较 */
    ModbusConfig_t *pCurrentConfig = (ModbusConfig_t *)MODBUS_FLASH_ADDR;
    if (pCurrentConfig->magic_key == FLASH_MAGIC_KEY &&
        pCurrentConfig->slave_addr == addr &&
        pCurrentConfig->baud_rate == baud) {
        /* 数据未改变，无需写入Flash */
        return;
    }

    HAL_FLASH_Unlock();

    /* 步骤1: 擦除Flash页/扇区 */
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    /* 根据不同STM32系列配置擦除参数
     * 请务必根据实际芯片型号修改此处代码 */
    #if defined(STM32F1)
        /* STM32F1系列: 按页擦除 */
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
        EraseInitStruct.PageAddress = MODBUS_FLASH_ADDR;
        EraseInitStruct.NbPages = 1;
    #elif defined(STM32F4)
        /* STM32F4系列: 按扇区擦除 */
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
        EraseInitStruct.Sector = FLASH_SECTOR_11;
        EraseInitStruct.NbSectors = 1;
        EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    #elif defined(STM32G0) || defined(STM32G4) || defined(STM32L4) || defined(STM32L5)
        /* STM32G0/G4/L4/L5系列: 按页擦除 */
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
        EraseInitStruct.PageAddress = MODBUS_FLASH_ADDR;
        EraseInitStruct.NbPages = 1;
    #elif defined(STM32H7)
        /* STM32H7系列: 按扇区擦除 */
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
        EraseInitStruct.Sector = FLASH_SECTOR_11;  /* 根据实际情况调整 */
        EraseInitStruct.NbSectors = 1;
        EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    #else
        /* 默认按页擦除 (适用于F0/L0等系列) */
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
        EraseInitStruct.PageAddress = MODBUS_FLASH_ADDR;
        EraseInitStruct.NbPages = 1;
    #endif

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
        HAL_FLASH_Lock();
        return;
    }

    /* 步骤2: 按字(32位)写入数据 */
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
