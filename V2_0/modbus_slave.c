/**
 * @file    modbus_slave.c
 * @brief   Modbus从站协议栈实现 (V2.0 多实例架构)
 * @version 2.0.0
 * @date    2025
 * 
 * @details 本文件实现Modbus RTU从站核心功能:
 *          - 多实例支持，无全局变量
 *          - 乒乓缓冲接收机制
 *          - 动态超时计算
 *          - 可选CRC查表法/移位法
 *          - 可选DMA/阻塞发送
 */

#include "modbus_slave.h"
#include <string.h>

/* ============================================================================
 *                              CRC16查找表
 * ============================================================================ */

/**
 * @brief   CRC16-Modbus查找表 (512字节)
 * @note    仅在 use_crc_table = true 时使用
 */
static const uint16_t CRC16_TABLE[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/* ============================================================================
 *                              内部函数声明
 * ============================================================================ */

static uint16_t Modbus_CRC16(const ModbusHandle_t *hmodbus, const uint8_t *buffer, uint16_t length);
static void Modbus_SendResponse(ModbusHandle_t *hmodbus, uint16_t len);
static void Modbus_SendException(ModbusHandle_t *hmodbus, uint8_t func_code, uint8_t exception_code);
static void Modbus_RS485_SetTxMode(const ModbusHandle_t *hmodbus);
static void Modbus_RS485_SetRxMode(const ModbusHandle_t *hmodbus);

/* ============================================================================
 *                              初始化函数
 * ============================================================================ */

/**
 * @brief   初始化Modbus从站实例
 */
bool Modbus_Init(ModbusHandle_t *hmodbus, const Modbus_Config_t *config) {
    /* 参数有效性检查 */
    if (hmodbus == NULL || config == NULL) {
        return false;
    }
    
    if (config->huart == NULL) {
        return false;
    }
    
    /* 缓冲区有效性检查 */
    if (config->buffer.rx_buf_a == NULL || 
        config->buffer.rx_buf_b == NULL || 
        config->buffer.tx_buf == NULL) {
        return false;
    }
    
    if (config->buffer.rx_buf_size < 8 || config->buffer.tx_buf_size < 8) {
        return false;  /* 缓冲区过小 */
    }
    
    /* 从站地址有效性检查 */
    if (config->slave_addr < 1 || config->slave_addr > 247) {
        return false;
    }
    
    /* 复制UART配置 */
    hmodbus->huart = config->huart;
    hmodbus->slave_addr = config->slave_addr;
    hmodbus->baud_rate = config->baud_rate;
    
    /* 复制缓冲区配置 */
    hmodbus->rx_buf_a = config->buffer.rx_buf_a;
    hmodbus->rx_buf_b = config->buffer.rx_buf_b;
    hmodbus->tx_buf = config->buffer.tx_buf;
    hmodbus->rx_buf_size = config->buffer.rx_buf_size;
    hmodbus->tx_buf_size = config->buffer.tx_buf_size;
    
    /* 初始化乒乓缓冲状态 */
    hmodbus->rx_active_buf = hmodbus->rx_buf_a;
    hmodbus->rx_process_buf = hmodbus->rx_buf_b;
    hmodbus->rx_len = 0;
    hmodbus->rx_ready = 0;
    
    /* 复制数据映射 */
    hmodbus->data_map = config->data_map;
    
    /* 复制RS485配置 */
    hmodbus->rs485 = config->rs485;
    
    /* 复制运行时选项 */
    hmodbus->use_dma_tx = config->use_dma_tx;
    hmodbus->use_crc_table = config->use_crc_table;
    
    /* 复制回调函数 */
    hmodbus->custom_config_cb = config->custom_config_cb;
    hmodbus->write_cb = config->write_cb;
    
    /* 初始化用户数据指针 */
    hmodbus->user_data = NULL;
    
    /* RS485初始化: 默认为接收模式 */
    if (hmodbus->rs485.enabled) {
        Modbus_RS485_SetRxMode(hmodbus);
    }
    
    /* 启动UART空闲中断接收 */
    Modbus_StartReceive(hmodbus);
    
    return true;
}

/* ============================================================================
 *                              中断回调接口
 * ============================================================================ */

/**
 * @brief   UART接收完成/空闲中断回调
 */
void Modbus_RxCallback(ModbusHandle_t *hmodbus, uint16_t size) {
    if (hmodbus == NULL) {
        return;
    }
    
    /* 乒乓缓冲切换:
     * 1. 将当前接收缓冲区设为处理缓冲区
     * 2. 记录接收数据长度
     * 3. 切换中断接收目标到另一个缓冲区
     * 4. 立即重新启动接收，减少数据丢失窗口 */
    
    /* 交换缓冲区指针 */
    uint8_t *completed_buf = hmodbus->rx_active_buf;
    hmodbus->rx_active_buf = (completed_buf == hmodbus->rx_buf_a) 
                            ? hmodbus->rx_buf_b : hmodbus->rx_buf_a;
    
    /* 设置处理缓冲区和数据长度 */
    hmodbus->rx_process_buf = completed_buf;
    hmodbus->rx_len = size;
    hmodbus->rx_ready = 1;
    
    /* 立即重新启动接收，指向新的缓冲区 */
    HAL_UARTEx_ReceiveToIdle_IT(hmodbus->huart, hmodbus->rx_active_buf, hmodbus->rx_buf_size);
}

/**
 * @brief   UART发送完成回调 (DMA模式)
 */
void Modbus_TxCallback(ModbusHandle_t *hmodbus) {
    if (hmodbus == NULL) {
        return;
    }
    
    if (hmodbus->use_dma_tx && hmodbus->rs485.enabled) {
        /* DMA发送完成，等待TC标志后切换RS485为接收模式
         * 注意: DMA完成中断时数据已全部移入发送缓冲，
         * 但需等待TC标志确保最后一个字节完全发出 */
        while (__HAL_UART_GET_FLAG(hmodbus->huart, UART_FLAG_TC) == RESET);
        Modbus_RS485_SetRxMode(hmodbus);
    }
}

/* ============================================================================
 *                              主处理函数
 * ============================================================================ */

/**
 * @brief   Modbus帧解析与响应处理
 */
void Modbus_Process(ModbusHandle_t *hmodbus) {
    if (hmodbus == NULL) {
        return;
    }
    
    if (!hmodbus->rx_ready) {
        return;  /* 无数据待处理 */
    }
    
    /* 获取待处理数据的本地副本 */
    uint8_t *rx_buf = hmodbus->rx_process_buf;
    uint16_t rx_len = hmodbus->rx_len;
    
    /* 清除接收标志，允许中断更新新数据 */
    hmodbus->rx_ready = 0;
    hmodbus->rx_len = 0;
    
    /* 步骤1: 检查帧最小长度 */
    if (rx_len < 4) {
        return;  /* 帧长度不足 */
    }
    
    /* 步骤2: 校验从站地址
     * 支持本机地址咀xFF广播地址(带返回) */
    uint8_t req_addr = rx_buf[0];
    if (req_addr != hmodbus->slave_addr && req_addr != 0xFF) {
        return;  /* 地址不匹配 */
    }
    
    /* 步骤3: CRC校验 */
    uint16_t received_crc = (rx_buf[rx_len - 1] << 8) | rx_buf[rx_len - 2];
    uint16_t calculated_crc = Modbus_CRC16(hmodbus, rx_buf, rx_len - 2);
    
    if (received_crc != calculated_crc) {
        return;  /* CRC错误 */
    }
    
    /* 步骤4: 解析功能码 */
    uint8_t func_code = rx_buf[1];
    
    /* 准备响应帧头部: 从站地址 + 功能码
     * 无论请求地址是0xFF还是本机地址，响应始终使用本机真实地址 */
    hmodbus->tx_buf[0] = hmodbus->slave_addr;
    hmodbus->tx_buf[1] = func_code;
    
    /* 功能码处理 */
    uint16_t start_addr, quantity;
    uint8_t byte_count;
    uint16_t i;
    
    switch (func_code) {
        /* ================================================================
         *                      线圈处理 (Coils)
         *                  功能码: 0x01, 0x05, 0x0F
         * ================================================================ */
        case MB_FUNC_READ_COILS:  /* 0x01: 读线圈状态 */
        {
            /* 检查是否支持此功能 */
            if (hmodbus->data_map.coils == NULL || hmodbus->data_map.coil_count == 0) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_FUNCTION);
                break;
            }
            
            start_addr = (rx_buf[2] << 8) | rx_buf[3];
            quantity = (rx_buf[4] << 8) | rx_buf[5];
            
            /* 参数校验 */
            if (quantity < 1 || quantity > 2000) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_DATA_VALUE);
                break;
            }
            if (start_addr + quantity > hmodbus->data_map.coil_count) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_DATA_ADDRESS);
                break;
            }
            
            byte_count = (quantity + 7) / 8;
            hmodbus->tx_buf[2] = byte_count;
            memset(&hmodbus->tx_buf[3], 0, byte_count);
            
            /* 读取线圈状态并打包响应
             * 注意: 访问共享数据时，在高安全要求场景应使用临界区保护 */
            for (i = 0; i < quantity; i++) {
                uint16_t bit_idx = start_addr + i;
                if (hmodbus->data_map.coils[bit_idx / 8] & (1 << (bit_idx % 8))) {
                    hmodbus->tx_buf[3 + i / 8] |= (1 << (i % 8));
                }
            }
            Modbus_SendResponse(hmodbus, 3 + byte_count);
            break;
        }
        
        case MB_FUNC_WRITE_SINGLE_COIL:  /* 0x05: 写单个线圈 */
        {
            if (hmodbus->data_map.coils == NULL || hmodbus->data_map.coil_count == 0) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_FUNCTION);
                break;
            }
            
            start_addr = (rx_buf[2] << 8) | rx_buf[3];
            uint16_t val = (rx_buf[4] << 8) | rx_buf[5];
            
            if (start_addr >= hmodbus->data_map.coil_count) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_DATA_ADDRESS);
                break;
            }
            
            /* 写入前回调检查 */
            if (hmodbus->write_cb != NULL) {
                if (!hmodbus->write_cb(hmodbus, func_code, start_addr, 1)) {
                    Modbus_SendException(hmodbus, func_code, MB_EX_SLAVE_DEVICE_FAILURE);
                    break;
                }
            }
            
            /* 0xFF00=置位, 0x0000=复位
             * 注意: 写入共享数据时，在高安全要求场景应使用原子操作 */
            if (val == 0xFF00) {
                hmodbus->data_map.coils[start_addr / 8] |= (1 << (start_addr % 8));
            } else if (val == 0x0000) {
                hmodbus->data_map.coils[start_addr / 8] &= ~(1 << (start_addr % 8));
            }
            
            /* 原样返回请求帧 */
            memcpy(hmodbus->tx_buf, rx_buf, 6);
            hmodbus->tx_buf[0] = hmodbus->slave_addr;
            Modbus_SendResponse(hmodbus, 6);
            break;
        }
        
        case MB_FUNC_WRITE_MULTI_COILS:  /* 0x0F: 写多个线圈 */
        {
            if (hmodbus->data_map.coils == NULL || hmodbus->data_map.coil_count == 0) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_FUNCTION);
                break;
            }
            
            start_addr = (rx_buf[2] << 8) | rx_buf[3];
            quantity = (rx_buf[4] << 8) | rx_buf[5];
            byte_count = rx_buf[6];
            
            if (start_addr + quantity > hmodbus->data_map.coil_count) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_DATA_ADDRESS);
                break;
            }
            
            /* 写入前回调检查 */
            if (hmodbus->write_cb != NULL) {
                if (!hmodbus->write_cb(hmodbus, func_code, start_addr, quantity)) {
                    Modbus_SendException(hmodbus, func_code, MB_EX_SLAVE_DEVICE_FAILURE);
                    break;
                }
            }
            
            /* 写入线圈状态
             * 注意: 批量写入共享数据时，建议使用临界区保护 */
            for (i = 0; i < quantity; i++) {
                uint8_t val = (rx_buf[7 + i / 8] >> (i % 8)) & 0x01;
                if (val) {
                    hmodbus->data_map.coils[(start_addr + i) / 8] |= (1 << ((start_addr + i) % 8));
                } else {
                    hmodbus->data_map.coils[(start_addr + i) / 8] &= ~(1 << ((start_addr + i) % 8));
                }
            }
            
            memcpy(&hmodbus->tx_buf[2], &rx_buf[2], 4);
            Modbus_SendResponse(hmodbus, 6);
            break;
        }
        
        /* ================================================================
         *                    离散输入处理 (Discrete Inputs)
         *                        功能码: 0x02
         * ================================================================ */
        case MB_FUNC_READ_DISCRETE:  /* 0x02: 读离散输入 */
        {
            if (hmodbus->data_map.discrete_inputs == NULL || hmodbus->data_map.discrete_count == 0) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_FUNCTION);
                break;
            }
            
            start_addr = (rx_buf[2] << 8) | rx_buf[3];
            quantity = (rx_buf[4] << 8) | rx_buf[5];
            
            if (quantity < 1 || quantity > 2000) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_DATA_VALUE);
                break;
            }
            if (start_addr + quantity > hmodbus->data_map.discrete_count) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_DATA_ADDRESS);
                break;
            }
            
            byte_count = (quantity + 7) / 8;
            hmodbus->tx_buf[2] = byte_count;
            memset(&hmodbus->tx_buf[3], 0, byte_count);
            
            /* 读取离散输入状态 */
            for (i = 0; i < quantity; i++) {
                uint16_t bit_idx = start_addr + i;
                if (hmodbus->data_map.discrete_inputs[bit_idx / 8] & (1 << (bit_idx % 8))) {
                    hmodbus->tx_buf[3 + i / 8] |= (1 << (i % 8));
                }
            }
            Modbus_SendResponse(hmodbus, 3 + byte_count);
            break;
        }
        
        /* ================================================================
         *                   保持寄存器处理 (Holding Registers)
         *                   功能码: 0x03, 0x06, 0x10
         * ================================================================ */
        case MB_FUNC_READ_HOLDING:  /* 0x03: 读保持寄存器 */
        {
            if (hmodbus->data_map.holding_regs == NULL || hmodbus->data_map.holding_reg_count == 0) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_FUNCTION);
                break;
            }
            
            start_addr = (rx_buf[2] << 8) | rx_buf[3];
            quantity = (rx_buf[4] << 8) | rx_buf[5];
            
            if (quantity < 1 || quantity > 125) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_DATA_VALUE);
                break;
            }
            if (start_addr + quantity > hmodbus->data_map.holding_reg_count) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_DATA_ADDRESS);
                break;
            }
            
            hmodbus->tx_buf[2] = quantity * 2;
            
            /* 读取保持寄存器数据
             * 注意: 对于16位寄存器，Cortex-M内核可保证单次读取的原子性 */
            for (i = 0; i < quantity; i++) {
                hmodbus->tx_buf[3 + i * 2] = (hmodbus->data_map.holding_regs[start_addr + i] >> 8) & 0xFF;
                hmodbus->tx_buf[4 + i * 2] = hmodbus->data_map.holding_regs[start_addr + i] & 0xFF;
            }
            Modbus_SendResponse(hmodbus, 3 + quantity * 2);
            break;
        }
        
        case MB_FUNC_WRITE_SINGLE_REG:  /* 0x06: 写单个寄存器 */
        {
            if (hmodbus->data_map.holding_regs == NULL || hmodbus->data_map.holding_reg_count == 0) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_FUNCTION);
                break;
            }
            
            start_addr = (rx_buf[2] << 8) | rx_buf[3];
            uint16_t val = (rx_buf[4] << 8) | rx_buf[5];
            
            if (start_addr >= hmodbus->data_map.holding_reg_count) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_DATA_ADDRESS);
                break;
            }
            
            /* 写入前回调检查 */
            if (hmodbus->write_cb != NULL) {
                if (!hmodbus->write_cb(hmodbus, func_code, start_addr, 1)) {
                    Modbus_SendException(hmodbus, func_code, MB_EX_SLAVE_DEVICE_FAILURE);
                    break;
                }
            }
            
            /* 写入寄存器
             * 注意: 寴于16位寄存器，Cortex-M内核可保证单次写入的原子性 */
            hmodbus->data_map.holding_regs[start_addr] = val;
            
            /* 原样返回请求帧 */
            memcpy(hmodbus->tx_buf, rx_buf, 6);
            hmodbus->tx_buf[0] = hmodbus->slave_addr;
            Modbus_SendResponse(hmodbus, 6);
            break;
        }
        
        case MB_FUNC_WRITE_MULTI_REGS:  /* 0x10: 写多个寄存器 */
        {
            if (hmodbus->data_map.holding_regs == NULL || hmodbus->data_map.holding_reg_count == 0) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_FUNCTION);
                break;
            }
            
            start_addr = (rx_buf[2] << 8) | rx_buf[3];
            quantity = (rx_buf[4] << 8) | rx_buf[5];
            
            if (start_addr + quantity > hmodbus->data_map.holding_reg_count) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_DATA_ADDRESS);
                break;
            }
            
            /* 写入前回调检查 */
            if (hmodbus->write_cb != NULL) {
                if (!hmodbus->write_cb(hmodbus, func_code, start_addr, quantity)) {
                    Modbus_SendException(hmodbus, func_code, MB_EX_SLAVE_DEVICE_FAILURE);
                    break;
                }
            }
            
            /* 写入保持寄存器数据
             * 注意: 批量写入共享数据时，建议使用临界区保护 */
            for (i = 0; i < quantity; i++) {
                hmodbus->data_map.holding_regs[start_addr + i] = 
                    (rx_buf[7 + i * 2] << 8) | rx_buf[8 + i * 2];
            }
            
            memcpy(&hmodbus->tx_buf[2], &rx_buf[2], 4);
            Modbus_SendResponse(hmodbus, 6);
            break;
        }
        
        /* ================================================================
         *                    输入寄存器处理 (Input Registers)
         *                         功能码: 0x04
         * ================================================================ */
        case MB_FUNC_READ_INPUT:  /* 0x04: 读输入寄存器 */
        {
            if (hmodbus->data_map.input_regs == NULL || hmodbus->data_map.input_reg_count == 0) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_FUNCTION);
                break;
            }
            
            start_addr = (rx_buf[2] << 8) | rx_buf[3];
            quantity = (rx_buf[4] << 8) | rx_buf[5];
            
            if (quantity < 1 || quantity > 125) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_DATA_VALUE);
                break;
            }
            if (start_addr + quantity > hmodbus->data_map.input_reg_count) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_DATA_ADDRESS);
                break;
            }
            
            hmodbus->tx_buf[2] = quantity * 2;
            
            /* 读取输入寄存器数据 */
            for (i = 0; i < quantity; i++) {
                hmodbus->tx_buf[3 + i * 2] = (hmodbus->data_map.input_regs[start_addr + i] >> 8) & 0xFF;
                hmodbus->tx_buf[4 + i * 2] = hmodbus->data_map.input_regs[start_addr + i] & 0xFF;
            }
            Modbus_SendResponse(hmodbus, 3 + quantity * 2);
            break;
        }
        
        /* ================================================================
         *                    自定义配置功能码 (0x64)
         *             通过回调函数实现，应用层处理Flash等操作
         * ================================================================ */
        case MB_FUNC_CUSTOM_CONFIG:  /* 0x64: 自定义配置指令 */
        {
            /* 帧格式: [Addr][64][RegHi][RegLo][ValHi][ValLo][CRC][CRC]
             * 长度固定为8字节 */
            if (rx_len != 8) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_DATA_VALUE);
                break;
            }
            
            uint16_t param_addr = (rx_buf[2] << 8) | rx_buf[3];
            uint16_t param_val  = (rx_buf[4] << 8) | rx_buf[5];
            
            /* 检查是否注册了回调函数 */
            if (hmodbus->custom_config_cb == NULL) {
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_FUNCTION);
                break;
            }
            
            /* 调用回调函数处理配置
             * 回调内部应处理:
             * - param_addr=0x0000: 修改从站地址
             * - param_addr=0x0001: 修改波特率
             * - 其他地址: 用户自定义参数 */
            if (hmodbus->custom_config_cb(hmodbus, param_addr, param_val)) {
                /* 处理成功，发送确认响应 */
                memcpy(hmodbus->tx_buf, rx_buf, 6);
                hmodbus->tx_buf[0] = hmodbus->slave_addr;
                Modbus_SendResponse(hmodbus, 6);
            } else {
                /* 处理失败，发送异常响应 */
                Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_DATA_VALUE);
            }
            break;
        }
        
        default:
            /* 不支持的功能码 */
            Modbus_SendException(hmodbus, func_code, MB_EX_ILLEGAL_FUNCTION);
            break;
    }
}

/* ============================================================================
 *                              配置接口函数
 * ============================================================================ */

/**
 * @brief   更新从站地址
 */
void Modbus_SetSlaveAddr(ModbusHandle_t *hmodbus, uint8_t addr) {
    if (hmodbus != NULL && addr >= 1 && addr <= 247) {
        hmodbus->slave_addr = addr;
    }
}

/**
 * @brief   更新波特率
 */
void Modbus_SetBaudRate(ModbusHandle_t *hmodbus, uint32_t baud_rate) {
    if (hmodbus != NULL && baud_rate > 0) {
        hmodbus->baud_rate = baud_rate;
    }
}

/**
 * @brief   启动UART接收
 */
void Modbus_StartReceive(ModbusHandle_t *hmodbus) {
    if (hmodbus != NULL && hmodbus->huart != NULL) {
        HAL_UARTEx_ReceiveToIdle_IT(hmodbus->huart, hmodbus->rx_active_buf, hmodbus->rx_buf_size);
    }
}

/* ============================================================================
 *                              底层辅助函数
 * ============================================================================ */

/**
 * @brief   计算CRC16校验码
 * @param   hmodbus Modbus句柄指针
 * @param   buffer  数据缓冲区指针
 * @param   length  数据长度
 * @return  16位CRC校验码
 */
static uint16_t Modbus_CRC16(const ModbusHandle_t *hmodbus, const uint8_t *buffer, uint16_t length) {
    uint16_t crc = 0xFFFF;
    
    if (hmodbus->use_crc_table) {
        /* 查表法: 速度快约10倍 */
        for (uint16_t i = 0; i < length; i++) {
            crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ buffer[i]) & 0xFF];
        }
    } else {
        /* 移位法: 代码体积小 */
        for (uint16_t i = 0; i < length; i++) {
            crc ^= buffer[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 1) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
    }
    
    return crc;
}

/**
 * @brief   RS485切换到发送模式
 */
static void Modbus_RS485_SetTxMode(const ModbusHandle_t *hmodbus) {
    if (hmodbus->rs485.enabled && hmodbus->rs485.de_port != NULL) {
        if (hmodbus->rs485.de_polarity) {
            HAL_GPIO_WritePin(hmodbus->rs485.de_port, hmodbus->rs485.de_pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(hmodbus->rs485.de_port, hmodbus->rs485.de_pin, GPIO_PIN_RESET);
        }
    }
}

/**
 * @brief   RS485切换到接收模式
 */
static void Modbus_RS485_SetRxMode(const ModbusHandle_t *hmodbus) {
    if (hmodbus->rs485.enabled && hmodbus->rs485.de_port != NULL) {
        if (hmodbus->rs485.de_polarity) {
            HAL_GPIO_WritePin(hmodbus->rs485.de_port, hmodbus->rs485.de_pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(hmodbus->rs485.de_port, hmodbus->rs485.de_pin, GPIO_PIN_SET);
        }
    }
}

/**
 * @brief   发送Modbus响应帧
 * @param   hmodbus Modbus句柄指针
 * @param   len     数据部分长度(不含CRC)
 */
static void Modbus_SendResponse(ModbusHandle_t *hmodbus, uint16_t len) {
    /* 检查发送缓冲区大小 */
    if (len + 2 > hmodbus->tx_buf_size) {
        return;  /* 缓冲区不足 */
    }
    
    /* 计算并追加CRC校验码 */
    uint16_t crc = Modbus_CRC16(hmodbus, hmodbus->tx_buf, len);
    hmodbus->tx_buf[len] = crc & 0xFF;
    hmodbus->tx_buf[len + 1] = (crc >> 8) & 0xFF;
    uint16_t total_len = len + 2;
    
    /* RS485方向控制: 切换为发送模式 */
    Modbus_RS485_SetTxMode(hmodbus);
    
    if (hmodbus->use_dma_tx) {
        /* DMA模式: 非阻塞发送，释放CPU
         * RS485方向切换在 Modbus_TxCallback 中处理 */
        HAL_UART_Transmit_DMA(hmodbus->huart, hmodbus->tx_buf, total_len);
    } else {
        /* 阻塞模式: 动态计算超时时间 (ms)
         * 公式: timeout = (字节数 * 10位 * 1000ms) / 波特率 + 安全余量
         * 10位 = 1起始位 + 8数据位 + 1停止位
         * 安全余量: +50ms 或 +10% (取较大值) */
        uint32_t tx_time_ms = ((uint32_t)total_len * 10 * 1000) / hmodbus->baud_rate;
        uint32_t margin = (tx_time_ms / 10) > 50 ? (tx_time_ms / 10) : 50;
        uint32_t timeout = tx_time_ms + margin;
        
        /* 确保最小超时时间为100ms */
        if (timeout < 100) {
            timeout = 100;
        }
        
        /* 阻塞式发送 */
        HAL_UART_Transmit(hmodbus->huart, hmodbus->tx_buf, total_len, timeout);
        
        /* RS485方向控制: 等待TC标志后切回接收模式 */
        if (hmodbus->rs485.enabled) {
            while (__HAL_UART_GET_FLAG(hmodbus->huart, UART_FLAG_TC) == RESET);
            Modbus_RS485_SetRxMode(hmodbus);
        }
    }
}

/**
 * @brief   发送Modbus异常响应帧
 * @param   hmodbus        Modbus句柄指针
 * @param   func_code      请求的功能码
 * @param   exception_code 异常码
 */
static void Modbus_SendException(ModbusHandle_t *hmodbus, uint8_t func_code, uint8_t exception_code) {
    hmodbus->tx_buf[0] = hmodbus->slave_addr;
    hmodbus->tx_buf[1] = func_code | 0x80;  /* 功能码最高位置1表示异常 */
    hmodbus->tx_buf[2] = exception_code;
    Modbus_SendResponse(hmodbus, 3);
}
