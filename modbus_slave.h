/*
* modbus_slave.h
 * Modbus Slave Logic Header
 */

#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include "modbus_config.h"
#include <stdint.h>
#include <stdbool.h>

// Modbus 功能码
#define MB_FUNC_READ_COILS           0x01
#define MB_FUNC_READ_DISCRETE        0x02
#define MB_FUNC_READ_HOLDING         0x03
#define MB_FUNC_READ_INPUT           0x04
#define MB_FUNC_WRITE_SINGLE_COIL    0x05
#define MB_FUNC_WRITE_SINGLE_REG     0x06
#define MB_FUNC_WRITE_MULTI_COILS    0x0F
#define MB_FUNC_WRITE_MULTI_REGS     0x10
#define MB_FUNC_CUSTOM_CONFIG        0x64  // 自定义功能码：修改配置

// 存储在Flash中的配置结构体
typedef struct {
    uint32_t magic_key;   // 用于判断是否初次初始化 (0xDEADBEEF)
    uint8_t  slave_addr;  // 从机地址
    uint32_t baud_rate;   // 波特率
    uint8_t  padding[3];  // 对齐填充
} ModbusConfig_t;

// Modbus 核心结构体
typedef struct {
    UART_HandleTypeDef* huart;
    uint8_t rx_buf[MB_RX_BUF_SIZE];
    uint8_t tx_buf[MB_TX_BUF_SIZE];
    uint16_t rx_len;
    ModbusConfig_t config;
} ModbusHandle_t;

// ================= 全局数据映射 (外部可访问) =================
// 主程序通过修改这些数组来改变Modbus数据
// 线圈使用位压缩存储
#if MB_COIL_COUNT > 0
extern uint8_t  mb_coils[(MB_COIL_COUNT + 7) / 8];
#endif

#if MB_DISCRETE_COUNT > 0
extern uint8_t  mb_discrete_inputs[(MB_DISCRETE_COUNT + 7) / 8];
#endif

#if MB_HOLDING_REG_COUNT > 0
extern uint16_t mb_holding_regs[MB_HOLDING_REG_COUNT];
#endif

#if MB_INPUT_REG_COUNT > 0
extern uint16_t mb_input_regs[MB_INPUT_REG_COUNT];
#endif

// ================= 函数声明 =================
void Modbus_Init(void);
void Modbus_Process(void); // 在主循环中调用
void Modbus_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size); // 放入 HAL_UARTEx_RxEventCallback

#endif // MODBUS_SLAVE_H
