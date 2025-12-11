/**
 * @file    modbus_slave.h
 * @brief   Modbus从站协议栈头文件
 * @details 定义数据结构、功能码、全局数据映射及接口函数
 */

#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include "modbus_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 *                              Modbus功能码定义
 * ============================================================================ */

#define MB_FUNC_READ_COILS           0x01    /* 读线圈状态 */
#define MB_FUNC_READ_DISCRETE        0x02    /* 读离散输入 */
#define MB_FUNC_READ_HOLDING         0x03    /* 读保持寄存器 */
#define MB_FUNC_READ_INPUT           0x04    /* 读输入寄存器 */
#define MB_FUNC_WRITE_SINGLE_COIL    0x05    /* 写单个线圈 */
#define MB_FUNC_WRITE_SINGLE_REG     0x06    /* 写单个寄存器 */
#define MB_FUNC_WRITE_MULTI_COILS    0x0F    /* 写多个线圈 */
#define MB_FUNC_WRITE_MULTI_REGS     0x10    /* 写多个寄存器 */
#define MB_FUNC_CUSTOM_CONFIG        0x64    /* 自定义功能码: 修改从站配置 */

/* ============================================================================
 *                              数据结构定义
 * ============================================================================ */

/**
 * @brief   Flash存储的配置参数结构体
 * @note    结构体大小需为4字节对齐，便于Flash按字写入
 * @warning Flash写入操作会暂停CPU执行，频繁写入会消耗Flash寿命
 */
typedef struct {
    uint32_t magic_key;     /* 魔数标识，用于判断Flash数据有效性 (0xDEADBEEF) */
    uint8_t  slave_addr;    /* 从站地址 (有效范围: 1~247) */
    uint32_t baud_rate;     /* 波特率 */
    uint8_t  padding[3];    /* 对齐填充，确保结构体大小为4的倍数 */
} ModbusConfig_t;

/**
 * @brief   Modbus从站核心句柄结构体
 * @note    包含UART句柄、收发缓冲区及运行时配置
 */
typedef struct {
    UART_HandleTypeDef* huart;              /* UART外设句柄 */
    uint8_t rx_buf[MB_RX_BUF_SIZE];         /* 接收缓冲区 */
    uint8_t tx_buf[MB_TX_BUF_SIZE];         /* 发送缓冲区 */
    uint16_t rx_len;                        /* 当前接收数据长度 */
    ModbusConfig_t config;                  /* 运行时配置参数 */
} ModbusHandle_t;

/* ============================================================================
 *                              全局数据映射
 * ============================================================================ */

/**
 * @brief   Modbus数据区全局变量声明
 * @note    用户程序通过访问这些数组来与Modbus主站交互数据
 *          线圈和离散输入采用位压缩存储，每字节存储8个状态
 */

#if MB_COIL_COUNT > 0
extern uint8_t  mb_coils[(MB_COIL_COUNT + 7) / 8];          /* 线圈数据区 (可读写) */
#endif

#if MB_DISCRETE_COUNT > 0
extern uint8_t  mb_discrete_inputs[(MB_DISCRETE_COUNT + 7) / 8];  /* 离散输入数据区 (只读) */
#endif

#if MB_HOLDING_REG_COUNT > 0
extern uint16_t mb_holding_regs[MB_HOLDING_REG_COUNT];      /* 保持寄存器数据区 (可读写) */
#endif

#if MB_INPUT_REG_COUNT > 0
extern uint16_t mb_input_regs[MB_INPUT_REG_COUNT];          /* 输入寄存器数据区 (只读) */
#endif

/* ============================================================================
 *                              接口函数声明
 * ============================================================================ */

/**
 * @brief   Modbus从站初始化
 * @note    应在系统启动时调用，完成以下操作：
 *          1. 初始化RS485方向控制(如启用)
 *          2. 从Flash加载配置参数
 *          3. 根据配置重新初始化UART波特率
 *          4. 开启空闲中断接收
 */
void Modbus_Init(void);

/**
 * @brief   Modbus主处理函数
 * @note    应在主循环中周期性调用，用于解析并响应Modbus请求
 */
void Modbus_Process(void);

/**
 * @brief   UART接收完成回调函数
 * @param   huart   UART句柄指针
 * @param   Size    接收到的数据长度
 * @note    需在 HAL_UARTEx_RxEventCallback() 中调用此函数
 *          示例:
 *          void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
 *              Modbus_RxCpltCallback(huart, Size);
 *          }
 */
void Modbus_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif /* MODBUS_SLAVE_H */
