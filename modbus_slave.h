/*
 * modbus_slave.h
 * Modbus从机逻辑头文件 / Modbus Slave Logic Header
 * 
 * 说明: 定义Modbus协议栈的核心数据结构、功能码和外部接口
 * Note: Defines core data structures, function codes, and external interfaces for Modbus stack
 */

#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include "modbus_config.h"
#include <stdint.h>
#include <stdbool.h>

// ================= Modbus 功能码定义 / Modbus Function Code Definitions =================
#define MB_FUNC_READ_COILS           0x01  // 读线圈 / Read Coils
#define MB_FUNC_READ_DISCRETE        0x02  // 读离散输入 / Read Discrete Inputs
#define MB_FUNC_READ_HOLDING         0x03  // 读保持寄存器 / Read Holding Registers
#define MB_FUNC_READ_INPUT           0x04  // 读输入寄存器 / Read Input Registers
#define MB_FUNC_WRITE_SINGLE_COIL    0x05  // 写单个线圈 / Write Single Coil
#define MB_FUNC_WRITE_SINGLE_REG     0x06  // 写单个寄存器 / Write Single Register
#define MB_FUNC_WRITE_MULTI_COILS    0x0F  // 写多个线圈 / Write Multiple Coils
#define MB_FUNC_WRITE_MULTI_REGS     0x10  // 写多个寄存器 / Write Multiple Registers
#define MB_FUNC_CUSTOM_CONFIG        0x64  // 自定义功能码:修改配置 / Custom Function: Modify Configuration

// ================= Flash配置结构体 / Flash Configuration Structure =================
// 存储在Flash中的配置结构体 / Configuration structure stored in Flash
typedef struct {
    uint32_t magic_key;   // 魔数,用于判断是否初次初始化 (0xDEADBEEF) / Magic key for first-time initialization check
    uint8_t  slave_addr;  // 从机地址 (1-247) / Slave address (1-247)
    uint32_t baud_rate;   // 波特率 / Baud rate
    uint8_t  padding[3];  // 对齐填充 / Alignment padding
} ModbusConfig_t;

// ================= Modbus核心句柄结构体 / Modbus Core Handle Structure =================
typedef struct {
    UART_HandleTypeDef* huart;              // UART句柄指针 / UART handle pointer
    uint8_t rx_buf[MB_RX_BUF_SIZE];         // 接收缓冲区 / Receive buffer
    uint8_t tx_buf[MB_TX_BUF_SIZE];         // 发送缓冲区 / Transmit buffer
    uint16_t rx_len;                        // 接收数据长度 / Received data length
    ModbusConfig_t config;                  // 当前配置 / Current configuration
} ModbusHandle_t;

// ================= 全局数据映射 / Global Data Mapping =================
// 主程序通过修改这些数组来改变Modbus数据 / Main program modifies these arrays to change Modbus data
// 线圈使用位压缩存储 / Coils use bit-packed storage
extern uint8_t  mb_coils[MB_COIL_COUNT / 8 + 1];           // 线圈区域 (可读写) / Coil area (read/write)
extern uint8_t  mb_discrete_inputs[MB_DISCRETE_COUNT / 8 + 1]; // 离散输入区域 (只读) / Discrete input area (read-only)
extern uint16_t mb_holding_regs[MB_HOLDING_REG_COUNT];     // 保持寄存器区域 (可读写) / Holding register area (read/write)
extern uint16_t mb_input_regs[MB_INPUT_REG_COUNT];         // 输入寄存器区域 (只读) / Input register area (read-only)

// ================= 函数声明 / Function Declarations =================
/**
 * @brief  初始化Modbus从机 / Initialize Modbus slave
 * @note   必须在主函数开始时调用,会从Flash加载配置并启动UART接收
 *         Must be called at the beginning of main function, loads config from Flash and starts UART reception
 */
void Modbus_Init(void);

/**
 * @brief  Modbus主处理函数 / Modbus main processing function
 * @note   在主循环中循环调用,处理接收到的Modbus帧
 *         Call continuously in main loop to process received Modbus frames
 */
void Modbus_Process(void);

/**
 * @brief  UART接收完成回调函数 / UART receive complete callback function
 * @param  huart: UART句柄 / UART handle
 * @param  Size: 接收到的数据长度 / Length of received data
 * @note   必须放入HAL_UARTEx_RxEventCallback中调用
 *         Must be called within HAL_UARTEx_RxEventCallback
 */
void Modbus_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif // MODBUS_SLAVE_H
