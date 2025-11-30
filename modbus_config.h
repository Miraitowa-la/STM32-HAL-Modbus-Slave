/*
 * modbus_config.h
 * Modbus从机配置文件 / Modbus Slave Configuration File
 * 
 * 说明: 本文件包含所有用户可配置的硬件参数和协议参数
 * Note: This file contains all user-configurable hardware and protocol parameters
 */

#ifndef MODBUS_CONFIG_H
#define MODBUS_CONFIG_H

#include "main.h" // 确保包含HAL库定义和UART句柄 / Ensure HAL library definitions and UART handle are included

// ================= 硬件配置 / Hardware Configuration =================
// 指定使用的UART句柄,例如 &huart1
// Specify the UART handle to use, e.g., &huart1
extern UART_HandleTypeDef 			huart1;
#define MODBUS_UART_HANDLE      &huart1

// ================= 物理层接口配置 / Physical Layer Interface Configuration =================
// 0: RS232 或 TTL (无流控引脚,全双工/透明传输)
// 0: RS232 or TTL (no flow control pins, full-duplex/transparent transmission)
// 1: RS485 (半双工,需要控制 DE/RE 引脚)
// 1: RS485 (half-duplex, requires DE/RE pin control)
#define MODBUS_USE_RS485        0

// --- 仅当 MODBUS_USE_RS485 为 1 时需要配置以下内容 ---
// --- Configure the following only when MODBUS_USE_RS485 is set to 1 ---
// 请在 CubeMX 中将该引脚配置为 GPIO_Output
// Configure this pin as GPIO_Output in CubeMX
#define RS485_PORT              GPIOC
#define RS485_PIN               GPIO_PIN_13

// RS485 逻辑电平定义 (通常: High=发送, Low=接收)
// RS485 logic level definition (typically: High=Transmit, Low=Receive)
#define RS485_TX_ENABLE()       HAL_GPIO_WritePin(RS485_PORT, RS485_PIN, GPIO_PIN_SET)
#define RS485_RX_ENABLE()       HAL_GPIO_WritePin(RS485_PORT, RS485_PIN, GPIO_PIN_RESET)

// ================= Flash 存储配置 / Flash Storage Configuration =================
// 请根据你的STM32型号选择一个扇区或页面的起始地址用于存储配置
// Select a sector or page start address based on your STM32 model for storing configuration
// 注意:该地址的数据在修改波特率/地址时会被擦除!
// Warning: Data at this address will be erased when modifying baud rate/address!
// 示例 (STM32F103C8T6 最后一页) / Example (STM32F103C8T6 last page): 0x0800F800
// 示例 (STM32F407 扇区11) / Example (STM32F407 Sector 11): 0x080E0000
#define MODBUS_FLASH_ADDR       0x0800F800 

// ================= Modbus 默认参数 / Modbus Default Parameters =================
// 当Flash第一次使用或校验失败时使用的默认值
// Default values used when Flash is first used or validation fails
#define DEFAULT_SLAVE_ADDR      0x01    // 默认从机地址 / Default slave address
#define DEFAULT_BAUD_RATE       9600    // 默认波特率 / Default baud rate

// ================= 寄存器数量配置 / Register Quantity Configuration =================
// 定义各区的最大数量 / Define the maximum quantity for each area
#define MB_COIL_COUNT           8       // 线圈 (0xxxx) / Coils (0xxxx)
#define MB_DISCRETE_COUNT       8       // 离散输入 (1xxxx) / Discrete Inputs (1xxxx)
#define MB_HOLDING_REG_COUNT    8       // 保持寄存器 (4xxxx) / Holding Registers (4xxxx)
#define MB_INPUT_REG_COUNT      8       // 输入寄存器 (3xxxx) / Input Registers (3xxxx)

// 接收/发送缓冲区大小 / Receive/Transmit buffer size
#define MB_RX_BUF_SIZE          256     // 接收缓冲区大小 / Receive buffer size
#define MB_TX_BUF_SIZE          256     // 发送缓冲区大小 / Transmit buffer size

#endif // MODBUS_CONFIG_H
