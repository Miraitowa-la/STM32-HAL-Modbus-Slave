/**
 * @file    modbus_slave.h
 * @brief   Modbus从站协议栈头文件 (V2.0 多实例架构)
 * @version 2.0.0
 * @date    2025
 * 
 * @details 本版本实现了以下架构升级:
 *          1. 多实例支持 - 可同时运行多个独立的Modbus从站
 *          2. 运行时配置 - 废弃静态宏定义，所有配置通过初始化函数传入
 *          3. 数据源解耦 - 库内部不定义任何寄存器数组，仅存储外部数据指针
 *          4. 回调机制 - 自定义功能码通过回调函数实现，Flash操作由应用层处理
 * 
 * @note    使用场景: 多个串口共享同一份物理寄存器数据
 */

#ifndef __MODBUS_SLAVE_H
#define __MODBUS_SLAVE_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 *                              Modbus功能码定义
 * ============================================================================ */

#define MB_FUNC_READ_COILS           0x01    /**< 读线圈状态 */
#define MB_FUNC_READ_DISCRETE        0x02    /**< 读离散输入 */
#define MB_FUNC_READ_HOLDING         0x03    /**< 读保持寄存器 */
#define MB_FUNC_READ_INPUT           0x04    /**< 读输入寄存器 */
#define MB_FUNC_WRITE_SINGLE_COIL    0x05    /**< 写单个线圈 */
#define MB_FUNC_WRITE_SINGLE_REG     0x06    /**< 写单个寄存器 */
#define MB_FUNC_WRITE_MULTI_COILS    0x0F    /**< 写多个线圈 */
#define MB_FUNC_WRITE_MULTI_REGS     0x10    /**< 写多个寄存器 */
#define MB_FUNC_CUSTOM_CONFIG        0x64    /**< 自定义功能码: 设备配置 */

/* ============================================================================
 *                              Modbus异常码定义
 * ============================================================================ */

#define MB_EX_ILLEGAL_FUNCTION       0x01    /**< 非法功能码 */
#define MB_EX_ILLEGAL_DATA_ADDRESS   0x02    /**< 非法数据地址 */
#define MB_EX_ILLEGAL_DATA_VALUE     0x03    /**< 非法数据值 */
#define MB_EX_SLAVE_DEVICE_FAILURE   0x04    /**< 从站设备故障 */

/* ============================================================================
 *                              类型前向声明
 * ============================================================================ */

typedef struct ModbusHandle ModbusHandle_t;

/* ============================================================================
 *                              回调函数类型定义
 * ============================================================================ */

/**
 * @brief   自定义功能码0x64回调函数类型
 * @param   hmodbus     Modbus句柄指针
 * @param   param_addr  参数地址 (0x0000=从站地址, 0x0001=波特率, 其他用户自定义)
 * @param   param_val   参数值
 * @retval  true  处理成功，协议栈将发送确认响应
 * @retval  false 处理失败，协议栈将发送异常响应
 * 
 * @note    此回调用于处理设备配置命令(如修改从站地址、波特率等)
 *          Flash保存、系统复位等操作应在回调内部完成
 * 
 * @warning 回调执行期间会阻塞Modbus处理，请尽量缩短执行时间
 *          如需延时操作(如Flash写入后复位)，建议设置标志位在主循环中处理
 */
typedef bool (*Modbus_CustomConfigCallback_t)(ModbusHandle_t *hmodbus, 
                                               uint16_t param_addr, 
                                               uint16_t param_val);

/**
 * @brief   数据写入前回调函数类型 (可选)
 * @param   hmodbus     Modbus句柄指针
 * @param   func_code   功能码
 * @param   start_addr  起始地址
 * @param   quantity    数量
 * @retval  true  允许写入
 * @retval  false 拒绝写入，将返回异常响应
 * 
 * @note    用于实现写入权限控制或数据验证
 */
typedef bool (*Modbus_WriteCallback_t)(ModbusHandle_t *hmodbus,
                                        uint8_t func_code,
                                        uint16_t start_addr,
                                        uint16_t quantity);

/* ============================================================================
 *                              数据结构定义
 * ============================================================================ */

/**
 * @brief   数据映射配置结构体
 * @details 用于将外部数据源绑定到Modbus实例
 *          支持多个Modbus实例共享同一份数据源
 * 
 * @warning 并发保护注意事项:
 *          当多个Modbus实例共享同一数据源时，存在竞态条件风险。
 *          建议措施:
 *          1. 对于16位及以下的原子操作，STM32 Cortex-M内核可保证原子性
 *          2. 对于涉及多个寄存器的复合操作，应使用临界区保护
 *          3. 可通过 __disable_irq() / __enable_irq() 实现简单的临界区
 *          4. 或使用RTOS的互斥锁(Mutex)机制
 */
typedef struct {
    /* 线圈区域 (Coils) - Modbus地址 0xxxx */
    uint8_t  *coils;              /**< 线圈数据指针 (位压缩存储, NULL表示不支持) */
    uint16_t  coil_count;         /**< 线圈数量 (0表示不支持此功能) */
    
    /* 离散输入区域 (Discrete Inputs) - Modbus地址 1xxxx */
    uint8_t  *discrete_inputs;    /**< 离散输入数据指针 (位压缩存储, NULL表示不支持) */
    uint16_t  discrete_count;     /**< 离散输入数量 (0表示不支持此功能) */
    
    /* 保持寄存器区域 (Holding Registers) - Modbus地址 4xxxx */
    uint16_t *holding_regs;       /**< 保持寄存器数据指针 (NULL表示不支持) */
    uint16_t  holding_reg_count;  /**< 保持寄存器数量 (0表示不支持此功能) */
    
    /* 输入寄存器区域 (Input Registers) - Modbus地址 3xxxx */
    uint16_t *input_regs;         /**< 输入寄存器数据指针 (NULL表示不支持) */
    uint16_t  input_reg_count;    /**< 输入寄存器数量 (0表示不支持此功能) */
} Modbus_DataMap_t;

/**
 * @brief   RS485硬件配置结构体
 */
typedef struct {
    bool          enabled;        /**< RS485模式使能标志 */
    GPIO_TypeDef *de_port;        /**< DE/RE控制引脚GPIO端口 (如GPIOA) */
    uint16_t      de_pin;         /**< DE/RE控制引脚编号 (如GPIO_PIN_8) */
    bool          de_polarity;    /**< DE极性: true=高电平发送, false=低电平发送 */
} Modbus_RS485Config_t;

/**
 * @brief   缓冲区配置结构体
 * @note    采用乒乓缓冲(双缓冲)机制，解决接收与处理的竞态问题
 */
typedef struct {
    uint8_t  *rx_buf_a;           /**< 接收缓冲区A (乒乓缓冲) */
    uint8_t  *rx_buf_b;           /**< 接收缓冲区B (乒乓缓冲) */
    uint8_t  *tx_buf;             /**< 发送缓冲区 */
    uint16_t  rx_buf_size;        /**< 接收缓冲区大小 (建议256字节) */
    uint16_t  tx_buf_size;        /**< 发送缓冲区大小 (建议256字节) */
} Modbus_BufferConfig_t;

/**
 * @brief   Modbus从站初始化配置结构体
 * @details 所有配置项均在运行时通过此结构体传入
 *          替代V1.0的静态宏定义方式
 */
typedef struct {
    /* UART配置 */
    UART_HandleTypeDef *huart;    /**< HAL UART句柄指针 */
    uint8_t             slave_addr;/**< 从站地址 (有效范围: 1~247) */
    uint32_t            baud_rate; /**< 波特率 (用于动态超时计算) */
    
    /* 缓冲区配置 */
    Modbus_BufferConfig_t buffer; /**< 缓冲区配置 */
    
    /* 数据映射 */
    Modbus_DataMap_t    data_map; /**< 数据区映射配置 */
    
    /* 硬件接口配置 */
    Modbus_RS485Config_t rs485;   /**< RS485配置 (不使用时enabled=false) */
    
    /* 运行时选项 */
    bool                use_dma_tx;    /**< 使用DMA发送 (需CubeMX配置DMA) */
    bool                use_crc_table; /**< 使用CRC查表法 (速度快, 需512B ROM) */
    
    /* 回调函数 */
    Modbus_CustomConfigCallback_t custom_config_cb; /**< 0x64功能码回调 (可为NULL) */
    Modbus_WriteCallback_t        write_cb;         /**< 写入前回调 (可为NULL) */
} Modbus_Config_t;

/**
 * @brief   Modbus从站核心句柄结构体
 * @details 包含独立的运行状态，支持多实例并行运行
 *          类似C++的类成员变量
 * 
 * @note    用户应使用 Modbus_Init() 初始化此结构体
 *          不建议直接访问内部成员，请使用提供的API
 */
struct ModbusHandle {
    /* ==================== UART接口 ==================== */
    UART_HandleTypeDef *huart;            /**< HAL UART句柄指针 */
    
    /* ==================== 从站配置 ==================== */
    uint8_t             slave_addr;       /**< 当前从站地址 */
    uint32_t            baud_rate;        /**< 当前波特率 (用于超时计算) */
    
    /* ==================== 乒乓缓冲区 ==================== */
    uint8_t            *rx_buf_a;         /**< 接收缓冲区A */
    uint8_t            *rx_buf_b;         /**< 接收缓冲区B */
    uint8_t            *tx_buf;           /**< 发送缓冲区 */
    uint16_t            rx_buf_size;      /**< 接收缓冲区大小 */
    uint16_t            tx_buf_size;      /**< 发送缓冲区大小 */
    
    uint8_t * volatile  rx_active_buf;    /**< 当前中断接收目标缓冲区 */
    uint8_t * volatile  rx_process_buf;   /**< 当前待处理数据缓冲区 */
    volatile uint16_t   rx_len;           /**< 待处理数据长度 */
    volatile uint8_t    rx_ready;         /**< 接收完成标志 (1=有数据待处理) */
    
    /* ==================== 数据映射 ==================== */
    Modbus_DataMap_t    data_map;         /**< 数据区指针映射 */
    
    /* ==================== 硬件配置 ==================== */
    Modbus_RS485Config_t rs485;           /**< RS485配置 */
    
    /* ==================== 运行时选项 ==================== */
    bool                use_dma_tx;       /**< DMA发送标志 */
    bool                use_crc_table;    /**< CRC查表法标志 */
    
    /* ==================== 回调函数 ==================== */
    Modbus_CustomConfigCallback_t custom_config_cb; /**< 0x64回调 */
    Modbus_WriteCallback_t        write_cb;         /**< 写入回调 */
    
    /* ==================== 用户数据 ==================== */
    void               *user_data;        /**< 用户自定义数据指针 (可选) */
};

/* ============================================================================
 *                              API函数声明
 * ============================================================================ */

/**
 * @brief   初始化Modbus从站实例
 * @param   hmodbus 待初始化的Modbus句柄指针
 * @param   config  初始化配置结构体指针
 * @retval  true    初始化成功
 * @retval  false   初始化失败 (参数无效)
 * 
 * @note    调用此函数后，Modbus从站开始监听UART数据
 *          必须确保UART已在CubeMX中正确配置并初始化
 * 
 * @code
 *          // 使用示例
 *          ModbusHandle_t hModbus1;
 *          Modbus_Config_t config = { ... };
 *          if (!Modbus_Init(&hModbus1, &config)) {
 *              Error_Handler();
 *          }
 * @endcode
 */
bool Modbus_Init(ModbusHandle_t *hmodbus, const Modbus_Config_t *config);

/**
 * @brief   Modbus主处理函数
 * @param   hmodbus Modbus句柄指针
 * 
 * @note    应在主循环中周期性调用
 *          此函数解析接收到的Modbus帧并生成响应
 *          采用乒乓缓冲机制，处理期间不会阻塞新数据接收
 * 
 * @warning 此函数非线程安全，在RTOS环境中需确保单一任务调用
 *          或使用互斥锁保护
 */
void Modbus_Process(ModbusHandle_t *hmodbus);

/**
 * @brief   UART接收完成/空闲中断回调
 * @param   hmodbus Modbus句柄指针
 * @param   size    接收到的数据长度
 * 
 * @note    需在 HAL_UARTEx_RxEventCallback() 中调用此函数
 * 
 * @code
 *          void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
 *              if (huart->Instance == hModbus1.huart->Instance) {
 *                  Modbus_RxCallback(&hModbus1, Size);
 *              } else if (huart->Instance == hModbus2.huart->Instance) {
 *                  Modbus_RxCallback(&hModbus2, Size);
 *              }
 *          }
 * @endcode
 */
void Modbus_RxCallback(ModbusHandle_t *hmodbus, uint16_t size);

/**
 * @brief   UART发送完成回调 (DMA模式专用)
 * @param   hmodbus Modbus句柄指针
 * 
 * @note    仅在 use_dma_tx = true 时需要调用
 *          需在 HAL_UART_TxCpltCallback() 中调用此函数
 *          用于DMA发送完成后切换RS485方向
 */
void Modbus_TxCallback(ModbusHandle_t *hmodbus);

/**
 * @brief   更新从站地址
 * @param   hmodbus Modbus句柄指针
 * @param   addr    新的从站地址 (1-247)
 * 
 * @note    此函数仅更新运行时地址，不涉及Flash保存
 *          持久化保存应在应用层实现
 */
void Modbus_SetSlaveAddr(ModbusHandle_t *hmodbus, uint8_t addr);

/**
 * @brief   更新波特率
 * @param   hmodbus   Modbus句柄指针
 * @param   baud_rate 新的波特率
 * 
 * @note    此函数仅更新运行时波特率配置(用于超时计算)
 *          实际UART波特率修改需调用 HAL_UART_Init()
 */
void Modbus_SetBaudRate(ModbusHandle_t *hmodbus, uint32_t baud_rate);

/**
 * @brief   启动UART接收
 * @param   hmodbus Modbus句柄指针
 * 
 * @note    通常由 Modbus_Init() 自动调用
 *          在某些错误恢复场景下可手动调用以重新启动接收
 */
void Modbus_StartReceive(ModbusHandle_t *hmodbus);

/**
 * @brief   获取当前从站地址
 * @param   hmodbus Modbus句柄指针
 * @return  当前从站地址
 */
static inline uint8_t Modbus_GetSlaveAddr(const ModbusHandle_t *hmodbus) {
    return hmodbus->slave_addr;
}

/**
 * @brief   获取当前波特率
 * @param   hmodbus Modbus句柄指针
 * @return  当前波特率
 */
static inline uint32_t Modbus_GetBaudRate(const ModbusHandle_t *hmodbus) {
    return hmodbus->baud_rate;
}

/**
 * @brief   设置用户自定义数据
 * @param   hmodbus   Modbus句柄指针
 * @param   user_data 用户数据指针
 * 
 * @note    可用于在回调函数中传递上下文信息
 */
static inline void Modbus_SetUserData(ModbusHandle_t *hmodbus, void *user_data) {
    hmodbus->user_data = user_data;
}

/**
 * @brief   获取用户自定义数据
 * @param   hmodbus Modbus句柄指针
 * @return  用户数据指针
 */
static inline void* Modbus_GetUserData(const ModbusHandle_t *hmodbus) {
    return hmodbus->user_data;
}

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_SLAVE_H */
