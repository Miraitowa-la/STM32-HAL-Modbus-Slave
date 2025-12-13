/**
 * @file    example_main.c
 * @brief   Modbus V2.0 多实例使用示例
 * @version 2.0.0
 * @date    2025
 * 
 * @details 本示例展示如何:
 *          1. 定义一份物理寄存器数据源
 *          2. 初始化两个独立的Modbus实例 (UART1和UART2)
 *          3. 让两个实例共享同一份数据源 ("双接口访问同一数据源")
 *          4. 实现自定义功能码0x64的回调处理
 * 
 * @note    此文件仅为示例代码，实际使用时需根据硬件配置修改
 *          CubeMX配置要求:
 *          - 开启USART1和USART2
 *          - 开启对应的NVIC中断
 *          - 如使用RS485，配置DE/RE控制引脚为GPIO Output
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "modbus_slave.h"
#include <string.h>

/* ============================================================================
 *                              硬件句柄声明 (CubeMX生成)
 * ============================================================================ */

extern UART_HandleTypeDef huart1;  /* UART1句柄 */
extern UART_HandleTypeDef huart2;  /* UART2句柄 */

/* ============================================================================
 *                              共享数据源定义
 * ============================================================================ 
 * 
 * 并发保护说明:
 * =============
 * 当多个Modbus实例共享同一份数据时，存在竞态条件风险。
 * 
 * 对于STM32 Cortex-M内核:
 * - 单个8位或16位读/写操作是原子的
 * - 32位对齐的读/写操作也是原子的
 * - 涉及多个寄存器的复合操作(如读取连续10个寄存器)不是原子的
 * 
 * 保护方案:
 * 1. 简单方案 - 关闭中断:
 *    __disable_irq();
 *    // 访问共享数据
 *    __enable_irq();
 * 
 * 2. RTOS方案 - 使用互斥锁:
 *    osMutexAcquire(dataMutexHandle, osWaitForever);
 *    // 访问共享数据
 *    osMutexRelease(dataMutexHandle);
 * 
 * 3. 无锁方案 - 使用volatile + 原子标志位:
 *    适用于单写者多读者场景
 * 
 * 对于本示例的简单应用场景(主循环更新数据, 两个Modbus只读访问):
 * - 16位寄存器的读取天然是原子的
 * - 应用层写入时可关闭中断保护
 */

/* 数据区大小定义 */
#define DATA_COIL_COUNT           16    /* 线圈数量 */
#define DATA_DISCRETE_COUNT       16    /* 离散输入数量 */
#define DATA_HOLDING_REG_COUNT    32    /* 保持寄存器数量 */
#define DATA_INPUT_REG_COUNT      32    /* 输入寄存器数量 */

/* 共享数据源 - 所有Modbus实例访问同一份数据 */
static uint8_t  g_coils[(DATA_COIL_COUNT + 7) / 8] = {0};           /* 线圈存储区 */
static uint8_t  g_discrete_inputs[(DATA_DISCRETE_COUNT + 7) / 8] = {0};  /* 离散输入存储区 */
static uint16_t g_holding_regs[DATA_HOLDING_REG_COUNT] = {0};       /* 保持寄存器存储区 */
static uint16_t g_input_regs[DATA_INPUT_REG_COUNT] = {0};           /* 输入寄存器存储区 */

/* ============================================================================
 *                              Modbus实例定义
 * ============================================================================ */

/* Modbus实例1 (UART1) */
static ModbusHandle_t hModbus1;

/* Modbus实例2 (UART2) */
static ModbusHandle_t hModbus2;

/* 缓冲区定义 - 每个实例需要独立的收发缓冲区 */
#define MODBUS_BUF_SIZE     256

/* 实例1缓冲区 */
static uint8_t mb1_rx_buf_a[MODBUS_BUF_SIZE];
static uint8_t mb1_rx_buf_b[MODBUS_BUF_SIZE];
static uint8_t mb1_tx_buf[MODBUS_BUF_SIZE];

/* 实例2缓冲区 */
static uint8_t mb2_rx_buf_a[MODBUS_BUF_SIZE];
static uint8_t mb2_rx_buf_b[MODBUS_BUF_SIZE];
static uint8_t mb2_tx_buf[MODBUS_BUF_SIZE];

/* ============================================================================
 *                              Flash存储配置 (应用层实现)
 * ============================================================================ */

/**
 * @brief   Flash存储的配置结构体
 */
typedef struct {
    uint32_t magic_key;     /* 魔数标识 (0xDEADBEEF) */
    uint8_t  slave_addr1;   /* 实例1从站地址 */
    uint8_t  slave_addr2;   /* 实例2从站地址 */
    uint32_t baud_rate;     /* 波特率 */
    uint8_t  reserved[2];   /* 对齐填充 */
} FlashConfig_t;

#define FLASH_MAGIC_KEY     0xDEADBEEF
#define DEFAULT_SLAVE_ADDR1 0x01
#define DEFAULT_SLAVE_ADDR2 0x02
#define DEFAULT_BAUD_RATE   9600

/* 存储的配置 (实际应用中应从Flash读取) */
static FlashConfig_t g_flash_config = {
    .magic_key = FLASH_MAGIC_KEY,
    .slave_addr1 = DEFAULT_SLAVE_ADDR1,
    .slave_addr2 = DEFAULT_SLAVE_ADDR2,
    .baud_rate = DEFAULT_BAUD_RATE
};

/* 波特率索引映射表 */
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

/* 待处理的配置更新标志 */
static volatile bool g_config_update_pending = false;

/* ============================================================================
 *                              回调函数实现
 * ============================================================================ */

/**
 * @brief   自定义功能码0x64回调函数
 * @param   hmodbus     Modbus句柄指针
 * @param   param_addr  参数地址
 * @param   param_val   参数值
 * @retval  true        处理成功
 * @retval  false       处理失败
 * 
 * @note    协议格式:
 *          param_addr = 0x0000: 修改从站地址, param_val = 1~247
 *          param_addr = 0x0001: 修改波特率, param_val = 1~8 (查表)
 * 
 * @warning Flash写入和系统复位应在主循环中执行，避免在中断上下文中操作
 */
static bool CustomConfigCallback(ModbusHandle_t *hmodbus, uint16_t param_addr, uint16_t param_val) {
    /* 参数地址 0x0000: 修改从站地址 */
    if (param_addr == 0x0000) {
        if (param_val >= 1 && param_val <= 247) {
            /* 更新运行时地址 */
            Modbus_SetSlaveAddr(hmodbus, (uint8_t)param_val);
            
            /* 标记需要保存配置
             * 实际Flash写入和复位应在主循环中执行 */
            g_config_update_pending = true;
            
            return true;
        }
        return false;  /* 地址范围无效 */
    }
    /* 参数地址 0x0001: 修改波特率 */
    else if (param_addr == 0x0001) {
        if (param_val >= 1 && param_val <= 8) {
            uint32_t new_baud = BAUD_RATE_TABLE[param_val];
            
            /* 更新运行时波特率 */
            Modbus_SetBaudRate(hmodbus, new_baud);
            
            /* 标记需要保存配置 */
            g_config_update_pending = true;
            
            return true;
        }
        return false;  /* 波特率索引无效 */
    }
    
    /* 其他地址: 不支持 */
    return false;
}

/**
 * @brief   写入前回调函数 (可选)
 * @param   hmodbus    Modbus句柄指针
 * @param   func_code  功能码
 * @param   start_addr 起始地址
 * @param   quantity   数量
 * @retval  true       允许写入
 * @retval  false      拒绝写入
 * 
 * @note    可用于实现写入权限控制或数据验证
 */
static bool WriteCallback(ModbusHandle_t *hmodbus, uint8_t func_code, 
                          uint16_t start_addr, uint16_t quantity) {
    /* 示例: 禁止写入地址100以后的寄存器 */
    if (func_code == MB_FUNC_WRITE_SINGLE_REG || func_code == MB_FUNC_WRITE_MULTI_REGS) {
        if (start_addr + quantity > 100) {
            return false;  /* 拒绝写入 */
        }
    }
    
    return true;  /* 允许写入 */
}

/* ============================================================================
 *                              Modbus初始化
 * ============================================================================ */

/**
 * @brief   初始化所有Modbus实例
 * @retval  true  初始化成功
 * @retval  false 初始化失败
 */
static bool Modbus_AppInit(void) {
    /* 配置共享的数据映射
     * 两个实例使用相同的数据指针，实现"双接口访问同一数据源" */
    Modbus_DataMap_t shared_data_map = {
        .coils = g_coils,
        .coil_count = DATA_COIL_COUNT,
        .discrete_inputs = g_discrete_inputs,
        .discrete_count = DATA_DISCRETE_COUNT,
        .holding_regs = g_holding_regs,
        .holding_reg_count = DATA_HOLDING_REG_COUNT,
        .input_regs = g_input_regs,
        .input_reg_count = DATA_INPUT_REG_COUNT
    };
    
    /* ==================== 初始化Modbus实例1 (UART1) ==================== */
    Modbus_Config_t config1 = {
        /* UART配置 */
        .huart = &huart1,
        .slave_addr = g_flash_config.slave_addr1,
        .baud_rate = g_flash_config.baud_rate,
        
        /* 缓冲区配置 */
        .buffer = {
            .rx_buf_a = mb1_rx_buf_a,
            .rx_buf_b = mb1_rx_buf_b,
            .tx_buf = mb1_tx_buf,
            .rx_buf_size = MODBUS_BUF_SIZE,
            .tx_buf_size = MODBUS_BUF_SIZE
        },
        
        /* 数据映射 - 使用共享数据 */
        .data_map = shared_data_map,
        
        /* RS485配置 - 实例1使用PA8作为DE/RE控制引脚 */
        .rs485 = {
            .enabled = true,
            .de_port = GPIOA,
            .de_pin = GPIO_PIN_8,
            .de_polarity = true  /* 高电平发送 */
        },
        
        /* 运行时选项 */
        .use_dma_tx = false,      /* 阻塞模式发送 */
        .use_crc_table = true,    /* 使用CRC查表法 */
        
        /* 回调函数 */
        .custom_config_cb = CustomConfigCallback,
        .write_cb = WriteCallback
    };
    
    if (!Modbus_Init(&hModbus1, &config1)) {
        return false;
    }
    
    /* ==================== 初始化Modbus实例2 (UART2) ==================== */
    Modbus_Config_t config2 = {
        /* UART配置 */
        .huart = &huart2,
        .slave_addr = g_flash_config.slave_addr2,
        .baud_rate = g_flash_config.baud_rate,
        
        /* 缓冲区配置 */
        .buffer = {
            .rx_buf_a = mb2_rx_buf_a,
            .rx_buf_b = mb2_rx_buf_b,
            .tx_buf = mb2_tx_buf,
            .rx_buf_size = MODBUS_BUF_SIZE,
            .tx_buf_size = MODBUS_BUF_SIZE
        },
        
        /* 数据映射 - 使用共享数据 (与实例1相同) */
        .data_map = shared_data_map,
        
        /* RS485配置 - 实例2使用PB5作为DE/RE控制引脚 */
        .rs485 = {
            .enabled = true,
            .de_port = GPIOB,
            .de_pin = GPIO_PIN_5,
            .de_polarity = true  /* 高电平发送 */
        },
        
        /* 运行时选项 */
        .use_dma_tx = false,
        .use_crc_table = true,
        
        /* 回调函数 */
        .custom_config_cb = CustomConfigCallback,
        .write_cb = NULL  /* 实例2不使用写入回调 */
    };
    
    if (!Modbus_Init(&hModbus2, &config2)) {
        return false;
    }
    
    return true;
}

/* ============================================================================
 *                              HAL回调函数
 * ============================================================================ */

/**
 * @brief   UART空闲中断回调函数
 * @note    需要在stm32xxxx_it.c中确保此函数被调用
 *          或者直接在此文件中实现
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    /* 根据UART实例分发到对应的Modbus处理 */
    if (huart->Instance == USART1) {
        Modbus_RxCallback(&hModbus1, Size);
    } else if (huart->Instance == USART2) {
        Modbus_RxCallback(&hModbus2, Size);
    }
}

/**
 * @brief   UART发送完成回调函数 (DMA模式时需要)
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        Modbus_TxCallback(&hModbus1);
    } else if (huart->Instance == USART2) {
        Modbus_TxCallback(&hModbus2);
    }
}

/* ============================================================================
 *                              主函数
 * ============================================================================ */

/**
 * @brief   应用程序入口点
 * @note    此示例展示了双Modbus实例共享数据源的典型用法
 */
int main(void) {
    /* MCU初始化 (CubeMX生成的代码) */
    HAL_Init();
    // SystemClock_Config();
    // MX_GPIO_Init();
    // MX_USART1_UART_Init();
    // MX_USART2_UART_Init();
    
    /* 初始化所有Modbus实例 */
    if (!Modbus_AppInit()) {
        Error_Handler();
    }
    
    /* 初始化共享数据 (示例) */
    g_holding_regs[0] = 0x1234;
    g_holding_regs[1] = 0x5678;
    g_input_regs[0] = 0;
    
    /* 主循环 */
    uint32_t tick_count = 0;
    
    while (1) {
        /* ==================== Modbus协议处理 ==================== */
        /* 两个实例并行工作，处理各自UART接收的数据 */
        Modbus_Process(&hModbus1);
        Modbus_Process(&hModbus2);
        
        /* ==================== 应用逻辑示例 ==================== */
        
        /* 示例1: 模拟传感器数据更新 (每1000次循环更新一次)
         * 注意: 如果对数据一致性要求高，建议使用临界区保护
         * 
         * __disable_irq();
         * g_input_regs[0] = new_value;
         * __enable_irq();
         */
        if (++tick_count >= 1000) {
            tick_count = 0;
            
            /* 模拟ADC采样值 */
            g_input_regs[0]++;  /* 16位写入，Cortex-M保证原子性 */
            
            /* 将保持寄存器0的值同步到输入寄存器1 */
            g_input_regs[1] = g_holding_regs[0];
        }
        
        /* 示例2: 使用线圈0控制LED (假设PC13连接LED) */
        if (g_coils[0] & 0x01) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  /* LED ON */
        } else {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);    /* LED OFF */
        }
        
        /* 示例3: 处理配置更新请求
         * 0x64功能码回调只设置标志位，实际Flash操作在这里执行
         * 这样避免了在中断上下文中进行耗时的Flash操作 */
        if (g_config_update_pending) {
            g_config_update_pending = false;
            
            /* 更新配置结构体 */
            g_flash_config.slave_addr1 = Modbus_GetSlaveAddr(&hModbus1);
            g_flash_config.slave_addr2 = Modbus_GetSlaveAddr(&hModbus2);
            g_flash_config.baud_rate = Modbus_GetBaudRate(&hModbus1);
            
            /* TODO: 保存配置到Flash
             * Flash_SaveConfig(&g_flash_config);
             * 
             * 延时后重启:
             * HAL_Delay(50);
             * NVIC_SystemReset();
             */
        }
        
        /* 其他应用逻辑... */
    }
}

/**
 * @brief   错误处理函数
 */
void Error_Handler(void) {
    __disable_irq();
    while (1) {
        /* 用户可在此添加错误指示，如闪烁LED */
    }
}
