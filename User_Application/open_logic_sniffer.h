#ifndef USER_OPEN_LOGIC_SNIFFER_H
#define USER_OPEN_LOGIC_SNIFFER_H
#include "main.h"
#ifdef HAL_UART_MODULE_ENABLED

/* 包含头文件 ----------------------------------------------------------------*/
#include "stdbool.h"
#include "../../User_Drives/user_uart.h"
#include "../../User_Drives/user_timer.h"

/* 宏定义 --------------------------------------------------------------------*/
/**
 * @name 硬件配置
 * @{
 */
#define OLS_MAX_CHANNELS              8U          /* 最大通道数 */
#define OLS_MAX_SAMPLE_RATE           500000U     /* 最大采样率 500KHz */
#define OLS_CLOCK_RATE                1000000U    /* 基准时钟 1MHz */
#define OLS_SAMPLE_BUFFER_SIZE        50000U      /* 采样缓冲区大小 50KB */
#define OLS_TX_BUFFER_SIZE            2048U       /* 发送缓冲区大小 */
#define OLS_CMD_BUFFER_SIZE           64U         /* 命令缓冲区大小 */
#define OLS_DEFAULT_READ_COUNT        4096U       /* 默认读取计数 */
#define OLS_MAX_TRIGGER_STAGES        4U          /* 最大触发级数 */
/** @} */

/**
 * @name SUMP 协议命令
 * @{
 */
#define OLS_CMD_RESET                 0x00U   /* 复位 */
#define OLS_CMD_ARM                   0x01U   /* 开始采集 */
#define OLS_CMD_GET_ID                0x02U   /* 获取设备ID */
#define OLS_CMD_GET_METADATA          0x04U   /* 获取元数据 */
#define OLS_CMD_STOP                  0x05U   /* 停止采集 */
#define OLS_CMD_GET_INPUT_DATA        0x06U   /* 获取当前输入数据 */
#define OLS_CMD_GET_CAPTURE_STATE     0x07U   /* 获取采集状态 */
#define OLS_CMD_GET_CAPTURE_DATA      0x08U   /* 获取采集数据 */
#define OLS_CMD_XON                   0x11U   /* 流量控制开启 */
#define OLS_CMD_XOFF                  0x13U   /* 流量控制关闭 */
#define OLS_CMD_SET_DIVIDER           0x80U   /* 设置分频器 */
#define OLS_CMD_SET_READ_COUNT        0x81U   /* 设置读取计数 */
#define OLS_CMD_SET_FLAGS             0x82U   /* 设置标志 */
#define OLS_CMD_SET_DELAY_COUNT       0x83U   /* 设置延迟计数 */
#define OLS_CMD_SET_READ_COUNT_ALT    0x84U   /* 设置读取计数 */
#define OLS_CMD_SET_TRIGGER_MASK0     0xC0U   /* 设置触发器掩码0 */
#define OLS_CMD_SET_TRIGGER_VALUE0    0xC1U   /* 设置触发器值0 */
#define OLS_CMD_SET_TRIGGER_CONFIG0   0xC2U   /* 设置触发器配置0 */
/** @} */

/**
 * @name 捕获标志位
 * @{
 */
#define OLS_FLAG_DEMUX                (1U << 0)  /* DEMUX 模式 */
#define OLS_FLAG_NOISE_FILTER         (1U << 1)  /* 噪声滤波 */
#define OLS_FLAG_DISABLE_CHANGROUP_1  (1U << 2) /* 禁用通道组1 */
#define OLS_FLAG_DISABLE_CHANGROUP_2  (1U << 3) /* 禁用通道组2 */
#define OLS_FLAG_DISABLE_CHANGROUP_3  (1U << 4) /* 禁用通道组3 */
#define OLS_FLAG_DISABLE_CHANGROUP_4  (1U << 5) /* 禁用通道组4 */
#define OLS_FLAG_CLOCK_EXTERNAL       (1U << 6) /* 外部时钟 */
#define OLS_FLAG_INVERT_EXT_CLOCK     (1U << 7) /* 反转外部时钟 */
#define OLS_FLAG_RLE                  (1U << 8) /* RLE 压缩 */
/** @} */

/**
 * @name 触发器配置
 * @{
 */
#define OLS_TRIGGER_START             (1U << 3)  /* 开始触发 */
/** @} */

/**
 * @name 元数据令牌
 * @{
 */
#define OLS_META_END                  0x00U   /* 元数据结束 */
#define OLS_META_DEVICE_NAME          0x01U   /* 设备名称 */
#define OLS_META_FPGA_VERSION         0x02U   /* FPGA 版本 */
#define OLS_META_NUM_PROBES           0x40U   /* 探针数量 */
#define OLS_META_PROTOCOL_VERSION     0x41U   /* 协议版本 */
#define OLS_META_SAMPLE_MEMORY        0x21U   /* 采样内存 */
#define OLS_META_MAX_SAMPLE_RATE      0x23U   /* 最大采样率 */
/** @} */

/**
 * @name 设备信息
 * @{
 */
#define OLS_DEVICE_NAME               "User Logic Sniffer"
#define OLS_FIRMWARE_VERSION          "v1.0"
#define OLS_PROTOCOL_VERSION          2U
#define OLS_DEVICE_ID                 "1SLO"
#define OLS_LIBSIGROK_CLOCK_RATE      100000000UL  /* libsigrok 使用 100MHz 时钟基准 */
/** @} */

/* 类型定义 ------------------------------------------------------------------*/

/**
 * @brief OLS 状态枚举
 */
typedef enum {
    OLS_STATE_IDLE = 0,      /* 空闲状态 */
    OLS_STATE_ARMED,         /* 已就绪状态 */
    OLS_STATE_TRIGGERED,     /* 已触发状态 */
    OLS_STATE_SAMPLING,      /* 采集中状态 */
    OLS_STATE_DONE          /* 完成状态 */
} OLS_State;

/**
 * @brief OLS 触发器配置结构体
 */
typedef struct {
    uint32_t mask[OLS_MAX_TRIGGER_STAGES];     /* 触发器掩码 */
    uint32_t value[OLS_MAX_TRIGGER_STAGES];     /* 触发器值 */
    uint8_t  config[OLS_MAX_TRIGGER_STAGES];   /* 触发器配置 */
    uint8_t  num_stages;                        /* 触发器级数 */
} OLS_Trigger;

/**
 * @brief OLS 逻辑分析仪驱动结构体
 */
typedef struct {
    UART_DRIVES  *uart;            /* UART 驱动 */
    TIMER_DRIVES *timer;           /* 定时器驱动 */
    GPIO_TypeDef *gpio_port;       /* GPIO 端口 */

    uint32_t sample_rate;          /* 采样率 */
    uint32_t divider;              /* 分频器值 */
    uint32_t read_count;           /* 读取计数 */
    uint32_t delay_count;          /* 延迟计数 */
    uint16_t flags;                /* 标志位 */

    OLS_Trigger trigger;           /* 触发器配置 */

    uint8_t  *sample_buffer;       /* 采样缓冲区指针 */
    uint32_t buffer_size;          /* 缓冲区大小 */
    uint32_t sample_count;         /* 当前采样计数 */

    bool rle_enabled;              /* RLE 压缩使能 */
    uint32_t rle_count;            /* RLE 计数 */
    uint8_t  last_sample;          /* 上一次采样值 */

    OLS_State state;               /* 当前状态 */
    bool      xon_xoff;            /* XON/XOFF 流控 */
} OLS_Handle;

/* 函数声明 ------------------------------------------------------------------*/
void OLS_Init(UART_DRIVES *uart, TIMER_DRIVES *timer, GPIO_TypeDef *gpio_port);


#endif /* HAL_UART_MODULE_ENABLED */
#endif /* USER_OPEN_LOGIC_SNIFFER_H */
