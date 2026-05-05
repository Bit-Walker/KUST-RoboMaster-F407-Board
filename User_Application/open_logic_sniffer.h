#ifndef USER_OPEN_LOGIC_SNIFFER_H
#define USER_OPEN_LOGIC_SNIFFER_H
#include "main.h"

/* 包含头文件 ----------------------------------------------------------------*/
#include "stdbool.h"
#include "../../User_Drives/user_uart.h"
#include "../../User_Drives/user_timer.h"

/* 宏定义 --------------------------------------------------------------------*/
#define OLS_MAX_CHANNELS          8
#define OLS_NUM_TRIGGER_STAGES    4
#define OLS_SAMPLE_BUFFER_SIZE    50000    /* 50KB 采样缓冲区（放在 CCMRAM） */
#define OLS_DEVICE_NAME           "F407-open-logic-sniffer"
#define OLS_FIRMWARE_VERSION      "v1.0"
#define OLS_PROTOCOL_VERSION      2
#define OLS_MAX_SAMPLE_RATE       1000000  /* 1MHz 最大采样率 */
#define OLS_CLOCK_RATE            1000000  /* 1MHz 基准时钟 */

/* SUMP 协议命令 */
#define CMD_RESET                      0x00
#define CMD_ARM_BASIC_TRIGGER          0x01
#define CMD_ID                         0x02
#define CMD_METADATA                   0x04
#define CMD_FINISH_NOW                 0x05
#define CMD_QUERY_INPUT_DATA           0x06
#define CMD_QUERY_CAPTURE_STATE        0x07
#define CMD_RETURN_CAPTURE_DATA        0x08
#define CMD_XON                        0x11
#define CMD_XOFF                       0x13
#define CMD_SET_DIVIDER                0x80
#define CMD_CAPTURE_SIZE               0x81
#define CMD_SET_FLAGS                  0x82
#define CMD_CAPTURE_DELAYCOUNT         0x83
#define CMD_CAPTURE_READCOUNT          0x84
#define CMD_SET_BASIC_TRIGGER_MASK0    0xC0
#define CMD_SET_BASIC_TRIGGER_VALUE0   0xC1
#define CMD_SET_BASIC_TRIGGER_CONFIG0  0xC2

/* 捕获标志 */
#define CAPTURE_FLAG_DEMUX                (1 << 0)
#define CAPTURE_FLAG_NOISE_FILTER         (1 << 1)
#define CAPTURE_FLAG_DISABLE_CHANGROUP_1  (1 << 2)
#define CAPTURE_FLAG_DISABLE_CHANGROUP_2  (1 << 3)
#define CAPTURE_FLAG_DISABLE_CHANGROUP_3  (1 << 4)
#define CAPTURE_FLAG_DISABLE_CHANGROUP_4  (1 << 5)
#define CAPTURE_FLAG_CLOCK_EXTERNAL       (1 << 6)
#define CAPTURE_FLAG_INVERT_EXT_CLOCK     (1 << 7)
#define CAPTURE_FLAG_RLE                  (1 << 8)

/* 触发器配置 */
#define TRIGGER_START                     (1 << 3)

/* 元数据令牌 */
#define METADATA_TOKEN_END                    0x00
#define METADATA_TOKEN_DEVICE_NAME            0x01
#define METADATA_TOKEN_FPGA_VERSION           0x02
#define METADATA_TOKEN_NUM_PROBES_SHORT       0x40
#define METADATA_TOKEN_PROTOCOL_VERSION_SHORT 0x41
#define METADATA_TOKEN_SAMPLE_MEMORY_BYTES    0x21
#define METADATA_TOKEN_MAX_SAMPLE_RATE_HZ     0x23

/* 类型定义 ------------------------------------------------------------------*/
/**
* @brief OLS 状态枚举
*/
typedef enum {
    OLS_STATE_IDLE = 0,    /* 空闲状态 */
    OLS_STATE_ARMED,       /* 已就绪状态 */
    OLS_STATE_TRIGGERED,   /* 已触发状态 */
    OLS_STATE_SAMPLING,    /* 采样中状态 */
    OLS_STATE_DONE         /* 完成状态 */
} OLS_State;

/**
* @brief OLS 触发器配置结构体
*/
typedef struct {
    uint32_t mask[OLS_NUM_TRIGGER_STAGES];    /* 触发器掩码 */
    uint32_t value[OLS_NUM_TRIGGER_STAGES];   /* 触发器值 */
    uint8_t config[OLS_NUM_TRIGGER_STAGES];   /* 触发器配置 */
    uint8_t num_stages;                       /* 触发器级数 */
} OLS_Trigger;

/**
* @brief OLS 逻辑分析仪驱动结构体
*/
typedef struct {
    /* 硬件句柄 */
    UART_DRIVES *uart;          /* UART 驱动 */
    TIMER_DRIVES *timer;        /* 定时器驱动 */
    
    /* 采样配置 */
    uint32_t sample_rate;    /* 采样率 */
    uint32_t divider;        /* 分频器 */
    uint32_t read_count;     /* 读取计数 */
    uint32_t delay_count;    /* 延迟计数 */
    uint16_t flags;          /* 标志位 */
    
    /* 触发器配置 */
    OLS_Trigger trigger;     /* 触发器 */
    
    /* 采样缓冲区 */
    uint8_t *sample_buffer;  /* 采样缓冲区指针 */
    uint32_t buffer_size;    /* 缓冲区大小 */
    uint32_t sample_count;   /* 采样计数 */
    
    /* RLE 压缩 */
    bool rle_enabled;        /* RLE 压缩使能 */
    uint32_t rle_count;      /* RLE 计数 */
    uint8_t last_sample;     /* 上一次采样值 */
    
    /* 状态 */
    OLS_State state;         /* 当前状态 */
    bool xon_xoff;           /* XON/XOFF 流控 */
    
    /* GPIO 配置 */
    GPIO_TypeDef *gpio_port;   /* GPIO 端口 */
    uint16_t gpio_pins;        /* GPIO 引脚 */
    
} OLS_DRIVES;

/* 函数声明 ------------------------------------------------------------------*/
void OLS_Init(OLS_DRIVES *user_ols, UART_DRIVES *uart, TIMER_DRIVES *timer, 
              GPIO_TypeDef *gpio_port, uint16_t gpio_pins);
void OLS_Reset(OLS_DRIVES *user_ols);
void OLS_UARTCallback(void *user_uart);
void OLS_ReturnCaptureData(OLS_DRIVES *ols);
uint32_t OLS_GetSampleCount(OLS_DRIVES *ols);
OLS_State OLS_GetState(OLS_DRIVES *ols);
bool OLS_IsCaptureDone(OLS_DRIVES *ols);

#endif /* USER_OPEN_LOGIC_SNIFFER_H */
