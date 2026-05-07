#include "../../Core/Inc/bsp.h"
#include "../open_logic_sniffer.h"
#include <string.h>

#ifdef HAL_UART_MODULE_ENABLED

/* 私有变量 ------------------------------------------------------------------*/
static OLS_Handle ols_instance = {0};
static uint8_t sample_buffer[OLS_SAMPLE_BUFFER_SIZE] __attribute__((section(".ccmram")));
static uint8_t tx_buffer[OLS_TX_BUFFER_SIZE];
static uint8_t cmd_buffer[OLS_CMD_BUFFER_SIZE];
static uint8_t cmd_index = 0;

static volatile bool capture_in_progress = false;
static volatile bool capture_done = false;

/* 私有函数 ------------------------------------------------------------------*/
static void OLS_SendCaptureData();

/**
 * @brief  限制读取计数在缓冲区范围内
 */
static void OLS_LimitReadCount(void) {
    if (ols_instance.read_count > ols_instance.buffer_size) {
        ols_instance.read_count = ols_instance.buffer_size;
    }
    if (ols_instance.read_count == 0) {
        ols_instance.read_count = OLS_DEFAULT_READ_COUNT;
    }
}

/**
 * @brief  采样回调函数
 */
static void OLS_SampleCallback(void *user_timer) {
    (void)user_timer;

    if (ols_instance.gpio_port != NULL) {
        const uint8_t port_value = (uint8_t)(ols_instance.gpio_port->IDR & 0x00FF);
        ols_instance.sample_buffer[ols_instance.sample_count++] = port_value;

        if (ols_instance.sample_count >= ols_instance.read_count) {
            capture_in_progress = false;
            capture_done = true;
            ols_instance.state = OLS_STATE_DONE;
            TIMER_Stop(ols_instance.timer);
            OLS_SendCaptureData();
        }
    }
}

/**
 * @brief  发送数据
 * @param  data 数据指针
 * @param  len  数据长度
 */
static void OLS_SendBytes(const uint8_t *data, const uint16_t len) {
    UART_Send_Data(ols_instance.uart, (const char *)data, len);
}

/**
 * @brief  发送单字节
 * @param  byte 字节值
 */
static void OLS_SendByte(const uint8_t byte) {
    UART_Send_Data(ols_instance.uart, (const char *)&byte, 1);
}

/**
 * @brief  发送设备 ID
 */
static void OLS_SendDeviceId(void) {
    const uint8_t id[4] = {OLS_DEVICE_ID[0], OLS_DEVICE_ID[1], OLS_DEVICE_ID[2], OLS_DEVICE_ID[3]};
    OLS_SendBytes(id, 4);
}

/**
 * @brief  发送元数据
 */
static void OLS_SendMetadata(void) {
    uint8_t metadata[64];
    uint16_t index = 0;

    metadata[index++] = OLS_META_DEVICE_NAME;
    memcpy(&metadata[index], OLS_DEVICE_NAME, 8);
    index += 8;
    metadata[index++] = '\0';

    metadata[index++] = OLS_META_NUM_PROBES;
    metadata[index++] = OLS_MAX_CHANNELS;

    metadata[index++] = OLS_META_SAMPLE_MEMORY;
    const uint32_t sample_mem = ols_instance.buffer_size;
    metadata[index++] = (uint8_t)(sample_mem & 0xFF);
    metadata[index++] = (uint8_t)((sample_mem >> 8) & 0xFF);
    metadata[index++] = (uint8_t)((sample_mem >> 16) & 0xFF);
    metadata[index++] = (uint8_t)((sample_mem >> 24) & 0xFF);

    metadata[index++] = OLS_META_MAX_SAMPLE_RATE;
    const uint32_t max_rate = OLS_CLOCK_RATE;
    metadata[index++] = (uint8_t)(max_rate & 0xFF);
    metadata[index++] = (uint8_t)((max_rate >> 8) & 0xFF);
    metadata[index++] = (uint8_t)((max_rate >> 16) & 0xFF);
    metadata[index++] = (uint8_t)((max_rate >> 24) & 0xFF);

    metadata[index++] = OLS_META_PROTOCOL_VERSION;
    metadata[index++] = OLS_PROTOCOL_VERSION;

    metadata[index++] = OLS_META_END;

    OLS_SendBytes(metadata, index);
}

/**
 * @brief  读取当前输入数据
 * @return 当前 GPIO 值
 */
static uint8_t OLS_ReadInputData(void) {
    return (uint8_t)(ols_instance.gpio_port->IDR & 0x00FF);
}

/**
 * @brief  发送采集状态
 */
static void OLS_SendCaptureState(void) {
    uint8_t state;

    if (capture_done) {
        state = 0x02;
    } else if (capture_in_progress) {
        state = 0x01;
    } else {
        state = 0x00;
    }

    OLS_SendByte(state);
}

/**
 * @brief  复位 OLS
 */
static void OLS_Reset(void) {
    if (capture_in_progress) {
        TIMER_Stop(ols_instance.timer);
        capture_in_progress = false;
    }

    ols_instance.state = OLS_STATE_IDLE;
    ols_instance.sample_count = 0;
    ols_instance.xon_xoff = true;
    ols_instance.read_count = OLS_DEFAULT_READ_COUNT;
    ols_instance.delay_count = 0;
    ols_instance.flags = 0;
    ols_instance.rle_enabled = false;
    ols_instance.sample_rate = OLS_MAX_SAMPLE_RATE;
    capture_done = false;
    cmd_index = 0;
}

/**
 * @brief  开始采集
 */
static void OLS_StartCapture() {
    if (capture_in_progress) {
        TIMER_Stop(ols_instance.timer);
    }

    ols_instance.sample_count = 0;
    ols_instance.state = OLS_STATE_ARMED;
    capture_in_progress = true;
    capture_done = false;

    OLS_LimitReadCount();

    TIMER_Set_Frequency(ols_instance.timer, ols_instance.sample_rate);
    TIMER_Start(ols_instance.timer);
    ols_instance.state = OLS_STATE_SAMPLING;
}

/**
 * @brief  停止采集
 */
static void OLS_StopCapture(void) {
    if (capture_in_progress) {
        TIMER_Stop(ols_instance.timer);
        capture_in_progress = false;
    }

    ols_instance.state = OLS_STATE_DONE;
    capture_done = true;
}

/**
 * @brief  发送采集数据
 */
static void OLS_SendCaptureData(void) {
    if (capture_in_progress) {
        TIMER_Stop(ols_instance.timer);
        capture_in_progress = false;
    }

    if (ols_instance.sample_count > 0) {
        const uint32_t total_samples = ols_instance.sample_count;
        uint32_t sent = 0;

        while (sent < total_samples) {
            const uint32_t remaining = total_samples - sent;
            const uint32_t chunk_size = (remaining > OLS_TX_BUFFER_SIZE) ? OLS_TX_BUFFER_SIZE : remaining;

            for (uint32_t i = 0; i < chunk_size; i++) {
                const uint32_t buffer_idx = total_samples - 1 - sent - i;
                tx_buffer[i] = ols_instance.sample_buffer[buffer_idx];
            }

            OLS_SendBytes(tx_buffer, (uint16_t)chunk_size);
            sent += chunk_size;
        }

        const uint8_t padding = (uint8_t)((4U - (total_samples % 4U)) % 4U);
        if (padding > 0) {
            const uint8_t pad_buffer[4] = {0x00, 0x00, 0x00, 0x00};
            OLS_SendBytes(pad_buffer, padding);
        }
    } else {
        const uint8_t empty_sample[4] = {0x00, 0x00, 0x00, 0x00};
        OLS_SendBytes(empty_sample, 4);
    }

    ols_instance.state = OLS_STATE_IDLE;
    capture_done = false;
}

/**
 * @brief  处理 SUMP 命令
 * @param  cmd  命令字节
 * @param  data 数据指针
 */
static void OLS_HandleCommand(const uint8_t cmd, const uint8_t *data) {
    switch (cmd) {
        case OLS_CMD_RESET:
            OLS_Reset();
            break;

        case OLS_CMD_ARM:
            OLS_StartCapture();
            break;

        case OLS_CMD_GET_ID:
            OLS_SendDeviceId();
            break;

        case OLS_CMD_GET_METADATA:
            OLS_SendMetadata();
            break;

        case OLS_CMD_STOP:
            OLS_StopCapture();
            break;

        case OLS_CMD_GET_INPUT_DATA:
            OLS_SendByte(OLS_ReadInputData());
            break;

        case OLS_CMD_GET_CAPTURE_STATE:
            OLS_SendCaptureState();
            break;

        case OLS_CMD_GET_CAPTURE_DATA:
            OLS_SendCaptureData();
            break;

        case OLS_CMD_SET_DIVIDER:
            if (data != NULL) {
                const uint32_t libsigrok_divider = (uint32_t)data[0] | (uint32_t)data[1] << 8 | (uint32_t)data[2] << 16;
                ols_instance.divider = libsigrok_divider;
                ols_instance.sample_rate = OLS_LIBSIGROK_CLOCK_RATE / (libsigrok_divider + 1U);
            }
            break;

        case OLS_CMD_SET_READ_COUNT:
        case OLS_CMD_SET_READ_COUNT_ALT:
            if (data != NULL) {
                const uint32_t read_count = (uint32_t)data[0] | ((uint32_t)data[1] << 8);
                ols_instance.read_count = (read_count + 1U) * 4U;
                OLS_LimitReadCount();
            }
            break;

        case OLS_CMD_SET_FLAGS:
            if (data != NULL) {
                ols_instance.flags = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
                ols_instance.rle_enabled = ((ols_instance.flags & OLS_FLAG_RLE) != 0);
            }
            break;

        case OLS_CMD_SET_TRIGGER_MASK0:
        case 0xC4U:
        case 0xC8U:
        case 0xCCU:
            if (data != NULL) {
                const uint8_t stage = (uint8_t)((cmd - OLS_CMD_SET_TRIGGER_MASK0) / 4U);
                if (stage < OLS_MAX_TRIGGER_STAGES) {
                    ols_instance.trigger.mask[stage] = (uint32_t)data[0] | (uint32_t)data[1] << 8  | (uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
                }
            }
            break;

        case OLS_CMD_SET_TRIGGER_VALUE0:
        case 0xC5U:
        case 0xC9U:
        case 0xCDU:
            if (data != NULL) {
                const uint8_t stage = (uint8_t)((cmd - OLS_CMD_SET_TRIGGER_VALUE0) / 4U);
                if (stage < OLS_MAX_TRIGGER_STAGES) {
                    ols_instance.trigger.value[stage] = (uint32_t)data[0] | (uint32_t)data[1] << 8  | (uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
                }
            }
            break;

        case OLS_CMD_SET_TRIGGER_CONFIG0:
        case 0xC6U:
        case 0xCAU:
        case 0xCEU:
            if (data != NULL) {
                const uint8_t stage = (uint8_t)((cmd - OLS_CMD_SET_TRIGGER_CONFIG0) / 4U);
                if (stage < OLS_MAX_TRIGGER_STAGES) {
                    ols_instance.trigger.config[stage] = data[3];
                    ols_instance.trigger.num_stages = stage + 1U;
                }
            }
            break;

        default:
            break;
    }
}

/**
 * @brief  UART 接收回调
 * @param  user_uart  UART 驱动结构体指针
 */
static void OLS_UARTCallback(void *user_uart) {
    UART_DRIVES *uart = (UART_DRIVES *)user_uart;
    const uint16_t len = UART_GetAllDate(uart, cmd_buffer);

    for (uint16_t i = 0; i < len; i++) {
        cmd_buffer[cmd_index++] = cmd_buffer[i];

        const uint8_t cmd = cmd_buffer[0];
        uint8_t expected_len = 1;

        if ((cmd >= 0x80U && cmd <= 0xBFU) || cmd >= 0xC0U) {
            expected_len = 5;
        }

        if (cmd_index >= expected_len) {
            const uint8_t *data = (expected_len > 1U) ? &cmd_buffer[1] : NULL;
            OLS_HandleCommand(cmd, data);
            cmd_index = 0;
        }

        if (cmd_index >= OLS_CMD_BUFFER_SIZE) {
            cmd_index = 0;
        }
    }
}

/* 公共函数 ------------------------------------------------------------------*/

/**
 * @brief  初始化 OLS 驱动
 * @param  uart        UART 驱动结构体指针
 * @param  timer       定时器驱动结构体指针
 * @param  gpio_port   GPIO 端口
 */
void OLS_Init(UART_DRIVES *uart, TIMER_DRIVES *timer, GPIO_TypeDef *gpio_port) {
    ols_instance.uart = uart;
    ols_instance.timer = timer;
    ols_instance.gpio_port = gpio_port;
    ols_instance.sample_buffer = sample_buffer;
    ols_instance.buffer_size = OLS_SAMPLE_BUFFER_SIZE;
    ols_instance.sample_rate = OLS_MAX_SAMPLE_RATE;
    ols_instance.read_count = OLS_DEFAULT_READ_COUNT;
    ols_instance.trigger.num_stages = 1;
    ols_instance.trigger.mask[0] = 0xFF;
    ols_instance.trigger.value[0] = 0x00;
    ols_instance.trigger.config[0] = OLS_TRIGGER_START;
    ols_instance.state = OLS_STATE_IDLE;
    ols_instance.xon_xoff = true;
    ols_instance.rle_enabled = false;

    TIMER_Set_Frequency(timer, OLS_MAX_SAMPLE_RATE);

    UART_RegisterCallback(uart, OLS_UARTCallback);
    TIMER_RegisterCallback(timer, OLS_SampleCallback);

}

#endif /* HAL_UART_MODULE_ENABLED */
