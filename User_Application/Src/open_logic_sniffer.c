#include "../../Core/Inc/bsp.h"
#include "../open_logic_sniffer.h"
#include <string.h>

/* 私有变量 ------------------------------------------------------------------*/
static OLS_DRIVES *ols_instance = NULL;

// 采样缓冲区
static uint8_t sample_buffer[50000] __attribute__((section("ccmram"))) = {0};

/* 发送缓冲区 - 放在普通 RAM 中，用于分块发送反向数据 */
static uint8_t tx_buffer[2048];
static volatile bool capture_in_progress = false;
static volatile bool capture_done = false;

/* 命令缓冲区 - 直接从 UART DMA 缓冲区读取 */
static uint8_t cmd_buffer[64];
static uint8_t cmd_index = 0;

/* 函数声明 */
static void OLS_HandleCommand(uint8_t cmd, const uint8_t *data);
static void OLS_SendBytes(const uint8_t *data, uint16_t len);
static void OLS_SendByte(uint8_t byte);
static void OLS_SendMetadata(void);
static void OLS_StartCapture(void);
static void OLS_StopCapture(void);
static void OLS_SendCaptureData(void);
static void OLS_SampleCallback(void* user_timer);

/**
* @brief 采样回调函数
*/
static void OLS_SampleCallback(void* user_timer) {
    (void)user_timer;
    
    if (!capture_in_progress || ols_instance == NULL)
        return;
    
    OLS_DRIVES *ols = ols_instance;
    
    /* 读取 GPIO 端口数据 - 使用 IDR 直读，避免多次读取 */
    const GPIO_TypeDef *port = ols->gpio_port;
    if (port != NULL) {
        /* 直接读取整个端口，然后组合位 */
        uint16_t port_value = (uint16_t)(port->IDR & 0x00FF);
        ols->sample_buffer[ols->sample_count++] = (uint8_t)port_value;
        
        /* LED 反馈：每次采样时翻转一次（采样速度快时会很快闪烁） */
        static uint32_t led_counter = 0;
        led_counter++;
        if (led_counter >= 10000) {  /* 每 10000 次采样翻转一次 LED */
            led_counter = 0;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }
        
        if (ols->sample_count >= ols->read_count) {
            capture_in_progress = false;
            capture_done = true;
            ols->state = OLS_STATE_DONE;
            TIMER_Stop(ols->timer);
            /* LED 灭：采集完成，准备发送数据 */
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            OLS_SendCaptureData();
        }
    }
}

/**
* @brief 初始化 OLS
*/
void OLS_Init(OLS_DRIVES *user_ols, UART_DRIVES *uart, TIMER_DRIVES *timer,
              GPIO_TypeDef *gpio_port, uint16_t gpio_pins) {
    memset(user_ols, 0, sizeof(OLS_DRIVES));
    
    user_ols->uart = uart;
    user_ols->timer = timer;
    user_ols->gpio_port = gpio_port;
    user_ols->gpio_pins = gpio_pins;
    user_ols->sample_buffer = sample_buffer;
    user_ols->buffer_size = OLS_SAMPLE_BUFFER_SIZE;
    user_ols->sample_count = 0;
    user_ols->sample_rate = 1000000;
    user_ols->divider = 0;
    user_ols->read_count = 4096;
    user_ols->delay_count = 0;
    user_ols->flags = 0;
    user_ols->trigger.num_stages = 1;
    user_ols->trigger.mask[0] = 0xFF;
    user_ols->trigger.value[0] = 0x00;
    user_ols->trigger.config[0] = TRIGGER_START;
    user_ols->state = OLS_STATE_IDLE;
    user_ols->xon_xoff = true;
    
    ols_instance = user_ols;
    cmd_index = 0;
    
    /* 注册回调 */
    UART_RegisterCallback(uart, OLS_UARTCallback);
    TIMER_RegisterCallback(timer, OLS_SampleCallback);
    TIMER_Set_Frequency(timer, 1000000);
    TIMER_Stop(timer);
}

/**
* @brief 重置 OLS
*/
void OLS_Reset(OLS_DRIVES *user_ols) {
    if (capture_in_progress) {
        TIMER_Stop(user_ols->timer);
        capture_in_progress = false;
    }
    user_ols->state = OLS_STATE_IDLE;
    user_ols->sample_count = 0;
    user_ols->xon_xoff = true;
    user_ols->read_count = 4096;
    user_ols->delay_count = 0;
    user_ols->flags = 0;
    user_ols->rle_enabled = false;
    user_ols->sample_rate = 1000000;
    capture_done = false;
    cmd_index = 0;
}

/**
* @brief UART 接收回调 - 直接从环形缓冲区读取
*/
void OLS_UARTCallback(void *user_uart) {
    (void)user_uart;
    
    if (ols_instance == NULL || ols_instance->uart == NULL)
        return;
    
    /* 从环形缓冲区读取所有数据 */
    uint16_t len = UART_GetAllDate(ols_instance->uart, cmd_buffer);
    
    for (uint16_t i = 0; i < len; i++) {
        uint8_t byte = cmd_buffer[i];
        
        /* 保存到命令缓冲区 */
        cmd_buffer[cmd_index++] = byte;
        
        /* 确定命令长度 */
        uint8_t cmd = cmd_buffer[0];
        uint8_t expected_len = 1;
        
        if (cmd >= 0x80 && cmd <= 0xBF) {
            expected_len = 5;
        } else if (cmd >= 0xC0) {
            expected_len = 5;
        }
        
        /* 检查是否收到完整命令 */
        if (cmd_index >= expected_len) {
            const uint8_t *data = (expected_len > 1) ? &cmd_buffer[1] : NULL;
            OLS_HandleCommand(cmd, data);
            cmd_index = 0;
        }
        
        /* 防止缓冲区溢出 */
        if (cmd_index >= sizeof(cmd_buffer)) {
            cmd_index = 0;
        }
    }
}

/**
* @brief 处理 SUMP 命令
*/
static void OLS_HandleCommand(uint8_t cmd, const uint8_t *data) {
    if (ols_instance == NULL)
        return;
    
    OLS_DRIVES *ols = ols_instance;
    
    switch (cmd) {
        case 0x00:  /* RESET */
            OLS_Reset(ols);
            break;
            
        case 0x01:  /* ARM */
            OLS_StartCapture();
            break;
            
        case 0x02:  /* GET_ID */
            {
                uint8_t id[4] = {'1', 'S', 'L', 'O'};
                OLS_SendBytes(id, 4);
            }
            break;
            
        case 0x04:  /* GET_METADATA */
            OLS_SendMetadata();
            break;
            
        case 0x05:  /* STOP */
            OLS_StopCapture();
            break;
            
        case 0x06:  /* GET_INPUT_DATA */
            {
                uint8_t sample = 0;
                GPIO_TypeDef *port = ols->gpio_port;
                if (port != NULL) {
                    sample |= (HAL_GPIO_ReadPin(port, GPIO_PIN_0) == GPIO_PIN_SET) ? 1 : 0;
                    sample |= (HAL_GPIO_ReadPin(port, GPIO_PIN_1) == GPIO_PIN_SET) ? 2 : 0;
                    sample |= (HAL_GPIO_ReadPin(port, GPIO_PIN_2) == GPIO_PIN_SET) ? 4 : 0;
                    sample |= (HAL_GPIO_ReadPin(port, GPIO_PIN_3) == GPIO_PIN_SET) ? 8 : 0;
                    sample |= (HAL_GPIO_ReadPin(port, GPIO_PIN_4) == GPIO_PIN_SET) ? 16 : 0;
                    sample |= (HAL_GPIO_ReadPin(port, GPIO_PIN_5) == GPIO_PIN_SET) ? 32 : 0;
                    sample |= (HAL_GPIO_ReadPin(port, GPIO_PIN_6) == GPIO_PIN_SET) ? 64 : 0;
                    sample |= (HAL_GPIO_ReadPin(port, GPIO_PIN_7) == GPIO_PIN_SET) ? 128 : 0;
                }
                OLS_SendByte(sample);
            }
            break;
            
        case 0x07:  /* GET_CAPTURE_STATE */
            /* libsigrok 期望: 0x00=空闲, 0x01=采集中, 0x02=完成 */
            if (capture_done) {
                OLS_SendByte(0x02);  /* 完成 */
            } else if (capture_in_progress) {
                OLS_SendByte(0x01);  /* 采集中 */
            } else {
                OLS_SendByte(0x00);  /* 空闲 */
            }
            break;
            
        case 0x08:  /* GET_CAPTURE_DATA */
            OLS_SendCaptureData();
            break;
            
        case 0x80:  /* SET_DIVIDER */
            if (data != NULL) {
                uint32_t libsigrok_divider = data[0] | (data[1] << 8) | (data[2] << 16);
                ols->divider = libsigrok_divider;
                
                /* libsigrok 使用 100MHz 时钟基准，我们需要转换为实际采样率
                 * 公式: actual_samplerate = CLOCK_RATE / (divider + 1)
                 * CLOCK_RATE = 100MHz (来自 libsigrok)
                 * 我们设备的采样率 = 100000000 / (libsigrok_divider + 1)
                 */
                uint32_t actual_samplerate = 100000000UL / (libsigrok_divider + 1);
                ols->sample_rate = actual_samplerate;
            }
            break;
            
        case 0x81:  /* SET_CAPTURE_SIZE */
            if (data != NULL) {
                /* SUMP 协议: 上位机发送 (样本数/4) - 1，设备需要 *4 恢复 */
                uint16_t read_count = data[0] | (data[1] << 8);
                ols->read_count = ((uint32_t)read_count + 1) * 4;
                if (ols->read_count > ols->buffer_size) {
                    ols->read_count = ols->buffer_size;
                }
            }
            break;
            
        case 0x82:  /* SET_FLAGS */
            if (data != NULL) {
                ols->flags = data[0] | (data[1] << 8);
                ols->rle_enabled = (ols->flags & CAPTURE_FLAG_RLE) != 0;
            }
            break;
            
        case 0x84:  /* SET_READ_COUNT */
            if (data != NULL) {
                /* SUMP 协议: 同 0x81，需要 *4 恢复 */
                uint32_t read_count = data[0] | (data[1] << 8);
                ols->read_count = ((uint32_t)read_count + 1) * 4;
                if (ols->read_count > ols->buffer_size) {
                    ols->read_count = ols->buffer_size;
                }
            }
            break;
            
        case 0xC0:  /* SET_TRIGGER_MASK0 */
        case 0xC4:
        case 0xC8:
        case 0xCC:
            if (data != NULL) {
                uint8_t stage = (cmd - 0xC0) / 4;
                if (stage < 4) {
                    ols->trigger.mask[stage] = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
                }
            }
            break;
            
        case 0xC1:  /* SET_TRIGGER_VALUE0 */
        case 0xC5:
        case 0xC9:
        case 0xCD:
            if (data != NULL) {
                uint8_t stage = (cmd - 0xC1) / 4;
                if (stage < 4) {
                    ols->trigger.value[stage] = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
                }
            }
            break;
            
        case 0xC2:  /* SET_TRIGGER_CONFIG0 */
        case 0xC6:
        case 0xCA:
        case 0xCE:
            if (data != NULL) {
                uint8_t stage = (cmd - 0xC2) / 4;
                if (stage < 4) {
                    ols->trigger.config[stage] = data[3];
                    ols->trigger.num_stages = stage + 1;
                }
            }
            break;
            
        default:
            break;
    }
}

/**
* @brief 发送数据
*/
static void OLS_SendBytes(const uint8_t *data, uint16_t len) {
    if (ols_instance == NULL || data == NULL || len == 0)
        return;
    
    UART_Send_Data(ols_instance->uart, (const char *)data, len);
}

static void OLS_SendByte(uint8_t byte) {
    if (ols_instance == NULL)
        return;
    
    UART_Send_Data(ols_instance->uart, (const char *)&byte, 1);
}

/**
* @brief 发送元数据
*/
static void OLS_SendMetadata(void) {
    if (ols_instance == NULL)
        return;
    
    uint8_t metadata[64];
    uint16_t index = 0;
    
    /* 设备名称 (type=0, key=0x01) - 字符串 */
    metadata[index++] = 0x01;
    strcpy((char *)&metadata[index], "F407-OLS");
    index += 9;  /* 8字符 + null 终止符 */
    
    /* 通道数量 (type=2, key=0x40) - 8位整数 */
    metadata[index++] = 0x40;
    metadata[index++] = 8;  /* 8个通道 */
    
    /* 采样内存 (type=1, key=0x21) - 32位整数 */
    metadata[index++] = 0x21;
    uint32_t sample_mem = OLS_SAMPLE_BUFFER_SIZE;  /* 缓冲区大小 */
    metadata[index++] = sample_mem & 0xFF;
    metadata[index++] = (sample_mem >> 8) & 0xFF;
    metadata[index++] = (sample_mem >> 16) & 0xFF;
    metadata[index++] = (sample_mem >> 24) & 0xFF;
    
    /* 最大采样率 (type=1, key=0x23) - 32位整数 */
    metadata[index++] = 0x23;
    uint32_t max_rate = 1000000;  /* 1MHz */
    metadata[index++] = max_rate & 0xFF;
    metadata[index++] = (max_rate >> 8) & 0xFF;
    metadata[index++] = (max_rate >> 16) & 0xFF;
    metadata[index++] = (max_rate >> 24) & 0xFF;
    
    /* 协议版本 (type=2, key=0x41) - 8位整数 */
    metadata[index++] = 0x41;
    metadata[index++] = 2;  /* 协议版本 2 */
    
    /* 元数据结束 */
    metadata[index++] = 0x00;
    
    OLS_SendBytes(metadata, index);
}

/**
* @brief 开始采集
*/
static void OLS_StartCapture(void) {
    if (ols_instance == NULL)
        return;
    
    OLS_DRIVES *ols = ols_instance;
    
    if (capture_in_progress) {
        TIMER_Stop(ols->timer);
    }
    
    ols->sample_count = 0;
    ols->state = OLS_STATE_ARMED;
    capture_in_progress = true;
    capture_done = false;
    
    /* LED 亮：表示开始采集 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    
    if (ols->read_count == 0) {
        ols->read_count = 4096;
    }
    if (ols->read_count > ols->buffer_size) {
        ols->read_count = ols->buffer_size;
    }
    
    /* 根据缓冲区大小限制最大样本数 */
    if (ols->read_count > OLS_SAMPLE_BUFFER_SIZE) {
        ols->read_count = OLS_SAMPLE_BUFFER_SIZE;
    }
    
    TIMER_Set_Frequency(ols->timer, ols->sample_rate);
    TIMER_Start(ols->timer);
    ols->state = OLS_STATE_SAMPLING;
}

/**
* @brief 停止采集
*/
static void OLS_StopCapture(void) {
    if (ols_instance == NULL)
        return;
    
    OLS_DRIVES *ols = ols_instance;
    
    if (capture_in_progress) {
        TIMER_Stop(ols->timer);
        capture_in_progress = false;
    }
    
    ols->state = OLS_STATE_DONE;
    capture_done = true;
}

/**
* @brief 发送采集数据
*/
static void OLS_SendCaptureData(void) {
    if (ols_instance == NULL)
        return;
    
    OLS_DRIVES *ols = ols_instance;
    
    if (capture_in_progress) {
        TIMER_Stop(ols->timer);
        capture_in_progress = false;
    }
    
    if (ols->sample_count > 0) {
        uint32_t total_samples = ols->sample_count;
        uint32_t sent = 0;
        
        while (sent < total_samples) {
            uint32_t remaining = total_samples - sent;
            uint32_t chunk_size = (remaining > 2048) ? 2048 : remaining;
            
            for (uint32_t i = 0; i < chunk_size; i++) {
                uint32_t buffer_idx = total_samples - 1 - sent - i;
                tx_buffer[i] = sample_buffer[buffer_idx];
            }
            
            OLS_SendBytes(tx_buffer, (uint16_t)chunk_size);
            sent += chunk_size;
        }
        
        uint8_t padding = (4 - (total_samples % 4)) % 4;
        if (padding > 0) {
            uint8_t pad_buffer[4] = {0x00, 0x00, 0x00, 0x00};
            OLS_SendBytes(pad_buffer, padding);
        }
    } else {
        /* 发送至少 4 字节空样本 */
        uint8_t empty_sample[4] = {0x00, 0x00, 0x00, 0x00};
        OLS_SendBytes(empty_sample, 4);
    }
    
    ols->state = OLS_STATE_IDLE;
    capture_done = false;
}

/* 公开函数 */
void OLS_ReturnCaptureData(OLS_DRIVES *ols) {
    (void)ols;
    OLS_SendCaptureData();
}

uint32_t OLS_GetSampleCount(OLS_DRIVES *ols) {
    if (ols == NULL)
        return 0;
    return ols->sample_count;
}

OLS_State OLS_GetState(OLS_DRIVES *ols) {
    if (ols == NULL)
        return OLS_STATE_IDLE;
    return ols->state;
}

bool OLS_IsCaptureDone(OLS_DRIVES *ols) {
    (void)ols;
    return capture_done;
}
