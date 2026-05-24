/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp.h"
#include <string.h>
#include <stdio.h>
#include "../../User_Algorithm/user_coord.h"


/* 主循环注册表 --------------------------------------------------------------*/
void (*loop_event[MAX_LOOP_EVENT])(void) = {0};
uint8_t loop_event_num = 0;

void LOOP_EVENT_Handle(void) {
    for (uint8_t event_index = 0 ; event_index < loop_event_num ; event_index++) {
        loop_event[event_index]();
    }
}

/* JScope ------------------------------------------------------------------*/
#ifdef HAL_TIM_MODULE_ENABLED
JScope_Transmit_t jscope_transmit = {0};
uint8_t JScope_RTT_UpBuffer[BUFFER_SIZE_UP] = {0};
#endif /* HAL_TIM_MODULE_ENABLED */


/* 接口定义 --------------------------------------------------------------------*/

LED_DRIVES blue_led = {0};

UART_DRIVES user_uart_debug = {0};
void user_uart_(void* user_uart);

TIMER_DRIVES user_timer_ols = {0};

// 蜂鸣器
BUZZER_DRIVES user_buzzer_1 = {0};

// 启动音乐
STARTUP_MUSIC_DRIVES user_startup_music = {0};
SysTick_Task user_startup_music_task = {0};

// LED 闪烁
SysTick_Task LED_Blink_Task = {0};
void LED_Blink_Callback(void *arg) {
    // const LED_DRIVES* led = (LED_DRIVES*)arg;
    // LED_Toggle(led);

    static uint8_t led_state = 0;

    if (led_state == 0x00) {
        led_state = 0x01;
    }

    if (key0_state == 0)
        GPIOA->ODR = ~led_state;
    else
        GPIOA->ODR = led_state;

    led_state <<= 1;
}


// 状态灯
LED_DRIVES user_led = {0};


uint8_t key0_state = {0};
uint8_t key1_state = {0};
uint8_t key2_state = {0};


// PWM
PWM_DRIVES user_pwm_1 = {0};