#ifndef USER_BSP_H
#define USER_BSP_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "../../SEGGER_RTT/SEGGER_RTT.h"
#include "../../User_Architect/user_systick.h"


/* 全局注册表 ----------------------------------------------------------------*/
#define MAX_LOOP_EVENT 32
void LOOP_EVENT_Handle(void);
typedef void (*LOOP_Event)(void);
extern LOOP_Event loop_event[MAX_LOOP_EVENT];
extern uint8_t loop_event_num;

/* JScope ------------------------------------------------------------------*/
#include "../../SEGGER_RTT/user_JScope_Transmit.h"
#ifdef HAL_TIM_MODULE_ENABLED
extern JScope_Transmit_t jscope_transmit;
extern uint8_t JScope_RTT_UpBuffer[BUFFER_SIZE_UP];
#endif /* HAL_TIM_MODULE_ENABLED */

/* 接口定义 ------------------------------------------------------------------*/

#include "../User_Drives/user_led.h"
extern LED_DRIVES blue_led;

#include "../User_Drives/user_uart.h"
extern UART_DRIVES user_uart_debug;

#include "../User_Drives/user_timer.h"
extern TIMER_DRIVES user_timer_ols;

#include "../User_Application/open_logic_sniffer.h"


#endif // USER_BSP_H
