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



