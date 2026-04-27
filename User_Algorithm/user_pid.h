#ifndef __USER_PID_H__
#define __USER_PID_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef float (*Err_Calculate)(const float feedback, const float target);

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    
    float max_out;
    float max_iout;
    
    float set;
    float fdb;
    
    float out;
    float Pout;
    float Iout;
    float Dout;

    Err_Calculate err_calculate;
    
    float err[3];   // [0]:当前误差, [1]:上一次误差, [2]:上上次误差
    
} PID_Controller;

/* 函数声明 ------------------------------------------------------------------*/
void PID_Init(PID_Controller *pid, Err_Calculate err_calculate, const float kp, const float ki, const float kd, const float max_out, const float max_iout);
float PID_Calculate(PID_Controller *pid, const float target, const float feedback);

#endif //__USER_PID_H__
