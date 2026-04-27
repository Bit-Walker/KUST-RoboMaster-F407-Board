/* 包含头文件 ----------------------------------------------------------------*/
#include "../user_pid.h"

/* 函数体 --------------------------------------------------------------------*/
void PID_Init(PID_Controller *pid, Err_Calculate err_calculate,
    const float kp, const float ki, const float kd,
    const float max_out, const float max_iout) {

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    
    pid->err[0] = 0.0f;
    pid->err[1] = 0.0f;
    pid->err[2] = 0.0f;
    
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->out = 0.0f;

    pid->err_calculate = err_calculate;
    
    pid->set = 0.0f;
    pid->fdb = 0.0f;
}

float PID_Calculate(PID_Controller *pid, const float target, const float feedback) {
    pid->set = target;
    pid->fdb = feedback;

    pid->err[0] = pid->err_calculate(pid->fdb,pid->set);

    pid->Pout =  pid->Kp *  pid->err[0];
    pid->Iout += pid->Ki *  pid->err[0];
    pid->Dout =  pid->Kd * (pid->err[0] - pid->err[1]);

    // 积分限幅
    if (pid->Iout > pid->max_iout) {
        pid->Iout = pid->max_iout;
    } else if (pid->Iout < -pid->max_iout) {
        pid->Iout = -pid->max_iout;
    }
    
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    
    // 输出限幅
    if (pid->out > pid->max_out) {
        pid->out = pid->max_out;
    } else if (pid->out < -pid->max_out) {
        pid->out = -pid->max_out;
    }

    pid->err[2] = pid->err[1];
    pid->err[1] = pid->err[0];
    
    return pid->out;
}
