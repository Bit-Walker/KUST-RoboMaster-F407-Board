#ifndef __USER_DJI_MOTOR_H__
#define __USER_DJI_MOTOR_H__
#ifdef HAL_CAN_MODULE_ENABLED

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "../../Core/Inc/bsp_config.h"
#include "../../User_Algorithm/user_pid.h"
#include "../../User_Drives/user_can.h"

/* 宏定义 -------------------------------------------------------------------*/
#define GM6020_CURRENT_CONTROL_ID_1   (0x1FE)
#define GM6020_CURRENT_CONTROL_ID_2   (0x2FE)
#define GM6020_FEEDBACK_BASE_ID       (0x204)

#define C6x0_CURRENT_CONTROL_ID_1     (0x200)
#define C6x0_CURRENT_CONTROL_ID_2     (0x1FF)
#define C6x0_FEEDBACK_BASE_ID         (0x200)

// 可以看到，大疆不同型号电调的ID存在冲突

/* 类型定义 ------------------------------------------------------------------*/
enum DJI_MOTOR_Type {
    GM6020 = 0,
    M3508_gear = 1,
    M3508_direct = 2,
    M2006 = 3,
};
enum DJI_MOTOR_Controller_Type {
    GM6020_Controller = 0,
    C620_Controller = 1,
    C610_Controller = 2,
};
enum DJI_Control_Mode {
    Rotor_angle = 0,
    Rotor_speed = 1,
    Torque_current = 2,
    OpenLoop_current = 3,
};

typedef struct {
    CAN_DRIVES *can;
    enum DJI_MOTOR_Type motor_type;
    enum DJI_MOTOR_Controller_Type controller_type;
    enum DJI_Control_Mode control_mode;
    float target;
    uint8_t id;
    uint16_t ctrl_id;
    uint16_t fdb_id;
    uint16_t rotor_angle;
    int16_t  rotor_speed;
    int16_t  torque_current;
    uint8_t  temperate;
    PID_Controller pid_controller;
} DJI_MOTOR_DRIVES;


/* 函数声明 ------------------------------------------------------------------*/
void DJI_Motor_Init(DJI_MOTOR_DRIVES *motor, CAN_DRIVES* can_drive, uint8_t id, enum DJI_MOTOR_Type motor_type, enum DJI_Control_Mode mode, float kp, float ki, float kd, float max_out, float max_iout);
void DJI_Motor_Target(DJI_MOTOR_DRIVES *motor,float target);
void DJI_Motor_Handle(CAN_DRIVES* can_drive);
void DJI_Motor_Execute(CAN_DRIVES* can_drive);

#endif /* HAL_CAN_MODULE_ENABLED */
#endif //__USER_DJI_MOTOR_H__
