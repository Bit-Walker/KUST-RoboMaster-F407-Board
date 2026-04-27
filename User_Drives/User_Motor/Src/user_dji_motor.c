/* 包含头文件 ----------------------------------------------------------------*/
#include <math.h>
#include "../user_dji_motor.h"
#include "../../Core/Inc/bsp.h"
#ifdef HAL_CAN_MODULE_ENABLED

/* 私有变量 ------------------------------------------------------------------*/
static DJI_MOTOR_DRIVES *motor_drives[DJI_MOTOR_NUM];
static uint8_t motor_num = 0;

/* 函数体 --------------------------------------------------------------------*/
float angle_error(const float feedback, const float target) {
    float error =  target - feedback;

    if (fabsf(target - feedback - 8192.0f) < fabsf(error))
        error =  target - feedback - 8192.0f;

    if (fabsf(target - feedback + 8192.0f) < fabsf(error))
        error = target - feedback + 8192.0f;

    return error;
}

float general_error(const float feedback, const float target) {
    return target - feedback;
}

/**
  * @brief  设置电机目标
  */
void DJI_Motor_Target(DJI_MOTOR_DRIVES *motor, float target){
    motor->target = target;
}

/**
  * @brief  初始化电机
  */
void DJI_Motor_Init(DJI_MOTOR_DRIVES* motor, CAN_DRIVES* can_drive, uint8_t id, enum DJI_MOTOR_Type motor_type, enum DJI_Control_Mode mode, float kp, float ki, float kd, float max_out, float max_iout) {
    motor->can = can_drive;
    motor->id = id;
    motor->motor_type = motor_type;
    motor->control_mode = mode;
    motor->target = 0;

    switch (motor_type) {
        case GM6020:
            motor->controller_type = GM6020_Controller;
            break;
        case M3508_direct:
        case M3508_gear:
            motor->controller_type = C620_Controller;
            break;
        case M2006:
            motor->controller_type = C610_Controller;
            break;
    }

    switch (motor->controller_type) {
        case GM6020_Controller:
            motor->fdb_id  = GM6020_FEEDBACK_BASE_ID + id;
            if (1 <= id && id <= 4) motor->ctrl_id = GM6020_CURRENT_CONTROL_ID_1;
            if (5 <= id && id <= 7) motor->ctrl_id = GM6020_CURRENT_CONTROL_ID_2;
            break;
        case C620_Controller:
        case C610_Controller:
            motor->fdb_id = C6x0_FEEDBACK_BASE_ID + id;
            if (1 <= id && id <= 4) motor->ctrl_id = C6x0_CURRENT_CONTROL_ID_1;
            if (5 <= id && id <= 8) motor->ctrl_id = C6x0_CURRENT_CONTROL_ID_2;
            break;
    }

    switch (motor->control_mode) {
        case Rotor_angle:
            PID_Init(&motor->pid_controller, angle_error, kp, ki, kd, max_out, max_iout);
            break;
        case Rotor_speed:
        case Torque_current:
            PID_Init(&motor->pid_controller, general_error, kp, ki, kd, max_out, max_iout);
            break;
        case OpenLoop_current:
            break;
    }

    motor_drives[motor_num] = motor;
    motor_num++;
}

/**
  * @brief  处理 CAN 接收数据
  * @note   需要在 CAN 回调中调用
  */
void DJI_Motor_Handle(CAN_DRIVES* can_drive) {
    for (uint8_t motor_index = 0; motor_index < motor_num; motor_index++) {
        DJI_MOTOR_DRIVES *motor = motor_drives[motor_index];
        
        if (motor->can->hcan == can_drive->hcan && motor->fdb_id == can_drive->rx_msg.StdId) {
            const uint8_t *data = can_drive->rx_msg.Data;
            motor->rotor_angle    = (uint16_t)(data[0] << 8 | data[1]);
            motor->rotor_speed    = (int16_t) (data[2] << 8 | data[3]);
            motor->torque_current = (int16_t) (data[4] << 8 | data[5]);
            motor->temperate = data[6];
            
            if (motor->control_mode == OpenLoop_current) {
                continue;
            }
            if (motor->control_mode == Rotor_speed) {
                const float speed = motor->rotor_speed;
                PID_Calculate(&motor->pid_controller, motor->target, speed);
            }
            if (motor->control_mode == Rotor_angle) {
                const float angle = motor->rotor_angle;
                PID_Calculate(&motor->pid_controller, motor->target, angle);
            }
            if (motor->control_mode == Torque_current) {
                const float current = motor->torque_current;
                PID_Calculate(&motor->pid_controller, motor->target, current);
            }
        }
    }
}

void DJI_Motor_Execute(CAN_DRIVES* can_drive) {
    uint8_t GM6020_control_id_1_frame [8] = {0};
    uint8_t GM6020_control_id_2_frame [8] = {0};
    uint8_t C6x0_control_id_1_frame   [8] = {0};
    uint8_t C6x0_control_id_2_frame   [8] = {0};

    for (uint8_t motor_index = 0; motor_index < motor_num; motor_index++) {
        const DJI_MOTOR_DRIVES *motor = motor_drives[motor_index];
        if (motor->can != can_drive)
            continue;
        int16_t current = 0;
        switch (motor->control_mode) {
            case OpenLoop_current:
                current = (int16_t) motor->target;
                break;
            case Rotor_angle:
            case Rotor_speed:
            case Torque_current:
                current = (int16_t) motor->pid_controller.out;
                break;
        }

        switch (motor->ctrl_id) {
            case GM6020_CURRENT_CONTROL_ID_1:
                GM6020_control_id_1_frame [2 * motor->id - 1  ] = (uint8_t) (current >> 0);
                GM6020_control_id_1_frame [2 * motor->id - 2  ] = (uint8_t) (current >> 8);
                break;
            case GM6020_CURRENT_CONTROL_ID_2:
                GM6020_control_id_2_frame [2 * motor->id - 9  ] = (uint8_t) (current >> 0);
                GM6020_control_id_2_frame [2 * motor->id - 10 ] = (uint8_t) (current >> 8);
                break;
            case C6x0_CURRENT_CONTROL_ID_1:
                C6x0_control_id_1_frame   [2 * motor->id - 1  ] = (uint8_t) (current >> 0);
                C6x0_control_id_1_frame   [2 * motor->id - 2  ] = (uint8_t) (current >> 8);
                break;
            case C6x0_CURRENT_CONTROL_ID_2:
                C6x0_control_id_2_frame   [2 * motor->id - 9  ] = (uint8_t) (current >> 0);
                C6x0_control_id_2_frame   [2 * motor->id - 10 ] = (uint8_t) (current >> 8);
                break;
            default:
                break;
        }
    }

    uint8_t GM6020_control_id_1_sign = 1;
    uint8_t GM6020_control_id_2_sign = 1;
    uint8_t C6x0_control_id_1_sign = 1;
    uint8_t C6x0_control_id_2_sign = 1;

    for (uint8_t motor_index = 0; motor_index < motor_num; motor_index++) {
        const DJI_MOTOR_DRIVES *motor = motor_drives[motor_index];
        if (motor->can != can_drive)
            continue;

        if (GM6020_control_id_1_sign) {
            CAN_Send(can_drive, GM6020_CURRENT_CONTROL_ID_1, GM6020_control_id_1_frame, 8);
            GM6020_control_id_1_sign = 0;
        }
        if (GM6020_control_id_2_sign) {
            CAN_Send(can_drive, GM6020_CURRENT_CONTROL_ID_2, GM6020_control_id_2_frame, 8);
            GM6020_control_id_2_sign = 0;
        }
        if (C6x0_control_id_1_sign) {
            CAN_Send(can_drive, C6x0_CURRENT_CONTROL_ID_1, C6x0_control_id_1_frame, 8);
            C6x0_control_id_1_sign = 0;
        }
        if (C6x0_control_id_2_sign) {
            CAN_Send(can_drive, C6x0_CURRENT_CONTROL_ID_2, C6x0_control_id_2_frame, 8);
            C6x0_control_id_2_sign = 0;
        }

    }
}

#endif /* HAL_CAN_MODULE_ENABLED */
