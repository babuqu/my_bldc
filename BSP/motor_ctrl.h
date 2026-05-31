/**
 ****************************************************************************************************
 * @file        motor_ctrl.h
 * @author      Motor Control Team
 * @version     V1.0
 * @date        2026-05-24
 * @brief       无刷电机综合控制头文�?
 ****************************************************************************************************
 */
#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H

#include "main.h"
#include "bldc.h"
#include "pid.h"
#include "zero_ctr.h"

/******************************************************************************************/
/* 控制模式定义 */
#define MOTOR_CTRL_MODE_OPENLOOP    0       /* 开环控�?*/
#define MOTOR_CTRL_MODE_CLOSEDLOOP  1       /* 闭环控制 */
#define MOTOR_CTRL_MODE_TORQUE      2       /* 转矩控制 */
#define MOTOR_CTRL_MODE_POSITION    3       /* 位置控制 */

/******************************************************************************************/
/* 电机状态机 */
typedef enum {
    MOTOR_STATE_IDLE = 0,       /* 空闲 */
    MOTOR_STATE_INIT,           /* 初始�?*/
    MOTOR_STATE_ALIGN,          /* 预定�?*/
    MOTOR_STATE_OPENLOOP,       /* 开环加�?*/
    MOTOR_STATE_CLOSEDLOOP,     /* 闭环运行 */
    MOTOR_STATE_STOP,           /* 停止 */
    MOTOR_STATE_FAULT           /* 故障 */
} MotorState_t;

/******************************************************************************************/
/* 电机控制结构�?*/
typedef struct {
    MotorState_t    state;              /* 当前状�?*/
    uint8_t         ctrl_mode;          /* 控制模式 */
    int32_t         target_speed;       /* 目标速度 (RPM) */
    int32_t         target_pos;         /* 目标位置 */
    int32_t         actual_speed;       /* 实际速度 */
    int32_t         actual_pos;         /* 实际位置 */
    int16_t         target_current;     /* 目标电流 */
    int16_t         actual_current;     /* 实际电流 */
    uint16_t        pwm_duty;           /* PWM 占空�?*/
    uint8_t         dir;                /* 转向 */
    uint8_t         fault_code;         /* 故障�?*/
    uint32_t        run_time;           /* 运行时间 (ms) */
    float           temperature;        /* 温度 */
    float           bus_voltage;        /* 母线电压 */
} MotorCtrl_t;

/******************************************************************************************/
/* 故障码定�?*/
#define MOTOR_FAULT_NONE            0x00    /* 无故�?*/
#define MOTOR_FAULT_OVER_CURRENT    0x01    /* 过流 */
#define MOTOR_FAULT_OVER_VOLTAGE    0x02    /* 过压 */
#define MOTOR_FAULT_UNDER_VOLTAGE   0x03    /* 欠压 */
#define MOTOR_FAULT_OVER_TEMP       0x04    /* 过温 */
#define MOTOR_FAULT_LOCKED_ROTOR   0x05    /* 堵转 */
#define MOTOR_FAULT_HALL_ERROR      0x06    /* 霍尔错误 */
#define MOTOR_FAULT_SPEED_ERROR     0x07    /* 速度错误 */

/******************************************************************************************/
/* 任务同步标志 */
#define COMMUTATION_TASK_PRIO       osPriorityRealtime7
#define BEMF_TASK_PRIO              osPriorityRealtime6
#define SPEED_TASK_PRIO             osPriorityNormal

/* 任务周期 (ms) */
#define COMMUTATION_TASK_PERIOD     1       /* 换相任务 1ms */
#define BEMF_TASK_PERIOD            2       /* BEMF 任务 2ms */
#define SPEED_TASK_PERIOD           10      /* 速度任务 10ms */
#define MONITOR_TASK_PERIOD         100     /* 监控任务 100ms */

/******************************************************************************************/
/* 外部变量声明 */
extern MotorCtrl_t g_motor_ctrl;
extern osThreadId_t CommutationTaskHandle;
extern osThreadId_t BEMF_TaskHandle;
extern osThreadId_t SpeedTaskHandle;

/* 信号�?*/
extern osSemaphoreId_t g_bemf_ready_sem;    /* BEMF 处理完成信号�?*/
extern osSemaphoreId_t g_commutation_ready_sem; /* 换相完成信号�?*/

/* 队列 */
extern QueueHandle_t g_motor_cmd_queue;     /* 电机命令队列 */
extern QueueHandle_t g_motor_data_queue;    /* 电机数据队列 */

/******************************************************************************************/
/* 函数声明 */
void MotorCtrl_Init(void);                          /* 电机控制初始�?*/
void MotorCtrl_StartMotor(int32_t speed, uint8_t dir);  /* 启动电机 */
void MotorCtrl_StopMotor(void);                     /* 停止电机 */
void MotorCtrl_SetSpeed(int32_t speed);             /* 设置速度 */
void MotorCtrl_SetPosition(int32_t pos);            /* 设置位置 */
void MotorCtrl_EmergencyStop(void);                 /* 急停 */
MotorState_t MotorCtrl_GetState(void);              /* 获取状�?*/
uint8_t MotorCtrl_GetFaultCode(void);               /* 获取故障�?*/
void MotorCtrl_ClearFault(void);                    /* 清除故障 */

/* 任务函数声明 */
void MotorCtrl_CommutationTaskFun(void *argument);
void MotorCtrl_BEMF_Task_Fun(void *argument);
void MotorCtrl_SpeedTaskFun(void *argument);

#endif /* __MOTOR_CTRL_H */
