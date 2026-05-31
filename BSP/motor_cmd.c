/**
 ****************************************************************************************************
 * @file        motor_cmd.c
 * @author      Motor Control Team
 * @version     V1.0
 * @date        2026-05-24
 * @brief       电机命令解析实现
 ****************************************************************************************************
 */

#include "motor_cmd.h"
#include "motor_ctrl.h"
#include "pid.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

extern PID_TypeDef g_speed_pid;
extern GetValue_t sensor_data;
extern _bldc_obj g_bldc_motor1;
extern MotorCtrl_t g_motor_ctrl;

/**
 * @brief       命令解析初始化
 * @param       无
 * @retval      无
 */
void MotorCmd_Init(void)
{
    UartPrintf("\r\n");
    UartPrintf("========================================\r\n");
    UartPrintf("   BLDC Motor Control System V1.0\r\n");
    UartPrintf("========================================\r\n");
    UartPrintf("Type 'help' for command list\r\n");
    UartPrintf("========================================\r\n");
}

/**
 * @brief       打印帮助信息
 * @param       无
 * @retval      无
 */
void MotorCmd_PrintHelp(void)
{
    UartPrintf("\r\nAvailable commands:\r\n");
    UartPrintf("  start [speed] [dir]  - Start motor (speed:RPM, dir:0=CCW,1=CW)\r\n");
    UartPrintf("  stop                 - Stop motor\r\n");
    UartPrintf("  speed [value]        - Set speed (RPM)\r\n");
    UartPrintf("  emergency            - Emergency stop\r\n");
    UartPrintf("  fault                - Clear fault\r\n");
    UartPrintf("  state                - Get motor state\r\n");
    UartPrintf("  data                 - Get motor data\r\n");
    UartPrintf("  pid [p] [i] [d]      - Set PID parameters\r\n");
    UartPrintf("  help                 - Show this help\r\n");
    UartPrintf("\r\n");
}

/**
 * @brief       处理命令
 * @param       buf: 命令缓冲区
 * @param       len: 命令长度
 * @retval      无
 */
void MotorCmd_Process(uint8_t *buf, uint16_t len)
{
    char cmd_str[32] = {0};
    char arg1[32] = {0};
    char arg2[32] = {0};
    int32_t speed = 0;
    uint8_t dir = 0;
    float kp, ki, kd;
    
    /* 解析命令 */
    int n = sscanf((char*)buf, "%31s %31s %31s", cmd_str, arg1, arg2);
    if(n < 1) return;
    
    /* 启动电机 */
    if(strcmp(cmd_str, "start") == 0)
    {
        if(n >= 3)
        {
            speed = atoi(arg1);
            dir = atoi(arg2);
            if(dir > 1) dir = 1;
            
            MotorCtrl_StartMotor(speed, dir == 0 ? CCW : CW);
            UartPrintf("Command: start motor at %d RPM, dir=%s\r\n", speed, dir == 0 ? "CCW" : "CW");
        }
        else
        {
            UartPrintf("Usage: start [speed] [dir]\r\n");
            UartPrintf("  speed: target speed in RPM (e.g., 1000)\r\n");
            UartPrintf("  dir: 0=CCW (default), 1=CW\r\n");
        }
    }
    /* 停止电机 */
    else if(strcmp(cmd_str, "stop") == 0)
    {
        MotorCtrl_StopMotor();
        UartPrintf("Command: stop motor\r\n");
    }
    /* 设置速度 */
    else if(strcmp(cmd_str, "speed") == 0)
    {
        if(n >= 2)
        {
            speed = atoi(arg1);
            MotorCtrl_SetSpeed(speed);
            UartPrintf("Command: set speed to %d RPM\r\n", speed);
        }
        else
        {
            UartPrintf("Usage: speed [value]\r\n");
        }
    }
    /* 急停 */
    else if(strcmp(cmd_str, "emergency") == 0)
    {
        MotorCtrl_EmergencyStop();
        UartPrintf("Command: emergency stop!\r\n");
    }
    /* 清除故障 */
    else if(strcmp(cmd_str, "fault") == 0)
    {
        MotorCtrl_ClearFault();
    }
    /* 获取状态 */
    else if(strcmp(cmd_str, "state") == 0)
    {
        MotorState_t state = MotorCtrl_GetState();
        uint8_t fault = MotorCtrl_GetFaultCode();
        
        UartPrintf("Motor State: ");
        switch(state)
        {
            case MOTOR_STATE_IDLE:        UartPrintf("IDLE\r\n"); break;
            case MOTOR_STATE_INIT:        UartPrintf("INIT\r\n"); break;
            case MOTOR_STATE_ALIGN:       UartPrintf("ALIGN\r\n"); break;
            case MOTOR_STATE_OPENLOOP:    UartPrintf("OPENLOOP\r\n"); break;
            case MOTOR_STATE_CLOSEDLOOP:  UartPrintf("CLOSEDLOOP\r\n"); break;
            case MOTOR_STATE_STOP:        UartPrintf("STOP\r\n"); break;
            case MOTOR_STATE_FAULT:       UartPrintf("FAULT\r\n"); break;
            default:                      UartPrintf("UNKNOWN\r\n"); break;
        }
        
        UartPrintf("Fault Code: 0x%02X\r\n", fault);
        UartPrintf("BEMF Status: %d\r\n", g_zero_ctr_status);
    }
    /* 获取数据 */
    else if(strcmp(cmd_str, "data") == 0)
    {
        UartPrintf("\r\n=== Motor Data ===\r\n");
        UartPrintf("Target Speed: %d RPM\r\n", g_motor_ctrl.target_speed);
        UartPrintf("Actual Speed: %d RPM\r\n", g_motor_ctrl.actual_speed);
        UartPrintf("PWM Duty: %d\r\n", g_motor_ctrl.pwm_duty);
        UartPrintf("Position: %d\r\n", g_motor_ctrl.actual_pos);
        UartPrintf("Temperature: %.2f C\r\n", g_motor_ctrl.temperature);
        UartPrintf("Bus Voltage: %.2f V\r\n", g_motor_ctrl.bus_voltage);
        UartPrintf("Run Time: %d ms\r\n", g_motor_ctrl.run_time);
        
        UartPrintf("\r\n=== ADC Data ===\r\n");
        UartPrintf("I_U: %.3f A\r\n", sensor_data.i_u);
        UartPrintf("I_V: %.3f A\r\n", sensor_data.i_v);
        UartPrintf("I_W: %.3f A\r\n", sensor_data.i_w);
        UartPrintf("BEMF_U: %.3f V\r\n", sensor_data.emf_u);
        UartPrintf("BEMF_V: %.3f V\r\n", sensor_data.emf_v);
        UartPrintf("BEMF_W: %.3f V\r\n", sensor_data.emf_w);
        UartPrintf("Vbus: %.2f V\r\n", sensor_data.Vbus_v);
        UartPrintf("NTC: %.2f C\r\n", sensor_data.ntc);
        
        UartPrintf("\r\n=== PID Data ===\r\n");
        UartPrintf("SetPoint: %.2f\r\n", g_speed_pid.SetPoint);
        UartPrintf("Error: %.2f\r\n", g_speed_pid.Error);
        UartPrintf("Up: %.2f\r\n", g_speed_pid.Up);
        UartPrintf("Ui: %.2f\r\n", g_speed_pid.Ui);
        UartPrintf("Ud: %.2f\r\n", g_speed_pid.Ud);
        UartPrintf("Output: %.2f\r\n", g_speed_pid.ActualValue);
    }
    /* 设置 PID 参数 */
    else if(strcmp(cmd_str, "pid") == 0)
    {
        if(n >= 4)
        {
            kp = atof(arg1);
            ki = atof(arg2);
            kd = atof(arg3);
            
            g_speed_pid.Proportion = kp;
            g_speed_pid.Integral = ki;
            g_speed_pid.Derivative = kd;
            
            UartPrintf("PID set: Kp=%.4f, Ki=%.6f, Kd=%.6f\r\n", kp, ki, kd);
        }
        else
        {
            UartPrintf("Current PID: Kp=%.4f, Ki=%.6f, Kd=%.6f\r\n", 
                       g_speed_pid.Proportion, g_speed_pid.Integral, g_speed_pid.Derivative);
            UartPrintf("Usage: pid [kp] [ki] [kd]\r\n");
        }
    }
    /* 帮助 */
    else if(strcmp(cmd_str, "help") == 0)
    {
        MotorCmd_PrintHelp();
    }
    /* 未知命令 */
    else
    {
        UartPrintf("Unknown command: %s\r\n", cmd_str);
        UartPrintf("Type 'help' for command list\r\n");
    }
}
