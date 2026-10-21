/**
 ****************************************************************************************************
 * @file        pid.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       PID算法代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 F407电机开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211014
 * 第一次发布
 *
 ****************************************************************************************************
 */

#include "pid.h"
#include "bldc.h"

PID_TypeDef  g_speed_pid;           /* 速度PID参数结构体 */
/**
 * @brief       初始化PID结构体成员
 * @param       无
 * @retval      无
 */
void pid_init(void)
{
    g_speed_pid.SetPoint = 0;       /* 设定目标Desired Value*/
    g_speed_pid.ActualValue = 0.0;  /* 设定目标Desired Value*/
    g_speed_pid.Ui=0.0;
    g_speed_pid.Up=0.0;
    g_speed_pid.Ud=0.0;
    g_speed_pid.Error = 0.0;        /* Error[1]*/
    g_speed_pid.LastError = 0.0;    /* Error[-1]*/
    g_speed_pid.PrevError = 0.0;    /* Error[-2]*/
    g_speed_pid.Proportion = KP;    /* 比例常数 Proportional Const*/
    g_speed_pid.Integral = KI;      /* 积分常数 Integral Const*/
    g_speed_pid.Derivative = KD;    /* 微分常数 Derivative Const*/ 
    g_speed_pid.IngMax = 8000;
    g_speed_pid.IngMin = -8000;
    g_speed_pid.OutMax = 8000;      /* 输出限制 */
    g_speed_pid.OutMin = -8000;   
    g_speed_pid.pid_gain = 10000; 	
}
float *user_setpoint = (float*)(&g_speed_pid.SetPoint);

/**
 * @brief       位置式PID算法
 * @param       *PID：PID结构体句柄所对应的目标值
 * @param       Feedback_value ： 实际值
 * @retval      目标控制量
 */
float increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value)
{
    PID->Error = (float)(PID->SetPoint - Feedback_value);   /* 偏差值 */
    PID->Up = PID->Proportion * PID->Error /PID->pid_gain;
    PID->Ui += (PID->Error * PID->Integral/PID->pid_gain);
    LIMIT_OUT(PID->Ui,PID->IngMax,PID->IngMin);
    PID->Ud = PID->Derivative /PID->pid_gain * (PID->Error - PID->LastError);
    PID->ActualValue = PID->Up + PID->Ui + PID->Ud;
    LIMIT_OUT(PID->ActualValue,PID->OutMax,PID->OutMin);
    PID->LastError = PID->Error;
    return ((PID->ActualValue));                   /* 返回实际控制数值 */
}
