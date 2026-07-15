/**
 ****************************************************************************************************
 * @file        zero_ctr.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-19
 * @brief       无感过零控制 驱动代码
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
 * V1.0 20220518
 * 第一次发布
 *
 ****************************************************************************************************
 */
#ifndef __ZERO_CTR_H
#define __ZERO_CTR_H

#include "main.h"

#define FirstOrderRC_LPF(Yn_1,Xn,a) Yn_1 = (1-a)*Yn_1 + a*Xn; //Yn:out;Xn:in;a:系数

/***************************** 电机驱动模式选择&过零信号过滤次数 *****************************/

#define DRIVE_MODE      1           /* 0为开环  1为闭环（过零点）*/
#define FILTER_LONG     0x00FF      /* 获取稳定过零信号的过滤次数 */

/***************************** 电机状态结构体 *****************************/

typedef struct
{
    uint8_t     HalllessUVW[3];     /* 读取三个反电动势的对应状态 */
    uint8_t     Hallless_State;     /* 当前反电动势状态 */
    uint8_t     OldHallless_State;  /* 历史反电动势状态 */
    int32_t     step_angle_error;   /* 步进角度误差 */
    int32_t     step_angle;         /* 步进角度 */
    int32_t     step_angleFitter;   /* 步进角度 滤波值 */
    uint16_t    Speed_count;        /* 速度计数值 */
    uint16_t    Speed_countFitter;  /* 速度计数滤波值 */
    uint16_t    Speed_count_old;    /* 速度计数历史值 */
    uint32_t    speed_coeff;        /* 速度系数 */
    uint8_t     Poles;              /* 电机极对数 */
    uint8_t     Move_State;         /* 电机旋转状态 */
    int16_t     Speed_RPM;          /* 电机旋转速度 */
    uint16_t    Speed_RPMF;         /* 电机旋转滤波速度 */
    uint16_t    Queue_Status[3];    /* 滤波 */
    uint16_t    Filter_Count;       /* 滤波计数 */
    uint16_t    Filter_CountF;      /* 滤波计数 */
    uint16_t    Filter_Math;        /* 滤波 */
    uint16_t    QFilter_Status[3];  /* 滤波 */
    uint16_t    Filter_Edge;        /* 滤波 */
    uint16_t    Filter_Delay;       /* 滤波 */
    uint16_t    Filter_Count_All;   /* 滤波计数 */
}Hallless;

extern uint8_t g_zero_ctr_status;   /* 过零启动标识信号 */
                                                
typedef struct
{
    uint16_t    Freq_T_Ref;         /* 开环根据固定霍尔状态启动频率 */
    uint16_t    Voilage_Ref;        /* 开环给定参考电压 */
    uint16_t    VvvF_huanxCount;    /* 换相计数 */
    uint16_t    VvvF_Count;         /* VF计算 */
    uint16_t    VvvF_state;         /* VF换相状态 */
    uint16_t    VvvF_num[6];        /* VF霍尔换相顺序 */
    uint16_t    OldVF_State;        /* 历史VF换相状态 */
    uint16_t    initial_state;      /* VF初始状态定位 */
    uint16_t    Freq_T_Ref_Count;   /* 换相计数 */
    uint16_t    Count;              /* 换相计数 */
}VvvF_start;

/***************************** 外部函数声明 *****************************/

void anwerfen_sw(void);             /* 无感六步换相顺序 */
void change_voltage(void);          /* 改变换向电压 */
void zero_ctr_init(void);           /* 无感控制初始化 */
void zero_ctr_loop(void);           /* 无感控制逻辑 */
uint8_t hallless_sw(void);          /* 过零闭环控制 */

#endif
