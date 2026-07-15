#include "main.h"
#ifndef __PARAM_H__
#define __PARAM_H__

typedef struct
{
    float kp;/* 比例常数 Proportional Const*/ 
    float ki;/* 积分常数 Integral Const*/
    float kd;/* 微分常数 Derivative Const*/ 

    float set_speed;/* 设定目标Desired Value*/
    float actual_speed;/* 实际转速*/

    float i_max;//积分相最大限幅
    float i_min;//积分相最小限幅
    float out_max;//PID输出最大限幅
    float out_min;//PID输出最小限幅
	  float flag;
} param_t;

typedef enum
{
    PARAM_U16,
    PARAM_S16,
    PARAM_FLOAT
}ParamType_t;

typedef struct
{
    float *addr;      // 参数地址
    uint16_t reg;     // Modbus寄存器位置
    float scale;      // 缩放系数
	  ParamType_t type; //参数类型
} param_map_t;

extern param_t g_param;

void Param_Init(param_t *p);
void Param_ToSystem(const param_t *p);

uint8_t Param_UpdateFromModbus(uint16_t *reg);
void Param_Publish(uint16_t *reg);
void Param_To_System(param_t *p);
uint8_t Param_LoadFromFlash(param_t *p);
uint8_t Param_SaveToFlash(param_t *p);
void Param_Apply(void);
void Param_InitDefault(void);
#endif
