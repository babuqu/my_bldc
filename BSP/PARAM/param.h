#ifndef __PARAM_H__
#define __PARAM_H__

#include <stdint.h>

typedef struct param_t
{
    float kp;           /* 速度环比例增益 */
    float ki;           /* 速度环积分增益 */
    float kd;           /* 速度环微分增益 */
    float actual_speed; /* 实际转速运行值，不属于可配置参数 */
    int32_t i_max;      /* 积分项上限 */
    int32_t i_min;      /* 积分项下限 */
    int32_t out_max;    /* PID 输出上限 */
    int32_t out_min;    /* PID 输出下限 */
    float flag;         /* 兼容旧参数结构的有效标记 */
} param_t;

extern param_t g_param;

/* 参数服务接口：参数值通过稳定 ID 访问，与通信地址无关。 */
uint8_t Param_GetValue(const param_t *p, uint16_t id, double *value);
uint8_t Param_SetValue(param_t *p, uint16_t id, double value);
uint8_t Param_Commit(param_t *candidate);
uint8_t Param_Sanitize(param_t *p);
void Param_To_System(param_t *p);
void Param_InitDefault(void);
uint8_t Param_Init_(void);

/* Modbus 适配接口：参数校验、持久化和运行应用由参数服务完成。 */
uint8_t Param_UpdateFromModbus(uint16_t *registers,
                               uint16_t start_reg, uint16_t reg_count);
void Param_Publish(uint16_t *registers);
uint8_t Param_Apply(uint16_t start_reg, uint16_t reg_count);

/* 目标转速是运行命令，不属于可持久化配置参数。 */
uint8_t Param_SetSpeedCommand(int32_t speed);
int32_t Param_GetSpeedCommand(void);

/* 从 Flash 恢复/保存 registry 权威值表；返回 0 表示成功。 */
uint8_t Param_LoadFromFlash(param_t *p);
uint8_t Param_SaveToFlash(void);

#endif
