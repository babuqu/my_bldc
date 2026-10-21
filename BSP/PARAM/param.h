#ifndef __PARAM_H__
#define __PARAM_H__

#include <stdint.h>
#include "param_list.h"

typedef struct param_t
{
#define PARAM_FIELD(name_, id_, type_, access_, min_, max_, def_, ctype_, field_, publish_) ctype_ field_;
    PARAM_LIST(PARAM_FIELD)
#undef PARAM_FIELD
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
uint8_t Param_UpdateFromModbusEx(uint16_t *registers,
                                 uint16_t start_reg, uint16_t reg_count,
                                 uint8_t *exception_code);
void Param_Publish(uint16_t *registers);
uint8_t Param_Apply(uint16_t start_reg, uint16_t reg_count);
uint8_t Param_ApplyEx(uint16_t start_reg, uint16_t reg_count,
                      uint8_t *exception_code);

/* 从 Flash 恢复/保存 registry 权威值表；返回 0 表示成功。 */
uint8_t Param_LoadFromFlash(param_t *p);
uint8_t Param_SaveToFlash(void);

#endif
