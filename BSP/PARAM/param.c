#include "param.h"
#include "param_registry.h"
#include <stdint.h>
#include <string.h>
#include "pid.h"

#define PARAM_FLAG_DEFAULT 0xA5A5u

extern PID_TypeDef g_speed_pid;
extern uint8_t ParamStorage_Load(void);
extern uint8_t ParamStorage_Save(void);

param_t g_param;//全局缓存参数变量

/* 按统一定义表下标取字段地址；表项负责提供唯一的字段偏移。 */
static uint8_t *Param_StateField(param_t *state, const param_desc_t *desc)
{
    if (state == NULL || desc == NULL || desc->state_offset == PARAM_NO_STATE_OFFSET)
    {
        return NULL;
    }
    return (uint8_t *)state + desc->state_offset;
}

/* 将当前运行状态需要掉电存储的变量参数全部逐项同步到 registry；范围和类型由统一定义表校验。 */
static uint8_t Param_RegistrySync(param_t *state)
{
    uint16_t i;

    if (state == NULL || ParamRegistry_Init() != 0)
    {
        return 0u; /* 参数状态无效或定义表初始化失败。 */
    }

    for (i = 0u; i < ParamRegistry_Count(); ++i)
    {
        const param_desc_t *desc = ParamRegistry_GetAt(i);
        uint8_t *field = Param_StateField(state, desc);
        double value;

        if (field == NULL)
        {
            return 0u;
        }

        if (desc->type == PARAM_TYPE_FLOAT)
        {
            float typed_value;
            memcpy(&typed_value, field, sizeof(typed_value));
            value = (double)typed_value;
        }
        else if (desc->type == PARAM_TYPE_INT32)
        {
            int32_t typed_value;
            memcpy(&typed_value, field, sizeof(typed_value));
            value = (double)typed_value;
        }
        else if (desc->type == PARAM_TYPE_UINT32)
        {
            uint32_t typed_value;
            memcpy(&typed_value, field, sizeof(typed_value));
            value = (double)typed_value;
        }
        else
        {
            return 0u;
        }

        /* 由 registry 校验权限、类型和范围，并更新其当前值。 */
        if (!ParamRegistry_SetCurrentValue(desc->id, value))
        {
            return 0u;
        }
    }
    return 1u;
}

/* 从 registry 当前值生成运行状态结构体，未配置字段保持为零。 */
static uint8_t Param_RegistryRead(param_t *state)
{
    uint16_t i;
    uint8_t record[8];
    uint16_t written;

    if (state == NULL || ParamRegistry_Init() != 0)
    {
        return 0u;
    }
    memset(state, 0, sizeof(*state));
    for (i = 0u; i < ParamRegistry_Count(); ++i)
    {
        const param_desc_t *desc = ParamRegistry_GetAt(i);
        uint8_t *field = Param_StateField(state, desc);
        uint32_t bits;

        if (field == NULL ||
            ParamValue_BuildRecord(desc->id, record, sizeof(record), &written) != 0 ||
            written != sizeof(record))
        {
            return 0u;
        }
        bits = (uint32_t)record[4] | ((uint32_t)record[5] << 8) |
               ((uint32_t)record[6] << 16) | ((uint32_t)record[7] << 24);
        memcpy(field, &bits, sizeof(bits));
    }
    return 1u;
}

/* 检查参数状态指针和统一定义表初始化状态。 */
uint8_t Param_Sanitize(param_t *state)
{
    if (state == NULL || ParamRegistry_Init() != 0)
    {
        return 1u;
    }
    return 0u;
}
//从registry中获取指定的ID数据到Param
uint8_t Param_GetValue(const param_t *state, uint16_t id, double *value)
{
    return ParamRegistry_GetStateValue(state, id, value);
}
//从中Param写指定的ID数据到registry
uint8_t Param_SetValue(param_t *state, uint16_t id, double value)
{
    return ParamRegistry_SetStateValue(state, id, value);
}

/* 先校验并更新 registry，再持久化；成功后才发布运行状态和 PID。 */
uint8_t Param_Commit(param_t *candidate)
{
    param_t accepted;
    param_t previous;
    uint8_t persistent_changed = 0u;
    uint16_t i;

    if (candidate == NULL)
    {
        return 0u;
    }
    accepted = *candidate;
    previous = g_param;
    /* 若参数越界或上下限关系非法，拒绝本次提交，不静默改写后再报告成功。 */
    if (Param_Sanitize(&accepted) != 0u)
    {
        return 0u;
    }
    for (i = 0u; i < ParamRegistry_Count(); ++i)
    {
        const param_desc_t *desc = ParamRegistry_GetAt(i);
        const uint8_t *new_field;
        const uint8_t *old_field;

        if (desc == NULL)
        {
            return 0u;
        }
        if ((desc->access & PARAM_ACCESS_PERSIST) == 0u)
        {
            continue;
        }
        new_field = Param_StateField(&accepted, desc);
        old_field = Param_StateField(&previous, desc);
        if (new_field == NULL || old_field == NULL)
        {
            return 0u;
        }
        if (memcmp(new_field, old_field, desc->length) != 0)
        {
            persistent_changed = 1u;
            break;
        }
    }
    if (persistent_changed && ParamStorage_Load() != 0u)
    {
        return 0u;
    }
    if (!Param_RegistrySync(&accepted))
    {
        (void)Param_RegistrySync(&previous);
        return 0u;
    }
    if (persistent_changed && !Param_SaveToFlash())
    {
        (void)Param_RegistrySync(&previous);
        return 0u;
    }
    g_param = accepted;
    *candidate = accepted;
    Param_To_System(&g_param);
    return 1u;
}

/* 用统一定义表中的默认值初始化配置字段。 */
void Param_InitDefault(void)
{
    uint16_t i;
    memset(&g_param, 0, sizeof(g_param));
    for (i = 0u; i < ParamRegistry_Count(); ++i)
    {
        const param_desc_t *desc = ParamRegistry_GetAt(i);
        if (desc != NULL)
        {
            (void)ParamRegistry_SetStateValue(&g_param, desc->id, desc->default_value);
        }
    }
    g_param.flag = (float)PARAM_FLAG_DEFAULT;
    (void)Param_RegistrySync(&g_param);
}

/* 启动时恢复 Flash；没有有效快照则回到定义表默认值。 */
uint8_t Param_Init_(void)
{
    param_t loaded;
    uint8_t status;

    /* App 入口显式清理 registry RAM 缓存，兼容 bootloader 直接跳转。 */
    ParamRegistry_Reset();
    if (ParamRegistry_Init() != 0)
    {   //registry初始化失败，将使用参数描述表中的默认参数
        Param_InitDefault();//初始化全局参数
        Param_To_System(&g_param);//发布全局参数
        return 1u;
    }
    status = Param_LoadFromFlash(&loaded);//加载flash数据
    if (status != 0u || Param_Sanitize(&loaded) != 0u)
    {
        Param_InitDefault();//flash加载失败，还是用默认参数
        status = 1u;
    }
    else
    {
        g_param = loaded;//加载成功，就赋值
        (void)Param_RegistrySync(&g_param);//同时更新到Registry中
    }
    Param_To_System(&g_param);//发布全局参数
    return status;
}

/* 从 Flash 恢复 registry，再生成供控制代码使用的状态视图。 */
uint8_t Param_LoadFromFlash(param_t *state)
{
    if (ParamStorage_Load() != 0u || !Param_RegistryRead(state))
    {
        return 1u;
    }
    return 0u;
}

/* 将 registry 当前值保存为一份完整 Flash 快照。 */
uint8_t Param_SaveToFlash(void)
{
    return ParamStorage_Save();
}

/* 把参数运行视图一次性发布。 */
void Param_To_System(param_t *state)
{
    if (state == NULL)
    {
        return;
    }
    g_speed_pid.Proportion = state->kp;
    g_speed_pid.Integral = state->ki;
    g_speed_pid.Derivative = state->kd;
}
