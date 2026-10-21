#include "param.h"
#include "param_registry.h"
#include <stdint.h>
#include <string.h>
#include "pid.h"

extern PID_TypeDef g_speed_pid;
extern uint8_t ParamStorage_Load(void);
extern uint8_t ParamStorage_Save(void);

/* 当前已经确认并供系统使用的参数状态。 */
param_t g_param;

/*
 * 根据描述表中的字段偏移取得 param_t 成员地址。
 * 参数 ID 与结构体字段的对应关系由统一参数表生成，这里不再单独维护映射。
 * 这个函数就是个"地址计算器"：给我结构体和它的描述信息，
 * 我算出那个成员住在内存的哪个位置，返回给你。
 */
static uint8_t *Param_StateField(param_t *state, const param_desc_t *desc)
{
    if (state == NULL || desc == NULL || desc->state_offset == PARAM_NO_STATE_OFFSET)
    {
        return NULL;
    }
    return (uint8_t *)state + desc->state_offset;
}

/*
 * 将 param_t 中的全部参数同步到 registry，后续可以存到flash。字段类型决定读取方式，
 * 参数权限、类型和范围由 registry 统一校验。中途失败时可能已有部分字段
 * 更新，因此由调用者负责恢复此前状态。
 */
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

/*
 * 将 registry 当前值复制到 param_t。先清零整个结构体，再按参数描述表
 * 填入已登记的字段；未登记或未映射的字段保持为零。
 */
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

/*
 * 检查状态指针，并确保统一参数描述表已初始化。
 * 参数表初始化。
 * 数值的类型和范围校验由 Param_SetValue / Param_RegistrySync 执行。
 */
uint8_t Param_Sanitize(param_t *state)
{
    if (state == NULL || ParamRegistry_Init() != 0)
    {
        return 1u;
    }
    return 0u;
}
/* 按 ID 从给定参数状态读取数值。 */
uint8_t Param_GetValue(const param_t *state, uint16_t id, double *value)
{
    return ParamRegistry_GetStateValue(state, id, value);
}
/* 按 ID 修改候选参数状态；参数值须通过 registry 的权限和范围校验。 */
uint8_t Param_SetValue(param_t *state, uint16_t id, double value)
{
    return ParamRegistry_SetStateValue(state, id, value);
}

/*
 * 提交失败后的恢复策略：优先恢复提交前的参数；若旧状态无法同步，
 * 则重置 registry，并让运行状态和 PID 回到参数表默认值，避免留下部分更新。
 */
static void Param_RestoreAfterCommitFailure(const param_t *previous)
{
    param_t defaults;
    param_t restore;

    if (previous != NULL)
    {
        restore = *previous;
        if (Param_RegistrySync(&restore))
        {
            return;
        }
    }

    ParamRegistry_Reset();
    if (ParamRegistry_Init() == 0 && Param_RegistryRead(&defaults))
    {
        g_param = defaults;
        Param_To_System(&g_param);
    }
}
uint8_t Param_Commit(param_t *candidate)
{
    param_t accepted;
    param_t previous;
    uint8_t persistent_changed = 0u;
    uint16_t i;

    /*
     * 提交流程：保存候选副本 -> 检查持久化字段是否变化 -> 准备存储 ->
     * 同步 registry -> 按需写 Flash -> 更新正式状态并发布。
     * 任一步失败都不把 candidate 接受为新的正式运行状态。
     */
    if (candidate == NULL)
    {
        return 0u;
    }
    accepted = *candidate;
    previous = g_param;
    /* 先使用局部副本处理，失败时不让调用者的 candidate 留下部分修改。 */
    /* 值非法或参数表不可用时拒绝提交，不做静默修正。 */
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
    /*
     * 仅持久化参数变化时才写 Flash，运行时参数变化不触发保存。
     * 先初始化存储再同步 accepted：首次 Save 可能先加载旧快照；如果先同步，
     * 旧值可能覆盖本次提交的新值。
     */
    if (persistent_changed && ParamStorage_Load() != 0u)
    {
        return 0u;
    }
    if (!Param_RegistrySync(&accepted))
    {
        /* 同步可能只完成了前几项，失败时恢复完整的旧状态。 */
        Param_RestoreAfterCommitFailure(&previous);
        return 0u;
    }
    if (persistent_changed && !Param_SaveToFlash())
    {
        /* Flash 保存失败时不接受本次提交，并恢复 registry 中的旧值。 */
        Param_RestoreAfterCommitFailure(&previous);
        return 0u;
    }
    g_param = accepted;
    *candidate = accepted;
    /* registry 与必要的 Flash 保存都成功后，才更新正式状态并发布到系统。 */
    Param_To_System(&g_param);
    return 1u;
}

/* 从统一参数表的默认值构造状态，并同步默认值到 registry。 */
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
    (void)Param_RegistrySync(&g_param);
}

/*
 * 启动初始化：清理 registry RAM 状态，尝试从 Flash 恢复参数，
 * 再发布最终状态。没有有效快照或数据不兼容时使用参数表默认值。
 */
uint8_t Param_Init_(void)
{
    param_t loaded;
    uint8_t status;

    /* App 入口显式清理 registry RAM 缓存，兼容 bootloader 直接跳转。 */
    ParamRegistry_Reset();
    if (ParamRegistry_Init() != 0)
    {
        /* registry 初始化失败时仍发布参数表默认值，保证运行参数有确定状态。 */
        Param_InitDefault();
        Param_To_System(&g_param);
        return 1u;
    }
    status = Param_LoadFromFlash(&loaded);
    if (status != 0u || Param_Sanitize(&loaded) != 0u)
    {
        Param_InitDefault(); /* Flash 加载失败或数据无效时回退到默认值。 */
        status = 1u;
    }
    else
    {
        g_param = loaded;
        (void)Param_RegistrySync(&g_param);
    }
    Param_To_System(&g_param);
    return status;
}

/* 从 Flash 恢复 registry，再生成供控制代码使用的 param_t 状态视图。 */
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

/*
 * 按 PARAM_LIST 中每个参数的发布配置，将状态一次性写入运行模块。
 * 没有配置发布语句的参数会展开为空操作，不需要在本函数另加分支。
 */
void Param_To_System(param_t *state)
{
    if (state == NULL)
    {
        return;
    }
#define PARAM_PUBLISH_FIELD(name_, id_, type_, access_, min_, max_, def_, ctype_, field_, publish_) publish_
    PARAM_LIST(PARAM_PUBLISH_FIELD)
#undef PARAM_PUBLISH_FIELD
}
