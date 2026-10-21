#ifndef PARAM_REGISTRY_H
#define PARAM_REGISTRY_H

#include "param.h"
#include "param_list.h"
#include <stdint.h>

/* 参数值编码类型；当前每种类型均使用 4 字节。 */
#define PARAM_TYPE_INT32   0u
#define PARAM_TYPE_FLOAT   1u
#define PARAM_TYPE_UINT32  2u

/* 参数属性可按位组合：读、写权限与 Flash 持久化标记相互独立。 */
#define PARAM_ACCESS_READ    0x01u
#define PARAM_ACCESS_WRITE   0x02u
#define PARAM_ACCESS_PERSIST 0x04u

#define PARAM_VALUE_MAX_LENGTH 4u
#define PARAM_NO_STATE_OFFSET  0xFFFFu

/* 参数 ID 固定不变，不随描述表顺序调整。 */
#define PARAM_DECLARE_ID(name_, id_, type_, access_, min_, max_, def_, ctype_, field_, publish_) name_ = (id_),
enum
{
    PARAM_LIST(PARAM_DECLARE_ID)
};
#undef PARAM_DECLARE_ID

/* 每项定义同时描述参数属性和 param_t 中对应字段的位置。 */
typedef struct
{
    uint16_t id;           /* 稳定参数 ID。 */
    uint8_t type;          /* PARAM_TYPE_* 数据类型。 */
    uint8_t length;        /* 参数值长度，当前固定为 4 字节。 */
    uint8_t access;        /* PARAM_ACCESS_* 权限组合。 */
    double min_value;      /* 最小工程值。 */
    double max_value;      /* 最大工程值。 */
    double default_value;  /* 默认工程值。 */
    uint16_t state_offset; /* 对应 param_t 字段的偏移。 */
} param_desc_t;

/* 参数值记录表的通用表头格式。 */
typedef struct
{
    uint32_t magic;   /* 表标识。 */
    uint16_t version; /* 格式版本。 */
    uint16_t count;   /* 记录数量。 */
    uint32_t crc32;   /* 整表校验值。 */
} value_table_header_t;

/* 单个参数值的运行时记录；字节数据按小端序保存。 */
typedef struct
{
    uint16_t id;                         /* 参数 ID。 */
    uint8_t type;                        /* 参数数据类型。 */
    uint8_t length;                      /* 有效数据长度。 */
    uint8_t data[PARAM_VALUE_MAX_LENGTH]; /* 参数原始数据。 */
} param_value_t;

/* 校验描述表，并用各项默认值初始化当前参数值。 */
int ParamRegistry_Init(void);

/* 清除 registry 的 RAM 缓存状态。 */
void ParamRegistry_Reset(void);

/* 按稳定 ID 查找描述项；失败时返回 NULL。 */
const param_desc_t *ParamRegistry_Find(uint16_t id);

/* 返回描述表中的参数数量。 */
uint16_t ParamRegistry_Count(void);

/* 按表下标获取描述项；失败时返回 NULL。 */
const param_desc_t *ParamRegistry_GetAt(uint16_t index);

/* 从运行状态字段读取工程值；成功返回非零。 */
uint8_t ParamRegistry_GetStateValue(const param_t *state,
                                    uint16_t id, double *value);

/* 将工程值写入运行状态字段；范围或类型不合法时失败。 */
uint8_t ParamRegistry_SetStateValue(param_t *state,
                                    uint16_t id, double value);

/* 校验工程值并更新 registry 中该参数的当前值。 */
uint8_t ParamRegistry_SetCurrentValue(uint16_t id, double value);

/* 按表下标获取参数 ID；成功返回非零。 */
uint8_t ParamRegistry_GetIdAt(uint16_t index, uint16_t *id);

/* 获取描述项的 CRC-16/Modbus；无效 ID 时返回 0。 */
uint16_t ParamRegistry_GetCrc16(uint16_t id);

/*
 * 解析并应用记录：[ID:LE16][TYPE:U8][LENGTH:U8][VALUE:LE]。
 * 返回 0 表示成功，非零表示格式、ID、权限、类型或范围错误。
 */
int ParamValue_ParseRecord(const uint8_t *record, uint16_t record_length);

/* 校验记录但不修改当前参数值。 */
int ParamValue_ValidateRecord(const uint8_t *record, uint16_t record_length);

/* 将指定 ID 的当前值编码为记录；成功返回 0。 */
int ParamValue_BuildRecord(uint16_t id, uint8_t *record,
                           uint16_t capacity, uint16_t *written);

#endif /* PARAM_REGISTRY_H */
