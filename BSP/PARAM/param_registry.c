/*
 * 参数描述表与当前值解析模块。
 * 描述表保存可读数值；只有值记录编解码时才转换为32位数据表示。
 * 本文件不负责Modbus地址、PID同步或Flash擦写。
 */
#include "param_registry.h"
#include "param.h"
#include <math.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* 用宏减少表项重复字段，同时让范围和默认值保持可读。 */
#define PARAM_DESC_STATE(id_, type_, access_, min_, max_, def_, field_) \
    { (id_), (type_), 4u, (access_), \
      (double)(min_), (double)(max_), (double)(def_), \
      (uint16_t)offsetof(param_t, field_) }

/*
 * 参数描述表：改参数范围或默认值时直接编辑普通数值。
 * FLOAT的min/max/default例如直接写0.0、6.55、0.0，不写IEEE位模式。
 */
/* 描述表保存ID、类型、范围和状态字段偏移；当前值按相同下标单独保存。 */
static const param_desc_t g_param_desc[] =
{
    PARAM_DESC_STATE(PARAM_ID_KP_SPEED, PARAM_TYPE_FLOAT,
                     PARAM_ACCESS_READ | PARAM_ACCESS_WRITE | PARAM_ACCESS_PERSIST,
                     0.0, 6.55, 0.5000f, kp),
    PARAM_DESC_STATE(PARAM_ID_KI_SPEED, PARAM_TYPE_FLOAT,
                     PARAM_ACCESS_READ | PARAM_ACCESS_WRITE | PARAM_ACCESS_PERSIST,
                     0.0, 6.55, 0.25f, ki),
    PARAM_DESC_STATE(PARAM_ID_KD_SPEED, PARAM_TYPE_FLOAT,
                     PARAM_ACCESS_READ | PARAM_ACCESS_WRITE | PARAM_ACCESS_PERSIST,
                     0.0, 6.55, 0.150f, kd),
    PARAM_DESC_STATE(Target_SPEED, PARAM_TYPE_INT32,
                     PARAM_ACCESS_READ | PARAM_ACCESS_WRITE,
                     -32760, 32760, 0, target_speed)
    //在这里添加新的参数ID的描述···
};

#define PARAM_DESC_COUNT (sizeof(g_param_desc) / sizeof(g_param_desc[0])) /* 描述项数量。 */
static param_value_t g_param_values[PARAM_DESC_COUNT]; /* 独立保存各参数当前值。 */
static uint16_t g_param_desc_crc16[PARAM_DESC_COUNT]; /* 缓存描述CRC，不修改只读描述表。 */
static uint8_t g_initialized; /* 标记默认值和描述CRC是否已初始化。 */

/* 显式清理 registry 的 RAM 状态，避免依赖启动前 RAM 内容为零。 */
void ParamRegistry_Reset(void)
{
    g_initialized = 0u;
    memset(g_param_values, 0, sizeof(g_param_values));
    memset(g_param_desc_crc16, 0, sizeof(g_param_desc_crc16));
}

/* ==================== 数值编码与CRC工具函数 ==================== */
/* 从字节数组读取little-endian 16位整数。 */
static uint16_t Param_ReadU16LE(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

/* 从字节数组读取little-endian 32位整数或浮点位模式。 */
static uint32_t Param_ReadU32LE(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

/* 将32位值拆为little-endian字节写入缓冲区。 */
static void Param_WriteU32LE(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

/* 将uint32原始位模式按位解释为int32，不进行数值转换。 */
static int32_t Param_BitsToInt32(uint32_t bits)
{
    int32_t value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

/* 将uint32原始位模式按位解释为IEEE-754 float。 */
static float Param_BitsToFloat(uint32_t bits)
{
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

/* 把表中的可读数值转换成线上/存储记录需要的32位表示。 */
/* 仅在序列化边界把表中的可读数值转换为32位存储位模式。 */
static uint32_t Param_NumberToBits(uint8_t type, double number)
{
    uint32_t bits;
    int32_t signed_value;
    uint32_t unsigned_value;
    float float_value;

    if (type == PARAM_TYPE_INT32)
    {
        signed_value = (int32_t)number;
        memcpy(&bits, &signed_value, sizeof(bits));
        return bits;
    }
    if (type == PARAM_TYPE_UINT32)
    {
        unsigned_value = (uint32_t)number;
        memcpy(&bits, &unsigned_value, sizeof(bits));
        return bits;
    }

    float_value = (float)number;
    memcpy(&bits, &float_value, sizeof(bits));
    return bits;
}

/* CRC-16/Modbus按0xFFFF初值和0xA001反转多项式计算。 */
static uint16_t Param_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    uint8_t bit;

    for (i = 0u; i < length; ++i)
    {
        crc ^= data[i];
        for (bit = 0u; bit < 8u; ++bit)
        {
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u)
                             : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

/* 将描述项按固定字段顺序序列化，再计算CRC，避免依赖结构体填充。 */
/* 按固定字段顺序编码后计算CRC，避免结构体填充字节影响结果。 */
static uint16_t Param_DescriptorCrc(const param_desc_t *desc)
{
    uint8_t bytes[19];
    uint32_t min_bits;
    uint32_t max_bits;
    uint32_t default_bits;

    min_bits = Param_NumberToBits(desc->type, desc->min_value);
    max_bits = Param_NumberToBits(desc->type, desc->max_value);
    default_bits = Param_NumberToBits(desc->type, desc->default_value);

    bytes[0] = (uint8_t)desc->id;
    bytes[1] = (uint8_t)(desc->id >> 8);
    bytes[2] = desc->type;
    bytes[3] = desc->length;
    bytes[4] = desc->access;
    Param_WriteU32LE(&bytes[5], min_bits);
    Param_WriteU32LE(&bytes[9], max_bits);
    Param_WriteU32LE(&bytes[13], default_bits);
    bytes[17] = (uint8_t)desc->state_offset;
    bytes[18] = (uint8_t)(desc->state_offset >> 8);
    return Param_Crc16(bytes, sizeof(bytes));
}

/* ==================== 描述查找与数据校验 ==================== */
/* 查找描述表项下标；未找到时返回-1。 */
static int Param_FindIndex(uint16_t id)
{
    size_t i;
    for (i = 0u; i < PARAM_DESC_COUNT; ++i)
    {
        if (g_param_desc[i].id == id)
        {
            return (int)i;
        }
    }
    return -1;
}

/* 按参数编码精度统一判断工程值是否处于定义范围内。 */
static int Param_ValueInRange(const param_desc_t *desc, double value);

/* 将32位值解码为工程数值，并按描述项类型检查范围。 */
static int Param_ValidateRaw(const param_desc_t *desc, uint32_t bits)
{
    double value;

    if (desc == NULL || desc->length != 4u || desc->min_value > desc->max_value)
    {
        return 0;
    }

    if (desc->type == PARAM_TYPE_INT32)
    {
        value = (double)Param_BitsToInt32(bits);
    }
    else if (desc->type == PARAM_TYPE_UINT32)
    {
        value = (double)bits;
    }
    else if (desc->type == PARAM_TYPE_FLOAT)
    {
        float float_value = Param_BitsToFloat(bits);
        if (!isfinite((double)float_value))
        {
            return 0;
        }
        value = (double)float_value;
    }
    else
    {
        return 0;
    }

    return Param_ValueInRange(desc, value);
}

/* 浮点参数按实际存储精度比较边界，避免 6.55f 被误判为超过 6.55。 */
static int Param_ValueInRange(const param_desc_t *desc, double value)
{
    if (desc == NULL || !isfinite(value))
    {
        return 0;
    }
    if (desc->type == PARAM_TYPE_FLOAT)
    {
        float float_value = (float)value;
        return (isfinite((double)float_value) &&
                float_value >= (float)desc->min_value &&
                float_value <= (float)desc->max_value);
    }
    return (value >= desc->min_value && value <= desc->max_value);
}

/* 根据参数编码类型返回运行状态字段应占用的字节数。 */
static uint8_t Param_StateTypeSize(uint8_t type)
{
    if (type == PARAM_TYPE_FLOAT || type == PARAM_TYPE_INT32 ||
        type == PARAM_TYPE_UINT32)
    {
        return 4u;
    }
    return 0u;
}

/* ==================== 初始化与描述表查询接口 ==================== */
/* 首次初始化时校验描述项，并用每项默认值建立当前值表。 */
/* 首次调用时校验描述表并装入默认值；后续调用不覆盖运行中的参数值。 */
int ParamRegistry_Init(void)
{
    size_t i;
    size_t j;
    uint32_t default_bits;

    if (g_initialized != 0u)//如果参数表已经加载，返回0
    {
        return 0;
    }
    //参数表还未加载
    for (i = 0u; i < PARAM_DESC_COUNT; ++i)
    {
        const param_desc_t *current_def = &g_param_desc[i];//提取第i个参数的描述
        //对参数描述表第i个参数值进行合理性检查，如果检查失败，认为参数描述表异常
        if (current_def->length != 4u ||
            (current_def->access &
             (uint8_t)~(PARAM_ACCESS_READ | PARAM_ACCESS_WRITE |
                        PARAM_ACCESS_PERSIST)) != 0u ||
            (current_def->access &
             (PARAM_ACCESS_READ | PARAM_ACCESS_WRITE)) == 0u ||
            !isfinite(current_def->min_value) || !isfinite(current_def->max_value) ||
            !isfinite(current_def->default_value) ||
            current_def->min_value > current_def->max_value ||
            current_def->default_value < current_def->min_value ||
            current_def->default_value > current_def->max_value ||
            current_def->state_offset == PARAM_NO_STATE_OFFSET ||
            Param_StateTypeSize(current_def->type) == 0u ||
            ((uint32_t)current_def->state_offset +
             Param_StateTypeSize(current_def->type)) > sizeof(param_t))
        {
            return 1;
        }
        //对描述表第i个参数值数据类型进行合理性检查，如果检查失败，认为参数描述表异常
        if (current_def->type != PARAM_TYPE_INT32 && current_def->type != PARAM_TYPE_UINT32 &&
            current_def->type != PARAM_TYPE_FLOAT)
        {
            return 1;
        }
        if (current_def->type == PARAM_TYPE_INT32 &&
            (current_def->min_value < (double)INT32_MIN ||
             current_def->max_value > (double)INT32_MAX ||
             floor(current_def->min_value) != current_def->min_value ||
             floor(current_def->max_value) != current_def->max_value ||
             floor(current_def->default_value) != current_def->default_value))
        {
            return 1;
        }
        if (current_def->type == PARAM_TYPE_UINT32 &&
            (current_def->min_value < 0.0 ||
             current_def->max_value > (double)UINT32_MAX ||
             floor(current_def->min_value) != current_def->min_value ||
             floor(current_def->max_value) != current_def->max_value ||
             floor(current_def->default_value) != current_def->default_value))
        {
            return 1;
        }
        if (current_def->type == PARAM_TYPE_FLOAT &&
            (!isfinite((double)(float)current_def->min_value) ||
             !isfinite((double)(float)current_def->max_value)))
        {
            return 1;
        }
        //如果存在重复的ID号，认为数据描述表异常
        for (j = 0u; j < i; ++j)
        {
            if (g_param_desc[j].id == current_def->id)
            {
                return 1;
            }
            if ((current_def->state_offset <
                 (uint32_t)g_param_desc[j].state_offset +
                 Param_StateTypeSize(g_param_desc[j].type)) &&
                (g_param_desc[j].state_offset <
                 (uint32_t)current_def->state_offset +
                 Param_StateTypeSize(current_def->type)))
            {
                return 1;
            }
        }
        //参数描述校验码
        g_param_desc_crc16[i] = Param_DescriptorCrc(current_def);
        //从参数描述默认值转换为32位无符号数
        default_bits = Param_NumberToBits(current_def->type, current_def->default_value);
        //对当前参数描述表中默认值范围进行检查
        if (!Param_ValidateRaw(current_def, default_bits))
        {
            return 1;
        }
        //如果上面检查都没有问题
        //将参数表中的默认参数赋值给参数缓冲区，完成一次参数默认值赋值
        g_param_values[i].id = current_def->id;
        g_param_values[i].type = current_def->type;
        g_param_values[i].length = current_def->length;
        Param_WriteU32LE(g_param_values[i].data, default_bits);
    }
    //如果上面的赋值都成功，参数初始值完成。
    g_initialized = 1u;
    return 0;
}

/* 返回指定ID的只读描述项指针；没有该ID或初始化失败时返回NULL。 */
const param_desc_t *ParamRegistry_Find(uint16_t id)
{
    int index;
    if (ParamRegistry_Init() != 0)
    {
        return NULL;
    }
    index = Param_FindIndex(id);
    return (index < 0) ? NULL : &g_param_desc[index];
}

/* 返回静态参数描述表的条目数量。 */
uint16_t ParamRegistry_Count(void)
{
    return (uint16_t)PARAM_DESC_COUNT;
}

/* 按统一定义表的固定顺序返回一项，用于存储层遍历。 */
const param_desc_t *ParamRegistry_GetAt(uint16_t index)
{
    if (ParamRegistry_Init() != 0 || index >= PARAM_DESC_COUNT)
    {
        return NULL;
    }
    return &g_param_desc[index];
}

/* 由定义项中的偏移访问状态字段，避免维护第二份 ID 到字段映射表。 */
uint8_t ParamRegistry_GetStateValue(const param_t *state,
                                    uint16_t id, double *value)
{
    const param_desc_t *definition = ParamRegistry_Find(id);
    const uint8_t *field;
    double state_value;

    if (state == NULL || value == NULL || definition == NULL ||
        definition->state_offset == PARAM_NO_STATE_OFFSET ||
        ((uint32_t)definition->state_offset +
         Param_StateTypeSize(definition->type)) > sizeof(param_t) ||
        (definition->access & PARAM_ACCESS_READ) == 0u)
    {
        return 0u;
    }
    field = (const uint8_t *)state + definition->state_offset;
    if (definition->type == PARAM_TYPE_FLOAT)
    {
        float typed_value;
        memcpy(&typed_value, field, sizeof(typed_value));
        state_value = (double)typed_value;
    }
    else if (definition->type == PARAM_TYPE_INT32)
    {
        int32_t typed_value;
        memcpy(&typed_value, field, sizeof(typed_value));
        state_value = (double)typed_value;
    }
    else if (definition->type == PARAM_TYPE_UINT32)
    {
        uint32_t typed_value;
        memcpy(&typed_value, field, sizeof(typed_value));
        state_value = (double)typed_value;
    }
    else
    {
        return 0u;
    }
    if (!Param_ValueInRange(definition, state_value))
    {
        return 0u;
    }
    *value = state_value;
    return 1u;
}

/* 写入状态视图前复用统一描述项进行有限值和范围校验。 */
uint8_t ParamRegistry_SetStateValue(param_t *state,
                                    uint16_t id, double value)
{
    const param_desc_t *desc = ParamRegistry_Find(id);
    uint8_t *field;

    if (state == NULL || desc == NULL ||
        desc->state_offset == PARAM_NO_STATE_OFFSET ||
        ((uint32_t)desc->state_offset + Param_StateTypeSize(desc->type)) > sizeof(param_t) ||
        (desc->access & PARAM_ACCESS_WRITE) == 0u ||
        !Param_ValueInRange(desc, value))
    {
        return 0u;
    }
    field = (uint8_t *)state + desc->state_offset;
    if (desc->type == PARAM_TYPE_FLOAT)
    {
        float typed_value = (float)value;
        if (!isfinite((double)typed_value) ||
            !Param_ValueInRange(desc, (double)typed_value))
        {
            return 0u;
        }
        memcpy(field, &typed_value, sizeof(typed_value));
    }
    else if (desc->type == PARAM_TYPE_INT32)
    {
        int32_t typed_value;
        if (value != (double)(int32_t)value)
        {
            return 0u;
        }
        typed_value = (int32_t)value;
        memcpy(field, &typed_value, sizeof(typed_value));
    }
    else if (desc->type == PARAM_TYPE_UINT32)
    {
        uint32_t typed_value;
        if (value < 0.0 || value != (double)(uint32_t)value)
        {
            return 0u;
        }
        typed_value = (uint32_t)value;
        memcpy(field, &typed_value, sizeof(typed_value));
    }
    else
    {
        return 0u;
    }
    return 1u;
}

/* 校验工程值后直接更新 registry 当前值，不经过 Flash 记录格式。 */
uint8_t ParamRegistry_SetCurrentValue(uint16_t id, double value)
{
    const param_desc_t *desc = ParamRegistry_Find(id);
    int index;
    uint32_t bits;

    if (desc == NULL || (desc->access & PARAM_ACCESS_WRITE) == 0u ||
        !Param_ValueInRange(desc, value))
    {
        return 0u;
    }

    if (desc->type == PARAM_TYPE_FLOAT)
    {
        float typed_value = (float)value;
        if (!isfinite((double)typed_value) ||
            !Param_ValueInRange(desc, (double)typed_value))
        {
            return 0u;
        }
        memcpy(&bits, &typed_value, sizeof(bits));
    }
    else if (desc->type == PARAM_TYPE_INT32)
    {
        int32_t typed_value;
        if (value != (double)(int32_t)value)
        {
            return 0u;
        }
        typed_value = (int32_t)value;
        memcpy(&bits, &typed_value, sizeof(bits));
    }
    else if (desc->type == PARAM_TYPE_UINT32)
    {
        uint32_t typed_value;
        if (value < 0.0 || value != (double)(uint32_t)value)
        {
            return 0u;
        }
        typed_value = (uint32_t)value;
        memcpy(&bits, &typed_value, sizeof(bits));
    }
    else
    {
        return 0u;
    }

    index = Param_FindIndex(id);
    if (index < 0)
    {
        return 0u;
    }
    g_param_values[index].id = desc->id;
    g_param_values[index].type = desc->type;
    g_param_values[index].length = desc->length;
    Param_WriteU32LE(g_param_values[index].data, bits);
    return 1u;
}

/* 按描述表下标读取稳定ID，供存储层遍历所有参数。 */
uint8_t ParamRegistry_GetIdAt(uint16_t index, uint16_t *id)
{
    if (id == NULL || ParamRegistry_Init() != 0 || index >= PARAM_DESC_COUNT)
    {
        return 0u;
    }
    *id = g_param_desc[index].id;
    return 1u;
}
/* 查询描述项CRC；描述表本身保持只读，CRC单独缓存。 */
uint16_t ParamRegistry_GetCrc16(uint16_t id)
{
    int index;
    if (ParamRegistry_Init() != 0)
    {
        return 0u;
    }
    index = Param_FindIndex(id);
    return (index < 0) ? 0u : g_param_desc_crc16[index];
}

/* ==================== 参数值记录读写接口 ==================== */
/*
 * 解析 [ID:LE16][TYPE:U8][LENGTH:U8][VALUE:LE] 记录。
 * 返回0表示成功；1为格式/初始化错误，2为未知ID，3为只读，
 * 4为类型或长度不匹配，5为值超出描述表范围。
 */
/* 只校验记录格式、权限、类型和范围，不修改注册表中的当前值。 */
static int ParamValue_ValidateRecordInternal(const uint8_t *record,
                                             uint16_t record_length,
                                             int *validated_index)
{
    uint16_t id;
    uint8_t type;
    uint8_t length;
    uint32_t bits;
    int index;
    const param_desc_t *desc;
    //数据合法性检查
    if (record == NULL || record_length < 4u || ParamRegistry_Init() != 0)
    {
        return 1;
    }

    id = Param_ReadU16LE(record);
    type = record[2];
    length = record[3];
    //如果长度超了，返回1
    if (length > PARAM_VALUE_MAX_LENGTH || record_length != (uint16_t)(4u + length))
    {
        return 1;
    }
    //在参数描述表中查找当前ID
    index = Param_FindIndex(id);
    //如果没找到，返回2
    if (index < 0)
    {
        return 2;
    }
    //找到参数描述
    desc = &g_param_desc[index];
    //如果数据是只读的，返回3
    if ((desc->access & PARAM_ACCESS_WRITE) == 0u)
    {
        return 3;
    }
    //如果长度和类型不匹配，返回4
    if (type != desc->type || length != desc->length)
    {
        return 4;
    }
     //如果数据超范围，返回5   
    bits = Param_ReadU32LE(&record[4]);
    if (!Param_ValidateRaw(desc, bits))
    {
        return 5;
    }
    if (validated_index != NULL)
    {
        *validated_index = index;
    }
    //如果所有条件都OK,返回0
    return 0;
}

/* 对外只校验记录，不修改注册表中的当前值。 */
int ParamValue_ValidateRecord(const uint8_t *record, uint16_t record_length)
{
    return ParamValue_ValidateRecordInternal(record, record_length, NULL);
}

/* 完整校验通过后，再一次性替换对应ID的当前值g_param_values。 */
int ParamValue_ParseRecord(const uint8_t *record, uint16_t record_length)
{
    //数据校验
    int index;
    int status = ParamValue_ValidateRecordInternal(record, record_length, &index);

    if (status != 0)//数据校验失败
    {
        return status;
    }

    //记录校验已确认ID、类型和长度与定义一致；只更新参数数据，不覆盖定义元信息
    g_param_values[index].id = g_param_desc[index].id;
    memcpy(g_param_values[index].data, &record[4], record[3]);
    return 0;
}

/* 将指定ID的当前值编码为记录；capacity不足或ID不存在时返回非零。 */
/* 将当前值进行编码，为 [ID:LE16][TYPE][LENGTH][VALUE:LE] 记录。 */
int ParamValue_BuildRecord(uint16_t id, uint8_t *record,
                           uint16_t capacity, uint16_t *written)
{
    int index;
    uint16_t required;
    const param_value_t *value;

    if (record == NULL || written == NULL || ParamRegistry_Init() != 0)
    {
        return 1;//校验失败
    }
    index = Param_FindIndex(id);//查找ID
    if (index < 0)
    {
        return 2;//没有当前ID
    }

    value = &g_param_values[index];//获取当前ID的值
    if (value->id != g_param_desc[index].id ||
        value->type != g_param_desc[index].type ||
        value->length != g_param_desc[index].length)
    {
        return 4;//当前值元信息与描述定义不一致
    }
    required = (uint16_t)(4u + value->length);
    if (capacity < required)
    {
        return 3;
    }

    record[0] = (uint8_t)id;
    record[1] = (uint8_t)(id >> 8);
    record[2] = value->type;
    record[3] = value->length;
    memcpy(&record[4], value->data, value->length);
    *written = required;
    return 0;
}
