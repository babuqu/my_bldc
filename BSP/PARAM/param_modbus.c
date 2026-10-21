// Modbus 保持寄存器与参数状态 g_param 之间的适配层。
#include "param.h"           // 参数结构体及参数读写、提交接口。
#include <stddef.h>           // 提供 NULL 定义。
#include <stdint.h>           // 提供 uint16_t、int32_t 等定宽整数类型。
#include "modbus.h"          // 提供保持寄存器数组和寄存器数量定义。
#include "param_registry.h"
#include "param_modbus_list.h"  // 提供 KP、KI、KD 对应的参数 ID。

/* KP、KI、KD 在 Modbus 寄存器中按 0.001 的分辨率传输。 */
#define PARAM_GAIN_SCALE 10000.0 // 写入时除以该值还原参数，读取时乘以该值编码。

// Modbus 单个 16 位寄存器的有符号性定义。
typedef enum
{
    PARAM_MODBUS_U16, // 无符号 16 位整数，范围为 0 到 65535。
    PARAM_MODBUS_S16  // 有符号 16 位补码整数，范围为 -32768 到 32767。
} param_modbus_type_t; // 映射项使用的线上数据类型。

// 描述一个 Modbus 保持寄存器对应的参数。
typedef struct
{
    uint16_t reg;       // 保持寄存器的零基偏移。
    uint16_t param_id;  // 参数注册表中的稳定参数 ID。
    double scale;       // Modbus 数值与参数实际值之间的缩放倍数。
    param_modbus_type_t type; // 该寄存器在线上的 U16 或 S16 编码类型。
} param_modbus_map_t;   // Modbus 参数映射项类型。

/* 只把配置参数映射到 g_param。 */
#define PARAM_MODBUS_ENTRY(reg_, id_, scale_, type_) {(reg_), (id_), (scale_), (type_)},
static const param_modbus_map_t g_modbus_map[] =
{
    PARAM_MODBUS_LIST(PARAM_MODBUS_ENTRY)
};
#undef PARAM_MODBUS_ENTRY
#define PARAM_MODBUS_MAP_COUNT (sizeof(g_modbus_map) / sizeof(g_modbus_map[0])) // 映射表中的参数数量。

/* 按映射项指定的 U16/S16 类型，把寄存器字解码为 int32_t。 */
static int32_t ParamModbus_Decode(uint16_t word, param_modbus_type_t type) // word 是原始寄存器值，type 决定符号解释。
{
    if (type == PARAM_MODBUS_U16) // U16 不需要补码转换。
    {
        return (int32_t)word; // 0 到 65535 都可由 int32_t 表示。
    }

    if (word <= 0x7FFFu) // 最高位为 0 时，数值为非负数。
    {
        return (int32_t)word; // 直接返回非负数。
    }
    return -((int32_t)0x10000u - (int32_t)word); // 最高位为 1 时按 16 位补码还原负数。
}

/* 把 g_param 中的参数值编码到对应的 Modbus 保持寄存器。 */
static void ParamModbus_Encode(const param_t *state, uint16_t *registers) // state 是参数源，registers 是寄存器目标。
{
    uint16_t i; // 映射表遍历索引。

    if (state == NULL || registers == NULL) // 检查输入参数和寄存器缓冲区是否有效。
    {
        return; // 任一指针为空时不进行编码。
    }

    for (i = 0u; i < PARAM_MODBUS_MAP_COUNT; ++i) // 遍历 KP、KI、KD 三项映射。
    {
        double value; // 保存从参数结构中读取的实际值。
        int32_t raw;  // 保存换算为寄存器精度后的整数值。

        if (!Param_GetValue(state, g_modbus_map[i].param_id, &value)) // 按参数 ID 从 state 读取参数值。
        {
            continue; // 读取失败时跳过该项，不影响其他寄存器。
        }

        value *= g_modbus_map[i].scale; // 根据映射缩放因子转换成寄存器整数值。
        raw = (int32_t)(value + ((value >= 0.0) ? 0.5 : -0.5)); // 对正负数分别进行四舍五入。

        if (g_modbus_map[i].type == PARAM_MODBUS_U16) // 根据映射类型选择无符号 16 位范围。
        {
            if (raw > 65535) raw = 65535; // U16 最大值为 65535。
            if (raw < 0) raw = 0; // U16 不表示负数，负值限制为 0。
            registers[g_modbus_map[i].reg] = (uint16_t)raw; // 按 U16 原值写入寄存器。
        }
        else // S16 映射使用 16 位补码表示有符号数。
        {
            if (raw > 32767) raw = 32767; // S16 最大值为 32767。
            if (raw < -32768) raw = -32768; // S16 最小值为 -32768。
            registers[g_modbus_map[i].reg] = (uint16_t)(int16_t)raw; // 将有符号值按补码写入寄存器。
        }
    }
}

/* 将本次 Modbus 写入的映射寄存器转换为参数值，并提交到 g_param。 */
uint8_t Param_UpdateFromModbusEx(uint16_t *registers, // Modbus 保持寄存器数据数组。
                                 uint16_t start_reg, // 本次写入的起始寄存器偏移。
                                 uint16_t reg_count, // 本次写入覆盖的寄存器数量。
                                 uint8_t *exception_code) // 可选的 Modbus 异常码输出。
{
    param_t candidate; // 暂存待校验、待提交的参数副本。
    uint32_t request_end; // 请求范围末端地址，采用 32 位避免加法溢出。
    uint8_t config_changed = 0u; // 标记本次请求是否包含已映射的参数。
    uint16_t i; // 映射表遍历索引。

    if (exception_code != NULL)
    {
        *exception_code = 0x04u; // 默认按设备内部故障处理，例如提交或 Flash 保存失败。
    }
    if (registers == NULL || reg_count == 0u) // 检查输入数组和写入数量。
    {
        if (exception_code != NULL) *exception_code = 0x03u;
        return 0u; // 输入无效，返回失败。
    }

    request_end = (uint32_t)start_reg + reg_count; // 计算请求覆盖区间的末端（不包含末端）。
    if (request_end > HOLDING_REG_NUM) // 确认请求区间没有超过保持寄存器数组。
    {
        ParamModbus_Encode(&g_param, registers); // 越界时用当前有效参数刷新已映射寄存器。
        if (exception_code != NULL) *exception_code = 0x02u;
        return 0u; // 拒绝越界请求。
    }

    candidate = g_param; // 复制当前参数，先在副本上处理整次请求。
    for (i = 0u; i < PARAM_MODBUS_MAP_COUNT; ++i) // 检查每个参数寄存器是否落在本次写入区间。
    {
        const param_modbus_map_t *map = &g_modbus_map[i]; // 取得当前寄存器映射项。
        int32_t raw; // 按映射项类型解码得到的寄存器整数值。
        double value; // 换算后的参数实际值。

        if (map->reg < start_reg || (uint32_t)map->reg >= request_end) // 判断本次请求是否包含当前映射寄存器。
        {
            continue; // 未写入的参数保持 candidate 中原值。
        }

        raw = ParamModbus_Decode(registers[map->reg], map->type); // 按映射项声明的 U16 或 S16 类型解码。
        value = (double)raw / map->scale; // 按映射精度还原为参数实际值。
        if (!Param_SetValue(&candidate, map->param_id, value)) // 写入候选参数并由参数注册表校验范围。
        {
            ParamModbus_Encode(&g_param, registers); // 任一参数无效时恢复当前有效值到寄存器镜像。
            if (exception_code != NULL) *exception_code = 0x03u;
            return 0u; // 拒绝整次请求，不提交部分参数。
        }

        config_changed = 1u; // 记录候选参数中至少有一项需要更新。
    }

    if (config_changed && !Param_Commit(&candidate)) // 有映射参数变化时统一校验并提交整个候选结构。
    {
        ParamModbus_Encode(&g_param, registers); // 提交失败时用原有 g_param 恢复寄存器镜像。
        return 0u; // 报告参数提交失败。
    }

    ParamModbus_Encode(&g_param, registers); // 成功后用最新 g_param 刷新 KP、KI、KD 寄存器。
    if (exception_code != NULL) *exception_code = 0u;
    return 1u; // 本次请求处理成功。
}

uint8_t Param_UpdateFromModbus(uint16_t *registers,
                               uint16_t start_reg, uint16_t reg_count)
{
    return Param_UpdateFromModbusEx(registers, start_reg, reg_count, NULL);
}

/* 读取保持寄存器前，将 g_param 当前值发布到寄存器镜像。 */
void Param_Publish(uint16_t *registers) // registers 指向 Modbus 保持寄存器数组。
{
    ParamModbus_Encode(&g_param, registers); // 把 KP、KI、KD 当前值编码到寄存器 0、1、2。
}

/* Modbus 写寄存器后的统一入口，处理指定的寄存器范围。 */
uint8_t Param_Apply(uint16_t start_reg, uint16_t reg_count) // start_reg 和 reg_count 描述本次写入范围。
{
    return Param_UpdateFromModbus(HoldingReg, start_reg, reg_count); // 从全局保持寄存器读取数据并更新参数。
}

uint8_t Param_ApplyEx(uint16_t start_reg, uint16_t reg_count,
                      uint8_t *exception_code)
{
    return Param_UpdateFromModbusEx(HoldingReg, start_reg, reg_count,
                                    exception_code);
}
