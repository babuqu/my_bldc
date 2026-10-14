#include "param.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include "modbus.h"
#include "param_registry.h"

/* KP/KI/KD 在 Modbus 中按 0.001 的分辨率传输。 */
#define PARAM_GAIN_SCALE 1000.0

typedef enum
{
    PARAM_MODBUS_S16,
    PARAM_MODBUS_SCALED_S16,
    PARAM_MODBUS_SPEED_COMMAND
} param_modbus_encoding_t;

typedef struct
{
    uint16_t reg;
    uint16_t param_id;
    double scale;
    param_modbus_encoding_t encoding;
} param_modbus_map_t;

/* Modbus 地址只存在于适配表中；目标转速是命令字，不是持久化参数。 */
static const param_modbus_map_t g_modbus_map[] =
{
    {0u, PARAM_ID_KP_SPEED,  PARAM_GAIN_SCALE, PARAM_MODBUS_SCALED_S16},
    {1u, PARAM_ID_KI_SPEED,  PARAM_GAIN_SCALE, PARAM_MODBUS_SCALED_S16},
    {2u, PARAM_ID_KD_SPEED,  PARAM_GAIN_SCALE, PARAM_MODBUS_SCALED_S16},
    {3u, 0u,                  1.0,              PARAM_MODBUS_SPEED_COMMAND}
};
#define PARAM_MODBUS_MAP_COUNT (sizeof(g_modbus_map) / sizeof(g_modbus_map[0]))

/* 把 Modbus 16 位补码按有符号数解释，避免依赖实现定义的窄化转换。 */
static int16_t ParamModbus_DecodeS16(uint16_t word)
{
    if (word <= 0x7FFFu)
    {
        return (int16_t)word;
    }
    return (int16_t)(-((int32_t)0x10000u - (int32_t)word));
}

/* 将当前 PID 命令和持久化参数发布到 Holding Register 镜像。 */
static void ParamModbus_Encode(const param_t *state, uint16_t *registers)
{
    uint16_t i;

    if (state == NULL || registers == NULL)
    {
        return;
    }

    for (i = 0u; i < PARAM_MODBUS_MAP_COUNT; ++i)
    {
        double value;
        int32_t raw;

        if (g_modbus_map[i].encoding == PARAM_MODBUS_SPEED_COMMAND)
        {
            raw = Param_GetSpeedCommand();
        }
        else
        {
            if (!Param_GetValue(state, g_modbus_map[i].param_id, &value))
            {
                continue;
            }
            if (g_modbus_map[i].encoding == PARAM_MODBUS_SCALED_S16)
            {
                value *= g_modbus_map[i].scale;
            }
            raw = (int32_t)(value + ((value >= 0.0) ? 0.5 : -0.5));
        }

        if (raw > 32767) raw = 32767;
        if (raw < -32768) raw = -32768;
        registers[g_modbus_map[i].reg] = (uint16_t)(int16_t)raw;
    }
}

/* 只处理本次 Modbus 请求覆盖的寄存器，避免镜像中的旧值覆盖其他参数。 */
uint8_t Param_UpdateFromModbus(uint16_t *registers,
                               uint16_t start_reg, uint16_t reg_count)
{
    param_t candidate;
    uint8_t config_changed = 0u;
    uint8_t command_seen = 0u;
    int32_t speed_command = 0;
    uint32_t request_end;
    uint16_t i;

    if (registers == NULL || reg_count == 0u)
    {
        return 0u;
    }
    request_end = (uint32_t)start_reg + reg_count;
    if (request_end > HOLDING_REG_NUM)
    {
        ParamModbus_Encode(&g_param, registers);
        return 0u;
    }

    candidate = g_param;
    for (i = 0u; i < PARAM_MODBUS_MAP_COUNT; ++i)
    {
        const param_modbus_map_t *map = &g_modbus_map[i];
        int32_t raw;
        double value;

        if (map->reg < start_reg || (uint32_t)map->reg >= request_end)
        {
            continue;
        }
        raw = ParamModbus_DecodeS16(registers[map->reg]);
        if (map->encoding == PARAM_MODBUS_SPEED_COMMAND)
        {
            if (raw < -6000 || raw > 6000)
            {
                ParamModbus_Encode(&g_param, registers);
                return 0u;
            }
            speed_command = raw;
            command_seen = 1u;
            continue;
        }

        value = (double)raw;
        if (map->encoding == PARAM_MODBUS_SCALED_S16)
        {
            value /= map->scale;
        }
        if (!Param_SetValue(&candidate, map->param_id, value))
        {
            ParamModbus_Encode(&g_param, registers);
            return 0u;
        }
        config_changed = 1u;
    }

    /* 配置参数先完整落盘；失败时目标速度命令也不执行。 */
    if (config_changed && !Param_Commit(&candidate))
    {
        ParamModbus_Encode(&g_param, registers);
        return 0u;
    }
    if (command_seen && !Param_SetSpeedCommand(speed_command))
    {
        ParamModbus_Encode(&g_param, registers);
        return 0u;
    }

    ParamModbus_Encode(&g_param, registers);
    return 1u;
}

/* 用当前运行值刷新 Modbus 寄存器镜像。 */
void Param_Publish(uint16_t *registers)
{
    ParamModbus_Encode(&g_param, registers);
}

/* Modbus 写寄存器后的统一入口，传入本次请求的地址范围。 */
uint8_t Param_Apply(uint16_t start_reg, uint16_t reg_count)
{
    return Param_UpdateFromModbus(HoldingReg, start_reg, reg_count);
}
