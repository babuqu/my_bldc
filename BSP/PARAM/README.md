# 参数模块说明

## 参数定义与 `param_t` 的对应关系

`param_registry.c` 中的 `g_param_desc` 是持久化配置参数的定义表。每项描述稳定 ID、数据类型、长度、读写权限、工程值范围、默认值，以及它在 `param_t` 中对应字段的偏移。

表项通过 `PARAM_DESC_STATE` 宏创建。最后一个参数是 `param_t` 的字段名，宏用 `offsetof` 将字段名转换成 `state_offset`：

```c
PARAM_DESC_STATE(PARAM_ID_KP_SPEED, PARAM_TYPE_FLOAT,
                 PARAM_ACCESS_READ | PARAM_ACCESS_WRITE,
                 0.0, 6.55, 0.0, kp)
```

这项定义表示：参数 ID `PARAM_ID_KP_SPEED` 是一个可读写的 `float`，范围为 0.0 到 6.55，默认值为 0.0，对应 `param_t.kp`。读写状态字段时，registry 根据 `state_offset` 找到字段，再根据 `type` 以对应的 C 类型访问它。结构体字段顺序可以调整，定义表中的字段名仍会由编译器重新计算偏移。

**当前实际列入定义表并保存到 Flash 的参数只有 `kp`、`ki`、`kd`。** `param_t` 中的 `actual_speed`、`i_max`、`i_min`、`out_max`、`out_min` 和 `flag` 当前都没有定义表项，因此不会作为配置参数保存。新增持久化参数时，需要同时增加正确类型的 `param_t` 字段、稳定 ID，以及一条带权限、范围、默认值和字段名的定义表项。

参数 ID 是持久化身份，不随表顺序变化；`state_offset` 只负责连接定义项和运行结构体。不要用数组下标或 Modbus 地址代替参数 ID。

## 运行值与持久化值

模块中有两种 RAM 表示：

- `g_param` 是控制和参数服务使用的运行结构体。
- `param_registry.c` 中的 `g_param_values` 是按参数 ID 保存类型化字节值的 registry 当前值表，也是 Flash 快照的输入。

它们不是两份独立配置来源。`Param_RegistrySync` 将 `g_param` 中已定义的字段校验后同步到 registry；启动恢复时，registry 值再通过 `Param_RegistryRead` 生成 `g_param`。Flash 只保存 registry 当前值，不直接保存整个 `param_t` 结构体。

定义表保存可读的普通数值，例如 `0.0`、`6.55`。浮点数只在值记录编解码边界转换成 32 位 IEEE-754 位模式；描述表中不维护浮点位常量。

## 配置参数写入流程

1. 调用方复制当前 `g_param`，通过 `Param_SetValue` 修改候选字段。registry 按 ID 查定义，并检查写权限、类型和范围。
2. `Param_Commit` 检查候选状态，调用 `Param_RegistrySync` 更新 registry 当前值。
3. `Param_SaveToFlash` 请求存储层追加完整快照。
4. Flash 写入及回读校验成功后，才用候选值更新 `g_param` 并调用 `Param_To_System` 应用运行配置。失败时恢复 registry 中原来的参数值。

`Param_Sanitize` 当前只检查状态指针和 registry 初始化状态，不负责逐参数裁剪或参数之间的关系检查。单项权限、类型和范围由 registry 接口检查。

## Modbus 与目标转速命令

Modbus 地址和缩放只在 `param_modbus.c` 中维护，通过映射表关联持久化参数 ID：

| Holding Register | 含义 | 处理方式 |
|---:|---|---|
| 0 | KP | 持久化配置，缩放系数 1000 |
| 1 | KI | 持久化配置，缩放系数 1000 |
| 2 | KD | 持久化配置，缩放系数 1000 |
| 3 | 目标转速 | 运行命令，范围 -6000 到 6000，不保存到 Flash |

适配层只处理本次 Modbus 请求覆盖的寄存器，避免镜像中的旧寄存器值覆盖未写入的参数。一次请求同时写配置和目标转速时，配置快照成功后才执行目标速度命令。

## Flash 存储格式

每条参数记录为：

```text
[ID:LE16][TYPE:U8][LENGTH:U8][DATA:LE][CRC16:LE]
```

记录集合保存在完整快照中。快照头含 magic、格式版本、记录数、递增序列号和负载长度，记录区之后有 CRC32。当前存储格式版本为 2；版本不匹配或校验失败的快照会被忽略，不做旧版本迁移。没有有效快照时，参数 registry 使用定义表默认值。

存储层在 STM32F407 的 Sector 10 和 Sector 11 之间追加、轮换快照，每个扇区按 128 字节槽位写入。链接程序区必须避开 `0x080C0000..0x080FFFFF`。更换芯片或调整链接布局前，需要重新核对 Flash 分区。

## 文件职责

| 文件 | 职责 |
|---|---|
| `param.c/.h` | 运行参数服务、运行状态访问、提交、PID 应用、启动恢复和目标转速命令接口。 |
| `param_registry.c/.h` | 参数定义表、registry 当前值、类型与范围校验、值记录编解码。 |
| `param_modbus.c` | Modbus 寄存器与参数 ID 的映射、寄存器缩放和命令字处理。 |
| `param_storage.c` | STM32F407 Flash 快照读写、扇区轮换和 CRC 校验。 |
