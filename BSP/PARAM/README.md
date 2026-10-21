# 参数管理模块

## 参数定义的唯一来源

`param_list.h` 是参数元数据的唯一来源。每行包含符号名和稳定数值 ID、registry 类型、访问权限、工程值范围、默认值、C 字段类型与字段名，以及可选的系统发布语句。X-Macro 根据该清单生成参数 ID 枚举、`param_t` 字段、registry 描述表和 `Param_To_System` 发布赋值。

新增参数时，在 `PARAM_LIST(X)` 中添加一行。参数发布后应保持数值 ID 不变。发布语句可以留空；非空时必须是有效的 C 语句，并且能在 `Param_To_System` 的上下文中编译，其中可使用 `state` 及目标变量声明。

`param_modbus_list.h` 是 Modbus 保持寄存器映射的独立来源。每行包含零基寄存器偏移、参数 ID、缩放倍数和线上类型（`U16` 或 `S16`）。只有需要通过 Modbus 暴露的参数才需要在此添加映射。寄存器偏移必须唯一、位于 `HOLDING_REG_NUM` 范围内，并与参数范围及缩放倍数相匹配。

当前参数定义如下：

| ID | 字段 | 类型 | 权限 | 范围 | 默认值 | Flash 持久化 | 发布目标 |
|---:|---|---|---|---:|---:|---|---|
| `0x0101` | `target_speed` | `int32_t` | 读/写 | -32760..32760 | 0 | 否 | 无 |
| `0x0102` | `kp` | `float` | 读/写 | 0..6.55 | 0.5 | 是 | `g_speed_pid.Proportion` |
| `0x0103` | `ki` | `float` | 读/写 | 0..6.55 | 0.25 | 是 | `g_speed_pid.Integral` |
| `0x0104` | `kd` | `float` | 读/写 | 0..6.55 | 0.15 | 是 | `g_speed_pid.Derivative` |

当前 Modbus 映射使用寄存器 0..3：目标转速使用有符号类型，缩放倍数为 1；KP、KI、KD 使用无符号类型，缩放倍数为 10000。增益上限 6.55 对应编码值 65500，不超过 U16 上限。

## 运行态与 registry

`g_param` 是参数服务使用的运行态视图。`param_registry.c` 维护按类型编码的当前值缓存。registry 描述表只读；当前值单独保存，并通过稳定参数 ID 序列化。

`Param_SetValue` 在候选状态中修改参数，并检查参数 ID、写权限、类型和范围。`Param_Commit` 复制候选状态，比较持久化参数是否变化；有变化时先初始化 Flash 存储，再将候选值同步到 registry，并在需要时写入快照。全部成功后，才更新 `g_param` 并发布到 PID。

如果同步 registry 或写 Flash 失败，提交会尝试恢复旧参数状态。恢复旧状态失败时，代码会重置 registry 到描述表默认值，再从 registry 重建 `g_param`。

`Param_Commit` 中较早调用 `ParamStorage_Load` 是有意的。若存储尚未就绪，`ParamStorage_Save` 会延迟调用 Load，而 Load 会把旧 Flash 快照应用到 registry。先初始化存储、再同步候选值，可避免 Save 后续加载旧快照并覆盖候选值。

启动时，`Param_Init_` 会重置并初始化 registry，加载最新有效的 Flash 快照，从 registry 生成运行态参数，再发布到 PID。没有有效快照时使用参数描述表中的默认值。

## 快照格式与固件升级

Flash 中每条参数记录都包含稳定 ID、类型、长度、值和 CRC16，记录顺序不用于匹配参数。快照整体另有 CRC32，用于检查数据是否损坏。当前写入版本为 3；读取同时兼容此前加入定义指纹的版本 4 快照，忽略其中的定义指纹，以便固件升级后仍能读取参数。

加载时按 ID 将记录逐项匹配到当前参数表。新增参数没有旧记录时使用当前默认值；固件中已删除的参数会被忽略；调整清单顺序或 `param_t` 结构体布局不会影响匹配。旧记录的 ID 不存在、类型或长度不匹配、权限不允许，或值超出当前范围时，只跳过该条记录，其他有效参数仍会加载。若快照整体 CRC 或记录 CRC 损坏，则整份快照不应用，并继续查找另一扇区的有效快照。

若参数含义发生变化但仍沿用相同 ID、类型和范围，存储层无法自动识别这种语义变化；需要为这类变更分配新 ID，或另行编写显式迁移逻辑。

## 校验与限制

`Param_Sanitize` 当前只检查状态指针和 registry 是否初始化，不检查参数间的关系。单项参数的类型、权限和范围由 registry 读写接口及同步流程检查。

访问权限定义支持只读参数，但当前同步流程调用 `ParamRegistry_SetCurrentValue`，该接口拒绝没有写权限的参数。目前 `PARAM_LIST` 中的参数都可写。新增只读参数前，需要先调整同步和默认值初始化逻辑。

描述表初始化会检查重复 ID、非法类型或范围，以及 `param_t` 字段偏移是否重叠。Modbus 映射目前不会在启动时检查寄存器偏移重复或是否超出寄存器数组范围；维护 `param_modbus_list.h` 时需自行确保这些约束成立。

`Param_RegistrySync` 按条目逐项写入。提交失败时会尝试恢复旧状态，再回退到描述表默认值；如果 registry 初始化或默认值读取本身失败，目前没有进一步恢复机制，无法保证运行态与 registry 仍保持一致。

## RAM 开销评估

按当前 4 个参数估算，参数管理模块的常驻 RAM 约为 65–68 B：`g_param` 约 16 B，registry 当前值表约 32 B，描述 CRC 缓存约 8 B，初始化标志约 1 B，Flash 存储运行状态约 8 B。具体值会受 ARMCC 对齐规则影响。参数描述表是只读数据，通常放在 Flash；当前每项描述约 40 B，4 项约 160 B，不计入常驻 RAM。

函数临时栈主要来自 Flash 快照缓冲区：`ParamStorage_Load` 和 `ParamStorage_Save` 各分配 128 B。若首次直接调用 Save，且存储尚未初始化，Save 会嵌套调用 Load，两份缓冲区可能同时占用约 256 B，另加调用帧和其他局部变量。`Param_Commit` 自身还会保存候选态和旧状态副本，当前共约 32 B。实际栈峰值需结合调用路径或运行时测量确认。

整个固件的 RAM 占用远大于参数模块。当前提供的 Keil 汇总为 RW Data 1,088 B、ZI Data 23,568 B；按列相加约 24,656 B，已经超过 20 KiB（20,480 B）的 RAM 上限。报告中的 `Total RW Size` 为 23,976 B，也超过该上限；应以 MAP 文件中的 SRAM 区段分布核对最终占用和统计口径。

工程配置 `configTOTAL_HEAP_SIZE` 为 15,360 B（15 KiB）。FreeRTOS 动态任务栈从这个堆中分配，当前动态任务配置的栈总量约 5,760 B；这部分包含在 FreeRTOS 堆的预留空间中，不能再与 15 KiB 重复相加。另有一个 512 B 的静态任务栈，以及启动文件配置的 1 KiB 主栈和 512 B 堆。若 RAM 上限确为 20 KiB，应先核实 MAP 中各 SRAM 段及堆的实际使用量，再评估缩小 FreeRTOS 堆和各任务栈；参数管理模块节省的空间不足以解决当前总量超限。

## Flash 布局

存储层在 STM32F407 的 Sector 10 与 Sector 11 中轮换保存快照。链接后的程序映像必须避开 `0x080C0000..0x080FFFFF`。Sector 10 和 Sector 11 各为 128 KiB，快照槽位为 128 字节。

只有设置 `PARAM_ACCESS_PERSIST` 的参数会写入 Flash。`Param_Commit` 仅在持久化参数发生变化时写入快照。像 `target_speed` 这样的运行时参数只更新 registry 和运行态，不持久化。

## 文件职责

| 文件 | 职责 |
|---|---|
| `param_list.h` | 参数元数据及可选发布语句 |
| `param_modbus_list.h` | Modbus 寄存器映射 |
| `param.c/.h` | 运行态、提交与回滚、启动恢复和发布 |
| `param_registry.c/.h` | 描述项查找、校验和类型化参数值记录 |
| `param_modbus.c` | Modbus 编码、解码和寄存器写入应用 |
| `param_storage.c` | Flash 快照存取、按 ID 加载及扇区轮换 |
