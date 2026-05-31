# 无感 BLDC（反电动势过零）工程说明

## 1. 控制流程（按时间顺序）

1. `ALIGN`：先固定 `U+ / V-` 矢量，把转子拉到已知电角度位置。  
2. `OPENLOOP`：按六步换相表强制换相，逐步缩短换相周期并提高占空比，让反电动势幅值建立起来。  
3. `CLOSEDLOOP`：检测到稳定过零后，切换到过零闭环；每次过零后延迟一小段时间（近似 30 电角度）再换相。  
4. `SPEED PID`：闭环阶段由速度 PID 细调 `pwm_duty`，保持目标转速。

---

## 2. 本次实现的关键点

- 统一了过零状态机：`ALIGN -> DELAY -> OPENLOOP -> CLOSEDLOOP`。  
- 去掉了 `tim.c` 里“强制 `run_flag = RUN`”的调试残留，避免上电自转。  
- 增加了串口命令入口（在 `MonitorTask` 中）：  
  - `start [rpm] [dir]`：启动电机（`dir: 0=CCW, 1=CW`）  
  - `stop`：停机  
  - `speed [rpm]`：改目标转速  
  - `dir [0|1]`：停机状态切方向  
  - `state`：查看运行状态  
  - `help`：命令帮助  
- 修正了板级引脚映射：门驱使能/关断改为 `PWM_SD(PD8)` 和 `CTRL_SD(PD2)`。  
- 修正了 BEMF GPIO 时钟使能（PA 口而非 PH 口）。

---

## 3. 重要参数与调参建议（`BSP/ZERO_CTR/zero_ctr.h`）

- `ZC_ALIGN_TICKS`：定位时间。  
  - 太小：启动抖动、失步。  
  - 太大：启动慢。  
- `ZC_OPENLOOP_DELAY_INIT / MIN`：开环起始换相周期与最小周期。  
  - `INIT` 大一些更稳，`MIN` 小一些速度爬升更快。  
- `ZC_OPENLOOP_STEPS_BEFORE_CLOSEDLOOP`：最少开环步数。  
  - 太小会提前切闭环导致丢步；太大切换慢。  
- `FILTER_LONG`：数字去抖深度。  
  - 当前 `0xFFFF`（16 连续样本）。噪声大时可保持；若切换太慢可适度减小。  
- `ZC_COMM_DELAY_DIV`：过零到换相的延时比例。  
  - 当前经验值可先用；不同电机、滤波网络和 PWM 频率下需要微调。

---

## 4. 推荐联调步骤

1. 上电后串口输入 `help`，确认命令链路正常。  
2. 输入 `start 400 0` 低速启动。  
3. 输入 `state` 观察 `zero_state` 是否进入 `3`（闭环）。  
4. 输入 `speed 800`、`speed 1200` 逐步提速，观察电流、母线电压和温升。  
5. 若启动抖动：先增大 `ZC_ALIGN_TICKS`，再增大 `ZC_OPENLOOP_DELAY_INIT`。  
6. 若闭环切入困难：增大 `ZC_OPENLOOP_STEPS_BEFORE_CLOSEDLOOP`，或检查 BEMF 比较器阈值与硬件 RC。

---

## 5. 工程注意事项

- 过零法本质依赖 BEMF 信噪比，低速区必须靠开环拉起。  
- 电流采样零点偏置必须先校准，否则保护阈值和 PID 会漂。  
- 若硬件上 BEMF 比较器前端 RC 偏大，软件里的延时角必须对应下调。  
- 建议先在低母线电压、限流电源下调试，再逐步提高工作点。

