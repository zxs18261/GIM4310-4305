# 可配置项手册

本固件的全部可配置项分三层，按"改动生效方式"区分：

| 层 | 载体 | 修改方式 | 生效 |
|---|---|---|---|
| 板级硬件参数 | [board_config.h](../Application/config/board_config.h) | 改宏 → 重新编译烧录 | 复位后 |
| 电机/控制/保护参数 | [motor_config.h](../Application/config/motor_config.h) | 改宏 → 重新编译烧录（`MOTOR_TYPE` 也可在 Keil 工程宏定义中覆盖，如 `MOTOR_TYPE=2`） | 复位后 |
| 运行参数（持久化） | `g_Params`（[app_params.h](../Application/app/app_params.h)），存 Flash page 63 | shell `set <name> <v>` + `save`，或 CAN 配置通道 0x11 + 0x12 | 见各项"生效"列 |

运行参数上电从参数页加载；无有效记录（首次烧录/`PARAMS_VERSION` 变更/CRC 坏）时自动回落编译期默认值。`save` 请求由后台在**封波状态**下落盘（页擦除约 20ms 关中断），落盘后自动重建编码器多圈基准。恢复默认：shell `defaults` / CAN 0x13（只回 RAM，不落盘，需再 `save`）。

---

## 1. 运行期持久化参数（shell / CAN 可改）

默认值来源列为编译期宏，即"改默认值去哪里改"。

| shell 名 | CAN 索引 | 含义 | 单位/类型 | 默认值（来源） | 合法范围 | 生效方式 |
|---|---|---|---|---|---|---|
| `id` | 0 | CAN 节点 ID | uint | 1 | 1~63（`BOARD_CAN_NODE_ID_MAX`，受 0x600/0x680 配置地址空间约束） | 写入后后台重配 CAN（应答用旧 ID 发出） |
| `baud` | 1 | CAN 波特率枚举 | 0=1M 1=500k 2=250k 3=125k | 0（1M） | 0~3 | 写入后后台重配 CAN |
| `term` | 2 | 120Ω 终端电阻 | 0/1 | 1（单机调试友好） | 0/1 | 即时（GPIO 直切） |
| `timeout` | 3 | CAN 指令超时 | ms | 500（`PROT_CAN_TIMEOUT_MS`） | 0~60000，0=关闭 | 即时；仅约束"由 CAN 发起使能"的闭环模式 |
| `kp` | 4 | 速度环 Kp | A/(rad/s)，输出轴 | 1.5（`CTRL_SPEED_KP`） | 无硬校验 | 即时（写入即刷新 PID） |
| `ki` | 5 | 速度环 Ki | 连续量纲，内部 ×1ms | 15.0（`CTRL_SPEED_KI`） | 无硬校验 | 即时 |
| `poskp` | 6 | 位置环 Kp | 1/s | 20.0（`CTRL_POS_KP`） | 无硬校验 | 即时（慢环直读） |
| `curmax` | 7 | 电流限幅 | A（q 轴） | 12.0（`MOTOR_CUR_PEAK_A`） | (0, `MOTOR_CUR_PEAK_A`] | 即时（慢环 1kHz 刷新，叠加温度降额） |
| `spdmax` | 8 | 输出轴速度限幅 | rad/s | 50.0 / 81.0（`MOTOR_SPEED_MAX_RADS`） | (0, 1.2×`MOTOR_SPEED_MAX_RADS`] | 即时 |
| `bright` | 9 | WS2812 亮度 | 0~255 | 40（`BOARD_LED_BRIGHTNESS_DEFAULT`） | 0~255 | 即时 |
| `offset` | 10 | 电角度偏置 | rad | 0（校准写入） | — | **只读**（预定位校准产物） |
| `pp` | 11 | 极对数 | uint | 14（`MOTOR_POLE_PAIRS`，校准实测覆盖） | — | **只读** |
| `encinv` | 12 | 编码器方向反转 | 0/1 | 0（校准写入） | — | **只读** |
| `caldone` | 13 | 已校准标志 | 0/1 | 0 | — | **只读** |

注意：
- 只读四项（10~13）是 `cal` 预定位校准的产物，校准成功后自动请求落盘；
- `kp`/`ki`/`poskp` 未做范围校验（调参自由度），错误增益会振荡——建议小电流下整定；
- 修改 `id`/`baud` 后节点会短暂离线（后台重配），上位机须按新地址重连。

---

## 2. motor_config.h（电机/控制/校准/保护/MIT）

### 2.1 电机选型与电机参数

`MOTOR_TYPE` 选择参数段：`MOTOR_TYPE_GIM4310_10`（默认）/ `MOTOR_TYPE_GIM4305_10`。数据来源为 doc/ 两张官方参数表 png（权威）；表中线电感单位标 mH 实为 µH。

| 宏 | 4310-10 | 4305-10 | 单位 | 说明 |
|---|---|---|---|---|
| `MOTOR_POLE_PAIRS` | 14 | 14 | 对 | 校准实测覆盖运行值，此宏仅默认值 |
| `MOTOR_GEAR_RATIO` | 10.0 | 10.0 | — | 行星减速比；内部转子量↔输出轴量换算唯一依据 |
| `MOTOR_RS_OHM` | 0.98 | 0.555 | Ω | **星形等效相电阻 = 线-线值/2**；电流环 Ki 整定与开环电压限幅使用 |
| `MOTOR_LS_H` | 334e-6 | 142e-6 | H | 星形等效相电感 = 线-线/2；电流环 Kp 整定与交叉解耦使用 |
| `MOTOR_KT_ROTOR` | 0.078 | 0.05 | N·m/A | 转子侧转矩常数 = 表列输出轴值/减速比；与转速常数不自洽（反推 0.095/0.059），MIT 转矩定标偏差时按实测修正此宏 |
| `MOTOR_CUR_RATED_A` | 2.1 | 2.0 | A | 额定电流（预留 I²t 保护用，当前未参与逻辑） |
| `MOTOR_CUR_PEAK_A` | 12.0 | 12.0 | A | 峰值限幅；电机本体堵转 15.3/19.55A，**受板载测量上限 13.2A 约束**，改大前先看第 4 节联动约束 |
| `MOTOR_SPEED_MAX_RADS` | 50.0 | 81.0 | rad/s | 输出轴最高转速（482.6/778.9 rpm） |

派生：`MOTOR_KT_OUT = MOTOR_KT_ROTOR × MOTOR_GEAR_RATIO`（MIT 转矩↔iq 换算、反馈转矩估计）。

### 2.2 控制参数

| 宏 | 默认 | 说明 |
|---|---|---|
| `CTRL_CURRENT_TS` | 1/`BOARD_PWM_FREQ_HZ` | 电流环周期，**已与板级 PWM 频率联动**，勿手改 |
| `CTRL_CURRENT_BW_RADS` | 6283.2（1kHz） | 电流环带宽；PI 增益内模法自动整定：Kp=Ls·ωc、Ki=Rs·ωc·Ts。带宽越高对采样延迟越敏感，20kHz 采样下建议 ≤ 1.5kHz |
| `CTRL_DECOUPLE_ENABLE` | 1 | d/q 交叉解耦 + 反电动势前馈总开关（调试电流环阶跃时可置 0 排除前馈影响） |
| `CTRL_SLOW_LOOP_DIV` | 20 | 慢环分频：速度/位置环频率 = PWM 频率/该值 = 1kHz；改动会影响 `ki`/`poskp` 的离散化 |
| `CTRL_SPEED_KP` / `CTRL_SPEED_KI` | 1.5 / 15.0 | 速度环 PI **默认值**（运行值在 g_Params，见第 1 节） |
| `CTRL_POS_KP` | 20.0 | 位置环 P 默认值（级联输出速度给定） |
| `CTRL_POS_SPEED_LIMIT` | 15.0 rad/s | 位置模式速度给定限幅（与 `spdmax` 取小） |
| `CTRL_PLL_BW_RADS` | 1256.6（200Hz） | 速度估计 PLL 自然频率；调大响应快噪声大，建议 5~10 倍速度环带宽 |

### 2.3 校准参数

| 宏 | 默认 | 说明 |
|---|---|---|
| `CAL_CURRENT_A` | 3.0 A | d 轴锁定电流（额定 1.3 倍，锁定相铜损约 8.5W；小负载可降 2~2.5A） |
| `CAL_RAMP_TIME_S` | 0.5 s | 电流爬升/半时长用于缓降 |
| `CAL_SETTLE_TIME_S` | 0.8 s | 锁定静置时长 |
| `CAL_ROTATE_EREV` | 8 | 方向/极对数检测旋转电角圈数；圈数越多极对数实测抗负载角差越稳 |
| `CAL_ROTATE_SPEED_ERADS` | 12.57 rad/s | 旋转电角速度（2 电角圈/s ≈ 输出轴 0.86 rpm） |
| `CAL_OFFSET_POINTS` | 16 | 偏置标定点数（正/反向各半，步距 45°e，双向平均抵消摩擦滞后） |

### 2.4 保护参数

| 宏 | 默认 | 说明 |
|---|---|---|
| `PROT_OC_LIMIT_A` | 13.0 A | 软件过流阈值 = 测量量程 13.2A − 0.2A 裕量；**必须 < `BOARD_CUR_MEAS_RANGE`**（超量程读数钳位，另有贴轨检测兜底） |
| `PROT_OV_LIMIT_V` / `PROT_UV_LIMIT_V` | 30.0 / 9.0 V | 母线过/欠压（24V 标称）；换供电电压等级时改这里 |
| `PROT_VBUS_FAULT_MS` | 10 ms | 电压越限确认时间（滤再生瞬态抬压） |
| `PROT_MOS_WARN_C` / `PROT_MOS_OT_LIMIT_C` | 75 / 90 ℃ | MOS 降额起点/过温故障；两点间电流限幅线性降额到 0 |
| `PROT_MOT_WARN_C` / `PROT_MOT_OT_LIMIT_C` | 90 / 110 ℃ | 电机绕组同上（NTC 开路时豁免并打印告警） |
| `PROT_ENC_ERR_LIMIT` | 16 | 编码器连续错误（CRC/磁场）故障阈值，= 0.8ms @20kHz |
| `PROT_CAN_TIMEOUT_MS` | 500 | `timeout` 运行参数的默认值 |

### 2.5 MIT 协议量程

与上位机打包/解包**必须逐字节一致**，改动即协议不兼容：

| 宏 | 默认 | 备注 |
|---|---|---|
| `MIT_P_MAX` | ±12.5 rad | SteadyWin/mini-cheetah 标准 |
| `MIT_V_MAX` | ±65 rad/s | 4305-10 极速 81.6 超出，反馈饱和属预期 |
| `MIT_KP_MAX` | 0~500 N·m/rad | |
| `MIT_KD_MAX` | 0~5 N·m·s/rad | |
| `MIT_T_MAX` | ±18 N·m | 取 SteadyWin GDZ34 默认值以兼容原厂上位机 |

---

## 3. board_config.h（板级硬件）

除非换板/改硬件，通常不动。

### 3.1 时钟与 PWM

| 宏 | 默认 | 说明 |
|---|---|---|
| `BOARD_HSE_FREQ_HZ` | 20MHz | 板载晶振（非常见 8MHz）；改晶振须同步改 bsp_clock.c 的 PLLM（当前 DIV5 硬编码对应 20MHz） |
| `BOARD_SYSCLK_HZ` / `BOARD_TIM1_CLK_HZ` | 170MHz | 记录值，供派生宏计算；实际时钟树在 SystemClock_Config/bsp_clock |
| `BOARD_PWM_FREQ_HZ` | 20kHz | PWM/电流环频率；`BOARD_PWM_ARR`、`CTRL_CURRENT_TS` 自动派生，慢环频率 = 该值/`CTRL_SLOW_LOOP_DIV` |
| `BOARD_PWM_DEADTIME_DTG` | 34（200ns） | TIM1 DTG 原始值，非 ns！换算：DTG<128 时 ns = DTG×5.88；FD6288 另叠加 100~300ns 内部死区 |
| `BOARD_PWM_MAX_DUTY` | 0.94 | 每相占空比上限。三重约束：低侧采样窗口 ≥3µs、FD6288 最小脉宽 ≥500ns、电压利用率（详见联动约束） |
| `BOARD_PWM_ADC_TRIG_ADVANCE` | 40 拍（235ns） | ADC 触发相对计数器峰值的提前量；**勿增大**（会早于最迟导通的低侧管），电流测量异常时优先降 MAX_DUTY |
| `BOARD_PWM_CCR_U/V/W` | CCR1/2/3 | 相别→TIM1 通道映射（网表定死，勿改） |

### 3.2 电流采样

| 宏 | 默认 | 说明 |
|---|---|---|
| `BOARD_SHUNT_OHM` | 5mΩ | 低侧采样电阻 |
| `BOARD_CSA_GAIN` | 25.0 | 运放差分增益（原理图注释值）；**存在硬件疑点**（见 Firmware.md §3），若实测定标偏差在此修正 |
| `BOARD_ADC_VREF` | 3.3V | VREF+ = AVCC |
| `BOARD_CUR_SIGN_U/V/BUS` | -1/-1/+1 | 电流符号链（网表推导：低侧续流流入电机为负压差）；校准若整体反向优先查此处 |
| `BOARD_CUR_MEAS_RANGE` | 13.2A | 硬件测量量程（偏置居中时）；`PROT_OC_LIMIT_A` 与 `MOTOR_CUR_PEAK_A` 的天花板 |
| `BOARD_ADC1_JCH_CUR/IBUS/VBUS`、`BOARD_ADC2_JCH_CUR` | IN12/IN3/IN11、IN3 | 注入组通道映射（网表定死） |
| `BOARD_ADC1_RCH_MOSNTC` / `BOARD_ADC2_RCH_MOTNTC` | IN14 / IN5 | NTC 规则组通道 |
| `BOARD_ADC_CUR_SMPTIME` | 6.5 周期 | 电流通道采样时间@42.5MHz；加长会挤占低侧窗口 |
| `BOARD_ADC_VBUS_SMPTIME` | 24.5 周期 | VBUS 注入 rank3 |
| `BOARD_ADC_NTC_SMPTIME` | 247.5 周期 | NTC 高阻源（≈5kΩ）必需的长采样；**只在注入组启动前写入生效**（HAL 限制，勿在运行期重配） |
| `BOARD_ADC_OFFSET_MIN/MAX` | 300/3900 LSB | 上电偏置合理窗口（因硬件偏置疑点刻意放宽），越界触发 `FAULT_ADC_OFFSET` |
| `BOARD_ADC_RAIL_LOW/HIGH` | 20/4075 LSB | 贴轨（饱和=过流）判定阈值 |

### 3.3 母线电压与 NTC

| 宏 | 默认 | 说明 |
|---|---|---|
| `BOARD_VBUS_R_TOP/R_BOT` | 100k/4.3k | 分压电阻；`BOARD_VBUS_LSB` 派生（满量程 ≈80V） |
| `BOARD_VBUS_LPF_ALPHA` | 0.05 | 母线电压一阶低通（20kHz 下截止 ≈160Hz） |
| `BOARD_NTC_PULLUP` | 10k | 两路 NTC 共用的上拉（接 VCC，NTC 接地） |
| `BOARD_NTC_MOS_R25/BETA` | 10k/3950 | 板载 KNTC0603/10KF3950（确定值） |
| `BOARD_NTC_MOT_R25/BETA` | 10k/3950 | 电机内置 NTC：R25 为厂家惯例高置信，**B 值未公开待实测标定** |
| `BOARD_NTC_FAULT_TEMP` | 250℃ | 开路/短路哨兵值（触发传感器缺失豁免逻辑） |

### 3.4 串口 / CAN / 灯 / 编码器 / 中断优先级

| 宏 | 默认 | 说明 |
|---|---|---|
| `BOARD_UART_BAUDRATE` | 921600 | 调试串口波特率；终端不支持时改 115200（CSV 日志高速率时注意带宽） |
| `BOARD_CAN_TERM_ACTIVE_HIGH` | 1 | EN_120 电平语义（TS5A3159 IN=H 连 NO，硬件定死） |
| `BOARD_CAN_BROADCAST_ID` | 0x000 | 广播 ID（仅特殊帧） |
| `BOARD_CAN_NODE_ID_MAX` | 63 | 节点 ID 上限（0x600+63 不越入 0x680 应答区） |
| `BOARD_CAN_CFG_ID_BASE` / `REPLY_BASE` | 0x600 / 0x680 | 配置协议地址规划；改动即协议变更须同步上位机 |
| `BOARD_CAN_MASTER_ID` | 0x000 | MIT 反馈帧目标 ID（mini-cheetah 约定） |
| `BOARD_WS2812_ARR` | 派生（211） | 800kHz 位周期 |
| `BOARD_WS2812_CCR_0/1` | 58/136（0.34/0.80µs） | 0/1 码高电平；该 LED 型号约束 T0H<0.47µs、T1H>0.58µs |
| `BOARD_WS2812_ORDER_RGB` | 0（GRB） | 颜色发送序；本板 XL-3528RGBW-WS2812B 实测为标准 24bit GRB |
| `BOARD_LED_BRIGHTNESS_DEFAULT` | 40 | `bright` 运行参数的默认值 |
| `BOARD_ENC_GPIO_PORT`、`BOARD_ENC_CSN/CLK/DO_PIN`、`BOARD_ENC_DO_PIN_POS` | GPIOB 4/5/6 | 位带 SSI 引脚（网表定死；CubeMX 的 SPI1 配置与实物不符已在代码回收） |
| `BOARD_IRQPRIO_FOC/CAN/UART/UART_DMA/LED_DMA` | 1/3/6/7/8 | NVIC 抢占优先级（组 4）；**FOC 必须最高**（SysTick 固定 15），并发正确性依赖此序 |

---

## 4. 联动约束清单（改 A 前必查 B）

1. **`BOARD_PWM_FREQ_HZ`** → `BOARD_PWM_ARR`、`CTRL_CURRENT_TS` 自动派生；但需人工复核：慢环频率（/`CTRL_SLOW_LOOP_DIV`）、`PROT_VBUS_FAULT_MS` 折算周期数、低侧采样窗口绝对宽度（= (1−MAX_DUTY)×周期，20kHz→3µs，40kHz 就只剩 1.5µs 不够）、ISR 预算（= SYSCLK/PWM_FREQ）。
2. **`BOARD_PWM_MAX_DUTY`** ↑ 电压利用率 vs ↓ 采样窗口/最小脉宽。下限约束：窗口 ≥ 注入序列 1.8µs + 振铃裕量，且 (1−duty)×周期 ≥ 500ns（FD6288 输入滤波）。电流测量在高调制区异常时**降**这个值（如 0.92），不要动 TRIG_ADVANCE。
3. **`MOTOR_CUR_PEAK_A` / `PROT_OC_LIMIT_A` / `BOARD_CUR_MEAS_RANGE`** 三者关系必须保持：`PEAK < OC_LIMIT < MEAS_RANGE`。想用满电机 15.3A 堵转能力必须先改硬件（增益降到 ~20 或换 shunt），单改宏会让过流保护失效。
4. **`CTRL_CURRENT_BW_RADS`** 隐含依赖 `MOTOR_RS_OHM`/`MOTOR_LS_H` 准确性（内模法整定）；换电机段后无需手调，但参数表阻感与实测差异大时电流环阶跃会过冲/迟缓。
5. **`CTRL_SLOW_LOOP_DIV`** → 慢环 Ts 变化影响 `ki` 的离散化与 `PROT_VBUS_FAULT_MS` 无关（后者在快环），只影响速度/位置环性态。
6. **`BOARD_HSE_FREQ_HZ`** 仅是记录值：真正的 PLL 配置在 [bsp_clock.c](../Application/bsp/bsp_clock.c)（HSE 20MHz→PLLM DIV5）与 CubeMX 的 SystemClock_Config（HSI 回退路径），三处必须一致。
7. **MIT 量程五宏**与上位机是同一份契约，单侧改动 = 静默的数值错误（不会报错）。
8. **`PARAMS_VERSION`**（app_params.h）：改 `APP_ParamsTypeDef` 布局必须递增，否则旧记录按新布局误读；递增后所有节点上电回默认值（含校准结果，需重新 `cal`）。新增字段一律追加到结构体末尾。
9. **Flash 参数页**：`PARAM_PAGE_ADDR = 0x0801F800`（bsp_flash.c）与 Keil 工程 IROM Size `0x1F800` 互补——改参数页位置两处同改，否则链接器可能把代码放进参数页。

---

## 5. 修改后的验证清单

- 任何宏改动：重新编译应保持 0 Error / 0 Warning（CLI 命令见 Firmware.md §11）；
- 涉及 PWM/采样时序：上电看 `status` 的 `isr` 行（last/max 周期数 vs 预算）与 `[init] current offsets`；
- 涉及电机参数：重新 `cal`（阻感只影响环路增益无需重校，但极对数/方向类必须重校）；
- 涉及协议（ID 规划/MIT 量程）：与上位机联调回归。
