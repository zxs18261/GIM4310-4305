# GIM4310/4305 关节电机控制器固件说明

固件版本 `APP_FW_VERSION = 0.1.0`（[app_main.h](../Application/app/app_main.h)），目标 MCU STM32G431CBU6 @170MHz，Keil MDK（Arm Compiler 6）构建。

配套文档：[CAN-Protocol.md](CAN-Protocol.md)（通信协议定义）、[Configuration.md](Configuration.md)（全部可配置项：编译期宏 + 运行期持久化参数 + 联动约束）。

## 1. 分层架构与文件归置

```
Application/
├─ config/                     统一参数抽象（全部硬件/电机/控制常数的唯一定义点）
│  ├─ board_config.h           板级：PWM/死区/采样映射/换算系数/NTC/CAN/灯/中断优先级
│  └─ motor_config.h           电机选型（MOTOR_TYPE）：极对数/阻感/Kt/限值 + 控制/校准/保护参数 + MIT 量程
├─ bsp/                        外设封装（重配 CubeMX 占位参数为实际工况参数）
│  ├─ bsp_clock.c/h            HSI→HSE 20MHz PLL 源切换（失败回退 HSI）
│  ├─ bsp_pwm.c/h              TIM1 中心对齐互补 PWM + 死区 + OC4→TRGO2 ADC 触发 + MOE 封波
│  ├─ bsp_adc.c/h              双 ADC 注入组（IU/IBUS/VBUS + IV）+ 偏置实测 + NTC 规则组 + OPAMP 启动
│  ├─ bsp_encoder.c/h          MT6701 SSI 位带模拟（PB4/PB5/PB6）+ CRC6 + 多圈展开
│  ├─ bsp_can.c/h              FDCAN1 经典 CAN + 位时序表 + 硬件滤波 + 120Ω 终端切换
│  ├─ bsp_uart.c/h             USART3 DMA TX 环形缓冲 + RXNE 中断接收（非阻塞 printf）
│  ├─ bsp_ws2812.c/h           TIM2_CH1 PWM+DMA 单灯驱动（24bit GRB，亮度缩放）
│  └─ bsp_flash.c/h            参数页（page 63 @0x0801F800）擦写 + 硬件 CRC32 校验
├─ control/                    控制域（与通信/外设解耦，参数经 Init 结构注入）
│  ├─ foc_math.c/h             查表 sin/cos、Clarke/Park、min-max 零序 SVPWM
│  ├─ foc.c/h                  d/q 电流 PI（内模法整定）+ 动态圆限幅 + 交叉解耦
│  ├─ pid.c/h                  clamping 抗饱和 PID
│  ├─ filter.c/h               一阶 LPF、PLL 角度跟踪速度估计器
│  ├─ motor_ctrl.c/h           模式状态机 + 20kHz FOC ISR 主循环（前台核心）
│  ├─ calibration.c/h          预定位：方向/极对数/电角偏置（ISR 内状态机）
│  └─ protection.c/h           过流/过压欠压/编码器/温度降额/CAN 看门狗判定
└─ app/                        应用与通信层
   ├─ app_main.c/h             APP_Init 初始化链 + APP_Loop 后台超级循环
   ├─ app_params.c/h           g_Params 运行参数（Flash 加载/落盘调度）
   ├─ app_can_protocol.c/h     MIT 运控协议 + 配置协议（详见 CAN-Protocol.md）
   ├─ app_shell.c/h            串口行式 shell
   ├─ app_monitor.c/h          CSV 周期日志 + 状态快照打印
   └─ app_led.c/h              灯语状态映射
```

CubeMX 覆盖区约束：`Core/` 下仅 `main.c`、`stm32g4xx_it.c` 的 `USER CODE BEGIN/END` 区内有改动；重新生成代码不会丢失任何应用逻辑。CubeMX 生成的外设初始化参数均为占位默认值，实际参数在各 `BSP_xxx_Init` 中重新配置（修改句柄 Init 字段后重新调用 HAL Init）。

## 2. 前后台执行模型

**前台（中断）**，优先级见 `board_config.h`：

| 中断 | 优先级 | 职责 |
|---|---|---|
| `ADC1_2_IRQHandler`（JEOS） | 1 | 20kHz 电流环节拍 → `MC_FocIsr` |
| `FDCAN1_IT0_IRQHandler` | 3 | CAN 帧解析、目标值写入、反馈帧入队 |
| `USART3_IRQHandler` | 6 | RXNE 入环形缓冲 + DMA TX 收尾 |
| `DMA1_Channel2/3_IRQHandler` | 7/8 | UART TX / WS2812 帧搬运 |
| `SysTick` | 15 | HAL tick |

**20kHz 电流环调用链**（`MC_FocIsr`，[motor_ctrl.c](../Application/control/motor_ctrl.c)）：

```
TIM1 OC4(计数器峰值前 235ns) → TRGO2 → ADC1/ADC2 注入组(准同步)
→ ADC1 JEOS 中断 → BSP_ADC_IRQHandler：读 JDR、减偏置、乘 LSB、VBUS 滤波
→ MC_FocIsr(meas)：
   BSP_ENC_Read (位带 SSI ~5us) → Counts/Raw
   PLL_Update(转子机械角) → Speed；Raw*PP&MASK → ThetaE
   PROT_CheckFast(过流/母线电压/编码器) → 触发即 MC_TripFault(MOE 封波)
   模式分派：
     CALIB    → CAL_Isr（d 轴电流闭环+强制角状态机）
     MIT      → iq = (Kp·Δp + Kd·Δv + Tff)/KT_OUT（每周期，20kHz 阻抗渲染）
     TORQUE/SPEED/POSITION → iq 给定由 20 分频(1kHz)慢环 MC_SlowLoop 更新
     OPENLOOP → FOC_OutputVoltage(强制角积分)
   FOC_Update：Clarke→Park→d/q PI(动态圆限幅)→反Park→SVPWM
   BSP_PWM_SetDuty → CCR1/2/3（预装载，下个周期谷底生效）
```

ISR 实测耗时通过 DWT 周期计数统计（`status` 命令 `isr` 行，预算 8500 cycle/周期）。

**后台**（`APP_Loop`）：shell 轮询、CSV 日志、灯语刷新、CAN 重配、100ms 慢任务（NTC 温度→`PROT_CheckSlow` 降额→过温故障、MIT 模式 CAN 指令超时）、参数落盘（仅封波状态执行——页擦除约 20ms 会 stall 取指令，期间电流环停摆）。

## 3. 电流采样链（本板拓扑特殊，重点）

网表实测拓扑为“两相腿 + 公共母线回流”混合低侧采样：

```
Q2(U低侧)源极 → IU+ → R1(5mΩ) ┐
Q4(V低侧)源极 → IV+ → R2(5mΩ) ┼ IBUS+(公共节点) → R3(5mΩ) → PGND
Q6(W低侧)源极 ────────────────┘
```

- OPAMP3 跨 R1（IU）、OPAMP2 跨 R2（IV）、OPAMP1 跨 R3（IBUS），差分增益 25（外部 1k/25k），输出进 ADC（IU=ADC1_IN12、IV=ADC2_IN3、IBUS=ADC1_IN3）。
- W 相无采样电阻：`IW = -(IU+IV)` 重构。
- 采样时刻在计数器峰值（三相低侧全导通）：此时 R3 上三相电流之和 ≈ 0，**OPAMP1 实测母线电流恒为 0**，仅作诊断；实际母线电流用功率法估计 `IbusEst = 1.5(Vd·Id+Vq·Iq)/Vbus`。
- 电流符号：低侧续流窗口内相电流流入电机时 shunt 压差为负 → `BOARD_CUR_SIGN_U/V = -1`。
- 换算：`I = (raw - offset) × 3.3/4096/(25×0.005)` ≈ 6.45mA/LSB，测量上限约 ±13.2A。
- 偏置：上电封波状态下 512 次平均实测（`BSP_ADC_MeasureOffsets`），越界（<300 或 >3900 LSB）触发 `FAULT_ADC_OFFSET`；偏置不对称导致单方向量程低于过流阈值时上电打印告警。
- **贴轨检测**：每周期对 IU/IV 原始值做 `≤20 / ≥4075` 判定（`BOARD_ADC_RAIL_LOW/HIGH`），命中即按过流封波——实际电流超出测量范围时换算值被钳位、阈值比较失效，贴轨检测是软件过流的兜底（本板无硬件 BKIN 过流通道）。
- NTC 规则组采样时间（247.5 周期）必须在注入组启动前一次性配置（注入组硬件触发常运行后 HAL 会静默跳过 SMPR 写入）；运行期只做启动-轮询-取值，**严禁调用 `HAL_ADC_Stop`**（本 HAL 版本会连注入组一起停掉并关闭 ADC，电流环节拍将永久丢失）。

**⚠ 硬件疑点（需实测确认）**：按网表电阻值（VINP 偏置网络 10k/10k）计算运放直流工作点会正轨饱和（理论 VOUT≈7.15V），与"1.65V 偏置"的设计注释矛盾——AVCC 侧偏置电阻应约 47k 而非 10k，怀疑 BOM 与网表标注不一致。固件已按"上电实测偏置"兜底，若上电打印的 offsets 明显偏离 2048（约 1.65V），请硬件同事复核 R29/R35/R40（AGND 侧）与 R45/R36/R41（AVCC 侧）实际阻值。

## 4. 采样触发时序

TIM1 中心对齐模式 1，ARR=4250（170MHz/(2×4250)=20kHz）。PWM 模式 1 下高侧在计数器谷底附近导通、峰值附近关断（低侧导通）。OC4 配 PWM 模式 2、CCR4=ARR-40：计数器上行穿越 CCR4 时 OC4REF 出上升沿 → TRGO2 → 注入组转换，落点在峰值前 235ns，位于低侧导通窗口中心附近。最大占空比 0.94 限制保证最窄低侧窗口 ≥3µs（ADC 注入序列约 1.8µs + 开关振铃裕量），同时满足 FD6288 输入滤波对 ≥500ns 脉宽的要求。

死区：TIM1 DTG=34 → 200ns；FD6288 内部另有 100~300ns 死区叠加（等效串联），MOSFET MCG53N06 关断需求约 71ns（typ），总裕量充分且不至于造成明显电流过零畸变。

电流环输出侧带两项前馈：**输出角度补偿**（占空比下周期生效的平均 1.5Ts 延迟，InvPark 用 `θe + we×1.5Ts` 超前角，高速段消除十几度电角的矢量滞后）与**反电动势前馈**（`vq += we×ψf`，ψf = Kt_rotor/(1.5×PP)，随 `CTRL_DECOUPLE_ENABLE` 一并开关）。开环模式电压矢量幅值被限制在 `Rs×峰值电流`（约 11.8V @GIM4310），防止误输入长时间大电流。

## 5. 编码器链

MT6701 实际接线 **CSN=PB4 / CLK=PB5 / DO=PB6**（CubeMX 里的 SPI1/PB3/PC11 配置与实物不符；PB5 无 SPI SCK 复用、PB6 无 MISO 复用，硬件 SPI 不可用）。`BSP_ENC_Init` 先 `HAL_SPI_DeInit` 回收 CubeMX 的 SPI 配置再改配位带 GPIO。

读取（`BSP_ENC_Read`，ISR 内阻塞约 5µs）：CSN↓ → 24 个软件时钟（上升沿移出/下降沿后采样，半周期 12×NOP≈70ns，约 5MHz）→ CSN↑。帧 = 角度14bit + 磁场状态 Mg[3:0] + CRC6（X⁶+X+1，初值 0，覆盖高 18bit）。CRC 错保持上次角度并计数，连续 16 次触发 `FAULT_ENCODER`；Mg[1:0]≠0（磁场过强/过弱）同样计错。

多圈处理刻意保持在**整数域**：`Turns`（int32，±21 亿圈）+ `Raw` 单圈计数，不合成连续计数（int32 合成计数在最高转速下约 1 小时溢出、float 合成在千圈级损失分辨率）。速度估计 PLL 跟踪**单圈角**（相位差回卷 ±π），长时间连续旋转无精度退化；位置 = `Turns×2π + Raw×2π/CPR` 分离合成。参数落盘（关中断 20ms+）后自动重建多圈基准（`MC_RebaseEncoder`）。

延迟预算：SSI 读取约 5µs + MT6701 内部 TDelay 5µs ≈ 10µs。GIM4310-10 最高转速 4826rpm（转子）时电角速度约 7071rad/s，对应电角滞后约 4°（电流环采样间隔内电角前进约 20°）；GIM4305-10 极速工况（转子 7789rpm）滞后约 6.5°、每采样周期电角前进约 33°——高速段建议后续版本加入基于 PLL 速度的角度前馈补偿（θe += we×Tdelay），中低速（关节应用主工况）当前精度足够。

## 6. 预定位校准（`cal` 命令 / CAN 配置命令 0x05）

`CAL_Isr` 状态机（全程 d 轴电流闭环 3A + 强制电角，不依赖已有校准）：

1. **RAMP/SETTLE**：电流 0.5s 缓升 → 0.8s 锁定 θ=0；
2. **ROTATE**：强制角 2 圈电角/s 正向旋转 8 个电角圈——编码器计数增量的符号判定**方向**（反向则写入 `EncInvert` 并即时生效）、增量幅值反推**极对数**（`PP = EREV×CPR/|ΔC|`，取整后写入 `g_Params.PolePairs`；圈数取 8 使始末负载角差的相对影响减半）、增量过小（<期望 30%）判失败 `FAULT_CAL_FAIL`（堵转/相线未接）；
3. **OFFSET**：16 点锁定（正向 8 点步距 45°e + 同点位反向回访，抵消摩擦滞后），每点记录 `编码器电角 - 强制角`，sin/cos 圆平均后 `atan2f` 得 **ElecOffset**；
4. **RAMPDOWN**：电流缓降，`Calibrated=1`，后台自动落盘（失败路径同样先缓降再报 `FAULT_CAL_FAIL`，避免大电流硬封波泵压）。

全流程约 10s；校准电流 3A 为额定的 1.3 倍（锁定相铜损约 8.5W），避免高温环境下连续反复校准。

校准结束后 MC 重新预置编码器多圈基准与 PLL，会话零位重置为当前位置。**相序处理说明**：本固件不交换相线映射，而是统一以"正电角方向 = 编码器计数增大方向"重建一致性（数学上与交换两相等效），接线任意相序均可正常闭环。

## 7. 保护与故障

| 码 | 故障 | 判定 | 位置 |
|---|---|---|---|
| 1 | OVERCURRENT | 任一相 \|i\|>13A（瞬时，20kHz） | PROT_CheckFast |
| 2/3 | OVER/UNDERVOLT | >30V / <9V 持续 10ms | PROT_CheckFast |
| 4/5 | MOS/MOTOR_OT | ≥90℃ / ≥110℃（75/90℃ 起线性降额电流限幅） | APP_SlowTask |
| 6 | ENCODER | CRC/磁场连续错 ≥16 次 | PROT_CheckFast |
| 7 | CAL_FAIL | 校准判定失败 | CAL_Isr |
| 8 | CAN_TIMEOUT | 由 CAN 发起使能的闭环模式 500ms 无指令（参数可调，0 关闭；shell 发起的使能豁免） | APP_SlowTask |
| 9 | ADC_OFFSET | 上电偏置实测越界 | APP_Init |

触发即 `MC_TripFault`：清 MOE（输出高阻，FD6288 内部下拉关断三相，电机 freewheel）→ 模式锁存 FAULT → LED 红色计数闪烁（次数=故障码）→ `clear` / CAN 0x04 清除。NTC 开路/短路视为传感器缺失（告警替代故障，便于裸板调试）。所有 fault handler（HardFault 等）与 `Error_Handler` 首行均直接清 MOE 兜底。

## 8. 灯语表（WS2812，亮度 `bright` 参数 0~255 持久化）

| 状态 | 灯语 |
|---|---|
| 故障 | 红色闪 N 次（N=故障码）+ 1.2s 间隔 |
| 校准中 | 蓝色 5Hz 快闪 |
| 运行(MIT) | 青色常亮 |
| 运行(其他闭环/开环) | 绿色常亮 |
| 温度降额 | 黄色（覆盖运行/空闲色） |
| 空闲·已校准 | 绿色呼吸（2s） |
| 空闲·未校准 | 白色呼吸 |

注：LED 实为 XL-3528RGBW-WS2812B（24bit GRB，"W"指雾状封装非白光通道），本板 VDD 接 3.3V 低于其数据手册 3.5V 下限，实测正常但亮度偏低属预期。

## 9. shell（USART3，默认 921600-8N1，`BOARD_UART_BAUDRATE` 可改）

`help` 查看全表。核心流：`cal` → `en <torque|speed|pos|mit|open>` → `iq/torque/speed/pos <v>` → `dis`；调参 `get` / `set <name> <v>` / `save`；调试 `status`、`log on [hz]`（CSV：`t_ms,mode,pos,spd,id,iq,vbus,tmos,tmot,fault,isr_cyc`）。

**BOOT0 风险**：PB8 = USART3_RX 兼 BOOT0。出厂选项字节 `nSWBOOT0=1` 时，若复位瞬间外部串口把 RX 拉高会误入系统 bootloader。上电横幅会检测并告警，执行 `boot0 fix confirm` 一次性把选项字节改为固定主 Flash 启动（编程后自动复位）。

## 10. 已知硬件注意事项汇总

1. **运放偏置网络疑点**（§3）——需实测 OPx_VOUT 零流电压；
2. **电机内置 NTC 的 B 值未公开**——`BOARD_NTC_MOT_BETA` 暂按 3950（SteadyWin 惯例 R25=10k 已多方佐证），建议两点法实测或向厂家（simon@steadywin.cn）索取后修正；
3. **EN_120 复位默认接入**——PA15 复位态为 JTDI 内部上拉，上电到 GPIO 初始化前 120Ω 终端是接入的，`CanTerm` 参数（默认 1）决定运行时状态；
4. **母线 TVS 为 SMBJ36CA、MOSFET 60V**，官方"设计最大输入 80V"仅指 ADC 量程，实际供电按 24V 标称；
5. MT6701 EEPROM 零点无法在板烧写（需 VDD>4.5V），零点全部由固件侧 `ElecOffset`/会话零位处理。

## 11. 构建

Keil ≥5.43 + Arm Compiler 6（工程已含 `<uAC6>1</uAC6>`）。命令行：

```powershell
Start-Process 'C:\Keil6\UV4\UV4.exe' -ArgumentList '-b','"...\MDK-ARM\GIM4310（V0.0）.uvprojx"','-j0','-o','"build.log"' -Wait
```

电机选型切换：`motor_config.h` 中 `MOTOR_TYPE` 改为 `MOTOR_TYPE_GIM4305_10`（或在工程宏定义中添加 `MOTOR_TYPE=2`）。

链接器内存布局已把 IROM 收缩为 `0x1F800`（126KB），主 Flash 最后一页（page 63，参数页）从代码区永久划出——代码体量增长越界会在链接期报错而不是运行期擦掉固件尾部。
