# GIM4310/GIM4305 关节电机控制器固件

STM32G431CBU6 的无刷关节电机（gimbal + 行星减速）FOC 控制器固件，适配 SteadyWin **GIM4310-10** / **GIM4305-10**（14 极对，10:1 行星减速，参数表见 [doc/](doc/)），裸机前后台架构，CAN 总线控制（MIT mini-cheetah 风格阻抗控制 + 配置协议），串口 shell 调试。

> 配套硬件：GIM4310/4305 电机控制器 V0.0 板（原理图与网表见 [doc/DataSheet/](doc/DataSheet/)）。

## 功能特性

- **FOC 磁场定向控制**：20kHz 电流环（双 ADC 注入组低侧采样同步触发），SVPWM（min-max 零序注入），d/q 交叉解耦 + 反电动势前馈 + PWM 更新延迟角度前馈
- **五种控制模式**：转矩（电流）/ 转速 / 位置（级联）/ **MIT 阻抗控制**（`τ = Kp·Δp + Kd·Δv + τ_ff`，力位混合）/ 电压开环
- **预定位校准**：一键实测编码器方向、极对数复核、电角度偏置（16 点双向平均抵消摩擦滞后），结果持久化
- **CAN 通信**：经典 CAN 1M/500k/250k/125k，MIT 运控通道 + 独立配置通道（参数读写/使能/校准/保存），硬件滤波，120Ω 终端电阻软件切换
- **保护体系**：软件过流 + ADC 贴轨（放大器饱和）检测、过压/欠压、MOS/电机双路 NTC 过温（含线性降额）、编码器故障、CAN 指令超时看门狗
- **WS2812 状态灯语**：初始化/校准/空闲/使能/告警/故障（红闪次数 = 故障码），亮度可调可持久化
- **串口 shell**（921600 8N1）：状态查询、模式控制、参数整定、CSV 数据流输出、BOOT0 选项字节修复
- **参数持久化**：Flash 末页（版本 + CRC32 校验），链接器已保留该页

## 硬件平台摘要

| 项 | 说明 |
|---|---|
| MCU | STM32G431CBU6 @170MHz（HSE 20MHz，**非常见 8MHz**） |
| 功率级 | FD6288Q 栅极驱动 + MCG53N06 三相桥，母线标称 24V |
| 电流采样 | U/V 相低侧 5mΩ + 母线回流 5mΩ，片内 OPAMP 差分 ×25，量程约 ±13.2A |
| 编码器 | MT6701 磁编码器，14bit，**GPIO 位带模拟 SSI**（PB4/PB5/PB6，硬件 SPI 不可用） |
| CAN | TJA1051 + TS5A3159 切换 120Ω 终端（EN_120 高 = 接入） |
| 调试口 | CN9：SWD + USART3（**PB8 兼 BOOT0**，建议烧写后执行 shell `boot0 fix confirm` 修复选项字节） |

## 目录结构

```
Application/            应用代码（本仓库主体）
  config/               统一参数抽象：board_config.h（板级）/ motor_config.h（电机/控制/保护）
  bsp/                  外设层：PWM/ADC/编码器/CAN/UART/WS2812/Flash/时钟
  control/              控制层：FOC/PID/滤波与PLL/模式状态机/校准/保护
  app/                  应用层：初始化与后台调度/CAN协议/灯语/shell/监控/参数
Core/                   CubeMX 生成（改动仅限 USER CODE 区）
Drivers/                ST HAL/CMSIS（未改动）
MDK-ARM/                Keil MDK 工程（Arm Compiler 6）
doc/                    文档与硬件资料
  Firmware.md           固件架构与实现说明（含硬件注意事项）
  CAN-Protocol.md       CAN 协议完整定义
  Configuration.md      全部可配置项手册（宏 + 运行参数 + 联动约束）
  DataSheet/            原理图/网表/器件手册
GIM4310（V0.0）.ioc     CubeMX 工程（注意：其外设参数为占位值，实际配置在 bsp 层重配）
```

## 构建

- 工具链：Keil MDK ≥5.38（**Arm Compiler 6**，工程已含 `<uAC6>` 标记），STM32Cube FW_G4 V1.6.3
- IDE：打开 `MDK-ARM/GIM4310（V0.0）.uvprojx` 直接编译
- 命令行（PowerShell）：

```powershell
Start-Process -FilePath 'C:\Keil6\UV4\UV4.exe' -Wait -ArgumentList '-b','"<repo>\MDK-ARM\GIM4310（V0.0）.uvprojx"','-j0','-o','"build.log"'
```

电机选型默认 GIM4310-10；切换 GIM4305-10 改 `motor_config.h` 的 `MOTOR_TYPE` 或在工程宏定义加 `MOTOR_TYPE=2`。

## 快速上手

1. **烧录**（SWD）后接 921600 8N1 串口，上电看横幅：核对时钟切换（HSE）、电流偏置打印（`offsets` 应在 2048 附近，异常见下文已知事项）；
2. **校准**（首次必做，电机会转动约 10s）：shell 输入 `cal`，完成后自动落盘方向/极对数/电角偏置；
3. **验证闭环**：`en torque` → `iq 0.5` 手感转矩，`status` 查看电流/速度/温度/ISR 耗时；
4. **CAN 运控**：向节点 ID（默认 1）发 `FF FF FF FF FF FF FF FC` 进入 MIT 模式，随后发 8 字节命令帧（P16/V12/KP12/KD12/T12 打包，量程见 [doc/CAN-Protocol.md](doc/CAN-Protocol.md)）；配置通道请求 ID `0x600+ID`、应答 `0x680+ID`。

完整命令表与协议字段：[doc/Firmware.md](doc/Firmware.md) · [doc/CAN-Protocol.md](doc/CAN-Protocol.md) · [doc/Configuration.md](doc/Configuration.md)。

## 已知事项（硬件相关）

- **运放偏置网络疑点**：按网表阻值计算差分放大器直流工作点会正轨饱和（期望 1.65V 中点）。固件用上电实测偏置 + 合理窗口 + 贴轨检测兜底，首次上电务必核对 `offsets` 打印；若贴 3900+ 需核对 BOM（R29/R35/R40 侧偏置电阻）。详见 [doc/Firmware.md](doc/Firmware.md) §10。
- **电机内置 NTC 的 B 值未公开**：暂按 10k/B3950（R25 有厂家佐证），过温阈值精度受限，建议两点法实测后修 `BOARD_NTC_MOT_BETA`。
- **PB8 串口 RX 兼 BOOT0**：外部串口若在复位瞬间拉高 RX 会误入系统 bootloader；shell `boot0` 查询状态，`boot0 fix confirm` 写 nSWBOOT0=0 固定从 Flash 启动（写完自动重启）。
- 电机堵转电流（15.3A/19.55A）超出板载测量量程 13.2A，固件峰值限幅 12A——需要满堵转能力须先改采样增益硬件。

## 许可证

本项目原创代码（`Application/`）以 [MIT License](LICENSE) 发布（Copyright (c) 2026 Z.Xusheng），各源文件头含 SPDX 标识。`Core/` 与 `Drivers/` 中 CubeMX 生成代码及 ST HAL/CMSIS 遵循其文件头所载的 ST 许可条款。

## 版本

- 固件 `v0.1.0`：首版完整实现（架构/五模式/校准/协议/保护/灯语/shell/持久化），Keil AC6 编译 0 警告；已完成多轮静态审查与修复，**尚未上电实测**。
