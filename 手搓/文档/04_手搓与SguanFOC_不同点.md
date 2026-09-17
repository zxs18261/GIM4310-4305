# 手搓 FOC 与 SguanFOC Library — 不同点

> 对比对象  
> - **手搓**：`E:\github\GIM4310-4305\手搓`  
> - **SguanFOC**：`E:\github\GIM4310-4305\SguanFOC库文件编写\SguanFOC_Library-main`

---

## 1. 产品形态（最大差异）

| 维度 | 手搓工程 | SguanFOC Library |
|---|---|---|
| 本质 | **整机固件**（板级产品） | **可移植算法库** + 例程/资料 |
| 与硬件关系 | 强绑定 GIM4310 V0.0 原理图 | 弱绑定，靠 `UserData_Function` 钩子 |
| 是否被对方引用 | **未包含** Sguan 源码 | 与手搓无代码共用 |
| 交付物 | 可烧录的 Keil 工程 | 多版本库 + G4 例程 + PDF/Simulink/上位机 |

**结论**：手搓不是「SguanFOC 的二次封装」，而是另一套自研控制栈。

---

## 2. 目录与模块划分

| | 手搓 | SguanFOC |
|---|---|---|
| FOC 核心路径 | `Application/control/*.c` | `SguanFOC库v3.x.x/*.c` |
| 硬件层 | `Application/bsp/*`（完整实现） | 用户填钩子；例程在「配套③」 |
| 参数 | `board_config.h` / `motor_config.h` | `UserData_Motor.h` / `UserData_Config.h` |
| 应用协议 | `app_can_protocol` + shell | `Sguan_printf` JustFloat 为主 |
| 版本并行 | 单一固件树 | **三套算法树**并存（3.0.0 / 3.0.1 / 3.1.0） |

---

## 3. 调度与中断模型

| | 手搓 | SguanFOC |
|---|---|---|
| 电流环触发 | TIM1→ADC 注入完成 **JEOS** → `MC_FocIsr` | 用户定时器调 `SguanFOC_High_Loop` |
| 慢任务 | `APP_Loop` 后台 + ISR 内 20 分频 | `SguanFOC_Low_Loop` |
| 初始化 | `APP_Init`（main USER CODE） | `SguanFOC_main_Loop` 内状态推进 |
| 看门狗/互斥 | PWM 超时、Flash 落盘关中断策略 | `MOTOR_FLAG_STRUCT`（PWM_Calc 等） |

---

## 4. 数值与算法实现差异

| 项目 | 手搓 | SguanFOC |
|---|---|---|
| 数值类型 | **仅 float** | v3.0.0 float；**v3.0.1 Q31**；v3.1.0 float 无感 |
| SVPWM | **min-max 零序注入** | **七段式扇区 SVPWM**（v3.0.1 注释明确） |
| 电流环 API | `FOC_Update(ia,ib,ic,θe,we,vbus)` | 库内封装在 High_Loop / Control_* |
| 输出延迟补偿 | InvPark 用 `θe+1.5Ts·we` | 视版本；v3.1 有相位延迟等进阶补偿 |
| 无感 | **无** | v3.1.0：**HFI / SMO / NLFO / Hall…** |
| 先进控制 | 标准 PI + MIT 阻抗 | LADRC、STA、SMC、DOB、MTPA、弱磁、齿槽…（随版本） |
| 参数辨识 | 无完整辨识器 | v3.1 `Sguan_Identify` 等 |

---

## 5. 控制模式命名与能力

| 能力 | 手搓 | SguanFOC |
|---|---|---|
| 开环 | `MC_MODE_OPENLOOP` | `Velocity_OPEN_MODE` |
| 电流/转矩 | `TORQUE` | `Current_SINGLE_MODE` |
| 速度环 | `SPEED` | `VelCur_DOUBLE_MODE` |
| 位置环 | `POSITION` | `PosVelCur_THREE_MODE` |
| **MIT 阻抗** | **有**（一等公民，CAN 主推） | **无原生 MIT** |
| 专用校准模式 | `CALIB` + `cal` 一键落盘 | 角度校正/极性等标志位，流程不同 |
| 故障锁存 | `FAULT` + 灯语闪码 | `Sguan_MotorStatus` 状态机 |

---

## 6. 硬件接口差异（本仓库语境）

| | 手搓（GIM4310 板） | Sguan 通用 / 例程③ |
|---|---|---|
| 编码器 | **MT6701**，GPIO 位带 SSI | 用户钩子；常见 SPI/ABZ 磁编 |
| 电流采样 | U/V 低侧 + 母线回流，IW 重构 | 例程多为下桥臂双电阻，钩子可配 AB/AC/BC |
| 栅驱 | FD6288Q + MCG53N06 | 随板而变 |
| CAN | **MIT + 配置协议**（完整） | 库核心不绑定 CAN |
| 调试串口 | **921600 文本 shell** | **JustFloat / VOFA**（`AO=` 等） |
| 状态灯 | WS2812 灯语 | 依赖用户板 |

---

## 7. 校准与参数持久化

| | 手搓 | SguanFOC |
|---|---|---|
| 校准流程 | 强制角旋转测方向/PP + 16 点 ElecOffset，约 10s | 库内角度校正/极性辨识路径（不同状态机） |
| 落盘 | Flash 末页 + CRC，`save` / 校准自动请求 | 依赖用户实现；库本身不规定板级 Flash 布局 |
| 会话零位 | `zero` 命令 | 由用户控制层定义 |

---

## 8. 文档与配套生态

| | 手搓 | SguanFOC |
|---|---|---|
| 自述文档 | `README` + `doc/Firmware.md` 等（板级详尽） | 上游 README + PDF + B 站移植视频 |
| Simulink | 无 | 配套算法原理图开源 |
| QT 上位机 | 无（shell/CAN） | 配套 QT / CubeTool |
| Doxygen | `doc/html` | 视版本 |

---

## 9. 选型建议（本仓库场景）

| 目标 | 建议 |
|---|---|
| 在 **GIM4310 自研板** 上尽快跑通 MIT / shell | 用 **手搓** 工程 |
| 学习通用 FOC、移植到其他 MCU、玩无感/HFI | 用 **SguanFOC**（建议先 v3.0.1 有感，再 v3.1.0） |
| 把 Sguan「嵌进」手搓 | 需重做钩子与中断模型，**工作量大**，且与现有 MIT/校准重复；一般不推荐混用两套电流环 |
| 算法对照学习 | 可对照阅读：手搓 `foc.c` ↔ Sguan `SguanFOC.c` + `Sguan_SVPWM.c` |

---

## 10. 一句话总结不同点

> **手搓 = 面向 GIM4310 的完整产品固件（float + min-max SVPWM + MIT/CAN/shell）；SguanFOC = 可移植多版本算法库（七段 SVPWM + JustFloat + 可选 Q31/无感全家桶）。二者理论同类，工程不可直接互换。**

相同点见：[`03_手搓与SguanFOC_相同点.md`](03_手搓与SguanFOC_相同点.md)。  
