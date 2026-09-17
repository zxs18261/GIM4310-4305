# 手搓工程 FOC 模块说明

> 路径：`E:\github\GIM4310-4305\手搓`  
> 对照对象：本仓库自研固件中的控制核心（**不是** SguanFOC 第三方库）

## 1. 定位

面向 **SteadyWin GIM4310/GIM4305** 关节电机的完整板级固件：FOC + 校准 + 保护 + CAN(MIT) + 串口 shell。  
控制算法写在 `Application/control/`，与 GIM4310 V0.0 硬件（双运放低侧采样、MT6701 SSI、FD6288）强绑定。

| 项 | 说明 |
|---|---|
| 作者/许可 | Z.Xusheng，MIT（`Application/`） |
| MCU | STM32G431CBU6 @170MHz |
| 数值 | 全浮点 `float`（无 Q31 双轨） |
| 电流环 | 20kHz（与 PWM 同频，ADC 注入 JEOS） |
| 慢环 | 1kHz（电流环 20 分频） |
| 版本 | 固件 `v0.1.0`（README：尚未上电实测） |

## 2. FOC 相关文件清单

```
Application/control/
├── foc_math.c/h      sin/cos 查表、Clarke/Park/InvPark、min-max SVPWM
├── foc.c/h           d/q 电流 PI（内模法）+ 解耦/反电势前馈 + 圆限幅
├── pid.c/h           clamping 抗饱和 PID
├── filter.c/h        LPF、PLL 测速
├── motor_ctrl.c/h    模式状态机 + MC_FocIsr（前台核心）
├── calibration.c/h   预定位：方向 / 极对数 / 电角偏置
└── protection.c/h    过流/电压/温度降额/编码器/CAN 看门狗

Application/config/
├── board_config.h    板级：PWM/ADC/引脚/IRQ
└── motor_config.h    电机与环路参数（MOTOR_TYPE）

Application/bsp/      PWM、ADC、编码器、CAN、UART…（硬件封装）
Application/app/      APP_Init、shell、CAN 协议、灯语、参数 Flash
```

## 3. 算法要点

- **电流环整定**：`Kp = Ls·ωc`，`Ki = Rs·ωc·Ts`（内模法）
- **SVPWM**：αβ → 三相后做 **min-max 零序注入**（中点平移），非经典七段扇区表
- **前馈**：d/q 交叉解耦 + `vq += we·ψf`；InvPark 使用 `θe + we·1.5Ts` 补偿 PWM 更新延迟
- **开环**：`FOC_OutputVoltage(vd,vq,θe)`，强制角积分在 `motor_ctrl` 中
- **校准**：d 轴锁定电流 + 强制电角旋转测方向/极对数 + 16 点电角偏置平均

## 4. 控制模式

| 模式 | 枚举 | 说明 |
|---|---|---|
| DISABLED | 0 | 封波 |
| OPENLOOP | 1 | 电压开环 |
| TORQUE | 2 | q 轴电流 / 转矩 |
| SPEED | 3 | 速度环 |
| POSITION | 4 | 位置→速度→电流 |
| MIT | 5 | `τ = Kp·Δp + Kd·Δv + τ_ff` |
| CALIB | 6 | 校准中 |
| FAULT | 7 | 故障锁存 |

## 5. 调试接口

- USART3 **921600** 行式 shell（`cal` / `en torque` / `iq` / `log on`）
- CAN MIT 运控 + 配置通道
- 遥测为 **CSV**，非 VOFA JustFloat

## 6. 阅读入口

1. `README.md` → `doc/Firmware.md`  
2. `app_main.c`（`APP_Init`）  
3. `motor_ctrl.c`（`MC_FocIsr`）  
4. `foc.c` / `foc_math.c`  
