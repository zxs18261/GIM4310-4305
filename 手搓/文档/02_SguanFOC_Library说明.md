# SguanFOC Library 说明

> 路径：`E:\github\GIM4310-4305\SguanFOC库文件编写\SguanFOC_Library-main`  
> 上游：开源项目 [Sguan-ZhouQing/SguanFOC_Library](https://github.com/Sguan-ZhouQing/SguanFOC_Library)（MIT）

## 1. 定位

跨 MCU 的 **通用 FOC 算法库**：坐标变换、SVPWM、多环 PID、状态机、串口 JustFloat、可选无感算法等。  
通过用户钩子文件 `UserData_*.h` 对接具体硬件（PWM 占空比、ADC、编码器），库本体尽量与芯片解耦。

| 项 | 说明 |
|---|---|
| 语言 | 纯 C |
| 调度模型 | `SguanFOC_main_Loop` + `High_Loop` + `Low_Loop` + `Printf_Loop` |
| 调试协议 | VOFA **JustFloat**（`AO=` / `BO=` / `CO=` / `VO=` 等） |
| 硬件依赖 | 需用户实现 `UserData_Function.h` 等钩子 |

## 2. 仓库内目录结构

```
SguanFOC_Library-main/
├── README.md
├── LICENSE
├── SguanFOC库v3.0.0(有感foc，浮点运算，初阶)/     ← 有感 + float
├── SguanFOC库v3.0.1(有感foc，Q31定点，初阶)/      ← 有感 + Q31/float 可切换
├── SguanFOC库v3.1.0(无感foc，浮点运算，进阶)/     ← 无感/HFI/SMO/NLFO 等
├── 配套最新例程开源③[STM32G4，下桥臂双电阻]/   ← 板级例程工程
├── 上官FOC硬件开源[CH32V3，机器人关节PCB]/
├── 上官FOC软件开源[SguanFOC_CubeTool(v1.0)]/
├── 配套QT上位机及FOC使用说明①[PDF]/
└── 配套Simulink模型开源②[算法原理图]/
```

**与手搓对比时，以库源码目录（v3.0.0 / v3.0.1 / v3.1.0）为准；例程与 PDF 为配套资料。**

## 3. 版本能力对照

| 版本 | 数值 | 传感 | 亮点模块（节选） |
|---|---|---|---|
| **v3.0.0** | float | 有感 | PID、PLL、LADRC、滤波、JustFloat、UserData 钩子 |
| **v3.0.1** | Q31（可配 float） | 有感 | IQmath、STA 滑模、Optimize、七段 SVPWM |
| **v3.1.0** | float | 有感+无感 | HFI、SMO、NLFO、Hall、DOB、齿槽补偿、弱磁、参数辨识、SMC… |

## 4. 典型文件（以 v3.0.1 为例）

| 文件 | 作用 |
|---|---|
| `SguanFOC.c/h` | 总控：High/Low 环、模式分派、全局结构体 |
| `Sguan_SVPWM.c/h` | **七段式扇区 SVPWM**（+ Q31 版本） |
| `Sguan_PID` / `Sguan_PLL` / `Sguan_Filter` | 环路与测速 |
| `Sguan_MotorStatus` | 状态机 / 故障 |
| `Sguan_printf` | JustFloat 收发 |
| `UserData_Motor.h` | 电机参数 |
| `UserData_Function.h` | PWM/ADC/编码器钩子（移植关键） |
| `UserData_UserControl.h` | 用户实时控制、上位机通道映射 |
| `UserData_Config.h` / `Sguan_Config.h` | 功能开关、Q31 等 |

v3.1.0 额外大量观测器与补偿文件（`Sguan_HFI`、`Sguan_SMO`、`Sguan_NLFO` 等）。

## 5. 控制模式（库约定）

| 宏 | 含义 |
|---|---|
| `Velocity_OPEN_MODE` | 开环（直接 `Uq_in` 等） |
| `Current_SINGLE_MODE` | 电流单环 |
| `VelCur_DOUBLE_MODE` | 速度-电流双环 |
| `PosVelCur_THREE_MODE` | 位置-速度-电流三环 |

无原生 MIT 阻抗通道；通信侧以串口 JustFloat / 用户扩展为主。

## 6. 移植骨架（官方 README）

```c
#include "SguanFOC.h"

int main(void) {
    MCU_Init();
    while (1) {
        SguanFOC_main_Loop();   // 初始化 + 串口
    }
}

void HighFreq_IRQ(void) {
    SguanFOC_High_Loop();       // 电流环 / 角度 / PWM
}

void LowFreq_IRQ(void) {
    SguanFOC_Low_Loop();        // 状态机 / 保护
}
```

用户必须在 `UserData_*.h` 中填好电机参数与硬件读写函数。

## 7. 阅读入口

1. 本目录 `README.md`  
2. 选定版本（建议 G4 有感先看 **v3.0.1** 或例程③）  
3. `UserData_Function.h` + `SguanFOC.c` 的 High_Loop  
4. `Sguan_SVPWM.c`、电流环控制函数  
5. 进阶再读 v3.1.0 无感模块  
