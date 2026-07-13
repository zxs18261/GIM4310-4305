/**
  ******************************************************************************
  * @file    board_config.h
  * @brief   板级硬件参数统一配置（GIM4310/4305 电机控制器 V0.0 板）
  * @note    本文件集中定义与电路板绑定的全部硬件参数与引脚/通道映射，
  *          依据为网表 Netlist_Schematic1_2026-07-11.tel 与原理图 V1.0：
  *          - TIM1 CH1/CH2/CH3 = U/V/W 相（FD6288Q 同相驱动）；
  *          - 电流采样：U/V 两相低侧腿独立 5mR + 公共母线回流 5mR，
  *            内部 OPAMP 差分放大 25 倍，偏置约半量程（上电实测补偿）；
  *          - MT6701 编码器为 GPIO 位带模拟 SSI（PB4=CSN/PB5=CLK/PB6=DO，
  *            无硬件 SPI 复用可用）；
  *          - EN_120（PA15）高电平接入 CAN 120R 终端电阻（TS5A3159 IN=H 连 NO）。
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Z.Xusheng
  * SPDX-License-Identifier: MIT
  *
  * This file is part of the GIM4310/GIM4305 joint motor controller firmware,
  * distributed under the MIT License. See the LICENSE file in the repository
  * root for full terms.
  ******************************************************************************
  */

#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== 时钟 ====================================== */
#define BOARD_HSE_FREQ_HZ           (20000000UL)    /*!< 板载晶振 20MHz（非常见 8MHz，勿照搬例程） */
#define BOARD_SYSCLK_HZ             (170000000UL)   /*!< 系统时钟 */
#define BOARD_TIM1_CLK_HZ           (170000000UL)   /*!< TIM1 内核时钟（APB2 x1） */

/* ============================== PWM ======================================= */
#define BOARD_PWM_FREQ_HZ           (20000UL)       /*!< PWM/电流环频率 */
/** 中心对齐模式：ARR = TIM1CLK / (2 * PWM_FREQ) = 4250 */
#define BOARD_PWM_ARR               ((uint32_t)(BOARD_TIM1_CLK_HZ / (2UL * BOARD_PWM_FREQ_HZ)))
/** 死区：200ns @ tDTS=5.88ns -> DTG=34；FD6288 另有 100~300ns 内部死区叠加 */
#define BOARD_PWM_DEADTIME_DTG      (34U)
/** 每相占空比上限：保证低侧 shunt 采样窗口 >= 3us 且高/低侧脉宽 >= 500ns
    （FD6288 输入滤波对窄脉冲的吞没风险） */
#define BOARD_PWM_MAX_DUTY          (0.94f)
/** ADC 注入组触发提前量 [计数拍]：OC4 上升沿相对计数器峰值的提前时间，
    40 拍 = 235ns，保证采样落在低侧导通窗口中心附近 */
#define BOARD_PWM_ADC_TRIG_ADVANCE  (40U)
/** 相别 -> TIM1 CCR 寄存器成员映射（网表确认 CH1=U/CH2=V/CH3=W） */
#define BOARD_PWM_CCR_U             CCR1
#define BOARD_PWM_CCR_V             CCR2
#define BOARD_PWM_CCR_W             CCR3

/* ============================ 电流采样 ==================================== */
#define BOARD_SHUNT_OHM             (0.005f)        /*!< 低侧采样电阻 */
#define BOARD_CSA_GAIN              (25.0f)         /*!< OPAMP 差分增益（原理图注释值） */
#define BOARD_ADC_VREF              (3.3f)          /*!< VREF+ = AVCC */
/** 电流分辨率 [A/LSB] = VREF / 4096 / (增益 * 采样电阻) ≈ 6.45mA */
#define BOARD_CUR_LSB               (BOARD_ADC_VREF / 4096.0f / (BOARD_CSA_GAIN * BOARD_SHUNT_OHM))
/** 电流符号：低侧续流窗口内，相电流流入电机时 shunt 压差为负（网表推导），
    正方向定义为电流流入电机 */
#define BOARD_CUR_SIGN_U            (-1.0f)
#define BOARD_CUR_SIGN_V            (-1.0f)
#define BOARD_CUR_SIGN_BUS          (1.0f)
/** 电流通道硬件量程 [A]（偏置在半量程时约 ±13.2A，超出即放大器/ADC 饱和） */
#define BOARD_CUR_MEAS_RANGE        (13.2f)

/** ADC 注入通道映射（网表确认）：
    IU  = OPAMP3_VOUT = PB1 = ADC1_IN12
    IBUS= OPAMP1_VOUT = PA2 = ADC1_IN3（峰值采样时刻恒≈0，仅诊断用）
    IV  = OPAMP2_VOUT = PA6 = ADC2_IN3
    VBUS= PB12 = ADC1_IN11 */
#define BOARD_ADC1_JCH_CUR          ADC_CHANNEL_12
#define BOARD_ADC1_JCH_IBUS         ADC_CHANNEL_3
#define BOARD_ADC1_JCH_VBUS         ADC_CHANNEL_11
#define BOARD_ADC2_JCH_CUR          ADC_CHANNEL_3
/** 规则组通道：MOS NTC = PB11 = ADC1_IN14，电机 NTC = PC4 = ADC2_IN5 */
#define BOARD_ADC1_RCH_MOSNTC       ADC_CHANNEL_14
#define BOARD_ADC2_RCH_MOTNTC       ADC_CHANNEL_5

/** 采样时间（ADC 时钟 = SYSCLK/4 = 42.5MHz） */
#define BOARD_ADC_CUR_SMPTIME       ADC_SAMPLETIME_6CYCLES_5
#define BOARD_ADC_VBUS_SMPTIME      ADC_SAMPLETIME_24CYCLES_5
#define BOARD_ADC_NTC_SMPTIME       ADC_SAMPLETIME_247CYCLES_5

/** 零流偏置合理窗口 [LSB]：标称 1.65V 对应 2048，但网表所示偏置网络存在
    直流工作点疑问（详见 doc/Firmware.md 硬件注意事项），窗口放宽并在
    偏置校准时打印实测值供核对 */
#define BOARD_ADC_OFFSET_MIN        (300.0f)
#define BOARD_ADC_OFFSET_MAX        (3900.0f)
/** 电流通道贴轨判定阈值 [LSB]：原始值越过即视为放大器/ADC 饱和（过流） */
#define BOARD_ADC_RAIL_LOW          (20U)
#define BOARD_ADC_RAIL_HIGH         (4075U)

/* ============================ 母线电压 ==================================== */
#define BOARD_VBUS_R_TOP            (100.0f)        /*!< 分压上电阻 [kOhm] */
#define BOARD_VBUS_R_BOT            (4.3f)          /*!< 分压下电阻 [kOhm] */
/** 电压分辨率 [V/LSB] ≈ 19.55mV，满量程 ≈ 80V */
#define BOARD_VBUS_LSB              (BOARD_ADC_VREF / 4096.0f * ((BOARD_VBUS_R_TOP + BOARD_VBUS_R_BOT) / BOARD_VBUS_R_BOT))
/** 母线电压一阶低通系数（20kHz 采样，约 160Hz 截止） */
#define BOARD_VBUS_LPF_ALPHA        (0.05f)

/* ============================== NTC ======================================= */
#define BOARD_NTC_PULLUP            (10000.0f)      /*!< 上拉电阻（两路同为 10k 接 VCC） */
#define BOARD_NTC_MOS_R25           (10000.0f)      /*!< 板载 KNTC0603/10KF3950 */
#define BOARD_NTC_MOS_BETA          (3950.0f)
#define BOARD_NTC_MOT_R25           (10000.0f)      /*!< 电机内置 NTC（R25 厂家惯例值，B 值未公开，
                                                         待实测标定，见 doc/Firmware.md 第 10 节） */
#define BOARD_NTC_MOT_BETA          (3950.0f)
#define BOARD_NTC_FAULT_TEMP        (250.0f)        /*!< 开路/短路时返回的故障温度 [C] */

/* ============================ 调试串口 ==================================== */
#define BOARD_UART_BAUDRATE         (921600UL)      /*!< 高速调试打印；终端不支持时改 115200 */

/* ============================== CAN ======================================= */
#define BOARD_CAN_TERM_ACTIVE_HIGH  (1)             /*!< EN_120 高电平 = 120R 接入（TS5A3159 IN=H 连 NO） */
#define BOARD_CAN_BROADCAST_ID      (0x000U)        /*!< 广播 ID（全体节点接收特殊帧） */
#define BOARD_CAN_NODE_ID_MAX       (63U)           /*!< 节点 ID 范围 1~63（配置协议地址空间约束） */
#define BOARD_CAN_CFG_ID_BASE       (0x600U)        /*!< 配置协议请求 ID = 0x600 + NodeId */
#define BOARD_CAN_CFG_REPLY_BASE    (0x680U)        /*!< 配置协议应答 ID = 0x680 + NodeId */
#define BOARD_CAN_MASTER_ID         (0x000U)        /*!< MIT 反馈帧目标 ID（mini-cheetah 主机约定） */

/* ============================ WS2812 灯 =================================== */
/** 位周期 1.25us：ARR = SYSCLK / 800kHz - 1 = 211 */
#define BOARD_WS2812_ARR            ((uint32_t)(BOARD_SYSCLK_HZ / 800000UL - 1UL))
#define BOARD_WS2812_CCR_0          (58U)           /*!< 0 码高电平 ~0.34us */
#define BOARD_WS2812_CCR_1          (136U)          /*!< 1 码高电平 ~0.80us */
#define BOARD_WS2812_ORDER_RGB      (0)             /*!< 0 = 标准 GRB 发送序 */
#define BOARD_LED_BRIGHTNESS_DEFAULT (40U)          /*!< 默认亮度（0~255） */

/* ========================= 编码器（位带 SSI） ============================= */
/** 网表实际接线：CSN=PB4、CLK=PB5、DO=PB6（无硬件 SPI 复用可用，位带模拟） */
#define BOARD_ENC_GPIO_PORT         GPIOB
#define BOARD_ENC_CSN_PIN           GPIO_PIN_4
#define BOARD_ENC_CLK_PIN           GPIO_PIN_5
#define BOARD_ENC_DO_PIN            GPIO_PIN_6
#define BOARD_ENC_DO_PIN_POS        (6U)            /*!< IDR 位号 */

/* =========================== 中断优先级 =================================== */
/** NVIC_PRIORITYGROUP_4：数值越小优先级越高，SysTick 已固定为 15 */
#define BOARD_IRQPRIO_FOC           (1U)            /*!< ADC 注入完成 = 电流环 */
#define BOARD_IRQPRIO_CAN           (3U)            /*!< CAN 接收解析 */
#define BOARD_IRQPRIO_UART          (6U)            /*!< 串口收发 */
#define BOARD_IRQPRIO_UART_DMA      (7U)            /*!< 串口 DMA */
#define BOARD_IRQPRIO_LED_DMA       (8U)            /*!< WS2812 DMA */

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_CONFIG_H */
