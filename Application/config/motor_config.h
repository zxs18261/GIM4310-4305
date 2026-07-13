/**
  ******************************************************************************
  * @file    motor_config.h
  * @brief   电机参数与控制参数统一配置
  * @note    - 通过 MOTOR_TYPE 选择目标电机，扩展新电机时新增参数段即可；
  *          - 标注 [待确认] 的参数为厂家资料未获取前的工程估计值：
  *            极对数与编码器方向由预定位校准程序实测校验（不一致时告警），
  *            Kt/相阻感建议以实测或厂家数据修正；
  *          - 内部约定：转子侧机械量后缀 Rotor，输出轴（关节）侧不带后缀；
  *            输出轴量 = 转子量 / MOTOR_GEAR_RATIO。
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

#ifndef __MOTOR_CONFIG_H
#define __MOTOR_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "board_config.h"   /* 电流环周期等派生参数与板级 PWM 频率联动 */

/* ============================ 电机选型 ==================================== */
#define MOTOR_TYPE_GIM4310_10       (1)
#define MOTOR_TYPE_GIM4305_10       (2)

#ifndef MOTOR_TYPE
#define MOTOR_TYPE                  MOTOR_TYPE_GIM4310_10
#endif

/* ======================= GIM4310-10 参数段 ================================ */
/* 来源：doc/GIM4310-10.png（随机附带的官方参数表，48V 额定版本，权威）。
   表中"线电阻/线电感"为线-线值，FOC 采用星形等效相参数 = 线-线值 / 2；
   线电感表列单位 mH 但数值 668.47 实为 uH（0.668mH）；
   极对数由预定位校准实测复核。 */
#if (MOTOR_TYPE == MOTOR_TYPE_GIM4310_10)

#define MOTOR_NAME                  "GIM4310-10"
#define MOTOR_POLE_PAIRS            (14U)       /*!< 参数表，校准实测复核 */
#define MOTOR_GEAR_RATIO            (10.0f)     /*!< 行星减速比 10:1 */
#define MOTOR_RS_OHM                (0.98f)     /*!< 星形等效相电阻 = 1.96(线-线)/2 */
#define MOTOR_LS_H                  (0.000334f) /*!< 星形等效相电感 = 668.47uH(线-线)/2 */
/** 转子侧转矩常数 = 表列扭矩常数 0.78(输出轴)/减速比。
    注意与转速常数不完全自洽（10.05rpm/V 反推约 0.095）：表列值可能已含
    减速器效率/测试口径差异，MIT 转矩标定如有偏差可按实测修正本宏 */
#define MOTOR_KT_ROTOR              (0.078f)
#define MOTOR_CUR_RATED_A           (2.1f)      /*!< 额定电流（参数表；预留 I2t 连续电流保护用） */
/** 峰值电流限幅：电机本体堵转 15.3A，但受板载电流测量上限约 13.2A 约束，
    留检测余量后取 12A（软件过流阈值 13A 见 PROT_OC_LIMIT_A） */
#define MOTOR_CUR_PEAK_A            (12.0f)
#define MOTOR_SPEED_MAX_RADS        (50.0f)     /*!< 输出轴最高 482.6rpm ≈ 50.5rad/s */

/* ======================= GIM4305-10 参数段 ================================ */
/* 来源：doc/GIM4305-10.png（随机附带的官方参数表，权威）。
   线电感表列 283.96（单位标 mH 实为 uH）。 */
#elif (MOTOR_TYPE == MOTOR_TYPE_GIM4305_10)

#define MOTOR_NAME                  "GIM4305-10"
#define MOTOR_POLE_PAIRS            (14U)       /*!< 参数表，校准实测复核 */
#define MOTOR_GEAR_RATIO            (10.0f)
#define MOTOR_RS_OHM                (0.555f)    /*!< 相电阻 = 1.11(线-线)/2 */
#define MOTOR_LS_H                  (0.000142f) /*!< 相电感 = 283.96uH(线-线)/2 */
/** 表列扭矩常数 0.5(输出轴)/减速比；转速常数 16.23rpm/V 反推约 0.059，
    偏差处理同 GIM4310 注释 */
#define MOTOR_KT_ROTOR              (0.05f)
#define MOTOR_CUR_RATED_A           (2.0f)      /*!< 额定电流（参数表；预留 I2t 连续电流保护用） */
/** 电机本体堵转 19.55A，同样受板载测量上限约束取 12A */
#define MOTOR_CUR_PEAK_A            (12.0f)
#define MOTOR_SPEED_MAX_RADS        (81.0f)     /*!< 输出轴最高 778.9rpm ≈ 81.6rad/s */

#else
#error "MOTOR_TYPE not supported"
#endif

/** 输出轴转矩常数 [N*m/A]（忽略减速器效率） */
#define MOTOR_KT_OUT                (MOTOR_KT_ROTOR * MOTOR_GEAR_RATIO)

/* ============================ 控制参数 ==================================== */
/** 电流环 */
#define CTRL_CURRENT_TS             (1.0f / (float)BOARD_PWM_FREQ_HZ)   /*!< 电流环周期 = PWM 周期（随板级 PWM 频率联动） */
#define CTRL_CURRENT_BW_RADS        (6283.2f)           /*!< 电流环带宽 1kHz */
#define CTRL_DECOUPLE_ENABLE        (1U)                /*!< d/q 交叉解耦前馈 */

/** 速度/位置环（电流环 20 分频 = 1kHz） */
#define CTRL_SLOW_LOOP_DIV          (20U)
#define CTRL_SLOW_TS                (CTRL_CURRENT_TS * (float)CTRL_SLOW_LOOP_DIV)
/** 速度环 PI 默认增益（输出轴速度误差 [rad/s] -> q 轴电流 [A]），运行时可调 */
#define CTRL_SPEED_KP               (1.5f)
#define CTRL_SPEED_KI               (15.0f)             /*!< 连续量纲 [A/(rad)]，离散化乘 Ts */
/** 位置环 P 默认增益（输出轴位置误差 [rad] -> 输出轴速度给定 [rad/s]） */
#define CTRL_POS_KP                 (20.0f)
/** 位置模式下速度给定限幅 [rad/s]（输出轴） */
#define CTRL_POS_SPEED_LIMIT        (15.0f)

/** 速度估计 PLL 自然频率 [rad/s] */
#define CTRL_PLL_BW_RADS            (1256.6f)           /*!< 200Hz */

/* ============================ 校准参数 ==================================== */
#define CAL_CURRENT_A               (3.0f)      /*!< 预定位/校准 d 轴锁定电流 */
#define CAL_RAMP_TIME_S             (0.5f)      /*!< 电流爬升时间 */
#define CAL_SETTLE_TIME_S           (0.8f)      /*!< 锁定静置时间 */
#define CAL_ROTATE_EREV             (8U)        /*!< 方向/极对数检测旋转电角圈数
                                                     （圈数越多，始末负载角差对极对数
                                                     实测的相对影响越小） */
#define CAL_ROTATE_SPEED_ERADS      (12.57f)    /*!< 校准旋转电角速度 [rad/s]（2 电角圈/s） */
#define CAL_OFFSET_POINTS           (16U)       /*!< 偏置标定采样点数（正反向各一半，步距 45 度电角） */

/* ============================ 保护参数 ==================================== */
#define PROT_OC_LIMIT_A             (13.0f)     /*!< 软件过流阈值 = BOARD_CUR_MEAS_RANGE(13.2A) 减 0.2A 检测裕量；
                                                     超出测量上限后放大器/ADC 饱和读数钳位，阈值必须低于量程 */
#define PROT_OV_LIMIT_V             (30.0f)     /*!< 过压阈值（额定 24V 母线） */
#define PROT_UV_LIMIT_V             (9.0f)      /*!< 欠压阈值 */
#define PROT_VBUS_FAULT_MS          (10U)       /*!< 电压越限确认时间 */
#define PROT_MOS_OT_LIMIT_C         (90.0f)     /*!< MOS 过温故障阈值 */
#define PROT_MOS_WARN_C             (75.0f)     /*!< MOS 温度告警/降额起点 */
#define PROT_MOT_OT_LIMIT_C         (110.0f)    /*!< 电机绕组过温故障阈值 */
#define PROT_MOT_WARN_C             (90.0f)     /*!< 电机温度告警/降额起点 */
#define PROT_ENC_ERR_LIMIT          (16U)       /*!< 编码器连续错误故障阈值 */
#define PROT_CAN_TIMEOUT_MS         (500U)      /*!< CAN 指令超时（0 = 关闭检测） */

/* ========================= MIT 协议量程 ==================================== */
/** 与上位机/驱动库必须一致的打包量程（输出轴量纲）。
    采用 SteadyWin GIM 家族/mini-cheetah 标准量程：P ±12.5 / V ±65 /
    KP 0~500 / KD 0~5；T_MAX 取 SteadyWin GDZ34 默认 18 N*m 以兼容原厂上位机 */
#define MIT_P_MAX                   (12.5f)     /*!< 位置 [-P_MAX, +P_MAX] rad */
#define MIT_V_MAX                   (65.0f)     /*!< 速度 [-V_MAX, +V_MAX] rad/s；
                                                     GIM4305-10 空载最高 81.6rad/s 超出
                                                     打包范围，反馈饱和到边界属预期 */
#define MIT_KP_MAX                  (500.0f)    /*!< 刚度 [0, KP_MAX] N*m/rad */
#define MIT_KD_MAX                  (5.0f)      /*!< 阻尼 [0, KD_MAX] N*m*s/rad */
#define MIT_T_MAX                   (18.0f)     /*!< 转矩 [-T_MAX, +T_MAX] N*m */

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONFIG_H */
