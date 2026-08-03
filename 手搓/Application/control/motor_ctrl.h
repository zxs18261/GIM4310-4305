/**
  ******************************************************************************
  * @file    motor_ctrl.h
  * @brief   电机控制核心模块头文件（模式状态机 + FOC 主循环）
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

#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "foc.h"
#include "pid.h"
#include "filter.h"
#include "protection.h"
#include "bsp_encoder.h"
#include "bsp_adc.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief 控制模式枚举
  * @note  数值用于 CAN/shell 上报，勿改动既有取值。
  */
typedef enum
{
    MC_MODE_DISABLED = 0U,      /*!< 封波（freewheel） */
    MC_MODE_OPENLOOP = 1U,      /*!< 开环电压模式（调试） */
    MC_MODE_TORQUE   = 2U,      /*!< 转矩（q 轴电流）环 */
    MC_MODE_SPEED    = 3U,      /*!< 转速环 */
    MC_MODE_POSITION = 4U,      /*!< 位置环（级联速度环） */
    MC_MODE_MIT      = 5U,      /*!< MIT 阻抗混合控制 */
    MC_MODE_CALIB    = 6U,      /*!< 预定位校准中 */
    MC_MODE_FAULT    = 7U,      /*!< 故障锁存 */
} MC_ModeTypeDef;

/**
  * @brief 电机控制运行数据结构体（telemetry 直接读取）
  */
typedef struct
{
    volatile MC_ModeTypeDef Mode;       /*!< 当前模式 */
    volatile PROT_FaultTypeDef Fault;   /*!< 锁存故障码 */
    volatile uint8_t Enabled;           /*!< 功率输出使能标志 */
    uint32_t PrechargeCnt;              /*!< 使能后自举预充倒计数 */

    /* 测量 */
    ENC_DataTypeDef  Enc;               /*!< 编码器数据 */
    PLL_HandleTypeDef Pll;              /*!< 转子机械角跟踪 PLL */
    float ThetaE;                       /*!< 电角度 [rad] */
    float Pos;                          /*!< 输出轴位置 [rad]（会话零位） */
    float Speed;                        /*!< 输出轴角速度 [rad/s] */
    float PosZero;                      /*!< 会话零位（转子机械角/减速比）[rad] */
    float Iu;                           /*!< U 相电流 [A] */
    float Iv;                           /*!< V 相电流 [A] */
    float Iw;                           /*!< W 相电流 [A]（重构） */
    float Vbus;                         /*!< 母线电压 [V] */
    float IbusEst;                      /*!< 母线电流估计 [A]（功率法） */

    /* 控制 */
    FOC_HandleTypeDef Foc;              /*!< 电流环 */
    PID_HandleTypeDef SpeedPid;         /*!< 速度环 PI */
    uint32_t SlowDivCnt;                /*!< 慢环分频计数 */
    float CurLimitNow;                  /*!< 当前生效电流限幅（含降额）[A] */

    /* 目标值 */
    float TgtIq;                        /*!< TORQUE 模式 q 轴电流给定 [A] */
    float TgtSpeed;                     /*!< SPEED 模式输出轴速度给定 [rad/s] */
    float TgtPos;                       /*!< POSITION 模式输出轴位置给定 [rad] */
    float MitPosDes;                    /*!< MIT 期望位置 [rad] */
    float MitVelDes;                    /*!< MIT 期望速度 [rad/s] */
    float MitKp;                        /*!< MIT 刚度 [N*m/rad] */
    float MitKd;                        /*!< MIT 阻尼 [N*m*s/rad] */
    float MitTff;                       /*!< MIT 前馈转矩 [N*m] */

    /* 开环模式 */
    float OpenVd;                       /*!< 开环 d 轴电压 [V] */
    float OpenVq;                       /*!< 开环 q 轴电压 [V] */
    float OpenSpeedE;                   /*!< 开环电角速度 [rad/s] */
    float OpenTheta;                    /*!< 开环电角度累加 [rad] */

    /* 诊断 */
    uint32_t IsrCount;                  /*!< 电流环计数 */
    uint32_t IsrCyclesMax;              /*!< 电流环最大耗时 [CPU cycle] */
    uint32_t IsrCyclesLast;             /*!< 电流环最近耗时 [CPU cycle] */
} MC_HandleTypeDef;

/* Exported variables --------------------------------------------------------*/
extern MC_HandleTypeDef g_Mc;

/* Exported functions prototypes ---------------------------------------------*/
void              MC_Init(void);
void              MC_FocIsr(const ADC_FocMeasTypeDef *meas);

HAL_StatusTypeDef MC_Enable(void);
void              MC_Disable(void);
HAL_StatusTypeDef MC_SetMode(MC_ModeTypeDef mode);
HAL_StatusTypeDef MC_StartCalibration(void);
HAL_StatusTypeDef MC_ClearFault(void);
void              MC_TripFault(PROT_FaultTypeDef fault);
void              MC_SetZeroHere(void);
void              MC_RebaseEncoder(void);

void              MC_SetIqRef(float iq);
void              MC_SetTorque(float torque);
void              MC_SetSpeedRef(float speed);
void              MC_SetPosRef(float pos);
void              MC_SetMitCommand(float pDes, float vDes, float kp, float kd, float tff);
void              MC_SetOpenLoop(float vd, float vq, float speedE);
void              MC_ApplyPidParams(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CTRL_H */
