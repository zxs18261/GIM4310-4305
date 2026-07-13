/**
  ******************************************************************************
  * @file    foc.h
  * @brief   FOC 电流环模块头文件
  * @note    仅包含磁场定向控制本体（变换 + d/q 电流 PI + SVPWM），
  *          不涉及外设与通信，硬件参数由上层通过 Init 结构注入。
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

#ifndef __FOC_H
#define __FOC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "foc_math.h"
#include "pid.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief FOC 初始化参数结构体
  */
typedef struct
{
    float Ts;               /*!< 电流环周期 [s]（= PWM 周期） */
    float CurrentBandwidth; /*!< 期望电流环带宽 [rad/s] */
    float Rs;               /*!< 相电阻 [Ohm]（用于 PI 增益整定） */
    float Ls;               /*!< 相电感 [H]（用于 PI 增益整定与解耦） */
    float Flux;             /*!< 转子磁链 [Wb] = Kt_rotor/(1.5*PP)，反电动势前馈用，0 关闭 */
    float MaxDuty;          /*!< 每相占空比上限 (0.5, 1)，低侧采样窗口约束 */
    uint8_t DecoupleEnable; /*!< 1 = 使能 d/q 交叉解耦 + 反电动势前馈 */
} FOC_InitTypeDef;

/**
  * @brief FOC 运行句柄结构体
  * @note  Update 之后可读取内部测量/输出用于监控。
  */
typedef struct
{
    PID_HandleTypeDef PidId;    /*!< d 轴电流 PI */
    PID_HandleTypeDef PidIq;    /*!< q 轴电流 PI */
    float   Ls;                 /*!< 相电感（解耦用） */
    float   Flux;               /*!< 转子磁链（反电动势前馈用） */
    float   Ts;                 /*!< 电流环周期（输出角度前馈补偿用） */
    float   MaxDuty;            /*!< 占空比上限 */
    uint8_t DecoupleEnable;     /*!< 解耦/前馈使能 */

    float IdRef;                /*!< d 轴电流给定 [A] */
    float IqRef;                /*!< q 轴电流给定 [A] */

    FOC_Vec2TypeDef IAlphaBeta; /*!< 测量电流 alpha/beta [A] */
    FOC_Vec2TypeDef Idq;        /*!< 测量电流 d/q [A] */
    FOC_Vec2TypeDef Vdq;        /*!< 输出电压 d/q [V] */
    float DutyA;                /*!< A 相占空比输出 */
    float DutyB;                /*!< B 相占空比输出 */
    float DutyC;                /*!< C 相占空比输出 */
} FOC_HandleTypeDef;

/* Exported functions prototypes ---------------------------------------------*/
void FOC_Init(FOC_HandleTypeDef *hfoc, const FOC_InitTypeDef *init);
void FOC_Reset(FOC_HandleTypeDef *hfoc);
void FOC_SetCurrentRef(FOC_HandleTypeDef *hfoc, float idRef, float iqRef);
void FOC_Update(FOC_HandleTypeDef *hfoc, float ia, float ib, float ic,
                float thetaE, float we, float vbus);
void FOC_OutputVoltage(FOC_HandleTypeDef *hfoc, float vd, float vq,
                       float thetaE, float vbus);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_H */
