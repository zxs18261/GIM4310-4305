/**
  ******************************************************************************
  * @file    pid.h
  * @brief   位置式抗饱和（clamping anti-windup）PID 控制器模块头文件
  * @note    GIM4310/GIM4305 关节电机控制器应用代码，供电流环/速度环/位置环复用。
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

#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief PID 控制器句柄结构体
  * @note  积分项采用 clamping 抗饱和（anti-windup）：输出饱和且误差同向时冻结积分。
  */
typedef struct
{
    float Kp;           /*!< 比例增益 */
    float Ki;           /*!< 积分增益（单位含采样周期，即离散增益 Ki*Ts） */
    float Kd;           /*!< 微分增益（单位含采样周期，即离散增益 Kd/Ts），不用时置 0 */
    float OutMin;       /*!< 输出下限 */
    float OutMax;       /*!< 输出上限 */
    float Integral;     /*!< 积分累加器 */
    float PrevErr;      /*!< 上次误差（微分用） */
    float Out;          /*!< 最近一次输出（饱和后） */
} PID_HandleTypeDef;

/* Exported functions prototypes ---------------------------------------------*/
void  PID_Init(PID_HandleTypeDef *hpid, float kp, float ki, float kd, float outMin, float outMax);
void  PID_SetGains(PID_HandleTypeDef *hpid, float kp, float ki, float kd);
void  PID_SetLimits(PID_HandleTypeDef *hpid, float outMin, float outMax);
void  PID_Reset(PID_HandleTypeDef *hpid);
float PID_Update(PID_HandleTypeDef *hpid, float error);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
