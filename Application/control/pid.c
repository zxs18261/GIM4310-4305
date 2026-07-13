/**
  ******************************************************************************
  * @file    pid.c
  * @brief   位置式抗饱和 PID 控制器模块
  * @note    固定周期调用（增益已折算采样周期），clamping 抗饱和。
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

/* Includes ------------------------------------------------------------------*/
#include "pid.h"

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 PID 控制器
  * @param  hpid   PID 句柄
  * @param  kp     比例增益
  * @param  ki     离散积分增益（连续 Ki * Ts）
  * @param  kd     离散微分增益（连续 Kd / Ts）
  * @param  outMin 输出下限
  * @param  outMax 输出上限
  * @retval None
  */
void PID_Init(PID_HandleTypeDef *hpid, float kp, float ki, float kd, float outMin, float outMax)
{
    hpid->Kp     = kp;
    hpid->Ki     = ki;
    hpid->Kd     = kd;
    hpid->OutMin = outMin;
    hpid->OutMax = outMax;
    PID_Reset(hpid);
}

/**
  * @brief  运行中修改增益（不复位积分，保证无扰切换）
  */
void PID_SetGains(PID_HandleTypeDef *hpid, float kp, float ki, float kd)
{
    hpid->Kp = kp;
    hpid->Ki = ki;
    hpid->Kd = kd;
}

/**
  * @brief  运行中修改输出限幅，并把积分收拢到新限幅内
  */
void PID_SetLimits(PID_HandleTypeDef *hpid, float outMin, float outMax)
{
    hpid->OutMin = outMin;
    hpid->OutMax = outMax;
    if (hpid->Integral > outMax)
    {
        hpid->Integral = outMax;
    }
    else if (hpid->Integral < outMin)
    {
        hpid->Integral = outMin;
    }
}

/**
  * @brief  复位 PID 内部状态（积分/历史误差/输出）
  */
void PID_Reset(PID_HandleTypeDef *hpid)
{
    hpid->Integral = 0.0f;
    hpid->PrevErr  = 0.0f;
    hpid->Out      = 0.0f;
}

/**
  * @brief  执行一次 PID 计算
  * @note   在固定周期任务中调用（电流环 ISR / 降采样速度环）。
  *         抗饱和策略：仅当输出未饱和、或误差方向有助于退出饱和时才累加积分。
  * @param  hpid  PID 句柄
  * @param  error 目标值 - 反馈值
  * @retval 饱和后的控制输出
  */
float PID_Update(PID_HandleTypeDef *hpid, float error)
{
    float out;
    float dTerm;

    /* 微分项（对误差微分，Kd 通常仅位置环使用） */
    dTerm = hpid->Kd * (error - hpid->PrevErr);
    hpid->PrevErr = error;

    out = hpid->Kp * error + hpid->Integral + dTerm;

    if (out > hpid->OutMax)
    {
        /* 正向饱和：仅允许反向误差消解积分 */
        if (error < 0.0f)
        {
            hpid->Integral += hpid->Ki * error;
        }
        out = hpid->OutMax;
    }
    else if (out < hpid->OutMin)
    {
        if (error > 0.0f)
        {
            hpid->Integral += hpid->Ki * error;
        }
        out = hpid->OutMin;
    }
    else
    {
        hpid->Integral += hpid->Ki * error;
        /* 积分自身限幅，防止 Kp 项反号时积分残留过大 */
        if (hpid->Integral > hpid->OutMax)
        {
            hpid->Integral = hpid->OutMax;
        }
        else if (hpid->Integral < hpid->OutMin)
        {
            hpid->Integral = hpid->OutMin;
        }
    }

    hpid->Out = out;
    return out;
}
