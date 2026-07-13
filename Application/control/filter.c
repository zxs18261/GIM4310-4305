/**
  ******************************************************************************
  * @file    filter.c
  * @brief   滤波与状态估计模块（一阶低通滤波器、PLL 速度估计器）
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
#include "filter.h"
#include "foc_math.h"

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化一阶低通滤波器
  * @param  hlpf 滤波器句柄
  * @param  fc   截止频率 [Hz]
  * @param  ts   采样周期 [s]
  * @retval None
  */
void LPF_Init(LPF_HandleTypeDef *hlpf, float fc, float ts)
{
    float rc = 1.0f / (FOC_2PI * fc);

    hlpf->Alpha = ts / (ts + rc);
    hlpf->Y     = 0.0f;
}

/**
  * @brief  预置滤波器状态，避免起始阶跃
  */
void LPF_Preload(LPF_HandleTypeDef *hlpf, float y0)
{
    hlpf->Y = y0;
}

/**
  * @brief  执行一次滤波
  * @param  hlpf 滤波器句柄
  * @param  x    本次输入
  * @retval 滤波输出
  */
float LPF_Update(LPF_HandleTypeDef *hlpf, float x)
{
    hlpf->Y += hlpf->Alpha * (x - hlpf->Y);
    return hlpf->Y;
}

/**
  * @brief  初始化 PLL 速度估计器
  * @note   增益按二阶系统整定：Kp = 2*zeta*wn（取 zeta=1），Ki = wn^2。
  * @param  hpll      PLL 句柄
  * @param  bandwidth 期望自然频率 wn [rad/s]（一般取速度环带宽的 5~10 倍）
  * @param  ts        采样周期 [s]
  * @retval None
  */
void PLL_Init(PLL_HandleTypeDef *hpll, float bandwidth, float ts)
{
    hpll->Kp    = 2.0f * bandwidth;
    hpll->Ki    = bandwidth * bandwidth;
    hpll->Ts    = ts;
    hpll->Theta = 0.0f;
    hpll->Speed = 0.0f;
}

/**
  * @brief  预置 PLL 跟踪角度（上电对准编码器初值，避免速度冲击）
  * @param  hpll   PLL 句柄
  * @param  theta0 当前单圈角度 [rad]
  * @retval None
  */
void PLL_Preload(PLL_HandleTypeDef *hpll, float theta0)
{
    hpll->Theta = FOC_NormalizeAngle(theta0);
    hpll->Speed = 0.0f;
}

/**
  * @brief  执行一次 PLL 跟踪
  * @note   输入为单圈角度 [0, 2pi)，相位误差回卷到 (-pi, pi] 后进入环路，
  *         跟踪角更新后同样回卷，长时间旋转无累积误差。
  *         可跟踪转速上限由相位误差半圈判据决定（每拍 < pi），
  *         20kHz 采样下约 10000 rad/s，远超本应用。
  * @param  hpll  PLL 句柄
  * @param  theta 本次测量单圈角度 [rad]，[0, 2pi)
  * @retval None（结果读 hpll->Speed）
  */
void PLL_Update(PLL_HandleTypeDef *hpll, float theta)
{
    float err = theta - hpll->Theta;

    /* 相位差回卷到 (-pi, pi] */
    if (err > FOC_PI)
    {
        err -= FOC_2PI;
    }
    else if (err < -FOC_PI)
    {
        err += FOC_2PI;
    }

    hpll->Speed += hpll->Ki * err * hpll->Ts;
    hpll->Theta += (hpll->Speed + hpll->Kp * err) * hpll->Ts;
    hpll->Theta  = FOC_NormalizeAngle(hpll->Theta);
}
