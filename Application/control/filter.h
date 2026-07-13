/**
  ******************************************************************************
  * @file    filter.h
  * @brief   滤波与状态估计模块头文件（一阶低通滤波器、PLL 速度估计器）
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

#ifndef __FILTER_H
#define __FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief 一阶低通滤波器（IIR）句柄
  * @note  y[n] = y[n-1] + alpha * (x[n] - y[n-1])，alpha = Ts / (Ts + 1/(2*pi*fc))。
  *        当前工程仅 PLL 部分被控制层使用，LPF 作为通用件预留
  *        （温度/电流显示滤波等场景）。
  */
typedef struct
{
    float Alpha;        /*!< 滤波系数 (0, 1] */
    float Y;            /*!< 滤波器状态（输出） */
} LPF_HandleTypeDef;

/**
  * @brief PLL 型角度跟踪速度估计器句柄
  * @note  以锁相环结构跟踪输入的单圈角度（回卷感知相位差），输出平滑角速度；
  *        内部跟踪角保持在 [0, 2pi) 回卷域，长时间连续旋转无精度退化/溢出
  *        （相比跟踪连续展开角，float 大数相减的灾难性抵消被规避）。
  *        相比直接差分对编码器量化噪声抑制显著。
  */
typedef struct
{
    float Kp;           /*!< PLL 比例增益（约 2*wn） */
    float Ki;           /*!< PLL 积分增益（约 wn^2），已折算采样周期外部保持连续量纲 */
    float Ts;           /*!< 采样周期 [s] */
    float Theta;        /*!< 跟踪角度 [rad]（回卷 [0, 2pi)，仅内部状态） */
    float Speed;        /*!< 角速度输出 [rad/s] */
} PLL_HandleTypeDef;

/* Exported functions prototypes ---------------------------------------------*/
void  LPF_Init(LPF_HandleTypeDef *hlpf, float fc, float ts);
void  LPF_Preload(LPF_HandleTypeDef *hlpf, float y0);
float LPF_Update(LPF_HandleTypeDef *hlpf, float x);

void  PLL_Init(PLL_HandleTypeDef *hpll, float bandwidth, float ts);
void  PLL_Preload(PLL_HandleTypeDef *hpll, float theta0);
void  PLL_Update(PLL_HandleTypeDef *hpll, float theta);

#ifdef __cplusplus
}
#endif

#endif /* __FILTER_H */
