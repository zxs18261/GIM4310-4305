/**
  ******************************************************************************
  * @file    foc.c
  * @brief   FOC 电流环模块
  * @note    电流 PI 按内模法整定：Kp = Ls*wc，Ki = Rs*wc（离散化乘 Ts），
  *          电压限幅按母线电压动态计算并做 d 轴优先的圆限幅。
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
#include "foc.h"

/* Private function prototypes -----------------------------------------------*/
static float FOC_FastSqrt(float x);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 FOC 电流环
  * @param  hfoc FOC 句柄
  * @param  init 初始化参数
  * @retval None
  */
void FOC_Init(FOC_HandleTypeDef *hfoc, const FOC_InitTypeDef *init)
{
    float kp = init->Ls * init->CurrentBandwidth;
    float ki = init->Rs * init->CurrentBandwidth * init->Ts;

    /* 输出限幅先给标称值，Update 中按实际 vbus 动态刷新 */
    PID_Init(&hfoc->PidId, kp, ki, 0.0f, -1.0f, 1.0f);
    PID_Init(&hfoc->PidIq, kp, ki, 0.0f, -1.0f, 1.0f);

    hfoc->Ls             = init->Ls;
    hfoc->Flux           = init->Flux;
    hfoc->Ts             = init->Ts;
    hfoc->MaxDuty        = init->MaxDuty;
    hfoc->DecoupleEnable = init->DecoupleEnable;
    hfoc->IdRef          = 0.0f;
    hfoc->IqRef          = 0.0f;

    FOC_Reset(hfoc);
}

/**
  * @brief  复位电流环状态（封波后重新使能前调用，防止积分残留冲击）
  */
void FOC_Reset(FOC_HandleTypeDef *hfoc)
{
    PID_Reset(&hfoc->PidId);
    PID_Reset(&hfoc->PidIq);
    hfoc->IdRef = 0.0f;
    hfoc->IqRef = 0.0f;
    hfoc->Vdq.A = 0.0f;
    hfoc->Vdq.B = 0.0f;
    hfoc->DutyA = 0.0f;
    hfoc->DutyB = 0.0f;
    hfoc->DutyC = 0.0f;
}

/**
  * @brief  设置 d/q 电流给定
  * @note   可在任意上下文调用；float 写入在 M4 上为单指令原子。
  */
void FOC_SetCurrentRef(FOC_HandleTypeDef *hfoc, float idRef, float iqRef)
{
    hfoc->IdRef = idRef;
    hfoc->IqRef = iqRef;
}

/**
  * @brief  执行一个电流环周期（在 ADC 注入转换完成 ISR 中调用）
  * @param  hfoc   FOC 句柄
  * @param  ia/ib/ic 三相电流 [A]（电机相序 U/V/W）
  * @param  thetaE 电角度 [rad]
  * @param  we     电角速度 [rad/s]（解耦前馈用，可传 0 关闭效果）
  * @param  vbus   母线电压 [V]
  * @retval None（占空比结果读 hfoc->DutyA/B/C）
  */
void FOC_Update(FOC_HandleTypeDef *hfoc, float ia, float ib, float ic,
                float thetaE, float we, float vbus)
{
    FOC_Vec2TypeDef vAlphaBeta;
    float sinT;
    float cosT;
    float vMax;
    float vd;
    float vq;
    float vqLim;

    FOC_SinCos(thetaE, &sinT, &cosT);

    /* 电流测量变换到 d/q */
    FOC_Clarke(ia, ib, ic, &hfoc->IAlphaBeta);
    FOC_Park(&hfoc->IAlphaBeta, sinT, cosT, &hfoc->Idq);

    /* 线性调制区最大相电压幅值：vbus/sqrt(3) 再乘占空比余量 */
    vMax = vbus * FOC_1_SQRT3 * 2.0f * (hfoc->MaxDuty - 0.5f);

    /* d 轴优先动态限幅 */
    PID_SetLimits(&hfoc->PidId, -vMax, vMax);
    vd = PID_Update(&hfoc->PidId, hfoc->IdRef - hfoc->Idq.A);

    vqLim = FOC_FastSqrt(vMax * vMax - vd * vd);
    PID_SetLimits(&hfoc->PidIq, -vqLim, vqLim);
    vq = PID_Update(&hfoc->PidIq, hfoc->IqRef - hfoc->Idq.B);

    /* d/q 交叉解耦 + 反电动势前馈：vd -= we*Ls*iq，vq += we*(Ls*id + Flux) */
    if (hfoc->DecoupleEnable != 0U)
    {
        vd -= we * hfoc->Ls * hfoc->Idq.B;
        vq += we * (hfoc->Ls * hfoc->Idq.A + hfoc->Flux);
        /* 前馈叠加后再次限幅，保证不超线性调制区 */
        if (vd > vMax)
        {
            vd = vMax;
        }
        else if (vd < -vMax)
        {
            vd = -vMax;
        }
        if (vq > vqLim)
        {
            vq = vqLim;
        }
        else if (vq < -vqLim)
        {
            vq = -vqLim;
        }
    }

    hfoc->Vdq.A = vd;
    hfoc->Vdq.B = vq;

    /* 输出角度前馈补偿：占空比本周期计算、下周期生效，平均延迟约 1.5Ts，
       高速时电压矢量滞后可达十几度电角（测量侧 Park 仍用当拍角度）。
       输出侧按 thetaE + we*1.5Ts 超前旋转 */
    {
        float sinO;
        float cosO;

        FOC_SinCos(thetaE + we * 1.5f * hfoc->Ts, &sinO, &cosO);
        FOC_InvPark(&hfoc->Vdq, sinO, cosO, &vAlphaBeta);
    }
    FOC_SVPWM(&vAlphaBeta, vbus, hfoc->MaxDuty, &hfoc->DutyA, &hfoc->DutyB, &hfoc->DutyC);
}

/**
  * @brief  直接输出 d/q 电压矢量（开环模式 / 校准注入用，不经过电流 PI）
  * @param  hfoc   FOC 句柄
  * @param  vd/vq  d/q 电压 [V]
  * @param  thetaE 电角度 [rad]
  * @param  vbus   母线电压 [V]
  * @retval None（占空比结果读 hfoc->DutyA/B/C）
  */
void FOC_OutputVoltage(FOC_HandleTypeDef *hfoc, float vd, float vq,
                       float thetaE, float vbus)
{
    FOC_Vec2TypeDef vAlphaBeta;
    float sinT;
    float cosT;

    FOC_SinCos(thetaE, &sinT, &cosT);

    hfoc->Vdq.A = vd;
    hfoc->Vdq.B = vq;

    FOC_InvPark(&hfoc->Vdq, sinT, cosT, &vAlphaBeta);
    FOC_SVPWM(&vAlphaBeta, vbus, hfoc->MaxDuty, &hfoc->DutyA, &hfoc->DutyB, &hfoc->DutyC);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  快速平方根（M4F 上编译为 VSQRT 单指令）
  * @note   负输入（数值误差导致）返回 0。
  */
static float FOC_FastSqrt(float x)
{
    if (x <= 0.0f)
    {
        return 0.0f;
    }
#if defined(__CC_ARM)
    /* AC5：__sqrtf 编译器内建 */
    return __sqrtf(x);
#else
    /* AC6 / GCC：GNU 风格内联汇编 */
    {
        float result;
        __asm volatile ("vsqrt.f32 %0, %1" : "=t" (result) : "t" (x));
        return result;
    }
#endif
}
