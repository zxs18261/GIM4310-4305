/**
  ******************************************************************************
  * @file    foc_math.c
  * @brief   FOC 数学基础模块（快速三角函数、坐标变换、SVPWM）
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
#include "foc_math.h"

/* Private defines -----------------------------------------------------------*/
#define SIN_TABLE_SIZE          (256U)                 /*!< 查表点数（1/4 周期无压缩，整周期表） */
#define SIN_TABLE_STEP          ((float)SIN_TABLE_SIZE / FOC_2PI)

/* Private variables ---------------------------------------------------------*/
static float sinTable[SIN_TABLE_SIZE + 1];             /*!< 末尾冗余一点便于插值不取模 */
static uint8_t sinTableReady = 0U;

/* Private function prototypes -----------------------------------------------*/
static void FOC_MATH_BuildTable(void);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  查表+线性插值同时计算 sin/cos
  * @note   首次调用自动建表（也可由初始化流程显式触发一次以避免首次开销进 ISR）。
  * @param  theta  角度 [rad]，任意范围
  * @param  sinOut sin 输出
  * @param  cosOut cos 输出
  * @retval None
  */
void FOC_SinCos(float theta, float *sinOut, float *cosOut)
{
    float    pos;
    float    frac;
    uint32_t idxS;
    uint32_t idxC;

    if (sinTableReady == 0U)
    {
        FOC_MATH_BuildTable();
    }

    /* 归一化到 [0, 2pi) 再折算表位置 */
    pos = FOC_NormalizeAngle(theta) * SIN_TABLE_STEP;

    idxS = (uint32_t)pos;
    frac = pos - (float)idxS;
    *sinOut = sinTable[idxS] + frac * (sinTable[idxS + 1U] - sinTable[idxS]);

    /* cos(x) = sin(x + pi/2)：表内偏移 1/4 表长 */
    idxC = idxS + (SIN_TABLE_SIZE / 4U);
    if (idxC >= SIN_TABLE_SIZE)
    {
        idxC -= SIN_TABLE_SIZE;
    }
    *cosOut = sinTable[idxC] + frac * (sinTable[idxC + 1U] - sinTable[idxC]);
}

/**
  * @brief  角度归一化到 [0, 2pi)
  * @param  theta 输入角度 [rad]，任意范围
  * @retval 归一化角度 [rad]
  */
float FOC_NormalizeAngle(float theta)
{
    /* fmodf 开销较大，控制环中角度偏移有限，用减法循环足够快 */
    while (theta >= FOC_2PI)
    {
        theta -= FOC_2PI;
    }
    while (theta < 0.0f)
    {
        theta += FOC_2PI;
    }
    return theta;
}

/**
  * @brief  Clarke 变换（等幅值变换，3 相 -> alpha/beta）
  * @note   使用三相全量形式，容忍三相电流和不严格为零（两相重构+计算第三相时和为零）。
  * @param  ia/ib/ic  三相电流 [A]
  * @param  alphaBeta 输出 alpha/beta
  * @retval None
  */
void FOC_Clarke(float ia, float ib, float ic, FOC_Vec2TypeDef *alphaBeta)
{
    alphaBeta->A = FOC_2_3 * (ia - 0.5f * (ib + ic));
    alphaBeta->B = FOC_2_3 * FOC_SQRT3_2 * (ib - ic);
}

/**
  * @brief  Park 变换（alpha/beta -> d/q）
  */
void FOC_Park(const FOC_Vec2TypeDef *alphaBeta, float sinTheta, float cosTheta, FOC_Vec2TypeDef *dq)
{
    dq->A =  alphaBeta->A * cosTheta + alphaBeta->B * sinTheta;
    dq->B = -alphaBeta->A * sinTheta + alphaBeta->B * cosTheta;
}

/**
  * @brief  Park 反变换（d/q -> alpha/beta）
  */
void FOC_InvPark(const FOC_Vec2TypeDef *dq, float sinTheta, float cosTheta, FOC_Vec2TypeDef *alphaBeta)
{
    alphaBeta->A = dq->A * cosTheta - dq->B * sinTheta;
    alphaBeta->B = dq->A * sinTheta + dq->B * cosTheta;
}

/**
  * @brief  SVPWM 调制（注入零序的 min-max 中点平移法）
  * @note   与七段式空间矢量等效，计算量更小；输出占空比已按 maxDuty 限幅，
  *         maxDuty < 1 用于保证低侧采样窗口（低侧 shunt 方案必须）。
  * @param  vAlphaBeta 目标电压矢量 [V]
  * @param  vbus       母线电压 [V]
  * @param  maxDuty    占空比上限 (0, 1)
  * @param  dutyA/B/C  三相占空比输出 [0, maxDuty]
  * @retval None
  */
void FOC_SVPWM(const FOC_Vec2TypeDef *vAlphaBeta, float vbus, float maxDuty, float *dutyA, float *dutyB, float *dutyC)
{
    float va;
    float vb;
    float vc;
    float vMin;
    float vMax;
    float offset;
    float invVbus;
    float scale;
    float range;

    /* alpha/beta -> 三相瞬时电压（等幅值反 Clarke） */
    va = vAlphaBeta->A;
    vb = -0.5f * vAlphaBeta->A + FOC_SQRT3_2 * vAlphaBeta->B;
    vc = -0.5f * vAlphaBeta->A - FOC_SQRT3_2 * vAlphaBeta->B;

    /* min-max 求零序偏移 */
    vMin = va;
    vMax = va;
    if (vb < vMin)
    {
        vMin = vb;
    }
    if (vb > vMax)
    {
        vMax = vb;
    }
    if (vc < vMin)
    {
        vMin = vc;
    }
    if (vc > vMax)
    {
        vMax = vc;
    }

    /* 矢量超限时等比缩小，保持矢量方向不变。
       占空比经中点平移后为 0.5 +/- range/(2*vbus)，
       约束 range <= 2*(maxDuty-0.5)*vbus 可保证每相占空比在 [1-maxDuty, maxDuty]，
       为低侧 shunt 电流采样保留导通窗口 */
    range = (vMax - vMin);
    if (vbus < 1.0f)
    {
        vbus = 1.0f;    /* 母线电压异常时的除零保护，此时保护逻辑应已封波 */
    }
    if (range > 2.0f * (maxDuty - 0.5f) * vbus)
    {
        scale = 2.0f * (maxDuty - 0.5f) * vbus / range;
        va *= scale;
        vb *= scale;
        vc *= scale;
        vMin *= scale;
        vMax *= scale;
    }

    offset  = -0.5f * (vMin + vMax);
    invVbus = 1.0f / vbus;

    *dutyA = (va + offset) * invVbus + 0.5f;
    *dutyB = (vb + offset) * invVbus + 0.5f;
    *dutyC = (vc + offset) * invVbus + 0.5f;

    /* 数值裕量限幅：前置缩放已保证 duty ∈ [1-maxDuty, maxDuty]，
       此处 0 下界仅为浮点误差兜底（更低的占空比只会加宽低侧窗口，无害） */
    if (*dutyA > maxDuty)
    {
        *dutyA = maxDuty;
    }
    else if (*dutyA < 0.0f)
    {
        *dutyA = 0.0f;
    }
    if (*dutyB > maxDuty)
    {
        *dutyB = maxDuty;
    }
    else if (*dutyB < 0.0f)
    {
        *dutyB = 0.0f;
    }
    if (*dutyC > maxDuty)
    {
        *dutyC = maxDuty;
    }
    else if (*dutyC < 0.0f)
    {
        *dutyC = 0.0f;
    }
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  构建 sin 查找表（仅初始化时执行一次）
  * @note   用递推法避免链接 libm 的 sinf；递推误差在 256 点下可忽略。
  */
static void FOC_MATH_BuildTable(void)
{
    /* 旋转矢量递推：(c,s) 每步乘 e^{j*dTheta}，dTheta = 2pi/256；
       cos/sin(dTheta) 预先算好写死，避免运行期三角函数调用 */
    const float cosD = 0.99969881869620f;
    const float sinD = 0.02454122852291f;
    float c = 1.0f;
    float s = 0.0f;
    float tmp;
    uint32_t i;

    for (i = 0U; i < SIN_TABLE_SIZE; i++)
    {
        sinTable[i] = s;
        tmp = c * cosD - s * sinD;
        s   = c * sinD + s * cosD;
        c   = tmp;
    }
    sinTable[SIN_TABLE_SIZE] = sinTable[0];

    sinTableReady = 1U;
}
