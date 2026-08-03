/**
  ******************************************************************************
  * @file    foc_math.h
  * @brief   FOC 数学基础模块头文件（快速三角函数、坐标变换、角度工具）
  * @note    面向 20kHz 电流环 ISR 的确定性开销实现：sin/cos 采用查表+线性插值，
  *          单次 SinCos 约数十个 cycle，远快于 libm 的 sinf/cosf。
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

#ifndef __FOC_MATH_H
#define __FOC_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
#define FOC_PI                  (3.14159265358979f)
#define FOC_2PI                 (6.28318530717959f)
#define FOC_PI_2                (1.57079632679490f)
#define FOC_1_SQRT3             (0.57735026918963f)   /*!< 1/sqrt(3) */
#define FOC_SQRT3_2             (0.86602540378444f)   /*!< sqrt(3)/2 */
#define FOC_2_3                 (0.66666666666667f)   /*!< 2/3 */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief 两相正交量（alpha-beta 或 d-q）
  */
typedef struct
{
    float A;            /*!< alpha 或 d 分量 */
    float B;            /*!< beta 或 q 分量 */
} FOC_Vec2TypeDef;

/* Exported functions prototypes ---------------------------------------------*/
void  FOC_SinCos(float theta, float *sinOut, float *cosOut);
float FOC_NormalizeAngle(float theta);
void  FOC_Clarke(float ia, float ib, float ic, FOC_Vec2TypeDef *alphaBeta);
void  FOC_Park(const FOC_Vec2TypeDef *alphaBeta, float sinTheta, float cosTheta, FOC_Vec2TypeDef *dq);
void  FOC_InvPark(const FOC_Vec2TypeDef *dq, float sinTheta, float cosTheta, FOC_Vec2TypeDef *alphaBeta);
void  FOC_SVPWM(const FOC_Vec2TypeDef *vAlphaBeta, float vbus, float maxDuty, float *dutyA, float *dutyB, float *dutyC);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_MATH_H */
