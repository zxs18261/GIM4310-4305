/**
  ******************************************************************************
  * @file    bsp_pwm.h
  * @brief   TIM1 三相互补 PWM BSP 头文件
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

#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/
void    BSP_PWM_Init(void);
void    BSP_PWM_Enable(void);
void    BSP_PWM_Disable(void);
uint8_t BSP_PWM_IsEnabled(void);
void    BSP_PWM_SetDuty(float dutyU, float dutyV, float dutyW);
void    BSP_PWM_SetDutyZero(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PWM_H */
