/**
  ******************************************************************************
  * @file    bsp_clock.h
  * @brief   时钟切换 BSP 头文件（HSI -> HSE 20MHz）
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

#ifndef __BSP_CLOCK_H
#define __BSP_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef BSP_CLOCK_SwitchToHse(void);
uint8_t           BSP_CLOCK_IsHse(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CLOCK_H */
