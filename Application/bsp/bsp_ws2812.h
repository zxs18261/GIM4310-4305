/**
  ******************************************************************************
  * @file    bsp_ws2812.h
  * @brief   WS2812 类 RGB 状态灯 BSP 头文件（TIM2_CH1 PWM + DMA）
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

#ifndef __BSP_WS2812_H
#define __BSP_WS2812_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/
void    BSP_WS2812_Init(void);
void    BSP_WS2812_SetBrightness(uint8_t brightness);
uint8_t BSP_WS2812_GetBrightness(void);
void    BSP_WS2812_SetColor(uint8_t r, uint8_t g, uint8_t b);
void    BSP_WS2812_Poll(void);

/* 中断服务入口（由 stm32g4xx_it.c 用户区调用） */
void    BSP_WS2812_DmaIRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_WS2812_H */
