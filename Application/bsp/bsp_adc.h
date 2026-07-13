/**
  ******************************************************************************
  * @file    bsp_adc.h
  * @brief   ADC 采样 BSP 头文件（电流/母线电压注入组 + 温度规则组）
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

#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief 一次注入组采样的换算结果（电流环输入）
  */
typedef struct
{
    float Iu;               /*!< U 相电流 [A]（低侧采样窗口内有效） */
    float Iv;               /*!< V 相电流 [A] */
    float Ibus;             /*!< 母线电流 [A] */
    float Vbus;             /*!< 母线电压 [V]（轻度滤波） */
    uint8_t CurSat;         /*!< 1 = 电流通道原始值贴轨（放大器/ADC 饱和，
                                 实际电流超出测量范围，按过流处理） */
} ADC_FocMeasTypeDef;

/**
  * @brief FOC 周期回调类型（JEOS ISR 上下文）
  */
typedef void (*ADC_FocCallbackTypeDef)(const ADC_FocMeasTypeDef *meas);

/* Exported functions prototypes ---------------------------------------------*/
void              BSP_ADC_Init(void);
HAL_StatusTypeDef BSP_ADC_MeasureOffsets(uint32_t samples);
void              BSP_ADC_StartFocIrq(ADC_FocCallbackTypeDef callback);
void              BSP_ADC_GetOffsets(float *offsU, float *offsV, float *offsBus);
void              BSP_ADC_SetOffsets(float offsU, float offsV, float offsBus);
float             BSP_ADC_GetVbus(void);
float             BSP_ADC_ReadMosTemp(void);
float             BSP_ADC_ReadMotorTemp(void);

/* 中断服务入口（由 stm32g4xx_it.c 用户区调用） */
void              BSP_ADC_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ADC_H */
