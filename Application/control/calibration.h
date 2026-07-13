/**
  ******************************************************************************
  * @file    calibration.h
  * @brief   预定位校准模块头文件（编码器方向/极对数/电角度偏置）
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

#ifndef __CALIBRATION_H
#define __CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "foc.h"
#include "bsp_encoder.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief 校准执行结果枚举
  */
typedef enum
{
    CAL_RES_RUNNING = 0U,   /*!< 进行中 */
    CAL_RES_DONE,           /*!< 成功完成，结果已写入 g_Params */
    CAL_RES_FAILED,         /*!< 失败（电机未转动/极对数异常） */
} CAL_ResultTypeDef;

/* Exported functions prototypes ---------------------------------------------*/
void              CAL_Start(void);
CAL_ResultTypeDef CAL_Isr(FOC_HandleTypeDef *hfoc, ENC_DataTypeDef *enc,
                          float iu, float iv, float iw, float vbus);
const char       *CAL_GetPhaseName(void);
uint8_t           CAL_GetProgress(void);

#ifdef __cplusplus
}
#endif

#endif /* __CALIBRATION_H */
