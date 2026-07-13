/**
  ******************************************************************************
  * @file    bsp_encoder.h
  * @brief   MT6701 磁编码器 BSP 头文件（GPIO 位带模拟 SSI）
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

#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
#define ENC_CPR                 (16384U)    /*!< MT6701 单圈计数：14bit */
#define ENC_CPR_MASK            (ENC_CPR - 1U)

/* Exported types ------------------------------------------------------------*/

/**
  * @brief 编码器读取状态枚举
  */
typedef enum
{
    ENC_OK = 0U,            /*!< 读取有效 */
    ENC_ERR_CRC,            /*!< CRC 校验失败 */
    ENC_ERR_FIELD,          /*!< 磁场强度异常（过强/过弱） */
    ENC_ERR_OVERSPEED,      /*!< 超速标志置位（角度仍有效） */
} ENC_StatusTypeDef;

/**
  * @brief 编码器运行数据结构体
  * @note  多圈信息保持在整数域（Turns int32，±21 亿圈不溢出），
  *        不提供合成的连续计数（int32 合成计数在高速下约 1 小时即溢出，
  *        浮点合成在大圈数下损失单 count 分辨率）——由使用方按
  *        Turns 与 Raw 分离计算。
  */
typedef struct
{
    uint16_t Raw;           /*!< 本次原始单圈计数 0~16383（已按方向设置矫正） */
    int32_t  Turns;         /*!< 多圈计数 */
    uint8_t  MgStatus;      /*!< 原始磁场状态位 Mg[3:0] */
    ENC_StatusTypeDef Status;   /*!< 本次读取状态 */
    uint32_t CrcErrCnt;     /*!< CRC 错误累计 */
    uint32_t FieldErrCnt;   /*!< 磁场异常累计 */
    uint8_t  ConsecutiveErr;/*!< 连续错误计数（判故障用） */
} ENC_DataTypeDef;

/* Exported functions prototypes ---------------------------------------------*/
void              BSP_ENC_Init(void);
void              BSP_ENC_SetInvert(uint8_t invert);
uint8_t           BSP_ENC_GetInvert(void);
ENC_StatusTypeDef BSP_ENC_Read(ENC_DataTypeDef *data);
void              BSP_ENC_Preload(ENC_DataTypeDef *data);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ENCODER_H */
