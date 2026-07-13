/**
  ******************************************************************************
  * @file    protection.h
  * @brief   保护与故障管理模块头文件
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

#ifndef __PROTECTION_H
#define __PROTECTION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief 故障码枚举
  * @note  数值同时用作 LED 红色闪烁次数与 CAN/shell 上报码，勿改动既有取值。
  */
typedef enum
{
    FAULT_NONE        = 0U,
    FAULT_OVERCURRENT = 1U,     /*!< 相电流超过软件过流阈值 */
    FAULT_OVERVOLT    = 2U,     /*!< 母线过压 */
    FAULT_UNDERVOLT   = 3U,     /*!< 母线欠压 */
    FAULT_MOS_OT      = 4U,     /*!< MOS 过温 */
    FAULT_MOTOR_OT    = 5U,     /*!< 电机绕组过温 */
    FAULT_ENCODER     = 6U,     /*!< 编码器连续错误（CRC/磁场） */
    FAULT_CAL_FAIL    = 7U,     /*!< 预定位校准失败 */
    FAULT_CAN_TIMEOUT = 8U,     /*!< 运行中 CAN 指令超时 */
    FAULT_ADC_OFFSET  = 9U,     /*!< 电流偏置校准越界（采样链异常） */
} PROT_FaultTypeDef;

/* Exported functions prototypes ---------------------------------------------*/
void              PROT_Init(void);
PROT_FaultTypeDef PROT_CheckFast(float iu, float iv, float iw, float vbus, uint8_t encConsErr);
void              PROT_CheckSlow(float mosTemp, float motorTemp);
float             PROT_GetDerating(void);
uint8_t           PROT_IsWarning(void);
void              PROT_FeedCanWatchdog(void);
uint8_t           PROT_CheckCanTimeout(uint32_t timeoutMs);

#ifdef __cplusplus
}
#endif

#endif /* __PROTECTION_H */
