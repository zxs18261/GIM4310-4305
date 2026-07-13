/**
  ******************************************************************************
  * @file    bsp_can.h
  * @brief   FDCAN1 通信 BSP 头文件（经典 CAN 模式）
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

#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief CAN 波特率枚举（枚举值用于参数持久化，勿改动既有取值）
  */
typedef enum
{
    CAN_BAUD_1M   = 0U,
    CAN_BAUD_500K = 1U,
    CAN_BAUD_250K = 2U,
    CAN_BAUD_125K = 3U,
    CAN_BAUD_COUNT,
} CAN_BaudTypeDef;

/**
  * @brief CAN 接收回调类型（ISR 上下文）
  * @param id   标准帧 ID
  * @param data 数据指针（8 字节缓冲）
  * @param len  数据长度
  */
typedef void (*CAN_RxCallbackTypeDef)(uint32_t id, const uint8_t *data, uint8_t len);

/* Exported functions prototypes ---------------------------------------------*/
void              BSP_CAN_Init(CAN_BaudTypeDef baud, uint16_t nodeId);
void              BSP_CAN_RegisterRxCallback(CAN_RxCallbackTypeDef callback);
HAL_StatusTypeDef BSP_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len);
void              BSP_CAN_SetTermination(uint8_t enable);
uint8_t           BSP_CAN_GetTermination(void);
uint32_t          BSP_CAN_GetRxCount(void);
uint32_t          BSP_CAN_GetErrCount(void);

/* 中断服务入口（由 stm32g4xx_it.c 用户区调用） */
void              BSP_CAN_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CAN_H */
