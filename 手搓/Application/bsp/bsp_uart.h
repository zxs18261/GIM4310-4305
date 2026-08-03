/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @brief   USART3 调试串口 BSP 头文件（DMA TX 环形缓冲 + RXNE 中断接收）
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

#ifndef __BSP_UART_H
#define __BSP_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/
void     BSP_UART_Init(void);
uint32_t BSP_UART_Write(const uint8_t *data, uint32_t len);
uint32_t BSP_UART_Printf(const char *fmt, ...);
int32_t  BSP_UART_ReadByte(void);
uint32_t BSP_UART_TxFree(void);
void     BSP_UART_Flush(void);

/* 中断服务入口（由 stm32g4xx_it.c 用户区调用） */
void     BSP_UART_IRQHandler(void);
void     BSP_UART_DmaTxIRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART_H */
