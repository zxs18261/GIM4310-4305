/**
  ******************************************************************************
  * @file    app_monitor.h
  * @brief   调试数据打印模块头文件
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

#ifndef __APP_MONITOR_H
#define __APP_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/
void APP_MON_Init(void);
void APP_MON_Poll(void);
void APP_MON_SetLog(uint8_t enable, uint32_t rateHz);
void APP_MON_PrintStatus(void);
void APP_MON_PrintBanner(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MONITOR_H */
