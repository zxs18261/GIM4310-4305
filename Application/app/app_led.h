/**
  ******************************************************************************
  * @file    app_led.h
  * @brief   状态灯语模块头文件
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

#ifndef __APP_LED_H
#define __APP_LED_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported functions prototypes ---------------------------------------------*/
void APP_LED_Init(void);
void APP_LED_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_LED_H */
