/**
  ******************************************************************************
  * @file    app_main.h
  * @brief   应用入口模块头文件（前后台调度）
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

#ifndef __APP_MAIN_H
#define __APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
#define APP_FW_VERSION          "0.1.0"

/* Exported functions prototypes ---------------------------------------------*/
void  APP_Init(void);
void  APP_Loop(void);
float APP_GetMosTemp(void);
float APP_GetMotorTemp(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MAIN_H */
