/**
  ******************************************************************************
  * @file    bsp_flash.h
  * @brief   片内 Flash 参数持久化 BSP 头文件
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

#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef BSP_FLASH_LoadParams(void *buf, uint32_t size);
HAL_StatusTypeDef BSP_FLASH_SaveParams(const void *buf, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_FLASH_H */
