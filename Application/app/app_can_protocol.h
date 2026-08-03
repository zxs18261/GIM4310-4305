/**
  ******************************************************************************
  * @file    app_can_protocol.h
  * @brief   CAN 应用协议头文件（MIT 运控协议 + 配置协议）
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

#ifndef __APP_CAN_PROTOCOL_H
#define __APP_CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

/** 配置协议命令码（请求帧 data[0]） */
#define CANCFG_CMD_STATUS       (0x01U)     /*!< 读运行状态 */
#define CANCFG_CMD_ENABLE       (0x02U)     /*!< 使能：data[1] = 模式枚举 */
#define CANCFG_CMD_DISABLE      (0x03U)     /*!< 失能 */
#define CANCFG_CMD_CLEARFAULT   (0x04U)     /*!< 清故障 */
#define CANCFG_CMD_CALIBRATE    (0x05U)     /*!< 启动预定位校准 */
#define CANCFG_CMD_SETZERO      (0x06U)     /*!< 设当前位置为零位 */
#define CANCFG_CMD_TARGET       (0x07U)     /*!< 目标值：data[4..7] = float LE，按当前模式解释 */
#define CANCFG_CMD_GETPARAM     (0x10U)     /*!< 读参数：data[1] = 参数索引 */
#define CANCFG_CMD_SETPARAM     (0x11U)     /*!< 写参数：data[1] = 索引，data[4..7] = float LE */
#define CANCFG_CMD_SAVE         (0x12U)     /*!< 保存参数到 Flash */
#define CANCFG_CMD_DEFAULTS     (0x13U)     /*!< 恢复默认参数（不落盘） */

/** 配置协议参数索引 */
#define CANCFG_P_NODEID         (0U)
#define CANCFG_P_CANBAUD        (1U)
#define CANCFG_P_CANTERM        (2U)
#define CANCFG_P_TIMEOUT_MS     (3U)
#define CANCFG_P_SPEED_KP       (4U)
#define CANCFG_P_SPEED_KI       (5U)
#define CANCFG_P_POS_KP         (6U)
#define CANCFG_P_CUR_MAX        (7U)
#define CANCFG_P_SPEED_MAX      (8U)
#define CANCFG_P_LED_BRIGHT     (9U)
#define CANCFG_P_ELEC_OFFSET    (10U)   /*!< 只读 */
#define CANCFG_P_POLE_PAIRS     (11U)   /*!< 只读 */
#define CANCFG_P_ENC_INVERT     (12U)   /*!< 只读 */
#define CANCFG_P_CALIBRATED     (13U)   /*!< 只读 */

/* Exported functions prototypes ---------------------------------------------*/
void    APP_CAN_Init(void);
void    APP_CAN_Poll(void);
void    APP_CAN_RequestReinit(void);
uint8_t APP_CAN_IsCanControlled(void);
void    APP_CAN_SetControlled(uint8_t controlled);
float   APP_CAN_GetParamByIndex(uint8_t index, uint8_t *ok);
uint8_t APP_CAN_SetParamByIndex(uint8_t index, float value);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CAN_PROTOCOL_H */
