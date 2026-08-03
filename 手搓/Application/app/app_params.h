/**
  ******************************************************************************
  * @file    app_params.h
  * @brief   运行参数管理头文件（RAM 副本 + Flash 持久化）
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

#ifndef __APP_PARAMS_H
#define __APP_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
#define PARAMS_VERSION          (1UL)   /*!< 结构布局版本，布局变更时递增使旧记录失效 */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief 持久化参数结构体
  * @note  新增字段一律追加到末尾并递增 PARAMS_VERSION。
  */
typedef struct
{
    uint32_t Version;       /*!< = PARAMS_VERSION */

    /* 通信 */
    uint16_t NodeId;        /*!< CAN 节点 ID（1~BOARD_CAN_NODE_ID_MAX，即 1~63） */
    uint8_t  CanBaud;       /*!< CAN_BaudTypeDef 枚举值 */
    uint8_t  CanTerm;       /*!< 1 = 接入 120R 终端电阻 */
    uint16_t CanTimeoutMs;  /*!< 指令超时 [ms]，0 = 关闭 */
    uint16_t Reserved0;

    /* 校准结果 */
    uint8_t  Calibrated;    /*!< 1 = 已完成预定位校准 */
    uint8_t  EncInvert;     /*!< 编码器方向反转（校准写入） */
    uint8_t  PolePairs;     /*!< 极对数（校准实测确认） */
    uint8_t  Reserved1;
    float    ElecOffset;    /*!< 电角度偏置 [rad] */

    /* 控制参数（运行时可调） */
    float    SpeedKp;       /*!< 速度环 Kp [A/(rad/s)] */
    float    SpeedKi;       /*!< 速度环 Ki（连续量纲） */
    float    PosKp;         /*!< 位置环 Kp [1/s] */
    float    CurMax;        /*!< 电流限幅 [A] */
    float    SpeedMax;      /*!< 输出轴速度限幅 [rad/s] */

    /* 界面/指示 */
    uint8_t  LedBrightness; /*!< WS2812 全局亮度 */
    uint8_t  Reserved2[3];
} APP_ParamsTypeDef;

/* Exported variables --------------------------------------------------------*/
extern APP_ParamsTypeDef g_Params;

/* Exported functions prototypes ---------------------------------------------*/
/** ProcessSaveRequest 返回码 */
#define PARAMS_SAVE_NONE        (0U)    /*!< 无请求或条件不满足，未执行 */
#define PARAMS_SAVE_OK          (1U)    /*!< 已执行且校验通过 */
#define PARAMS_SAVE_FAILED      (2U)    /*!< 已执行但擦写/回读校验失败 */

void              APP_PARAMS_Init(void);
void              APP_PARAMS_LoadDefaults(void);
HAL_StatusTypeDef APP_PARAMS_Save(void);
void              APP_PARAMS_RequestSave(void);
uint8_t           APP_PARAMS_IsSavePending(void);
uint8_t           APP_PARAMS_ProcessSaveRequest(uint8_t motorEnabled);

#ifdef __cplusplus
}
#endif

#endif /* __APP_PARAMS_H */
