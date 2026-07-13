/**
  ******************************************************************************
  * @file    app_params.c
  * @brief   运行参数管理（RAM 副本 + Flash 持久化）
  * @note    - 上电从参数页加载，无有效记录时写入编译期默认值；
  *          - 保存动作只允许在电机封波时执行（页擦除约 20ms 会 stall 取指，
  *            期间电流环停摆），运行中请求以标志缓存、后台择机落盘。
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

/* Includes ------------------------------------------------------------------*/
#include "app_params.h"
#include "bsp_flash.h"
#include "bsp_can.h"
#include "motor_config.h"
#include "board_config.h"

/* Exported variables --------------------------------------------------------*/
APP_ParamsTypeDef g_Params;

/* Private variables ---------------------------------------------------------*/
static volatile uint8_t saveRequested = 0U;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  加载编译期默认参数
  */
void APP_PARAMS_LoadDefaults(void)
{
    g_Params.Version       = PARAMS_VERSION;

    g_Params.NodeId        = 1U;
    g_Params.CanBaud       = (uint8_t)CAN_BAUD_1M;
    g_Params.CanTerm       = 1U;    /* 单机调试默认接入终端电阻 */
    g_Params.CanTimeoutMs  = PROT_CAN_TIMEOUT_MS;
    g_Params.Reserved0     = 0U;

    g_Params.Calibrated    = 0U;
    g_Params.EncInvert     = 0U;
    g_Params.PolePairs     = MOTOR_POLE_PAIRS;
    g_Params.Reserved1     = 0U;
    g_Params.ElecOffset    = 0.0f;

    g_Params.SpeedKp       = CTRL_SPEED_KP;
    g_Params.SpeedKi       = CTRL_SPEED_KI;
    g_Params.PosKp         = CTRL_POS_KP;
    g_Params.CurMax        = MOTOR_CUR_PEAK_A;
    g_Params.SpeedMax      = MOTOR_SPEED_MAX_RADS;

    g_Params.LedBrightness = BOARD_LED_BRIGHTNESS_DEFAULT;
    g_Params.Reserved2[0]  = 0U;
    g_Params.Reserved2[1]  = 0U;
    g_Params.Reserved2[2]  = 0U;
}

/**
  * @brief  参数初始化：从 Flash 加载，失败（首次上电/版本不符）则用默认值
  */
void APP_PARAMS_Init(void)
{
    if (BSP_FLASH_LoadParams(&g_Params, sizeof(g_Params)) != HAL_OK)
    {
        APP_PARAMS_LoadDefaults();
        return;
    }
    if (g_Params.Version != PARAMS_VERSION)
    {
        APP_PARAMS_LoadDefaults();
    }
}

/**
  * @brief  立即保存参数（仅限电机封波状态下调用）
  */
HAL_StatusTypeDef APP_PARAMS_Save(void)
{
    return BSP_FLASH_SaveParams(&g_Params, sizeof(g_Params));
}

/**
  * @brief  请求保存参数（任意上下文安全，后台择机执行）
  */
void APP_PARAMS_RequestSave(void)
{
    saveRequested = 1U;
}

/**
  * @brief  查询是否有挂起的保存请求
  * @retval 1 = 有挂起请求
  */
uint8_t APP_PARAMS_IsSavePending(void)
{
    return saveRequested;
}

/**
  * @brief  处理挂起的保存请求（后台循环调用）
  * @note   "仅封波时落盘"的判定在关中断临界区内直读 TIM1 MOE 复核：
  *         后台粗判与落盘之间存在微秒级窗口，CAN ISR 可能恰好使能电机
  *         （页擦除约 20ms 关中断会令使能中的电流环停摆），复核不通过
  *         则保留请求下轮再试。
  * @param  motorEnabled 当前电机是否使能（粗判）
  * @retval PARAMS_SAVE_NONE / PARAMS_SAVE_OK / PARAMS_SAVE_FAILED
  */
uint8_t APP_PARAMS_ProcessSaveRequest(uint8_t motorEnabled)
{
    uint32_t primask;
    uint8_t  result = PARAMS_SAVE_NONE;

    if ((saveRequested == 0U) || (motorEnabled != 0U))
    {
        return PARAMS_SAVE_NONE;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
    {
        saveRequested = 0U;
        result = (APP_PARAMS_Save() == HAL_OK) ? PARAMS_SAVE_OK : PARAMS_SAVE_FAILED;
    }

    if (primask == 0U)
    {
        __enable_irq();
    }
    return result;
}
