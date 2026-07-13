/**
  ******************************************************************************
  * @file    protection.c
  * @brief   保护与故障管理模块
  * @note    - 快速检查（电流环 ISR，20kHz）：过流瞬时判定、母线电压带
  *            确认时间判定、编码器连续错误；
  *          - 慢速检查（后台，约 10Hz）：MOS/电机温度，含告警与线性降额；
  *          - CAN 指令看门狗：收到有效指令喂狗，超时由控制层决定动作；
  *          - 本模块只做"判定"，封波与故障锁存动作由 motor_ctrl 执行，
  *            保持判定逻辑可独立测试。
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
#include "protection.h"
#include "motor_config.h"
#include "board_config.h"
#include "main.h"

/* Private defines -----------------------------------------------------------*/
/** 电压越限确认周期数（快速检查 20kHz 执行） */
#define PROT_VBUS_FAULT_CYCLES  ((uint32_t)PROT_VBUS_FAULT_MS * (BOARD_PWM_FREQ_HZ / 1000U))

/* Private variables ---------------------------------------------------------*/
static uint32_t ovCounter = 0U;         /*!< 过压持续计数 */
static uint32_t uvCounter = 0U;         /*!< 欠压持续计数 */
static volatile float   derating = 1.0f;    /*!< 温度降额系数 0~1（后台写，FOC ISR 读） */
static volatile uint8_t warning  = 0U;      /*!< 告警标志（后台写，LED 轮询读） */
static volatile uint32_t canFeedTick = 0U;  /*!< 最近一次有效 CAN 指令时刻 */

/* Private function prototypes -----------------------------------------------*/
static float PROT_DeratingOf(float temp, float warnTemp, float faultTemp);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化保护模块
  */
void PROT_Init(void)
{
    ovCounter   = 0U;
    uvCounter   = 0U;
    derating    = 1.0f;
    warning     = 0U;
    canFeedTick = HAL_GetTick();
}

/**
  * @brief  快速保护检查（电流环 ISR 调用）
  * @param  iu/iv/iw 三相电流 [A]
  * @param  vbus     母线电压 [V]
  * @param  encConsErr 编码器连续错误计数
  * @retval 触发的故障码；FAULT_NONE 表示正常
  */
PROT_FaultTypeDef PROT_CheckFast(float iu, float iv, float iw, float vbus, uint8_t encConsErr)
{
    float absU = (iu < 0.0f) ? -iu : iu;
    float absV = (iv < 0.0f) ? -iv : iv;
    float absW = (iw < 0.0f) ? -iw : iw;

    /* 过流：瞬时判定，任一相超限即触发 */
    if ((absU > PROT_OC_LIMIT_A) || (absV > PROT_OC_LIMIT_A) || (absW > PROT_OC_LIMIT_A))
    {
        return FAULT_OVERCURRENT;
    }

    /* 过压/欠压：带确认时间，滤除瞬时波动（如再生瞬间抬压） */
    if (vbus > PROT_OV_LIMIT_V)
    {
        if (++ovCounter >= PROT_VBUS_FAULT_CYCLES)
        {
            return FAULT_OVERVOLT;
        }
    }
    else
    {
        ovCounter = 0U;
    }

    if (vbus < PROT_UV_LIMIT_V)
    {
        if (++uvCounter >= PROT_VBUS_FAULT_CYCLES)
        {
            return FAULT_UNDERVOLT;
        }
    }
    else
    {
        uvCounter = 0U;
    }

    /* 编码器连续错误 */
    if (encConsErr >= PROT_ENC_ERR_LIMIT)
    {
        return FAULT_ENCODER;
    }

    return FAULT_NONE;
}

/**
  * @brief  慢速保护检查（后台约 10Hz 调用）
  * @note   更新温度降额系数与告警标志；过温故障由返回的降额系数=0 体现，
  *          由调用方（app 层）触发 FAULT_MOS_OT/FAULT_MOTOR_OT。
  * @param  mosTemp   MOS 温度 [C]
  * @param  motorTemp 电机绕组温度 [C]
  */
void PROT_CheckSlow(float mosTemp, float motorTemp)
{
    float dMos = PROT_DeratingOf(mosTemp, PROT_MOS_WARN_C, PROT_MOS_OT_LIMIT_C);
    float dMot = PROT_DeratingOf(motorTemp, PROT_MOT_WARN_C, PROT_MOT_OT_LIMIT_C);

    derating = (dMos < dMot) ? dMos : dMot;
    warning  = (derating < 1.0f) ? 1U : 0U;
}

/**
  * @brief  读取温度降额系数（乘到电流限幅上；0 表示已达过温故障）
  */
float PROT_GetDerating(void)
{
    return derating;
}

/**
  * @brief  是否处于告警区（LED 黄色显示用）
  */
uint8_t PROT_IsWarning(void)
{
    return warning;
}

/**
  * @brief  喂 CAN 指令看门狗（收到有效运动指令时调用）
  */
void PROT_FeedCanWatchdog(void)
{
    canFeedTick = HAL_GetTick();
}

/**
  * @brief  检查 CAN 指令超时
  * @param  timeoutMs 超时时间 [ms]，0 = 关闭检测
  * @retval 1 = 已超时
  */
uint8_t PROT_CheckCanTimeout(uint32_t timeoutMs)
{
    if (timeoutMs == 0U)
    {
        return 0U;
    }
    return ((HAL_GetTick() - canFeedTick) > timeoutMs) ? 1U : 0U;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  线性降额：温度低于告警点为 1，达到故障点为 0
  */
static float PROT_DeratingOf(float temp, float warnTemp, float faultTemp)
{
    if (temp <= warnTemp)
    {
        return 1.0f;
    }
    if (temp >= faultTemp)
    {
        return 0.0f;
    }
    return (faultTemp - temp) / (faultTemp - warnTemp);
}
