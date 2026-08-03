/**
  ******************************************************************************
  * @file    calibration.c
  * @brief   预定位校准模块（编码器方向/极对数/电角度偏置）
  * @note    在电流环 ISR 上下文内以状态机方式执行，全程使用 d 轴电流闭环 +
  *          强制电角度（不依赖编码器校准状态），流程：
  *          1. RAMP    ：d 轴电流从 0 缓升至 CAL_CURRENT_A，theta 强制 0；
  *          2. SETTLE  ：静置，转子锁定到强制角，记录编码器基准；
  *          3. ROTATE  ：强制角匀速正向旋转 CAL_ROTATE_EREV 个电角圈，
  *                       由编码器增量判定方向、实测极对数；
  *          4. OFFSET  ：分点锁定（正向/反向各 CAL_OFFSET_POINTS/2 点），
  *                       每点记录 电角(编码器换算) - 强制角 的差，
  *                       正反双向平均以抵消摩擦/齿槽引起的滞后；
  *          5. RAMPDOWN：电流缓降，写入校准结果。
  *          方向反转的处理：ROTATE 结束若编码器反向计数，立即写入反转标志并
  *          重新预置编码器，后续 OFFSET 阶段在新方向语义下测量。
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
#include "calibration.h"
#include "app_params.h"
#include "motor_config.h"
#include "board_config.h"
#include <math.h>

/* Private types -------------------------------------------------------------*/

/**
  * @brief 校准状态机状态枚举
  */
typedef enum
{
    CAL_ST_IDLE = 0U,
    CAL_ST_RAMP,
    CAL_ST_SETTLE,
    CAL_ST_ROTATE,
    CAL_ST_RESETTLE,
    CAL_ST_OFFSET,
    CAL_ST_RAMPDOWN,
    CAL_ST_FAILDOWN,    /*!< 失败路径的电流缓降（与成功路径对称，避免硬封波泵压） */
    CAL_ST_DONE,
    CAL_ST_FAILED,
} CAL_StateTypeDef;

/* Private defines -----------------------------------------------------------*/
#define CAL_RAMP_CYCLES         ((uint32_t)(CAL_RAMP_TIME_S   * (float)BOARD_PWM_FREQ_HZ))
#define CAL_SETTLE_CYCLES       ((uint32_t)(CAL_SETTLE_TIME_S * (float)BOARD_PWM_FREQ_HZ))
#define CAL_POINT_SETTLE_CYCLES ((uint32_t)(0.25f * (float)BOARD_PWM_FREQ_HZ))
#define CAL_ROTATE_TOTAL_RAD    ((float)CAL_ROTATE_EREV * FOC_2PI)
#define CAL_ELEC_PER_COUNT      (FOC_2PI / (float)ENC_CPR)

/* Private variables ---------------------------------------------------------*/
static CAL_StateTypeDef calState = CAL_ST_IDLE;
static uint32_t stateCnt   = 0U;        /*!< 状态内周期计数 */
static float    forcedTheta = 0.0f;     /*!< 强制电角度 */
static float    curRef      = 0.0f;     /*!< d 轴电流给定 */
static int32_t  baseTurns   = 0;        /*!< ROTATE 起始编码器圈数 */
static int32_t  baseRaw     = 0;        /*!< ROTATE 起始编码器单圈计数 */
static float    rotatedRad  = 0.0f;     /*!< ROTATE 已旋转电角度 */
static uint32_t pointIdx    = 0U;       /*!< OFFSET 阶段点序号 */
static float    offsetSinSum = 0.0f;    /*!< 偏置圆平均：sin 累加 */
static float    offsetCosSum = 0.0f;    /*!< 偏置圆平均：cos 累加 */
static uint8_t  measuredPp  = 0U;       /*!< 实测极对数 */
static uint8_t  measuredInvert = 0U;    /*!< 实测方向反转标志 */

/* Private function prototypes -----------------------------------------------*/
static float CAL_OffsetPointTheta(uint32_t idx);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  启动校准状态机
  */
void CAL_Start(void)
{
    calState     = CAL_ST_RAMP;
    stateCnt     = 0U;
    forcedTheta  = 0.0f;
    curRef       = 0.0f;
    rotatedRad   = 0.0f;
    pointIdx     = 0U;
    offsetSinSum = 0.0f;
    offsetCosSum = 0.0f;
    measuredPp   = 0U;
    measuredInvert = BSP_ENC_GetInvert();
}

/**
  * @brief  校准状态机单步（电流环 ISR 调用，模式 = MC_MODE_CALIB）
  * @param  hfoc 电流环句柄（本模块驱动其 d 轴电流 + 强制角）
  * @param  enc  编码器数据（由 MC ISR 每周期更新）
  * @param  iu/iv/iw 三相电流 [A]
  * @param  vbus 母线电压 [V]
  * @retval CAL_RES_RUNNING / CAL_RES_DONE / CAL_RES_FAILED
  */
CAL_ResultTypeDef CAL_Isr(FOC_HandleTypeDef *hfoc, ENC_DataTypeDef *enc,
                          float iu, float iv, float iw, float vbus)
{
    stateCnt++;

    switch (calState)
    {
        case CAL_ST_RAMP:
            /* d 轴电流线性爬升，转子渐进锁定强制角 0 */
            curRef = CAL_CURRENT_A * (float)stateCnt / (float)CAL_RAMP_CYCLES;
            if (stateCnt >= CAL_RAMP_CYCLES)
            {
                curRef   = CAL_CURRENT_A;
                calState = CAL_ST_SETTLE;
                stateCnt = 0U;
            }
            break;

        case CAL_ST_SETTLE:
            if (stateCnt >= CAL_SETTLE_CYCLES)
            {
                baseTurns = enc->Turns;
                baseRaw   = (int32_t)enc->Raw;
                calState  = CAL_ST_ROTATE;
                stateCnt  = 0U;
            }
            break;

        case CAL_ST_ROTATE:
        {
            forcedTheta += CAL_ROTATE_SPEED_ERADS * CTRL_CURRENT_TS;
            rotatedRad  += CAL_ROTATE_SPEED_ERADS * CTRL_CURRENT_TS;
            if (forcedTheta > FOC_2PI)
            {
                forcedTheta -= FOC_2PI;
            }

            if (rotatedRad >= CAL_ROTATE_TOTAL_RAD)
            {
                /* 编码器增量在整数域计算（校准行程内远离 int32 边界） */
                int32_t deltaCounts = (enc->Turns - baseTurns) * (int32_t)ENC_CPR
                                    + ((int32_t)enc->Raw - baseRaw);
                int32_t absDelta    = (deltaCounts < 0) ? -deltaCounts : deltaCounts;
                float   ppf;

                /* 判定 1：编码器必须跟随转动（阈值 = 期望角的 30%） */
                if (absDelta < (int32_t)((float)CAL_ROTATE_EREV * (float)ENC_CPR * 0.3f
                                          / (float)MOTOR_POLE_PAIRS))
                {
                    calState = CAL_ST_FAILDOWN;
                    stateCnt = 0U;
                    break;
                }

                /* 判定 2：实测极对数 = 电角圈数 * CPR / |机械计数增量| */
                ppf = (float)CAL_ROTATE_EREV * (float)ENC_CPR / (float)absDelta;
                measuredPp = (uint8_t)(ppf + 0.5f);
                if ((measuredPp == 0U) || (measuredPp > 40U))
                {
                    calState = CAL_ST_FAILDOWN;
                    stateCnt = 0U;
                    break;
                }

                /* 判定 3：方向。编码器反向计数则翻转方向标志并立即生效，
                   使后续 OFFSET 在"正电角方向 = 编码器增大"语义下测量 */
                if (deltaCounts < 0)
                {
                    measuredInvert ^= 1U;
                    BSP_ENC_SetInvert(measuredInvert);
                    /* 反转后 Raw 语义突变，多圈计数在校准结束后由 MC 重建 */
                }

                calState = CAL_ST_RESETTLE;
                stateCnt = 0U;
                pointIdx = 0U;
                forcedTheta = CAL_OffsetPointTheta(0U);
            }
            break;
        }

        case CAL_ST_RESETTLE:
            /* 方向翻转后重新锁定第一个偏置测量点 */
            if (stateCnt >= CAL_SETTLE_CYCLES)
            {
                calState = CAL_ST_OFFSET;
                stateCnt = 0U;
            }
            break;

        case CAL_ST_OFFSET:
        {
            if (stateCnt >= CAL_POINT_SETTLE_CYCLES)
            {
                /* 记录本点偏置：编码器换算电角 - 强制角（圆平均消回卷） */
                uint32_t elecCounts = ((uint32_t)enc->Raw * (uint32_t)measuredPp) & ENC_CPR_MASK;
                float    elecFromEnc = (float)elecCounts * CAL_ELEC_PER_COUNT;
                float    diff = elecFromEnc - forcedTheta;
                float    s;
                float    c;

                FOC_SinCos(diff, &s, &c);
                offsetSinSum += s;
                offsetCosSum += c;

                pointIdx++;
                if (pointIdx >= CAL_OFFSET_POINTS)
                {
                    g_Params.ElecOffset = atan2f(offsetSinSum, offsetCosSum);
                    g_Params.PolePairs  = measuredPp;
                    g_Params.EncInvert  = measuredInvert;
                    g_Params.Calibrated = 1U;
                    calState = CAL_ST_RAMPDOWN;
                    stateCnt = 0U;
                }
                else
                {
                    forcedTheta = CAL_OffsetPointTheta(pointIdx);
                    stateCnt    = 0U;
                }
            }
            break;
        }

        case CAL_ST_RAMPDOWN:
            curRef = CAL_CURRENT_A * (1.0f - (float)stateCnt / (float)(CAL_RAMP_CYCLES / 2U));
            if (stateCnt >= (CAL_RAMP_CYCLES / 2U))
            {
                curRef   = 0.0f;
                calState = CAL_ST_DONE;
            }
            break;

        case CAL_ST_FAILDOWN:
            /* 失败路径同样缓降电流，避免 3A 锁定态硬封波的电感能量泵压 */
            curRef = CAL_CURRENT_A * (1.0f - (float)stateCnt / (float)(CAL_RAMP_CYCLES / 2U));
            if (stateCnt >= (CAL_RAMP_CYCLES / 2U))
            {
                curRef   = 0.0f;
                calState = CAL_ST_FAILED;
            }
            break;

        case CAL_ST_DONE:
            return CAL_RES_DONE;

        case CAL_ST_FAILED:
        case CAL_ST_IDLE:
        default:
            return CAL_RES_FAILED;
    }

    /* 统一执行 d 轴电流闭环（强制角）。OFFSET 各点间跳变 45 度电角
       （机械约 3.2 度），转子在每点 0.25s 静置窗口内重新锁定 */
    FOC_SetCurrentRef(hfoc, curRef, 0.0f);
    FOC_Update(hfoc, iu, iv, iw, forcedTheta, 0.0f, vbus);

    return CAL_RES_RUNNING;
}

/**
  * @brief  获取当前校准阶段名称（monitor/shell 显示）
  */
const char *CAL_GetPhaseName(void)
{
    switch (calState)
    {
        case CAL_ST_IDLE:     return "idle";
        case CAL_ST_RAMP:     return "ramp";
        case CAL_ST_SETTLE:   return "settle";
        case CAL_ST_ROTATE:   return "rotate";
        case CAL_ST_RESETTLE: return "resettle";
        case CAL_ST_OFFSET:   return "offset";
        case CAL_ST_RAMPDOWN: return "rampdown";
        case CAL_ST_FAILDOWN: return "faildown";
        case CAL_ST_DONE:     return "done";
        case CAL_ST_FAILED:
        default:              return "failed";
    }
}

/**
  * @brief  校准进度估计（0~100）
  */
uint8_t CAL_GetProgress(void)
{
    switch (calState)
    {
        case CAL_ST_IDLE:     return 0U;
        case CAL_ST_RAMP:     return 5U;
        case CAL_ST_SETTLE:   return 15U;
        case CAL_ST_ROTATE:   return (uint8_t)(20U + (uint32_t)(50.0f * rotatedRad / CAL_ROTATE_TOTAL_RAD));
        case CAL_ST_RESETTLE: return 72U;
        case CAL_ST_OFFSET:   return (uint8_t)(75U + (20U * pointIdx) / CAL_OFFSET_POINTS);
        case CAL_ST_RAMPDOWN:
        case CAL_ST_FAILDOWN: return 97U;
        case CAL_ST_DONE:
        case CAL_ST_FAILED:
        default:              return 100U;
    }
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  偏置测量点的强制角序列
  * @note   前半程正向步进、后半程按相同角度反向回访，双向平均抵消
  *         摩擦滞后；步距 = 2pi / (点数/2)。
  */
static float CAL_OffsetPointTheta(uint32_t idx)
{
    uint32_t half = CAL_OFFSET_POINTS / 2U;
    float    step = FOC_2PI / (float)half;
    uint32_t k;

    if (idx < half)
    {
        k = idx;
    }
    else
    {
        k = CAL_OFFSET_POINTS - 1U - idx;   /* 反向回访相同点位 */
    }
    return (float)k * step;
}
