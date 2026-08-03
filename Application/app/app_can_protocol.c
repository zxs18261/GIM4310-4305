/**
  ******************************************************************************
  * @file    app_can_protocol.c
  * @brief   CAN 应用协议（MIT 运控协议 + 配置协议）
  * @note    协议分两个通道，均为经典 CAN 标准帧：
  *
  *          【MIT 运控通道】CAN ID = NodeId（兼容 SteadyWin GIM/mini-cheetah）：
  *          - 命令帧 8 字节：P[16] V[12] KP[12] KD[12] T[12]，高位先行，
  *            量程 P±12.5rad / V±65rad/s / KP 0~500 / KD 0~5 / T±18Nm
  *            （输出轴量纲，见 motor_config.h）；
  *          - 特殊帧：FF..FF FC 进入电机模式（MIT），FF..FF FD 退出，
  *            FF..FF FE 设当前位置为零位；
  *          - 反馈帧 6 字节发往 ID 0x000（主机）：B0=NodeId，P[16] V[12] T[12]；
  *            每收到一条有效命令帧回发一条。
  *
  *          【配置通道】请求 ID = 0x600+NodeId，应答 ID = 0x680+NodeId：
  *          命令码见头文件，覆盖 模式切换/使能/校准/调参/保存 等管理操作，
  *          浮点均为小端 float32。
  *
  *          回调运行于 CAN ISR 上下文：仅做解析与目标值写入（轻量），
  *          耗时操作（保存参数等）置标志由后台执行。
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
#include "app_can_protocol.h"
#include "app_params.h"
#include "motor_ctrl.h"
#include "bsp_can.h"
#include "board_config.h"
#include "motor_config.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define MIT_SPECIAL_ENTER       (0xFCU)
#define MIT_SPECIAL_EXIT        (0xFDU)
#define MIT_SPECIAL_ZERO        (0xFEU)

/* Private variables ---------------------------------------------------------*/
static volatile uint8_t reinitRequest = 0U;     /*!< 通信参数变更后重新初始化请求 */
static volatile uint8_t canControlled = 0U;     /*!< 1 = 当前使能由 CAN 发起（指令超时保护适用） */

/* Private function prototypes -----------------------------------------------*/
static void  APP_CAN_RxHandler(uint32_t id, const uint8_t *data, uint8_t len);
static void  APP_CAN_HandleMit(const uint8_t *data, uint8_t len, uint8_t isBroadcast);
static void  APP_CAN_HandleCfg(const uint8_t *data, uint8_t len);
static void  APP_CAN_SendMitFeedback(void);
static void  APP_CAN_CfgReply(uint8_t cmd, uint8_t status, float value);
static uint16_t MIT_FloatToUint(float x, float xMin, float xMax, uint8_t bits);
static float    MIT_UintToFloat(uint16_t x, float xMin, float xMax, uint8_t bits);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 CAN 应用协议（按持久化参数配置 BSP）
  */
void APP_CAN_Init(void)
{
    uint16_t nodeId = g_Params.NodeId;

    if ((nodeId == 0U) || (nodeId > BOARD_CAN_NODE_ID_MAX))
    {
        nodeId = 1U;
        g_Params.NodeId = nodeId;
    }

    BSP_CAN_Init((CAN_BaudTypeDef)g_Params.CanBaud, nodeId);
    BSP_CAN_SetTermination(g_Params.CanTerm);
    BSP_CAN_RegisterRxCallback(APP_CAN_RxHandler);
}

/**
  * @brief  后台轮询：处理通信参数变更后的重新初始化
  * @note   波特率/节点 ID 修改不能在 ISR 内重配 FDCAN，延迟到后台执行。
  */
void APP_CAN_Poll(void)
{
    if (reinitRequest != 0U)
    {
        reinitRequest = 0U;
        APP_CAN_Init();
    }
}

/**
  * @brief  请求后台重新初始化 CAN（shell 恢复默认参数等场景）
  */
void APP_CAN_RequestReinit(void)
{
    reinitRequest = 1U;
}

/**
  * @brief  查询当前使能是否由 CAN 发起（指令超时保护范围判定）
  */
uint8_t APP_CAN_IsCanControlled(void)
{
    return canControlled;
}

/**
  * @brief  设置 CAN 控制标志（shell 使能时清零，豁免超时保护）
  */
void APP_CAN_SetControlled(uint8_t controlled)
{
    canControlled = (controlled != 0U) ? 1U : 0U;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CAN 接收分发（ISR 上下文）
  */
static void APP_CAN_RxHandler(uint32_t id, const uint8_t *data, uint8_t len)
{
    if ((id == g_Params.NodeId) || (id == BOARD_CAN_BROADCAST_ID))
    {
        APP_CAN_HandleMit(data, len, (uint8_t)(id == BOARD_CAN_BROADCAST_ID));
    }
    else if (id == (uint32_t)(BOARD_CAN_CFG_ID_BASE + g_Params.NodeId))
    {
        APP_CAN_HandleCfg(data, len);
    }
    else
    {
        /* 硬件过滤器已排除其余 ID，此分支正常不可达 */
    }
}

/**
  * @brief  MIT 运控通道处理
  * @param  isBroadcast 1 = 帧来自广播 ID：仅执行特殊帧且不回发反馈
  *         （多节点同时以 0x000 回发会产生仲裁冲突），常规命令帧忽略
  */
static void APP_CAN_HandleMit(const uint8_t *data, uint8_t len, uint8_t isBroadcast)
{
    if (len != 8U)
    {
        return;
    }

    /* 特殊帧：FF FF FF FF FF FF FF (FC|FD|FE) */
    if ((data[0] == 0xFFU) && (data[1] == 0xFFU) && (data[2] == 0xFFU) &&
        (data[3] == 0xFFU) && (data[4] == 0xFFU) && (data[5] == 0xFFU) &&
        (data[6] == 0xFFU))
    {
        switch (data[7])
        {
            case MIT_SPECIAL_ENTER:
                if (g_Mc.Enabled == 0U)
                {
                    if (MC_SetMode(MC_MODE_MIT) == HAL_OK)
                    {
                        if (MC_Enable() == HAL_OK)
                        {
                            canControlled = 1U;
                        }
                    }
                }
                break;

            case MIT_SPECIAL_EXIT:
                MC_Disable();
                (void)MC_SetMode(MC_MODE_DISABLED);
                canControlled = 0U;
                break;

            case MIT_SPECIAL_ZERO:
                MC_SetZeroHere();
                break;

            default:
                return;     /* 非法特殊帧不回发 */
        }
        if (isBroadcast == 0U)
        {
            APP_CAN_SendMitFeedback();
        }
        return;
    }

    /* 常规命令帧：仅本节点 ID 且 MIT 模式接受 */
    if ((isBroadcast == 0U) && (g_Mc.Mode == MC_MODE_MIT))
    {
        float p  = MIT_UintToFloat((uint16_t)(((uint16_t)data[0] << 8) | data[1]),
                                   -MIT_P_MAX, MIT_P_MAX, 16U);
        float v  = MIT_UintToFloat((uint16_t)(((uint16_t)data[2] << 4) | (data[3] >> 4)),
                                   -MIT_V_MAX, MIT_V_MAX, 12U);
        float kp = MIT_UintToFloat((uint16_t)((((uint16_t)data[3] & 0x0FU) << 8) | data[4]),
                                   0.0f, MIT_KP_MAX, 12U);
        float kd = MIT_UintToFloat((uint16_t)(((uint16_t)data[5] << 4) | (data[6] >> 4)),
                                   0.0f, MIT_KD_MAX, 12U);
        float t  = MIT_UintToFloat((uint16_t)((((uint16_t)data[6] & 0x0FU) << 8) | data[7]),
                                   -MIT_T_MAX, MIT_T_MAX, 12U);

        MC_SetMitCommand(p, v, kp, kd, t);
        PROT_FeedCanWatchdog();
        APP_CAN_SendMitFeedback();
    }
}

/**
  * @brief  发送 MIT 反馈帧（B0=ID，P16 V12 T12，发往主机 ID）
  */
static void APP_CAN_SendMitFeedback(void)
{
    uint8_t  buf[6];
    uint16_t p = MIT_FloatToUint(g_Mc.Pos, -MIT_P_MAX, MIT_P_MAX, 16U);
    uint16_t v = MIT_FloatToUint(g_Mc.Speed, -MIT_V_MAX, MIT_V_MAX, 12U);
    /* 转矩估计 = q 轴电流 * 输出轴转矩常数 */
    uint16_t t = MIT_FloatToUint(g_Mc.Foc.Idq.B * MOTOR_KT_OUT, -MIT_T_MAX, MIT_T_MAX, 12U);

    buf[0] = (uint8_t)g_Params.NodeId;
    buf[1] = (uint8_t)(p >> 8);
    buf[2] = (uint8_t)p;
    buf[3] = (uint8_t)(v >> 4);
    buf[4] = (uint8_t)(((v & 0x0FU) << 4) | (t >> 8));
    buf[5] = (uint8_t)t;

    (void)BSP_CAN_Send(BOARD_CAN_MASTER_ID, buf, 6U);
}

/**
  * @brief  配置通道处理
  */
static void APP_CAN_HandleCfg(const uint8_t *data, uint8_t len)
{
    float   value;
    uint8_t ok;

    if (len < 1U)
    {
        return;
    }

    switch (data[0])
    {
        case CANCFG_CMD_STATUS:
        {
            uint8_t buf[8];
            int16_t pos100 = (int16_t)(g_Mc.Pos * 100.0f);
            int16_t spd100 = (int16_t)(g_Mc.Speed * 100.0f);

            buf[0] = CANCFG_CMD_STATUS;
            buf[1] = (uint8_t)g_Mc.Mode;
            buf[2] = (uint8_t)g_Mc.Fault;
            buf[3] = (uint8_t)((g_Mc.Enabled != 0U ? 0x01U : 0x00U) |
                               (g_Params.Calibrated != 0U ? 0x02U : 0x00U));
            buf[4] = (uint8_t)(pos100 >> 8);
            buf[5] = (uint8_t)pos100;
            buf[6] = (uint8_t)(spd100 >> 8);
            buf[7] = (uint8_t)spd100;
            (void)BSP_CAN_Send((uint32_t)(BOARD_CAN_CFG_REPLY_BASE + g_Params.NodeId), buf, 8U);
            break;
        }

        case CANCFG_CMD_ENABLE:
            ok = 1U;
            if (len < 2U)
            {
                ok = 0U;
            }
            else
            {
                if (MC_SetMode((MC_ModeTypeDef)data[1]) != HAL_OK)
                {
                    ok = 0U;
                }
                else if (MC_Enable() != HAL_OK)
                {
                    ok = 0U;
                }
                else
                {
                    canControlled = 1U;
                }
            }
            APP_CAN_CfgReply(CANCFG_CMD_ENABLE, ok, 0.0f);
            break;

        case CANCFG_CMD_DISABLE:
            MC_Disable();
            (void)MC_SetMode(MC_MODE_DISABLED);
            canControlled = 0U;
            APP_CAN_CfgReply(CANCFG_CMD_DISABLE, 1U, 0.0f);
            break;

        case CANCFG_CMD_CLEARFAULT:
            APP_CAN_CfgReply(CANCFG_CMD_CLEARFAULT,
                             (MC_ClearFault() == HAL_OK) ? 1U : 0U, 0.0f);
            break;

        case CANCFG_CMD_CALIBRATE:
            APP_CAN_CfgReply(CANCFG_CMD_CALIBRATE,
                             (MC_StartCalibration() == HAL_OK) ? 1U : 0U, 0.0f);
            break;

        case CANCFG_CMD_SETZERO:
            MC_SetZeroHere();
            APP_CAN_CfgReply(CANCFG_CMD_SETZERO, 1U, 0.0f);
            break;

        case CANCFG_CMD_TARGET:
            ok = 0U;
            value = 0.0f;
            if (len == 8U)
            {
                memcpy(&value, &data[4], 4U);
                ok = 1U;
                switch (g_Mc.Mode)
                {
                    case MC_MODE_TORQUE:   MC_SetTorque(value);   break;
                    case MC_MODE_SPEED:    MC_SetSpeedRef(value); break;
                    case MC_MODE_POSITION: MC_SetPosRef(value);   break;
                    default:               ok = 0U;               break;
                }
                if (ok != 0U)
                {
                    PROT_FeedCanWatchdog();
                }
            }
            APP_CAN_CfgReply(CANCFG_CMD_TARGET, ok, value);
            break;

        case CANCFG_CMD_GETPARAM:
            ok = 0U;
            value = 0.0f;
            if (len >= 2U)
            {
                value = APP_CAN_GetParamByIndex(data[1], &ok);
            }
            APP_CAN_CfgReply(CANCFG_CMD_GETPARAM, ok, value);
            break;

        case CANCFG_CMD_SETPARAM:
            ok = 0U;
            value = 0.0f;
            if (len == 8U)
            {
                memcpy(&value, &data[4], 4U);
                ok = APP_CAN_SetParamByIndex(data[1], value);
            }
            APP_CAN_CfgReply(CANCFG_CMD_SETPARAM, ok, value);
            break;

        case CANCFG_CMD_SAVE:
            APP_PARAMS_RequestSave();
            APP_CAN_CfgReply(CANCFG_CMD_SAVE, 1U, 0.0f);
            break;

        case CANCFG_CMD_DEFAULTS:
            if (g_Mc.Enabled == 0U)
            {
                APP_PARAMS_LoadDefaults();
                MC_ApplyPidParams();
                /* 先用旧配置回帧，再请求按新 NodeId/波特率/终端重配
                   （否则默认值覆盖 NodeId 后节点在旧滤波配置下失联） */
                APP_CAN_CfgReply(CANCFG_CMD_DEFAULTS, 1U, 0.0f);
                reinitRequest = 1U;
            }
            else
            {
                APP_CAN_CfgReply(CANCFG_CMD_DEFAULTS, 0U, 0.0f);
            }
            break;

        default:
            APP_CAN_CfgReply(data[0], 0U, 0.0f);
            break;
    }
}

/**
  * @brief  配置通道应答：cmd + 状态 + float 值
  */
static void APP_CAN_CfgReply(uint8_t cmd, uint8_t status, float value)
{
    uint8_t buf[8] = {0};

    buf[0] = cmd;
    buf[1] = status;
    memcpy(&buf[4], &value, 4U);
    (void)BSP_CAN_Send((uint32_t)(BOARD_CAN_CFG_REPLY_BASE + g_Params.NodeId), buf, 8U);
}

/**
  * @brief  按索引读参数（CAN 配置通道与 shell 共用）
  */
float APP_CAN_GetParamByIndex(uint8_t index, uint8_t *ok)
{
    *ok = 1U;
    switch (index)
    {
        case CANCFG_P_NODEID:      return (float)g_Params.NodeId;
        case CANCFG_P_CANBAUD:     return (float)g_Params.CanBaud;
        case CANCFG_P_CANTERM:     return (float)g_Params.CanTerm;
        case CANCFG_P_TIMEOUT_MS:  return (float)g_Params.CanTimeoutMs;
        case CANCFG_P_SPEED_KP:    return g_Params.SpeedKp;
        case CANCFG_P_SPEED_KI:    return g_Params.SpeedKi;
        case CANCFG_P_POS_KP:      return g_Params.PosKp;
        case CANCFG_P_CUR_MAX:     return g_Params.CurMax;
        case CANCFG_P_SPEED_MAX:   return g_Params.SpeedMax;
        case CANCFG_P_LED_BRIGHT:  return (float)g_Params.LedBrightness;
        case CANCFG_P_ELEC_OFFSET: return g_Params.ElecOffset;
        case CANCFG_P_POLE_PAIRS:  return (float)g_Params.PolePairs;
        case CANCFG_P_ENC_INVERT:  return (float)g_Params.EncInvert;
        case CANCFG_P_CALIBRATED:  return (float)g_Params.Calibrated;
        default:
            *ok = 0U;
            return 0.0f;
    }
}

/**
  * @brief  按索引写参数（只读项与非法值拒绝，CAN 配置通道与 shell 共用）
  */
uint8_t APP_CAN_SetParamByIndex(uint8_t index, float value)
{
    switch (index)
    {
        case CANCFG_P_NODEID:
            if ((value < 1.0f) || (value > (float)BOARD_CAN_NODE_ID_MAX))
            {
                return 0U;
            }
            g_Params.NodeId = (uint16_t)value;
            reinitRequest = 1U;
            return 1U;

        case CANCFG_P_CANBAUD:
            if (value >= (float)CAN_BAUD_COUNT)
            {
                return 0U;
            }
            g_Params.CanBaud = (uint8_t)value;
            reinitRequest = 1U;
            return 1U;

        case CANCFG_P_CANTERM:
            g_Params.CanTerm = (value != 0.0f) ? 1U : 0U;
            BSP_CAN_SetTermination(g_Params.CanTerm);
            return 1U;

        case CANCFG_P_TIMEOUT_MS:
            if ((value < 0.0f) || (value > 60000.0f))
            {
                return 0U;
            }
            g_Params.CanTimeoutMs = (uint16_t)value;
            return 1U;

        case CANCFG_P_SPEED_KP:
            g_Params.SpeedKp = value;
            MC_ApplyPidParams();
            return 1U;

        case CANCFG_P_SPEED_KI:
            g_Params.SpeedKi = value;
            MC_ApplyPidParams();
            return 1U;

        case CANCFG_P_POS_KP:
            g_Params.PosKp = value;
            return 1U;

        case CANCFG_P_CUR_MAX:
            if ((value <= 0.0f) || (value > MOTOR_CUR_PEAK_A))
            {
                return 0U;
            }
            g_Params.CurMax = value;
            return 1U;

        case CANCFG_P_SPEED_MAX:
            if ((value <= 0.0f) || (value > MOTOR_SPEED_MAX_RADS * 1.2f))
            {
                return 0U;
            }
            g_Params.SpeedMax = value;
            return 1U;

        case CANCFG_P_LED_BRIGHT:
            if ((value < 0.0f) || (value > 255.0f))
            {
                return 0U;
            }
            g_Params.LedBrightness = (uint8_t)value;
            return 1U;

        default:
            return 0U;     /* 只读或未知索引 */
    }
}

/**
  * @brief  mini-cheetah 打包：float -> 无符号整数
  */
static uint16_t MIT_FloatToUint(float x, float xMin, float xMax, uint8_t bits)
{
    float span = xMax - xMin;

    if (x < xMin)
    {
        x = xMin;
    }
    else if (x > xMax)
    {
        x = xMax;
    }
    return (uint16_t)((x - xMin) * (float)((1UL << bits) - 1UL) / span);
}

/**
  * @brief  mini-cheetah 解包：无符号整数 -> float
  */
static float MIT_UintToFloat(uint16_t x, float xMin, float xMax, uint8_t bits)
{
    float span = xMax - xMin;

    return ((float)x) * span / (float)((1UL << bits) - 1UL) + xMin;
}
