/**
  ******************************************************************************
  * @file    motor_ctrl.c
  * @brief   电机控制核心模块（模式状态机 + 20kHz FOC 主循环）
  * @note    前后台架构中的"前台"：MC_FocIsr 由 ADC 注入完成中断（TIM1 触发，
  *          20kHz）驱动，完成 编码器读取 -> 角度/速度解算 -> 保护 -> 各模式
  *          控制律 -> SVPWM 输出 的完整链路；速度/位置环在 ISR 内 20 分频
  *          （1kHz）执行。后台（app 层）只做慢速任务与通信。
  *
  *          约定：
  *          - 正方向 = 校准后编码器计数增大方向，正 q 轴电流产生正方向转矩；
  *          - 输出轴量 = 转子量 / MOTOR_GEAR_RATIO，对外接口均为输出轴量纲；
  *          - 模式切换仅允许在封波状态进行（MIT 进入指令 = 切模式 + 使能）。
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
#include "motor_ctrl.h"
#include "calibration.h"
#include "bsp_pwm.h"
#include "app_params.h"
#include "motor_config.h"
#include "board_config.h"
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define MC_RAD_PER_COUNT        (FOC_2PI / (float)ENC_CPR)          /*!< 转子机械角分辨率 */
#define MC_INV_GEAR             (1.0f / MOTOR_GEAR_RATIO)
#define MC_PRECHARGE_CYCLES     (40U)   /*!< 使能后自举预充周期数（2ms @20kHz） */

/* Exported variables --------------------------------------------------------*/
MC_HandleTypeDef g_Mc;

/* Private function prototypes -----------------------------------------------*/
static void  MC_SlowLoop(void);
static float MC_Clamp(float value, float limit);
static void  MC_UpdateElecAngle(void);
static float MC_PosRotor(void);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化电机控制模块
  * @note   须在参数加载（APP_PARAMS_Init）与编码器初始化之后调用。
  */
void MC_Init(void)
{
    FOC_InitTypeDef focInit;

    g_Mc.Mode         = MC_MODE_DISABLED;
    g_Mc.Fault        = FAULT_NONE;
    g_Mc.Enabled      = 0U;
    g_Mc.PrechargeCnt = 0U;
    g_Mc.PosZero      = 0.0f;
    g_Mc.SlowDivCnt   = 0U;
    g_Mc.CurLimitNow  = g_Params.CurMax;

    focInit.Ts               = CTRL_CURRENT_TS;
    focInit.CurrentBandwidth = CTRL_CURRENT_BW_RADS;
    focInit.Rs               = MOTOR_RS_OHM;
    focInit.Ls               = MOTOR_LS_H;
    focInit.Flux             = MOTOR_KT_ROTOR / (1.5f * (float)MOTOR_POLE_PAIRS);
    focInit.MaxDuty          = BOARD_PWM_MAX_DUTY;
    focInit.DecoupleEnable   = CTRL_DECOUPLE_ENABLE;
    FOC_Init(&g_Mc.Foc, &focInit);

    PID_Init(&g_Mc.SpeedPid, g_Params.SpeedKp, g_Params.SpeedKi * CTRL_SLOW_TS, 0.0f,
             -g_Params.CurMax, g_Params.CurMax);

    /* 编码器方向按校准结果恢复，预读建立多圈基准 */
    BSP_ENC_SetInvert(g_Params.EncInvert);
    BSP_ENC_Preload(&g_Mc.Enc);

    PLL_Init(&g_Mc.Pll, CTRL_PLL_BW_RADS, CTRL_CURRENT_TS);
    PLL_Preload(&g_Mc.Pll, (float)g_Mc.Enc.Raw * MC_RAD_PER_COUNT);

    /* 会话零位 = 上电位置 */
    g_Mc.PosZero = MC_PosRotor() * MC_INV_GEAR;

    /* DWT cycle counter 用于 ISR 耗时统计 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* 预热 sin 查找表：首次建表约数千 cycle，不能落在第一拍电流环 ISR 内 */
    {
        float s;
        float c;
        FOC_SinCos(0.0f, &s, &c);
    }
}

/**
  * @brief  FOC 主循环（ADC 注入完成 ISR，20kHz）
  * @param  meas 本周期电流/电压测量
  */
void MC_FocIsr(const ADC_FocMeasTypeDef *meas)
{
    uint32_t tStart = DWT->CYCCNT;
    float rawAngle;
    float iqCmd;
    PROT_FaultTypeDef fault;

    /* ---------------- 测量与解算 ---------------- */
    (void)BSP_ENC_Read(&g_Mc.Enc);

    /* PLL 跟踪单圈角（回卷域，长时间旋转无精度退化）；
       位置由整数圈数 + 单圈角分离合成，规避 float 大数精度损失 */
    rawAngle = (float)g_Mc.Enc.Raw * MC_RAD_PER_COUNT;
    PLL_Update(&g_Mc.Pll, rawAngle);

    g_Mc.Speed = g_Mc.Pll.Speed * MC_INV_GEAR;
    g_Mc.Pos   = MC_PosRotor() * MC_INV_GEAR - g_Mc.PosZero;
    MC_UpdateElecAngle();

    g_Mc.Iu   = meas->Iu;
    g_Mc.Iv   = meas->Iv;
    g_Mc.Iw   = -(meas->Iu + meas->Iv);
    g_Mc.Vbus = meas->Vbus;
    /* 母线电流功率法估计（低侧采样时刻实测恒为 0，不可用） */
    g_Mc.IbusEst = 1.5f * (g_Mc.Foc.Vdq.A * g_Mc.Foc.Idq.A + g_Mc.Foc.Vdq.B * g_Mc.Foc.Idq.B)
                   / ((g_Mc.Vbus > 1.0f) ? g_Mc.Vbus : 1.0f);

    /* ---------------- 快速保护（仅使能状态触发） ---------------- */
    if (g_Mc.Enabled != 0U)
    {
        /* 电流通道贴轨 = 实际电流超出测量范围，换算值已钳位，
           必须独立于阈值比较立即封波 */
        if (meas->CurSat != 0U)
        {
            MC_TripFault(FAULT_OVERCURRENT);
        }
        fault = PROT_CheckFast(g_Mc.Iu, g_Mc.Iv, g_Mc.Iw, g_Mc.Vbus, g_Mc.Enc.ConsecutiveErr);
        if (fault != FAULT_NONE)
        {
            MC_TripFault(fault);
        }
    }

    /* ---------------- 模式控制律 ---------------- */
    switch (g_Mc.Mode)
    {
        case MC_MODE_CALIB:
        {
            CAL_ResultTypeDef res = CAL_Isr(&g_Mc.Foc, &g_Mc.Enc, g_Mc.Iu, g_Mc.Iv, g_Mc.Iw, g_Mc.Vbus);
            if (res == CAL_RES_RUNNING)
            {
                BSP_PWM_SetDuty(g_Mc.Foc.DutyA, g_Mc.Foc.DutyB, g_Mc.Foc.DutyC);
            }
            else
            {
                /* 校准结束：封波，按新方向重建角度基准（零位取当前位置） */
                MC_Disable();
                BSP_ENC_SetInvert(g_Params.EncInvert);
                BSP_ENC_Preload(&g_Mc.Enc);
                PLL_Preload(&g_Mc.Pll, (float)g_Mc.Enc.Raw * MC_RAD_PER_COUNT);
                g_Mc.PosZero = MC_PosRotor() * MC_INV_GEAR;
                if (res == CAL_RES_DONE)
                {
                    APP_PARAMS_RequestSave();
                    g_Mc.Mode = MC_MODE_DISABLED;
                }
                else
                {
                    g_Mc.Fault = FAULT_CAL_FAIL;
                    g_Mc.Mode  = MC_MODE_FAULT;
                }
            }
            break;
        }

        case MC_MODE_OPENLOOP:
        case MC_MODE_TORQUE:
        case MC_MODE_SPEED:
        case MC_MODE_POSITION:
        case MC_MODE_MIT:
        {
            if (g_Mc.Enabled == 0U)
            {
                break;  /* 已切模式但未使能：保持封波 */
            }

            /* 自举电容预充：零占空比（低侧全导通）维持数毫秒 */
            if (g_Mc.PrechargeCnt != 0U)
            {
                g_Mc.PrechargeCnt--;
                BSP_PWM_SetDutyZero();
                break;
            }

            /* 慢环（1kHz）：速度/位置环与限幅刷新 */
            if (++g_Mc.SlowDivCnt >= CTRL_SLOW_LOOP_DIV)
            {
                g_Mc.SlowDivCnt = 0U;
                MC_SlowLoop();
            }

            if (g_Mc.Mode == MC_MODE_OPENLOOP)
            {
                g_Mc.OpenTheta += g_Mc.OpenSpeedE * CTRL_CURRENT_TS;
                if (g_Mc.OpenTheta > FOC_2PI)
                {
                    g_Mc.OpenTheta -= FOC_2PI;
                }
                else if (g_Mc.OpenTheta < 0.0f)
                {
                    g_Mc.OpenTheta += FOC_2PI;
                }
                FOC_OutputVoltage(&g_Mc.Foc, g_Mc.OpenVd, g_Mc.OpenVq, g_Mc.OpenTheta, g_Mc.Vbus);
            }
            else
            {
                if (g_Mc.Mode == MC_MODE_MIT)
                {
                    /* MIT 阻抗控制律（每周期执行，20kHz 阻抗渲染） */
                    float torque = g_Mc.MitKp * (g_Mc.MitPosDes - g_Mc.Pos)
                                 + g_Mc.MitKd * (g_Mc.MitVelDes - g_Mc.Speed)
                                 + g_Mc.MitTff;
                    iqCmd = MC_Clamp(torque / MOTOR_KT_OUT, g_Mc.CurLimitNow);
                    FOC_SetCurrentRef(&g_Mc.Foc, 0.0f, iqCmd);
                }
                /* TORQUE/SPEED/POSITION 的 iq 给定在慢环中更新 */

                FOC_Update(&g_Mc.Foc, g_Mc.Iu, g_Mc.Iv, g_Mc.Iw, g_Mc.ThetaE,
                           g_Mc.Pll.Speed * (float)g_Params.PolePairs, g_Mc.Vbus);
            }

            BSP_PWM_SetDuty(g_Mc.Foc.DutyA, g_Mc.Foc.DutyB, g_Mc.Foc.DutyC);
            break;
        }

        case MC_MODE_DISABLED:
        case MC_MODE_FAULT:
        default:
            /* 封波状态：无输出，测量链持续运行供监控 */
            break;
    }

    g_Mc.IsrCount++;
    g_Mc.IsrCyclesLast = DWT->CYCCNT - tStart;
    if (g_Mc.IsrCyclesLast > g_Mc.IsrCyclesMax)
    {
        g_Mc.IsrCyclesMax = g_Mc.IsrCyclesLast;
    }
}

/**
  * @brief  使能功率输出
  * @retval HAL_OK；故障未清除 / 校准缺失（闭环模式）/ 模式为封波时返回 HAL_ERROR
  */
HAL_StatusTypeDef MC_Enable(void)
{
    if ((g_Mc.Mode == MC_MODE_FAULT) || (g_Mc.Mode == MC_MODE_DISABLED) ||
        (g_Mc.Mode == MC_MODE_CALIB))
    {
        return HAL_ERROR;
    }
    /* 闭环模式依赖电角度：必须已校准（开环模式豁免） */
    if ((g_Mc.Mode != MC_MODE_OPENLOOP) && (g_Params.Calibrated == 0U))
    {
        return HAL_ERROR;
    }
    if (g_Mc.Enabled != 0U)
    {
        return HAL_OK;
    }

    FOC_Reset(&g_Mc.Foc);
    PID_Reset(&g_Mc.SpeedPid);
    g_Mc.TgtIq        = 0.0f;
    g_Mc.TgtSpeed     = 0.0f;
    g_Mc.TgtPos       = g_Mc.Pos;      /* 位置模式无扰启动 */
    g_Mc.MitPosDes    = g_Mc.Pos;
    g_Mc.MitVelDes    = 0.0f;
    g_Mc.MitKp        = 0.0f;
    g_Mc.MitKd        = 0.0f;
    g_Mc.MitTff       = 0.0f;
    g_Mc.OpenTheta    = 0.0f;
    g_Mc.PrechargeCnt = MC_PRECHARGE_CYCLES;

    PROT_FeedCanWatchdog();     /* 使能瞬间重置超时基准 */
    BSP_PWM_Enable();
    g_Mc.Enabled = 1U;
    return HAL_OK;
}

/**
  * @brief  封波（ISR 安全）
  */
void MC_Disable(void)
{
    BSP_PWM_Disable();
    g_Mc.Enabled = 0U;
    FOC_Reset(&g_Mc.Foc);
    PID_Reset(&g_Mc.SpeedPid);
}

/**
  * @brief  切换控制模式（仅封波状态允许）
  */
HAL_StatusTypeDef MC_SetMode(MC_ModeTypeDef mode)
{
    if ((g_Mc.Enabled != 0U) || (g_Mc.Mode == MC_MODE_CALIB) || (g_Mc.Mode == MC_MODE_FAULT))
    {
        return HAL_ERROR;
    }
    /* 白名单校验：仅允许 DISABLED/OPENLOOP/TORQUE/SPEED/POSITION/MIT，
       CALIB/FAULT 为内部状态、越界枚举（CAN 注入的非法值）一律拒绝 */
    if (mode > MC_MODE_MIT)
    {
        return HAL_ERROR;
    }
    g_Mc.Mode = mode;
    return HAL_OK;
}

/**
  * @brief  启动预定位校准（封波状态下）
  */
HAL_StatusTypeDef MC_StartCalibration(void)
{
    if ((g_Mc.Enabled != 0U) || (g_Mc.Mode == MC_MODE_FAULT) || (g_Mc.Mode == MC_MODE_CALIB))
    {
        return HAL_ERROR;
    }

    FOC_Reset(&g_Mc.Foc);
    CAL_Start();
    g_Mc.Mode = MC_MODE_CALIB;
    g_Mc.PrechargeCnt = 0U;
    BSP_PWM_Enable();
    g_Mc.Enabled = 1U;
    return HAL_OK;
}

/**
  * @brief  清除故障（故障源已消失时）
  */
HAL_StatusTypeDef MC_ClearFault(void)
{
    if (g_Mc.Mode != MC_MODE_FAULT)
    {
        return HAL_OK;
    }
    g_Mc.Fault = FAULT_NONE;
    g_Mc.Mode  = MC_MODE_DISABLED;
    PROT_Init();
    return HAL_OK;
}

/**
  * @brief  触发故障：立即封波并锁存（ISR 安全）
  */
void MC_TripFault(PROT_FaultTypeDef fault)
{
    MC_Disable();
    g_Mc.Fault = fault;
    g_Mc.Mode  = MC_MODE_FAULT;
}

/**
  * @brief  以当前位置为输出轴零位
  */
void MC_SetZeroHere(void)
{
    g_Mc.PosZero += g_Mc.Pos;
    g_Mc.TgtPos    = 0.0f;
    g_Mc.MitPosDes = 0.0f;
}

/**
  * @brief  设置 q 轴电流给定（TORQUE 模式）
  * @param  iq q 轴电流 [A]，慢环中按限幅裁剪
  * @retval None
  */
void MC_SetIqRef(float iq)
{
    g_Mc.TgtIq = iq;
}

/**
  * @brief  设置输出轴转矩给定（TORQUE 模式，内部折算为 iq）
  * @param  torque 输出轴转矩 [N*m]
  * @retval None
  */
void MC_SetTorque(float torque)
{
    g_Mc.TgtIq = torque / MOTOR_KT_OUT;
}

/**
  * @brief  设置输出轴速度给定（SPEED 模式）
  * @param  speed 输出轴角速度 [rad/s]，按 SpeedMax 限幅
  * @retval None
  */
void MC_SetSpeedRef(float speed)
{
    g_Mc.TgtSpeed = MC_Clamp(speed, g_Params.SpeedMax);
}

/**
  * @brief  设置输出轴位置给定（POSITION 模式）
  * @param  pos 输出轴位置 [rad]（会话零位坐标）
  * @retval None
  */
void MC_SetPosRef(float pos)
{
    g_Mc.TgtPos = pos;
}

/**
  * @brief  设置 MIT 阻抗指令（五元组原子更新）
  * @note   调用上下文为 CAN ISR（优先级低于 FOC ISR），短暂关中断保证
  *         五个字段不被 FOC ISR 撕裂成新旧混合的一拍指令。
  * @param  pDes/vDes 期望位置 [rad] / 速度 [rad/s]（输出轴）
  * @param  kp/kd     刚度 [N*m/rad] / 阻尼 [N*m*s/rad]
  * @param  tff       前馈转矩 [N*m]
  * @retval None
  */
void MC_SetMitCommand(float pDes, float vDes, float kp, float kd, float tff)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_Mc.MitPosDes = pDes;
    g_Mc.MitVelDes = vDes;
    g_Mc.MitKp     = kp;
    g_Mc.MitKd     = kd;
    g_Mc.MitTff    = tff;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

/**
  * @brief  设置开环电压矢量（OPENLOOP 模式）
  * @note   开环不经电流 PI，电压矢量幅值限制为 Rs*峰值电流对应的
  *         稳态压降，防止误输入造成持续大电流（开环无温度降额通道，
  *         贴轨检测与软件过流阈值仍然生效）。
  * @param  vd/vq  d/q 轴电压 [V]
  * @param  speedE 电角速度 [rad/s]（内部积分生成旋转角）
  * @retval None
  */
void MC_SetOpenLoop(float vd, float vq, float speedE)
{
    const float vLimit = MOTOR_RS_OHM * MOTOR_CUR_PEAK_A;
    float magSq = vd * vd + vq * vq;

    if (magSq > (vLimit * vLimit))
    {
        float scale = vLimit / sqrtf(magSq);

        vd *= scale;
        vq *= scale;
    }

    g_Mc.OpenVd     = vd;
    g_Mc.OpenVq     = vq;
    g_Mc.OpenSpeedE = speedE;
}

/**
  * @brief  重建编码器多圈基准并保持用户位置连续（参数落盘后调用）
  * @note   落盘期间全程关中断约 20ms+，编码器停采；若期间关节被外力
  *         反拖超过转子半圈，多圈跳变判定会滑圈。本函数重新预读编码器
  *         并调整会话零位使 Pos 读数连续（假设落盘期间关节未动）。
  *         须在 FOC 中断屏蔽状态下由后台调用。
  * @retval None
  */
void MC_RebaseEncoder(void)
{
    float posPrev = g_Mc.Pos;

    BSP_ENC_Preload(&g_Mc.Enc);
    PLL_Preload(&g_Mc.Pll, (float)g_Mc.Enc.Raw * MC_RAD_PER_COUNT);
    g_Mc.PosZero = MC_PosRotor() * MC_INV_GEAR - posPrev;
}

/**
  * @brief  参数修改后刷新 PID 增益（shell/CAN 调参后调用）
  */
void MC_ApplyPidParams(void)
{
    PID_SetGains(&g_Mc.SpeedPid, g_Params.SpeedKp, g_Params.SpeedKi * CTRL_SLOW_TS, 0.0f);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  慢环（1kHz）：电流限幅刷新与速度/位置外环
  */
static void MC_SlowLoop(void)
{
    float curLimit = g_Params.CurMax * PROT_GetDerating();
    float iqCmd;
    float spdRef;

    if (curLimit > MOTOR_CUR_PEAK_A)
    {
        curLimit = MOTOR_CUR_PEAK_A;
    }
    g_Mc.CurLimitNow = curLimit;

    switch (g_Mc.Mode)
    {
        case MC_MODE_TORQUE:
            iqCmd = MC_Clamp(g_Mc.TgtIq, curLimit);
            /* 速度越限保护：同向转矩折零，防止空载飞车 */
            if (((g_Mc.Speed > g_Params.SpeedMax) && (iqCmd > 0.0f)) ||
                ((g_Mc.Speed < -g_Params.SpeedMax) && (iqCmd < 0.0f)))
            {
                iqCmd = 0.0f;
            }
            FOC_SetCurrentRef(&g_Mc.Foc, 0.0f, iqCmd);
            break;

        case MC_MODE_SPEED:
            PID_SetLimits(&g_Mc.SpeedPid, -curLimit, curLimit);
            iqCmd = PID_Update(&g_Mc.SpeedPid, g_Mc.TgtSpeed - g_Mc.Speed);
            FOC_SetCurrentRef(&g_Mc.Foc, 0.0f, iqCmd);
            break;

        case MC_MODE_POSITION:
            /* 级联：位置 P -> 速度给定 -> 速度 PI -> iq */
            spdRef = g_Params.PosKp * (g_Mc.TgtPos - g_Mc.Pos);
            spdRef = MC_Clamp(spdRef, (g_Params.SpeedMax < CTRL_POS_SPEED_LIMIT) ?
                                       g_Params.SpeedMax : CTRL_POS_SPEED_LIMIT);
            PID_SetLimits(&g_Mc.SpeedPid, -curLimit, curLimit);
            iqCmd = PID_Update(&g_Mc.SpeedPid, spdRef - g_Mc.Speed);
            FOC_SetCurrentRef(&g_Mc.Foc, 0.0f, iqCmd);
            break;

        default:
            /* MIT 每周期在主循环处理；OPENLOOP 不经电流环 */
            break;
    }
}

/**
  * @brief  由单圈原始计数计算电角度
  * @note   theta_e = raw * PP * (2pi/CPR) - ElecOffset；整数乘法自然回卷。
  */
static void MC_UpdateElecAngle(void)
{
    uint32_t elecCounts = ((uint32_t)g_Mc.Enc.Raw * (uint32_t)g_Params.PolePairs) & ENC_CPR_MASK;

    g_Mc.ThetaE = (float)elecCounts * MC_RAD_PER_COUNT - g_Params.ElecOffset;
}

/**
  * @brief  转子展开机械角 [rad]：整数圈 + 单圈角分离合成
  * @note   Turns 为 int32 整数域（±21 亿圈），float 合成精度损失只发生在
  *         Turns*2pi 的大数区（千圈级误差 <0.5mrad），无溢出风险。
  */
static float MC_PosRotor(void)
{
    return (float)g_Mc.Enc.Turns * FOC_2PI + (float)g_Mc.Enc.Raw * MC_RAD_PER_COUNT;
}

/**
  * @brief  对称限幅
  */
static float MC_Clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}
