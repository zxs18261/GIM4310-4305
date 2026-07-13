/**
  ******************************************************************************
  * @file    app_monitor.c
  * @brief   调试数据打印模块
  * @note    - 周期日志：CSV 格式便于上位机绘图，shell "log on [hz]" 开启，
  *            默认 20Hz，字段见 APP_MON_Poll 表头；
  *          - 状态快照：shell "status" 触发的人读格式完整状态；
  *          - 所有输出经 bsp_uart 非阻塞环形缓冲，不影响控制环。
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
#include "app_monitor.h"
#include "app_main.h"
#include "app_params.h"
#include "motor_ctrl.h"
#include "calibration.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "bsp_clock.h"
#include "board_config.h"
#include "motor_config.h"

/* Private variables ---------------------------------------------------------*/
static uint8_t  logEnable   = 0U;
static uint32_t logPeriodMs = 50U;      /*!< 默认 20Hz */
static uint32_t logLastTick = 0U;

/* Private constants ---------------------------------------------------------*/
static const char *const modeNames[] =
{
    "DISABLED", "OPENLOOP", "TORQUE", "SPEED", "POSITION", "MIT", "CALIB", "FAULT",
};

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化监控模块
  */
void APP_MON_Init(void)
{
    logEnable   = 0U;
    logLastTick = HAL_GetTick();
}

/**
  * @brief  周期日志轮询（后台调用）
  */
void APP_MON_Poll(void)
{
    uint32_t tick = HAL_GetTick();

    if ((logEnable == 0U) || ((tick - logLastTick) < logPeriodMs))
    {
        return;
    }
    logLastTick = tick;

    /* TX 缓冲余量不足时整行跳过，避免截断产生破损 CSV 行 */
    if (BSP_UART_TxFree() < 160U)
    {
        return;
    }

    /* CSV：时间[ms],模式,位置[rad],速度[rad/s],Id[A],Iq[A],Vbus[V],
            MOS温度[C],电机温度[C],故障码,ISR周期[cycle] */
    BSP_UART_Printf("%lu,%u,%.4f,%.3f,%.3f,%.3f,%.2f,%.1f,%.1f,%u,%lu\r\n",
                    (unsigned long)tick,
                    (unsigned int)g_Mc.Mode,
                    g_Mc.Pos, g_Mc.Speed,
                    g_Mc.Foc.Idq.A, g_Mc.Foc.Idq.B,
                    g_Mc.Vbus,
                    APP_GetMosTemp(), APP_GetMotorTemp(),
                    (unsigned int)g_Mc.Fault,
                    (unsigned long)g_Mc.IsrCyclesLast);
}

/**
  * @brief  设置周期日志开关与速率
  */
void APP_MON_SetLog(uint8_t enable, uint32_t rateHz)
{
    if ((rateHz == 0U) || (rateHz > 200U))
    {
        rateHz = 20U;
    }
    logPeriodMs = 1000U / rateHz;
    logEnable   = enable;

    if (enable != 0U)
    {
        BSP_UART_Printf("# t_ms,mode,pos,spd,id,iq,vbus,tmos,tmot,fault,isr_cyc\r\n");
    }
}

/**
  * @brief  打印完整状态快照（shell "status"）
  */
void APP_MON_PrintStatus(void)
{
    float offsU;
    float offsV;
    float offsBus;

    BSP_ADC_GetOffsets(&offsU, &offsV, &offsBus);

    BSP_UART_Printf("---- %s status ----\r\n", MOTOR_NAME);
    BSP_UART_Printf("mode      : %s", modeNames[g_Mc.Mode]);
    if (g_Mc.Mode == MC_MODE_CALIB)
    {
        BSP_UART_Printf(" (%s %u%%)", CAL_GetPhaseName(), (unsigned int)CAL_GetProgress());
    }
    BSP_UART_Printf("\r\nfault     : %u\r\n", (unsigned int)g_Mc.Fault);
    BSP_UART_Printf("enabled   : %u  calibrated: %u\r\n",
                    (unsigned int)g_Mc.Enabled, (unsigned int)g_Params.Calibrated);
    BSP_UART_Printf("pos/spd   : %.4f rad  %.3f rad/s (output)\r\n", g_Mc.Pos, g_Mc.Speed);
    BSP_UART_Printf("id/iq     : %.3f  %.3f A  (limit %.2f A)\r\n",
                    g_Mc.Foc.Idq.A, g_Mc.Foc.Idq.B, g_Mc.CurLimitNow);
    BSP_UART_Printf("iu/iv/iw  : %.3f  %.3f  %.3f A\r\n", g_Mc.Iu, g_Mc.Iv, g_Mc.Iw);
    BSP_UART_Printf("vbus      : %.2f V  ibus_est: %.3f A\r\n", g_Mc.Vbus, g_Mc.IbusEst);
    BSP_UART_Printf("temp      : mos %.1f C  motor %.1f C  derate %.2f\r\n",
                    APP_GetMosTemp(), APP_GetMotorTemp(), PROT_GetDerating());
    BSP_UART_Printf("encoder   : raw %u  turns %ld  err(crc/mg) %lu/%lu  invert %u\r\n",
                    (unsigned int)g_Mc.Enc.Raw, (long)g_Mc.Enc.Turns,
                    (unsigned long)g_Mc.Enc.CrcErrCnt, (unsigned long)g_Mc.Enc.FieldErrCnt,
                    (unsigned int)g_Params.EncInvert);
    BSP_UART_Printf("elec      : pp %u  offset %.4f rad\r\n",
                    (unsigned int)g_Params.PolePairs, g_Params.ElecOffset);
    BSP_UART_Printf("adc offs  : u %.1f  v %.1f  bus %.1f LSB\r\n", offsU, offsV, offsBus);
    BSP_UART_Printf("can       : id %u  baud %u  term %u  rx %lu  err %lu\r\n",
                    (unsigned int)g_Params.NodeId, (unsigned int)g_Params.CanBaud,
                    (unsigned int)BSP_CAN_GetTermination(),
                    (unsigned long)BSP_CAN_GetRxCount(), (unsigned long)BSP_CAN_GetErrCount());
    BSP_UART_Printf("isr       : last %lu  max %lu cycle (budget %lu)\r\n",
                    (unsigned long)g_Mc.IsrCyclesLast, (unsigned long)g_Mc.IsrCyclesMax,
                    (unsigned long)(BOARD_SYSCLK_HZ / BOARD_PWM_FREQ_HZ));
    BSP_UART_Printf("clock     : %s\r\n", (BSP_CLOCK_IsHse() != 0U) ? "HSE 20MHz" : "HSI16 (fallback!)");
}

/**
  * @brief  打印上电横幅
  */
void APP_MON_PrintBanner(void)
{
    BSP_UART_Printf("\r\n==== GIM Joint Motor Controller ====\r\n");
    BSP_UART_Printf("fw    : %s\r\n", APP_FW_VERSION);
    BSP_UART_Printf("motor : %s (pp=%u, gear=%.0f:1)\r\n",
                    MOTOR_NAME, (unsigned int)g_Params.PolePairs, MOTOR_GEAR_RATIO);
    BSP_UART_Printf("clock : %s\r\n", (BSP_CLOCK_IsHse() != 0U) ? "HSE 20MHz -> 170MHz" : "HSI16 -> 170MHz (HSE FAIL!)");
    BSP_UART_Printf("can   : id=%u baud_idx=%u term=%u\r\n",
                    (unsigned int)g_Params.NodeId, (unsigned int)g_Params.CanBaud,
                    (unsigned int)g_Params.CanTerm);
    BSP_UART_Printf("type 'help' for shell commands\r\n> ");
}
