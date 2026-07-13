/**
  ******************************************************************************
  * @file    app_main.c
  * @brief   应用入口模块（前后台调度）
  * @note    裸机前后台架构：
  *          - 前台：MC_FocIsr（ADC 注入完成中断，20kHz，见 motor_ctrl.c）、
  *            CAN 接收解析（FDCAN ISR）、串口收发（USART/DMA ISR）；
  *          - 后台：APP_Loop 超级循环，非阻塞轮询 shell / 灯语 / 温度保护 /
  *            周期日志 / 参数落盘，均以 HAL tick 分频，无任何忙等。
  *
  *          APP_Init 由 main.c 的 USER CODE BEGIN 2 调用（MX_*_Init 之后），
  *          APP_Loop 由 while(1) 内调用。
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
#include "app_main.h"
#include "app_params.h"
#include "app_can_protocol.h"
#include "app_led.h"
#include "app_shell.h"
#include "app_monitor.h"
#include "motor_ctrl.h"
#include "protection.h"
#include "bsp_clock.h"
#include "bsp_uart.h"
#include "bsp_pwm.h"
#include "bsp_adc.h"
#include "bsp_encoder.h"
#include "bsp_ws2812.h"
#include "board_config.h"
#include "motor_config.h"

/* Private defines -----------------------------------------------------------*/
#define APP_SLOW_TASK_MS        (100U)      /*!< 温度/超时等慢任务周期 */

/* Private variables ---------------------------------------------------------*/
static uint32_t slowTaskTick = 0U;
static float    mosTemp   = 25.0f;      /*!< MOS 温度缓存 [C] */
static float    motorTemp = 25.0f;      /*!< 电机温度缓存 [C] */
static uint8_t  mosNtcOk  = 1U;         /*!< 板载 NTC 有效标志（开路/短路时打印告警并退出过温判定） */
static uint8_t  motNtcOk  = 1U;         /*!< 电机 NTC 有效标志（同上，未接时不故障便于裸板调试） */

/* Private function prototypes -----------------------------------------------*/
static void APP_SlowTask(void);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  应用初始化（main 的 USER CODE BEGIN 2 调用）
  */
void APP_Init(void)
{
    HAL_StatusTypeDef clkStatus;
    HAL_StatusTypeDef offsStatus;

    /* 1. 时钟切换到板载 HSE 20MHz（失败自动回退 HSI 并在横幅告警） */
    clkStatus = BSP_CLOCK_SwitchToHse();

    /* 2. 调试串口（后续初始化过程即可打印） */
    BSP_UART_Init();

    /* 3. 参数加载 */
    APP_PARAMS_Init();

    /* 4. 状态灯 */
    BSP_WS2812_Init();
    APP_LED_Init();

    /* 5. 编码器（MT6701 上电就绪最长 32ms，HAL_Init 至此已远超） */
    BSP_ENC_Init();

    /* 6. 功率级 PWM（计数器启动、输出封波）与 ADC 采样链 */
    BSP_PWM_Init();
    BSP_ADC_Init();

    /* 7. 电流零流偏置实测（封波状态，512 次平均约 26ms） */
    offsStatus = BSP_ADC_MeasureOffsets(512U);

    /* 8. 保护与控制核心 */
    PROT_Init();
    MC_Init();

    /* 9. CAN 协议栈 */
    APP_CAN_Init();

    /* 10. 启动电流环中断（前台开始运行） */
    BSP_ADC_StartFocIrq(MC_FocIsr);

    /* 11. shell 与监控 */
    APP_SHELL_Init();
    APP_MON_Init();
    APP_MON_PrintBanner();

    if (clkStatus != HAL_OK)
    {
        BSP_UART_Printf("[warn] HSE start failed, running on HSI16 (CAN timing margin reduced)\r\n");
    }
    if (offsStatus != HAL_OK)
    {
        float u;
        float v;
        float b;

        BSP_ADC_GetOffsets(&u, &v, &b);
        BSP_UART_Printf("[fault] current offset out of range: u=%.0f v=%.0f bus=%.0f LSB\r\n", u, v, b);
        MC_TripFault(FAULT_ADC_OFFSET);
    }
    else
    {
        float u;
        float v;
        float b;
        float usable;
        float usableV;

        BSP_ADC_GetOffsets(&u, &v, &b);
        BSP_UART_Printf("[init] current offsets: u=%.0f v=%.0f bus=%.0f LSB (nominal 2048)\r\n", u, v, b);

        /* 偏置不对称会收缩单方向可测电流范围（贴轨检测仍兜底过流），
           低于软件过流阈值时提示定标风险 */
        usable  = ((u < (4095.0f - u)) ? u : (4095.0f - u)) * BOARD_CUR_LSB;
        usableV = ((v < (4095.0f - v)) ? v : (4095.0f - v)) * BOARD_CUR_LSB;
        if (usableV < usable)
        {
            usable = usableV;
        }
        if (usable < PROT_OC_LIMIT_A)
        {
            BSP_UART_Printf("[warn] offset asymmetry limits current range to %.1f A "
                            "(OC threshold %.1f A relies on rail detection)\r\n",
                            usable, PROT_OC_LIMIT_A);
        }
    }

    /* PB8 与 BOOT0 复用风险检查 */
    if ((FLASH->OPTR & FLASH_OPTR_nSWBOOT0) != 0U)
    {
        BSP_UART_Printf("[warn] nSWBOOT0=1: uart RX high at reset enters bootloader, run 'boot0 fix confirm'\r\n");
    }
    if (g_Params.Calibrated == 0U)
    {
        BSP_UART_Printf("[note] not calibrated, run 'cal' before closed-loop modes\r\n");
    }
}

/**
  * @brief  后台超级循环体（main 的 while(1) 内调用）
  */
void APP_Loop(void)
{
    uint32_t tick = HAL_GetTick();

    /* 快速轮询组：shell / 日志 / 灯语 / CAN 重配 */
    APP_SHELL_Poll();
    APP_MON_Poll();
    APP_LED_Poll();
    APP_CAN_Poll();

    /* 慢任务组：100ms */
    if ((tick - slowTaskTick) >= APP_SLOW_TASK_MS)
    {
        slowTaskTick = tick;
        APP_SlowTask();
    }

    /* 参数落盘（仅封波状态执行，避免擦写 stall 打断电流环）。
       落盘期间关中断约 20ms+ 编码器停采，完成后重建多圈基准，
       消除期间外力反拖可能造成的滑圈。
       FOC 中断在落盘前先屏蔽：否则落盘结束到重建之间，挂起的 JEOS
       中断会用 20ms 前的旧角度做半圈跳变判定，滑圈误差被固化进零位 */
    if ((APP_PARAMS_IsSavePending() != 0U) && (g_Mc.Enabled == 0U))
    {
        uint8_t saveResult;

        HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
        saveResult = APP_PARAMS_ProcessSaveRequest((uint8_t)(g_Mc.Enabled));
        if (saveResult != PARAMS_SAVE_NONE)
        {
            MC_RebaseEncoder();     /* 无论成败中断都已停 20ms+，均需重建 */
        }
        HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

        if (saveResult == PARAMS_SAVE_OK)
        {
            BSP_UART_Printf("[param] saved to flash\r\n");
        }
        else if (saveResult == PARAMS_SAVE_FAILED)
        {
            BSP_UART_Printf("[param] save FAILED (flash erase/verify error)\r\n");
        }
    }
}

/**
  * @brief  读取 MOS 温度缓存 [C]
  */
float APP_GetMosTemp(void)
{
    return mosTemp;
}

/**
  * @brief  读取电机温度缓存 [C]
  */
float APP_GetMotorTemp(void)
{
    return motorTemp;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  慢任务：温度采样/过温保护/CAN 指令超时
  */
static void APP_SlowTask(void)
{
    float tMos = BSP_ADC_ReadMosTemp();
    float tMot = BSP_ADC_ReadMotorTemp();

    /* NTC 开路/短路判别：视为传感器缺失，状态跳变时告警一次，不参与过温判定 */
    if (tMos >= (BOARD_NTC_FAULT_TEMP - 1.0f))
    {
        if (mosNtcOk != 0U)
        {
            BSP_UART_Printf("[warn] MOS NTC open/short, overtemp protection suspended\r\n");
        }
        mosNtcOk = 0U;
        tMos = 25.0f;
    }
    else
    {
        mosNtcOk = 1U;
        mosTemp  = tMos;
    }
    if (tMot >= (BOARD_NTC_FAULT_TEMP - 1.0f))
    {
        if (motNtcOk != 0U)
        {
            BSP_UART_Printf("[warn] motor NTC open/short, overtemp protection suspended\r\n");
        }
        motNtcOk = 0U;
        tMot = 25.0f;
    }
    else
    {
        motNtcOk = 1U;
        motorTemp = tMot;
    }

    PROT_CheckSlow(tMos, tMot);

    /* 过温故障（降额到 0 即故障阈值） */
    if (g_Mc.Enabled != 0U)
    {
        if ((mosNtcOk != 0U) && (tMos >= PROT_MOS_OT_LIMIT_C))
        {
            MC_TripFault(FAULT_MOS_OT);
            BSP_UART_Printf("[fault] MOS overtemp %.1fC\r\n", tMos);
        }
        if ((motNtcOk != 0U) && (tMot >= PROT_MOT_OT_LIMIT_C))
        {
            MC_TripFault(FAULT_MOTOR_OT);
            BSP_UART_Printf("[fault] motor overtemp %.1fC\r\n", tMot);
        }

        /* CAN 指令超时保护：适用于所有由 CAN 发起使能的闭环模式
           （shell 调试发起的使能豁免） */
        if ((APP_CAN_IsCanControlled() != 0U) &&
            (PROT_CheckCanTimeout(g_Params.CanTimeoutMs) != 0U))
        {
            MC_TripFault(FAULT_CAN_TIMEOUT);
            BSP_UART_Printf("[fault] CAN command timeout\r\n");
        }
    }
}
