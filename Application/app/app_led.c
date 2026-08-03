/**
  ******************************************************************************
  * @file    app_led.c
  * @brief   状态灯语模块（单颗 WS2812，亮度全局可调）
  * @note    灯语表（优先级从高到低）：
  *          - 故障      ：红色计数闪烁，闪烁次数 = 故障码，组间隔 1.2s；
  *          - 校准中    ：蓝色快闪（5Hz）；
  *          - 运行(MIT) ：青色常亮；
  *          - 运行(其他)：绿色常亮；
  *          - 温度告警  ：上述运行/空闲颜色替换为黄色（降额提示）；
  *          - 空闲已校准：绿色呼吸（2s 周期）；
  *          - 空闲未校准：白色呼吸（提示需要预定位校准）；
  *          亮度取 g_Params.LedBrightness，随参数修改即时生效。
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
#include "app_led.h"
#include "app_params.h"
#include "motor_ctrl.h"
#include "bsp_ws2812.h"
#include "main.h"

/* Private defines -----------------------------------------------------------*/
#define LED_FAULT_BLINK_MS      (250U)      /*!< 故障闪烁半周期 */
#define LED_FAULT_PAUSE_MS      (1200U)     /*!< 故障闪烁组间隔 */
#define LED_CAL_BLINK_MS        (100U)      /*!< 校准快闪半周期 */
#define LED_BREATH_PERIOD_MS    (2000U)     /*!< 呼吸周期 */

/* Private function prototypes -----------------------------------------------*/
static uint8_t APP_LED_BreathLevel(uint32_t tick);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化灯语模块
  */
void APP_LED_Init(void)
{
    BSP_WS2812_SetBrightness(g_Params.LedBrightness);
    BSP_WS2812_SetColor(0U, 0U, 0U);
}

/**
  * @brief  灯语轮询（后台调用，内部按 HAL tick 计时）
  */
void APP_LED_Poll(void)
{
    uint32_t tick = HAL_GetTick();

    /* 亮度参数随动 */
    if (BSP_WS2812_GetBrightness() != g_Params.LedBrightness)
    {
        BSP_WS2812_SetBrightness(g_Params.LedBrightness);
    }

    if (g_Mc.Mode == MC_MODE_FAULT)
    {
        /* 红色计数闪烁：N 次 (亮250/灭250) + 组间隔 */
        uint32_t n      = (uint32_t)g_Mc.Fault;
        uint32_t period = n * 2U * LED_FAULT_BLINK_MS + LED_FAULT_PAUSE_MS;
        uint32_t phase  = tick % period;
        uint32_t slot   = phase / LED_FAULT_BLINK_MS;

        if ((slot < n * 2U) && ((slot & 0x01U) == 0U))
        {
            BSP_WS2812_SetColor(255U, 0U, 0U);
        }
        else
        {
            BSP_WS2812_SetColor(0U, 0U, 0U);
        }
    }
    else if (g_Mc.Mode == MC_MODE_CALIB)
    {
        /* 蓝色快闪 */
        if (((tick / LED_CAL_BLINK_MS) & 0x01U) == 0U)
        {
            BSP_WS2812_SetColor(0U, 0U, 255U);
        }
        else
        {
            BSP_WS2812_SetColor(0U, 0U, 0U);
        }
    }
    else if (g_Mc.Enabled != 0U)
    {
        if (PROT_IsWarning() != 0U)
        {
            BSP_WS2812_SetColor(255U, 160U, 0U);    /* 黄色：降额运行 */
        }
        else if (g_Mc.Mode == MC_MODE_MIT)
        {
            BSP_WS2812_SetColor(0U, 200U, 200U);    /* 青色：MIT 运行 */
        }
        else
        {
            BSP_WS2812_SetColor(0U, 255U, 0U);      /* 绿色：运行 */
        }
    }
    else
    {
        /* 空闲呼吸 */
        uint8_t level = APP_LED_BreathLevel(tick);

        if (PROT_IsWarning() != 0U)
        {
            BSP_WS2812_SetColor(level, (uint8_t)((uint16_t)level * 160U / 255U), 0U);
        }
        else if (g_Params.Calibrated != 0U)
        {
            BSP_WS2812_SetColor(0U, level, 0U);     /* 绿色呼吸：就绪 */
        }
        else
        {
            BSP_WS2812_SetColor(level, level, level);   /* 白色呼吸：待校准 */
        }
    }

    BSP_WS2812_Poll();
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  呼吸亮度三角波（16~255）
  */
static uint8_t APP_LED_BreathLevel(uint32_t tick)
{
    uint32_t phase = tick % LED_BREATH_PERIOD_MS;
    uint32_t half  = LED_BREATH_PERIOD_MS / 2U;
    uint32_t level;

    if (phase >= half)
    {
        phase = LED_BREATH_PERIOD_MS - phase;
    }
    level = 16U + (phase * (255U - 16U)) / half;
    return (uint8_t)level;
}
