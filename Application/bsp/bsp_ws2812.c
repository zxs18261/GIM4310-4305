/**
  ******************************************************************************
  * @file    bsp_ws2812.c
  * @brief   WS2812 类 RGB 状态灯 BSP（TIM2_CH1 PWM + DMA，单灯）
  * @note    - 每个数据位映射为一个 PWM 周期，占空比区分 0/1 码，DMA 逐周期
  *            刷新 CCR1，帧尾附零占空比时隙形成 reset 码；
  *          - SetColor 仅缓存目标色，Poll 在后台检测变化并触发一次 DMA 发送，
  *            发送期间的新颜色请求自动合并到下一帧，无阻塞；
  *          - 亮度全局缩放（0~255），由 app 层持久化。
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
#include "bsp_ws2812.h"
#include "board_config.h"
#include "tim.h"

/* Private defines -----------------------------------------------------------*/
#define WS2812_BIT_COUNT        (24U)                       /*!< 单灯 24bit GRB（XL-3528RGBW-WS2812B 实为 3 通道，"W"指雾状封装） */
#define WS2812_RESET_SLOTS      (80U)                       /*!< reset 码时隙数：80*1.25us = 100us（该型号要求 >= 80us） */
#define WS2812_BUF_LEN          (WS2812_BIT_COUNT + WS2812_RESET_SLOTS)

/* Private variables ---------------------------------------------------------*/
static DMA_HandleTypeDef hdma_tim2_ch1;

static uint32_t pwmBuf[WS2812_BUF_LEN];                     /*!< CCR 序列缓冲 */
static volatile uint8_t  busy = 0U;                         /*!< DMA 发送中标志 */
static volatile uint8_t  pending = 0U;                      /*!< 有待发送的新颜色 */
static volatile uint32_t targetGrb = 0U;                    /*!< 目标颜色（已按亮度缩放，G<<16|R<<8|B） */
static uint8_t brightness = BOARD_LED_BRIGHTNESS_DEFAULT;   /*!< 全局亮度 */
static uint8_t curR = 0U;
static uint8_t curG = 0U;
static uint8_t curB = 0U;

/* Private function prototypes -----------------------------------------------*/
static void BSP_WS2812_Encode(uint32_t grb);
static void BSP_WS2812_RefreshTarget(void);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 WS2812 驱动（重配 TIM2 + DMA）
  * @note   须在 MX_TIM2_Init 之后调用；CubeMX 的 ARR/CCR 为占位值，此处重配。
  */
void BSP_WS2812_Init(void)
{
    /* 按位周期重配 TIM2：170MHz / 212 = 801.9kHz ≈ 1.25us 位周期 */
    htim2.Init.Prescaler         = 0U;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = BOARD_WS2812_ARR;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

    /* DMA 通道：TIM2_CH1 请求，字传输（TIM2 为 32bit 定时器） */
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_tim2_ch1.Instance                 = DMA1_Channel3;
    hdma_tim2_ch1.Init.Request             = DMA_REQUEST_TIM2_CH1;
    hdma_tim2_ch1.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tim2_ch1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tim2_ch1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_ch1.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_tim2_ch1.Init.Mode                = DMA_NORMAL;
    hdma_tim2_ch1.Init.Priority            = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);

    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, BOARD_IRQPRIO_LED_DMA, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    /* 上电熄灭 */
    BSP_WS2812_SetColor(0U, 0U, 0U);
    BSP_WS2812_Poll();
}

/**
  * @brief  设置全局亮度（0~255，线性缩放）
  */
void BSP_WS2812_SetBrightness(uint8_t value)
{
    brightness = value;
    BSP_WS2812_RefreshTarget();
}

/**
  * @brief  读取当前全局亮度
  */
uint8_t BSP_WS2812_GetBrightness(void)
{
    return brightness;
}

/**
  * @brief  设置目标颜色（缓存，Poll 时发送）
  * @param  r/g/b 原始颜色分量（未缩放亮度）
  */
void BSP_WS2812_SetColor(uint8_t r, uint8_t g, uint8_t b)
{
    curR = r;
    curG = g;
    curB = b;
    BSP_WS2812_RefreshTarget();
}

/**
  * @brief  后台轮询：颜色有更新且 DMA 空闲时发送一帧
  */
void BSP_WS2812_Poll(void)
{
    if ((pending != 0U) && (busy == 0U))
    {
        busy    = 1U;
        pending = 0U;
        BSP_WS2812_Encode(targetGrb);
        if (HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, pwmBuf, WS2812_BUF_LEN) != HAL_OK)
        {
            busy = 0U;  /* 启动失败则下轮重试 */
            pending = 1U;
        }
    }
}

/**
  * @brief  DMA1 通道 3（TIM2_CH1）中断服务
  */
void BSP_WS2812_DmaIRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_tim2_ch1);
}

/**
  * @brief  HAL PWM DMA 发送完成回调：停止 PWM 释放通道
  * @note   本工程仅 TIM2_CH1 使用 PWM DMA，回调独占。
  */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim == &htim2) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
    {
        HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
        busy = 0U;
    }
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  将 24bit 颜色编码为 CCR 序列
  * @param  grb 发送序颜色字（bit23 首发）
  */
static void BSP_WS2812_Encode(uint32_t grb)
{
    uint32_t i;

    for (i = 0U; i < WS2812_BIT_COUNT; i++)
    {
        pwmBuf[i] = ((grb & (1UL << (23U - i))) != 0U) ? BOARD_WS2812_CCR_1 : BOARD_WS2812_CCR_0;
    }
    for (; i < WS2812_BUF_LEN; i++)
    {
        pwmBuf[i] = 0U;     /* reset 码：持续低电平 */
    }
}

/**
  * @brief  按亮度缩放当前颜色并生成发送序颜色字，标记待发送
  */
static void BSP_WS2812_RefreshTarget(void)
{
    uint32_t r = ((uint32_t)curR * (uint32_t)brightness + 127U) / 255U;
    uint32_t g = ((uint32_t)curG * (uint32_t)brightness + 127U) / 255U;
    uint32_t b = ((uint32_t)curB * (uint32_t)brightness + 127U) / 255U;

#if (BOARD_WS2812_ORDER_RGB != 0)
    targetGrb = (r << 16) | (g << 8) | b;
#else
    targetGrb = (g << 16) | (r << 8) | b;   /* WS2812B 标准 GRB 序 */
#endif
    pending = 1U;
}
