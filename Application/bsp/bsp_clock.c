/**
  ******************************************************************************
  * @file    bsp_clock.c
  * @brief   时钟切换 BSP（HSI -> HSE 20MHz，SYSCLK 维持 170MHz）
  * @note    CubeMX 生成的 SystemClock_Config 使用 HSI16 作 PLL 源；板载晶振
  *          实为 20MHz（O_IN/O_OUT），HSI ±1% 精度对 1Mbps CAN 位时序裕量
  *          不足，上电后应尽早切换到 HSE：
  *          HSE 20MHz / PLLM 5 * PLLN 85 / PLLR 2 = 170MHz（与 HSI 配置同频，
  *          外设分频参数无需变更）。
  *          切换失败（晶振异常）时自动回退 HSI 配置并返回错误，由应用层
  *          打印告警，系统仍可降级运行。
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
#include "bsp_clock.h"

/* Private variables ---------------------------------------------------------*/
static uint8_t clockOnHse = 0U;

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef BSP_CLOCK_ConfigPll(uint32_t pllSource, uint32_t pllm);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  切换系统时钟 PLL 源为 HSE
  * @note   在外设初始化早期调用（切换前后 SYSCLK 均为 170MHz，
  *         已初始化外设的分频寄存器保持有效）。
  * @retval HAL_OK 已切换；HAL_ERROR HSE 启动失败，已回退 HSI 配置
  */
HAL_StatusTypeDef BSP_CLOCK_SwitchToHse(void)
{
    RCC_OscInitTypeDef oscInit = {0};

    /* 启动 HSE 并等待就绪（HAL 内部带超时） */
    oscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscInit.HSEState       = RCC_HSE_ON;
    oscInit.PLL.PLLState   = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&oscInit) != HAL_OK)
    {
        clockOnHse = 0U;
        return HAL_ERROR;
    }

    /* 重配 PLL 源为 HSE：20/5*85/2 = 170MHz */
    if (BSP_CLOCK_ConfigPll(RCC_PLLSOURCE_HSE, RCC_PLLM_DIV5) != HAL_OK)
    {
        /* 回退 HSI 配置：16/4*85/2 = 170MHz */
        (void)BSP_CLOCK_ConfigPll(RCC_PLLSOURCE_HSI, RCC_PLLM_DIV4);
        clockOnHse = 0U;
        return HAL_ERROR;
    }

    clockOnHse = 1U;
    return HAL_OK;
}

/**
  * @brief  查询当前 PLL 源是否为 HSE
  */
uint8_t BSP_CLOCK_IsHse(void)
{
    return clockOnHse;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  以指定源重配 PLL 并切回 PLL 供 SYSCLK
  * @note   流程：SYSCLK 暂切 HSI16 -> 停 PLL 重配 -> 回切 PLL。
  *         全程 FLASH_LATENCY_4（对低频运行同样安全）。
  */
static HAL_StatusTypeDef BSP_CLOCK_ConfigPll(uint32_t pllSource, uint32_t pllm)
{
    RCC_OscInitTypeDef oscInit = {0};
    RCC_ClkInitTypeDef clkInit = {0};

    /* SYSCLK 暂切 HSI16，释放 PLL */
    clkInit.ClockType    = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
                         | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clkInit.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clkInit.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clkInit.APB1CLKDivider = RCC_HCLK_DIV1;
    clkInit.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clkInit, FLASH_LATENCY_4) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 重配 PLL（HSI 保持开启作为回退源） */
    oscInit.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    oscInit.HSIState       = RCC_HSI_ON;
    oscInit.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscInit.PLL.PLLState   = RCC_PLL_ON;
    oscInit.PLL.PLLSource  = pllSource;
    oscInit.PLL.PLLM       = pllm;
    oscInit.PLL.PLLN       = 85U;
    oscInit.PLL.PLLP       = RCC_PLLP_DIV2;
    oscInit.PLL.PLLQ       = RCC_PLLQ_DIV2;
    oscInit.PLL.PLLR       = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&oscInit) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* SYSCLK 切回 PLL */
    clkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    return HAL_RCC_ClockConfig(&clkInit, FLASH_LATENCY_4);
}
