/**
  ******************************************************************************
  * @file    bsp_encoder.c
  * @brief   MT6701 磁编码器 BSP（GPIO 位带模拟 SSI）
  * @note    - 网表确认接线为 CSN=PB4 / CLK=PB5 / DO=PB6：PB5 无 SPI SCK 复用、
  *            PB6 无 SPI MISO 复用，硬件 SPI 不可用，采用 GPIO 位带模拟；
  *            （CubeMX 的 SPI1(PB3/PB4/PB5)与 PC11 片选配置与实物不符，
  *            初始化时整体回收并反配为 GPIO）
  *          - SSI 帧 24bit：D[13:0] 角度 + Mg[3:0] 磁场状态 + CRC[5:0]，
  *            要求精确 24 个 CLK 脉冲；
  *          - 时序：CSN 下降沿开帧，DO 在 CLK 上升沿移出、下降沿后采样；
  *            CLK 高/低电平最短 30ns，位带速率约 5MHz，全帧约 5us，
  *            供 20kHz 电流环 ISR 调用；
  *          - CRC6 多项式 X^6+X^1+1，覆盖高 18bit，初值 0；
  *          - 上电就绪时间最大 32ms，首次读取前由上层保证延时；
  *          - 方向反转为运行时设置（预定位校准写入，参数持久化）。
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
#include "bsp_encoder.h"
#include "board_config.h"
#include "spi.h"

/* Private defines -----------------------------------------------------------*/
#define ENC_CONSECUTIVE_ERR_MAX (0xFFU)

/**
  * @brief 半时钟延时：12 NOP 约 70ns（另叠加环内 GPIO 访问开销），
  *        相对 MT6701 TCLKL/TCLKH 最小 30ns 留有 2 倍以上裕量
  */
#define ENC_HALF_CLK_DELAY()    do { __NOP(); __NOP(); __NOP(); __NOP(); \
                                     __NOP(); __NOP(); __NOP(); __NOP(); \
                                     __NOP(); __NOP(); __NOP(); __NOP(); } while (0)

/* Private variables ---------------------------------------------------------*/
static uint8_t encInvert = 0U;      /*!< 方向反转标志（校准结果，运行时设置） */

/* Private function prototypes -----------------------------------------------*/
static uint8_t BSP_ENC_Crc6(uint32_t data18);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化编码器接口（回收 CubeMX 的 SPI 配置，改配位带 GPIO）
  * @note   须在 MX_SPI1_Init 与 MX_GPIO_Init 之后调用。
  */
void BSP_ENC_Init(void)
{
    GPIO_InitTypeDef gpioInit = {0};

    /* CubeMX 将 SPI1 配置在 PB3/PB4/PB5（与实物接线不符），先整体回收：
       DeInit 释放外设与复用引脚，随后关闭 SPI1 时钟 */
    (void)HAL_SPI_DeInit(&hspi1);
    __HAL_RCC_SPI1_CLK_DISABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 输出空闲态先行写入，避免配置为输出瞬间出现毛刺：CSN 高、CLK 低 */
    HAL_GPIO_WritePin(BOARD_ENC_GPIO_PORT, BOARD_ENC_CSN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BOARD_ENC_GPIO_PORT, BOARD_ENC_CLK_PIN, GPIO_PIN_RESET);

    /* CSN / CLK：推挽输出 */
    gpioInit.Pin   = BOARD_ENC_CSN_PIN | BOARD_ENC_CLK_PIN;
    gpioInit.Mode  = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull  = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BOARD_ENC_GPIO_PORT, &gpioInit);

    /* DO：输入（MT6701 推挽输出，无需上下拉） */
    gpioInit.Pin  = BOARD_ENC_DO_PIN;
    gpioInit.Mode = GPIO_MODE_INPUT;
    gpioInit.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BOARD_ENC_GPIO_PORT, &gpioInit);
}

/**
  * @brief  设置方向反转标志（预定位校准结果）
  */
void BSP_ENC_SetInvert(uint8_t invert)
{
    encInvert = (invert != 0U) ? 1U : 0U;
}

/**
  * @brief  读取方向反转标志
  */
uint8_t BSP_ENC_GetInvert(void)
{
    return encInvert;
}

/**
  * @brief  读取一次编码器角度（ISR 安全，阻塞约 5us）
  * @param  data 编码器数据结构体（Raw/Turns/状态在内部更新）
  * @retval 本次读取状态；CRC/磁场错误时 Raw/Turns 保持上次有效值
  */
ENC_StatusTypeDef BSP_ENC_Read(ENC_DataTypeDef *data)
{
    GPIO_TypeDef *port = BOARD_ENC_GPIO_PORT;
    uint32_t frame = 0U;
    uint32_t i;
    uint16_t raw;
    uint8_t  mg;
    int32_t  delta;

    /* CSN 下降沿开帧；CSN 到首个 CLK 边沿 >= 100ns */
    port->BRR = BOARD_ENC_CSN_PIN;
    ENC_HALF_CLK_DELAY();

    /* 24 位：上升沿移出 -> 下降沿后采样 */
    for (i = 0U; i < 24U; i++)
    {
        port->BSRR = BOARD_ENC_CLK_PIN;
        ENC_HALF_CLK_DELAY();
        port->BRR = BOARD_ENC_CLK_PIN;
        ENC_HALF_CLK_DELAY();
        frame = (frame << 1) | ((port->IDR >> BOARD_ENC_DO_PIN_POS) & 0x01U);
    }

    /* CSN 拉高结束帧 */
    port->BSRR = BOARD_ENC_CSN_PIN;

    /* CRC 校验：覆盖高 18bit（角度 14 + 状态 4） */
    if (BSP_ENC_Crc6(frame >> 6) != (uint8_t)(frame & 0x3FU))
    {
        data->CrcErrCnt++;
        if (data->ConsecutiveErr < ENC_CONSECUTIVE_ERR_MAX)
        {
            data->ConsecutiveErr++;
        }
        data->Status = ENC_ERR_CRC;
        return ENC_ERR_CRC;
    }

    mg = (uint8_t)((frame >> 6) & 0x0FU);
    data->MgStatus = mg;

    /* Mg[1:0]：01=磁场过强，10=磁场过弱 */
    if ((mg & 0x03U) != 0U)
    {
        data->FieldErrCnt++;
        if (data->ConsecutiveErr < ENC_CONSECUTIVE_ERR_MAX)
        {
            data->ConsecutiveErr++;
        }
        data->Status = ENC_ERR_FIELD;
        return ENC_ERR_FIELD;
    }

    raw = (uint16_t)(frame >> 10);

    if (encInvert != 0U)
    {
        raw = (uint16_t)((ENC_CPR - raw) & ENC_CPR_MASK);
    }

    /* 多圈展开：半圈跳变判定（20kHz 采样下等效转速裕量巨大） */
    delta = (int32_t)raw - (int32_t)data->Raw;
    if (delta > (int32_t)(ENC_CPR / 2U))
    {
        data->Turns--;
    }
    else if (delta < -(int32_t)(ENC_CPR / 2U))
    {
        data->Turns++;
    }

    data->Raw    = raw;
    data->ConsecutiveErr = 0U;

    /* Mg[3] 超速仅告警，角度仍有效 */
    if ((mg & 0x08U) != 0U)
    {
        data->Status = ENC_ERR_OVERSPEED;
        return ENC_ERR_OVERSPEED;
    }

    data->Status = ENC_OK;
    return ENC_OK;
}

/**
  * @brief  预置编码器状态（上电首读，避免多圈计数误跳）
  * @note   重试取得首个角度有效的帧（ENC_OK 或仅超速告警）作为基准，
  *         CRC 错与磁场异常帧的 Raw 未更新，不能用作基准。
  * @param  data 编码器数据结构体
  * @retval None
  */
void BSP_ENC_Preload(ENC_DataTypeDef *data)
{
    uint32_t retry;

    data->Turns          = 0;
    data->ConsecutiveErr = 0U;
    data->CrcErrCnt      = 0U;
    data->FieldErrCnt    = 0U;

    for (retry = 0U; retry < 8U; retry++)
    {
        ENC_DataTypeDef tmp = *data;
        ENC_StatusTypeDef st = BSP_ENC_Read(&tmp);

        if ((st == ENC_OK) || (st == ENC_ERR_OVERSPEED))
        {
            data->Raw      = tmp.Raw;
            data->MgStatus = tmp.MgStatus;
            break;
        }
    }
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CRC6 计算（X^6 + X + 1，初值 0，MSB 先入）
  * @param  data18 待校验的 18bit 数据（低 18 位有效）
  * @retval 6bit CRC
  */
static uint8_t BSP_ENC_Crc6(uint32_t data18)
{
    uint32_t crc = 0U;
    int32_t  i;

    for (i = 17; i >= 0; i--)
    {
        uint32_t bit = ((data18 >> (uint32_t)i) ^ (crc >> 5)) & 0x01U;
        crc = ((crc << 1) & 0x3FU);
        if (bit != 0U)
        {
            crc ^= 0x03U;   /* X^6+X+1 -> 反馈 0b000011 */
        }
    }
    return (uint8_t)crc;
}
