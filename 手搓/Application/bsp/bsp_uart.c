/**
  ******************************************************************************
  * @file    bsp_uart.c
  * @brief   USART3 调试串口 BSP（DMA TX 环形缓冲 + RXNE 中断接收）
  * @note    - TX：应用写入环形缓冲，空闲时启动 DMA 分段搬运，完全非阻塞；
  *            缓冲满时丢弃新数据并计数（调试链路不允许阻塞控制环）。
  *          - RX：RXNE 逐字节中断入环形缓冲，shell 在后台轮询取出。
  *          - CubeMX 生成的 MX_USART3_UART_Init 参数为占位默认值，
  *            本模块按 board_config.h 的实际波特率重新初始化。
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
#include "bsp_uart.h"
#include "board_config.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define UART_TX_BUF_SIZE        (2048U)     /*!< TX 环形缓冲大小（2 的幂） */
#define UART_RX_BUF_SIZE        (256U)      /*!< RX 环形缓冲大小（2 的幂） */
#define UART_PRINTF_MAX         (256U)      /*!< 单次 Printf 格式化上限 */

/* Private variables ---------------------------------------------------------*/
static DMA_HandleTypeDef hdma_usart3_tx;

static uint8_t  txBuf[UART_TX_BUF_SIZE];
static volatile uint32_t txHead = 0U;       /*!< 写入位置（应用侧） */
static volatile uint32_t txTail = 0U;       /*!< 读出位置（DMA 侧） */
static volatile uint32_t txDmaLen = 0U;     /*!< 当前 DMA 段长度，0 表示 DMA 空闲 */
static volatile uint32_t txDropCnt = 0U;    /*!< 缓冲满丢弃字节计数（诊断用） */

static uint8_t  rxBuf[UART_RX_BUF_SIZE];
static volatile uint32_t rxHead = 0U;
static volatile uint32_t rxTail = 0U;

/* Private function prototypes -----------------------------------------------*/
static void BSP_UART_KickDma(void);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化调试串口（重配波特率 + DMA + 中断）
  * @note   须在 MX_USART3_UART_Init 之后调用。
  */
void BSP_UART_Init(void)
{
    /* 按实际参数重新初始化 UART（覆盖 CubeMX 占位波特率） */
    huart3.Init.BaudRate = BOARD_UART_BAUDRATE;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }

    /* DMA 时钟与通道配置（CubeMX 未配置 DMA，此处自行接管） */
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_usart3_tx.Instance                 = DMA1_Channel2;
    hdma_usart3_tx.Init.Request             = DMA_REQUEST_USART3_TX;
    hdma_usart3_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode                = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority            = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_LINKDMA(&huart3, hdmatx, hdma_usart3_tx);

    /* 中断使能：DMA TX 完成 + UART（RXNE / TC 由 HAL 链路使用） */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, BOARD_IRQPRIO_UART_DMA, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_SetPriority(USART3_IRQn, BOARD_IRQPRIO_UART, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    /* 直接使能 RXNE 中断（接收不经 HAL 状态机，减小开销） */
    SET_BIT(huart3.Instance->CR1, USART_CR1_RXNEIE);
}

/**
  * @brief  向 TX 环形缓冲写入数据（非阻塞）
  * @param  data 数据指针
  * @param  len  长度
  * @retval 实际写入字节数（缓冲满时小于 len，差值计入丢弃计数）
  */
uint32_t BSP_UART_Write(const uint8_t *data, uint32_t len)
{
    uint32_t written = 0U;
    uint32_t head    = txHead;

    while (written < len)
    {
        uint32_t next = (head + 1U) & (UART_TX_BUF_SIZE - 1U);
        if (next == txTail)
        {
            txDropCnt += (len - written);
            break;  /* 缓冲满 */
        }
        txBuf[head] = data[written];
        head = next;
        written++;
    }
    txHead = head;

    BSP_UART_KickDma();
    return written;
}

/**
  * @brief  格式化打印到调试串口（非阻塞）
  * @retval 实际写入字节数
  */
uint32_t BSP_UART_Printf(const char *fmt, ...)
{
    char    buf[UART_PRINTF_MAX];
    va_list args;
    int     n;

    va_start(args, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (n <= 0)
    {
        return 0U;
    }
    if ((uint32_t)n >= sizeof(buf))
    {
        n = (int)sizeof(buf) - 1;   /* vsnprintf 截断时返回期望长度，按实际写入截断 */
    }
    return BSP_UART_Write((const uint8_t *)buf, (uint32_t)n);
}

/**
  * @brief  从 RX 环形缓冲读取一个字节
  * @retval 0~255 为有效字节；-1 表示无数据
  */
int32_t BSP_UART_ReadByte(void)
{
    int32_t c;

    if (rxTail == rxHead)
    {
        return -1;
    }
    c = (int32_t)rxBuf[rxTail];
    rxTail = (rxTail + 1U) & (UART_RX_BUF_SIZE - 1U);
    return c;
}

/**
  * @brief  查询 TX 环形缓冲剩余空间
  */
uint32_t BSP_UART_TxFree(void)
{
    uint32_t head = txHead;
    uint32_t tail = txTail;

    return (tail - head - 1U) & (UART_TX_BUF_SIZE - 1U);
}

/**
  * @brief  阻塞等待 TX 缓冲发空（仅限故障复位前等场景，不得在 ISR 中调用）
  */
void BSP_UART_Flush(void)
{
    while ((txHead != txTail) || (txDmaLen != 0U))
    {
        /* 等待 DMA 清空 */
    }
}

/**
  * @brief  USART3 中断服务（stm32g4xx_it.c 用户区调用）
  * @note   先于 HAL 消费 RXNE，再交 HAL 处理 DMA TX 收尾的 TC 中断。
  */
void BSP_UART_IRQHandler(void)
{
    USART_TypeDef *uart = huart3.Instance;

    /* 接收：逐字节入环，缓冲满则丢弃最旧数据以保持最新输入 */
    while ((uart->ISR & USART_ISR_RXNE) != 0U)
    {
        uint8_t  byte = (uint8_t)uart->RDR;
        uint32_t next = (rxHead + 1U) & (UART_RX_BUF_SIZE - 1U);

        if (next != rxTail)
        {
            rxBuf[rxHead] = byte;
            rxHead = next;
        }
    }

    /* 溢出错误清除（RXNE 消费不及时或线路噪声） */
    if ((uart->ISR & USART_ISR_ORE) != 0U)
    {
        uart->ICR = USART_ICR_ORECF;
    }
    /* 噪声/帧错误清除 */
    if ((uart->ISR & (USART_ISR_NE | USART_ISR_FE)) != 0U)
    {
        uart->ICR = USART_ICR_NECF | USART_ICR_FECF;
    }

    /* DMA TX 完成后的 TC 收尾由 HAL 处理 */
    HAL_UART_IRQHandler(&huart3);
}

/**
  * @brief  DMA1 通道 2（USART3_TX）中断服务
  */
void BSP_UART_DmaTxIRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart3_tx);
}

/**
  * @brief  HAL UART 发送完成回调：推进环形缓冲并续发下一段
  * @note   本工程 UART 仅 USART3 使用 HAL DMA 发送，回调独占。
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        txTail   = (txTail + txDmaLen) & (UART_TX_BUF_SIZE - 1U);
        txDmaLen = 0U;
        BSP_UART_KickDma();
    }
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  若 DMA 空闲且缓冲有数据，则启动下一段 DMA 传输
  * @note   段长取到缓冲物理末尾为止（环回部分下一段再发）。
  *         关中断保护判定-启动窗口，可从任务与 ISR 两个上下文安全调用。
  */
static void BSP_UART_KickDma(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if ((txDmaLen == 0U) && (txHead != txTail))
    {
        uint32_t head = txHead;
        uint32_t len;

        if (head > txTail)
        {
            len = head - txTail;
        }
        else
        {
            len = UART_TX_BUF_SIZE - txTail;    /* 先发到物理末尾 */
        }

        txDmaLen = len;
        if (HAL_UART_Transmit_DMA(&huart3, &txBuf[txTail], (uint16_t)len) != HAL_OK)
        {
            txDmaLen = 0U;  /* 启动失败（不应发生），放弃本段等待下次触发 */
        }
    }

    if (primask == 0U)
    {
        __enable_irq();
    }
}
