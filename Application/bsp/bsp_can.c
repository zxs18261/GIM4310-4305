/**
  ******************************************************************************
  * @file    bsp_can.c
  * @brief   FDCAN1 通信 BSP（经典 CAN 模式）
  * @note    - FDCAN 内核时钟 = PCLK1 = 170MHz，位时序按波特率查表配置，
  *            采样点约 88%（CiA 推荐区间）；
  *          - 硬件滤波：仅接收本节点 ID 与广播 ID 到 FIFO0，其余全部丢弃，
  *            多电机总线下不占用 CPU；
  *          - RX FIFO0 新报文中断内直接取出并回调应用层协议解析（轻量）；
  *          - TX 走 TX FIFO，满则返回错误由上层决定丢弃/重试；
  *          - 120Ω 终端电阻经 TS5A3159 模拟开关切换（EN_120，电平语义
  *            由 board_config.h 宏定义）；
  *          - CubeMX 生成的位时序为非法占位值，本模块整体重配。
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
#include "bsp_can.h"
#include "board_config.h"
#include "fdcan.h"

/* Private types -------------------------------------------------------------*/

/**
  * @brief 位时序表项（内核时钟 170MHz）
  */
typedef struct
{
    uint16_t Prescaler;
    uint16_t Seg1;      /*!< 不含 SYNC 段 */
    uint8_t  Seg2;
    uint8_t  Sjw;
} CAN_BitTimingTypeDef;

/* Private constants ---------------------------------------------------------*/

/* 170MHz / (Prescaler * (1 + Seg1 + Seg2)) = 波特率，采样点 = (1+Seg1)/总tq */
static const CAN_BitTimingTypeDef bitTimingTable[CAN_BAUD_COUNT] =
{
    /* 1M:   170/5  = 34tq, SP = 30/34 = 88.2% */
    { 5U,   29U, 4U, 4U },
    /* 500k: 170/10 = 34tq */
    { 10U,  29U, 4U, 4U },
    /* 250k: 170/20 = 34tq */
    { 20U,  29U, 4U, 4U },
    /* 125k: 170/40 = 34tq */
    { 40U,  29U, 4U, 4U },
};

/* Private variables ---------------------------------------------------------*/
static CAN_RxCallbackTypeDef rxCallback = NULL;
static volatile uint32_t rxCount  = 0U;
static volatile uint32_t errCount = 0U;
static uint8_t termEnabled = 0U;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 CAN（重配位时序 + 滤波器 + 中断）
  * @note   须在 MX_FDCAN1_Init 之后调用。
  * @param  baud   波特率枚举
  * @param  nodeId 本节点 ID（标准帧，作硬件滤波）
  */
void BSP_CAN_Init(CAN_BaudTypeDef baud, uint16_t nodeId)
{
    FDCAN_FilterTypeDef filter = {0};
    const CAN_BitTimingTypeDef *bt;

    if (baud >= CAN_BAUD_COUNT)
    {
        baud = CAN_BAUD_1M;
    }
    bt = &bitTimingTable[baud];

    /* 重配前先停止（MX 初始化后处于停止态，此处兼容重复调用） */
    (void)HAL_FDCAN_Stop(&hfdcan1);
    (void)HAL_FDCAN_DeInit(&hfdcan1);

    hfdcan1.Init.ClockDivider       = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat        = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode               = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = ENABLE;
    hfdcan1.Init.TransmitPause      = DISABLE;
    hfdcan1.Init.ProtocolException  = DISABLE;
    hfdcan1.Init.NominalPrescaler   = bt->Prescaler;
    hfdcan1.Init.NominalSyncJumpWidth = bt->Sjw;
    hfdcan1.Init.NominalTimeSeg1    = bt->Seg1;
    hfdcan1.Init.NominalTimeSeg2    = bt->Seg2;
    /* 经典 CAN 模式下数据段位时序不使用，填合法值即可 */
    hfdcan1.Init.DataPrescaler      = 1U;
    hfdcan1.Init.DataSyncJumpWidth  = 4U;
    hfdcan1.Init.DataTimeSeg1       = 13U;
    hfdcan1.Init.DataTimeSeg2       = 2U;
    hfdcan1.Init.StdFiltersNbr      = 2U;
    hfdcan1.Init.ExtFiltersNbr      = 0U;
    hfdcan1.Init.TxFifoQueueMode    = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    /* 滤波器 0：本节点 ID（经典单 ID 匹配 -> FIFO0） */
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0U;
    filter.FilterType   = FDCAN_FILTER_DUAL;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = nodeId;
    filter.FilterID2    = nodeId;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    /* 滤波器 1：广播 ID + 配置协议 ID（board_config 宏） */
    filter.FilterIndex = 1U;
    filter.FilterID1   = BOARD_CAN_BROADCAST_ID;
    filter.FilterID2   = (uint32_t)(BOARD_CAN_CFG_ID_BASE + nodeId);
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    /* 未匹配帧与远程帧全部拒绝 */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    /* RX FIFO0 新报文中断 */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, BOARD_IRQPRIO_CAN, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  注册接收回调（ISR 上下文执行，须轻量）
  */
void BSP_CAN_RegisterRxCallback(CAN_RxCallbackTypeDef callback)
{
    rxCallback = callback;
}

/**
  * @brief  发送一帧经典 CAN 数据帧（非阻塞，入 TX FIFO）
  * @param  id   标准帧 ID
  * @param  data 数据
  * @param  len  数据长度 0~8
  * @retval HAL_OK 已入队；HAL_ERROR FIFO 满或参数非法
  */
HAL_StatusTypeDef BSP_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef txHeader = {0};

    if (len > 8U)
    {
        return HAL_ERROR;
    }

    txHeader.Identifier          = id;
    txHeader.IdType              = FDCAN_STANDARD_ID;
    txHeader.TxFrameType         = FDCAN_DATA_FRAME;
    txHeader.DataLength          = (uint32_t)len;          /* 本 HAL 版本 DLC 宏即字节数 */
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    txHeader.FDFormat            = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker       = 0U;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, (uint8_t *)data);
}

/**
  * @brief  切换 120 欧终端电阻（TS5A3159 模拟开关）
  * @param  enable 1 = 接入终端电阻
  */
void BSP_CAN_SetTermination(uint8_t enable)
{
    GPIO_PinState level;

#if (BOARD_CAN_TERM_ACTIVE_HIGH != 0)
    level = (enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
#else
    level = (enable != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET;
#endif
    HAL_GPIO_WritePin(EN_120_GPIO_Port, EN_120_Pin, level);
    termEnabled = (enable != 0U) ? 1U : 0U;
}

/**
  * @brief  查询终端电阻状态
  */
uint8_t BSP_CAN_GetTermination(void)
{
    return termEnabled;
}

/**
  * @brief  读取接收帧累计计数（诊断用）
  * @retval 接收帧数
  */
uint32_t BSP_CAN_GetRxCount(void)
{
    return rxCount;
}

/**
  * @brief  读取错误事件累计计数（诊断用）
  * @retval 错误计数（FIFO 读取失败 + 总线错误状态事件）
  */
uint32_t BSP_CAN_GetErrCount(void)
{
    return errCount;
}

/**
  * @brief  FDCAN1 中断线 0 服务（stm32g4xx_it.c 用户区调用）
  * @note   直接在 ISR 内取出 FIFO0 报文并回调，避免 HAL 通知链开销。
  */
void BSP_CAN_IRQHandler(void)
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if ((hfdcan1.Instance->IR & FDCAN_IR_RF0N) != 0U)
    {
        hfdcan1.Instance->IR = FDCAN_IR_RF0N;   /* 写 1 清除 */

        while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U)
        {
            if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
            {
                rxCount++;
                if ((rxCallback != NULL) && (rxHeader.IdType == FDCAN_STANDARD_ID))
                {
                    rxCallback(rxHeader.Identifier, rxData,
                               (uint8_t)rxHeader.DataLength);
                }
            }
            else
            {
                errCount++;
                break;
            }
        }
    }

    /* 其余中断标志（总线关闭等）清除并计数，由后台读取诊断 */
    if ((hfdcan1.Instance->IR & (FDCAN_IR_BO | FDCAN_IR_EP | FDCAN_IR_EW)) != 0U)
    {
        hfdcan1.Instance->IR = FDCAN_IR_BO | FDCAN_IR_EP | FDCAN_IR_EW;
        errCount++;
    }
}
