/**
  ******************************************************************************
  * @file    bsp_vofa.c
  * @brief   VOFA+ JustFloat 上位机数据帧 BSP
 * @note    - JustFloat：N 个小端 float 紧接 4 字节帧尾（+Inf = 0x7F800000）；
 *            上位机 VOFA+ 选择“JustFloat”协议即可自动按通道解析；
 *          - 复用 BSP_UART 的非阻塞 DMA 发送（环形缓冲），不阻塞主循环/
 *            电流环；缓冲满时静默丢弃，保证实时性优先；
 *          - 帧尾每帧重新写入，避免固定帧尾只对特定通道数有效的经典问题。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_vofa.h"
#include "bsp_uart.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define VOFA_TAIL               (0x7F800000UL)      /*!< +Inf 作为帧尾 */

/* Private variables ---------------------------------------------------------*/
static float  vofaData[BSP_VOFA_CHANNEL_NUM];
static uint8_t vofaBuf[BSP_VOFA_CHANNEL_NUM * 4U + 4U];

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 VOFA 数据帧模块（清空通道）
  */
void BSP_VOFA_Init(void)
{
    memset(vofaData, 0, sizeof(vofaData));
}

/**
  * @brief  写入一个通道的浮点数据
  * @param  ch    通道号（0 ~ BSP_VOFA_CHANNEL_NUM-1）
  * @param  value 数据
  */
void BSP_VOFA_SetChannel(uint8_t ch, float value)
{
    if (ch < BSP_VOFA_CHANNEL_NUM)
    {
        vofaData[ch] = value;
    }
}

/**
  * @brief  组帧并发送一帧 JustFloat 数据
  */
void BSP_VOFA_Send(void)
{
    uint32_t tail = VOFA_TAIL;

    memcpy(vofaBuf, vofaData, sizeof(vofaData));
    memcpy(&vofaBuf[BSP_VOFA_CHANNEL_NUM * 4U], &tail, sizeof(tail));

    (void)BSP_UART_Write(vofaBuf, (uint32_t)sizeof(vofaBuf));
}
