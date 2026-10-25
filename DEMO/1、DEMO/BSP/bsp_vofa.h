/**
  ******************************************************************************
  * @file    bsp_vofa.h
  * @brief   VOFA+ JustFloat 上位机数据帧 BSP 头文件
  * @note    协议：N 个小端 IEEE-754 float + 4 字节帧尾（+Inf = 0x7F800000），
  *          经 USART3（921600）DMA 非阻塞发送。
  ******************************************************************************
  */

#ifndef __BSP_VOFA_H
#define __BSP_VOFA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
/**
  * @brief JustFloat 通道数
  * @note  通道含义由应用层定义，本工程 13 通道（见 main.c 文件头注释）：
  *        ch0~ch11 = Iu/Iv/Ibus/Vbus/机械角/电角度/dutyU/dutyV/dutyW/Tmos/Tmot/CAN计数，
  *        ch12 = Iw（由 Iu+Iv 重构的 W 相电流）。
  *        通道数可自由修改；单帧长度 = CHANNEL_NUM*4 + 4 字节。
  */
#define BSP_VOFA_CHANNEL_NUM    (13U)

/* Exported functions prototypes ---------------------------------------------*/
void BSP_VOFA_Init(void);
void BSP_VOFA_SetChannel(uint8_t ch, float value);
void BSP_VOFA_Send(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_VOFA_H */
