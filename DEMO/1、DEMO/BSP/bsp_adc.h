/**
  ******************************************************************************
  * @file    bsp_adc.h
  * @brief   ADC 采样 BSP 头文件（电流/母线电压注入组 + 温度规则组）
  ******************************************************************************
  */

#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief 一次注入组采样的换算结果
  */
typedef struct
{
    float Iu;               /*!< U 相电流 [A]（低侧采样窗口内有效） */
    float Iv;               /*!< V 相电流 [A] */
    float Iw;               /*!< W 相电流 [A]（三线制由 Iu+Iv 重构：Iw = -(Iu+Iv)） */
    float Ibus;             /*!< 母线电流 [A]（低侧采样时刻≈0，仅诊断用） */
    float Vbus;             /*!< 母线电压 [V]（轻度滤波） */
    uint16_t RawU;          /*!< U 相电流通道原始值 [LSB]（诊断） */
    uint16_t RawV;          /*!< V 相电流通道原始值 [LSB]（诊断） */
    uint16_t RawIbus;       /*!< 母线电流通道原始值 [LSB]（诊断） */
    uint16_t RawVbus;       /*!< 母线电压通道原始值 [LSB]（诊断） */
    uint8_t CurSat;         /*!< 1 = 电流通道原始值贴轨（放大器/ADC 饱和） */
} ADC_FocMeasTypeDef;

/**
  * @brief FOC 周期回调类型（JEOS ISR 上下文）
  */
typedef void (*ADC_FocCallbackTypeDef)(const ADC_FocMeasTypeDef *meas);

/* Exported functions prototypes ---------------------------------------------*/
void              BSP_ADC_Init(void);
HAL_StatusTypeDef BSP_ADC_MeasureOffsets(uint32_t samples);
void              BSP_ADC_StartFocIrq(ADC_FocCallbackTypeDef callback);
void              BSP_ADC_GetOffsets(float *offsU, float *offsV, float *offsBus);
void              BSP_ADC_SetOffsets(float offsU, float offsV, float offsBus);
float             BSP_ADC_GetVbus(void);
float             BSP_ADC_ReadMosTemp(void);
float             BSP_ADC_ReadMotorTemp(void);
void              BSP_ADC_GetLastMeas(ADC_FocMeasTypeDef *meas);
uint32_t          BSP_ADC_GetIrqCount(void);

/* 中断服务入口（由 stm32g4xx_it.c 用户区调用） */
void              BSP_ADC_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ADC_H */
