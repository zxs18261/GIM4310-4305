/**
  ******************************************************************************
  * @file    bsp_pwm.h
  * @brief   TIM1 三相互补 PWM BSP 头文件
  ******************************************************************************
  */

#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/
void    BSP_PWM_Init(void);
void    BSP_PWM_Enable(void);
void    BSP_PWM_Disable(void);
uint8_t BSP_PWM_IsEnabled(void);
void    BSP_PWM_SetDuty(float dutyU, float dutyV, float dutyW);
void    BSP_PWM_SetDutyZero(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PWM_H */
