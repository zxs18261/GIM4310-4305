/**
  ******************************************************************************
  * @file    bsp_adc.c
  * @brief   ADC 采样 BSP（电流/母线电压注入组 + 温度规则组）
  * @note    采样架构：
  *          - 注入组由 TIM1 TRGO2（OC4，计数器峰值附近）硬件触发，20kHz：
  *            ADC1 注入序列 = 相电流(U) + 母线电流 + 母线电压，
  *            ADC2 注入序列 = 相电流(V)；两 ADC 同触发准同步采样；
  *          - ADC1 JEOS（注入序列转换完成）中断即电流环节拍，
  *            换算物理量后回调控制层；
  *          - NTC 温度走规则组，后台软件触发轮询（毫秒级）；
  *          - 电流换算：I = (raw - offset) * LSB / (增益*采样电阻)，
  *            偏置在上电封波状态下实测平均（运放偏置 + 1.65V 基准合并补偿）；
  *          - CubeMX 生成的 ADC 参数（软件触发/单通道）为占位值，本模块重配。
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
#include "bsp_adc.h"
#include "board_config.h"
#include "adc.h"
#include "opamp.h"
#include <math.h>

/* Private variables ---------------------------------------------------------*/
static float offsetU   = 2048.0f;   /*!< U 相电流通道零流偏置 [LSB] */
static float offsetV   = 2048.0f;   /*!< V 相电流通道零流偏置 [LSB] */
static float offsetBus = 2048.0f;   /*!< 母线电流通道零流偏置 [LSB] */
static volatile float vbusFilt = 0.0f;  /*!< 母线电压轻度低通结果 [V] */
static ADC_FocCallbackTypeDef focCallback = NULL;

/* Private function prototypes -----------------------------------------------*/
static float BSP_ADC_NtcToTemp(uint16_t raw, float r25, float beta);
static float BSP_ADC_ReadRegularOnce(ADC_HandleTypeDef *hadc);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 ADC（重配注入组/触发/校准）
  * @note   须在 MX_ADC1/2_Init 与 BSP_PWM_Init 之后调用（依赖 TIM1 触发源配置）。
  */
void BSP_ADC_Init(void)
{
    ADC_InjectionConfTypeDef jConfig = {0};

    /* 启动三路运放（MX 仅完成 Init；standalone 模式、外部反馈网络增益 25） */
    if (HAL_OPAMP_Start(&hopamp1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_OPAMP_Start(&hopamp2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_OPAMP_Start(&hopamp3) != HAL_OK)
    {
        Error_Handler();
    }

    /* 单端自校准（内部失调），须在使能前执行 */
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    /* ---------------- ADC1 注入组：相电流 U + 母线电流 + 母线电压 ------------ */
    jConfig.InjectedSamplingTime         = BOARD_ADC_CUR_SMPTIME;
    jConfig.InjectedSingleDiff           = ADC_SINGLE_ENDED;
    jConfig.InjectedOffsetNumber         = ADC_OFFSET_NONE;
    jConfig.InjectedOffset               = 0U;
    jConfig.InjectedNbrOfConversion      = 3U;
    jConfig.InjectedDiscontinuousConvMode = DISABLE;
    jConfig.AutoInjectedConv             = DISABLE;
    jConfig.QueueInjectedContext         = DISABLE;
    jConfig.ExternalTrigInjecConv        = ADC_EXTERNALTRIGINJEC_T1_TRGO2;
    jConfig.ExternalTrigInjecConvEdge    = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    jConfig.InjecOversamplingMode        = DISABLE;

    jConfig.InjectedRank    = ADC_INJECTED_RANK_1;
    jConfig.InjectedChannel = BOARD_ADC1_JCH_CUR;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &jConfig) != HAL_OK)
    {
        Error_Handler();
    }
    jConfig.InjectedRank    = ADC_INJECTED_RANK_2;
    jConfig.InjectedChannel = BOARD_ADC1_JCH_IBUS;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &jConfig) != HAL_OK)
    {
        Error_Handler();
    }
    jConfig.InjectedRank         = ADC_INJECTED_RANK_3;
    jConfig.InjectedChannel      = BOARD_ADC1_JCH_VBUS;
    jConfig.InjectedSamplingTime = BOARD_ADC_VBUS_SMPTIME;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &jConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* ---------------- ADC2 注入组：相电流 V --------------------------------- */
    jConfig.InjectedSamplingTime    = BOARD_ADC_CUR_SMPTIME;
    jConfig.InjectedNbrOfConversion = 1U;
    jConfig.InjectedRank            = ADC_INJECTED_RANK_1;
    jConfig.InjectedChannel         = BOARD_ADC2_JCH_CUR;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &jConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* ---------------- 规则组：NTC 温度通道 ----------------------------------
       必须在注入组启动之前完成配置：注入组经硬件触发常年运行（JADSTART=1）
       后，HAL_ADC_ConfigChannel 会静默跳过 SMPR 采样时间写入。
       每个 ADC 仅有一个 NTC 通道，rank1 一次配好，运行期只做软件启动。 */
    {
        ADC_ChannelConfTypeDef chConfig = {0};

        chConfig.Rank         = ADC_REGULAR_RANK_1;
        chConfig.SamplingTime = BOARD_ADC_NTC_SMPTIME;
        chConfig.SingleDiff   = ADC_SINGLE_ENDED;
        chConfig.OffsetNumber = ADC_OFFSET_NONE;
        chConfig.Offset       = 0U;

        chConfig.Channel = BOARD_ADC1_RCH_MOSNTC;
        if (HAL_ADC_ConfigChannel(&hadc1, &chConfig) != HAL_OK)
        {
            Error_Handler();
        }
        chConfig.Channel = BOARD_ADC2_RCH_MOTNTC;
        if (HAL_ADC_ConfigChannel(&hadc2, &chConfig) != HAL_OK)
        {
            Error_Handler();
        }
    }

    /* 使能 ADC 并挂起注入组等待硬件触发 */
    if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  测量电流通道零流偏置（阻塞，封波状态下调用）
  * @note   依赖 TIM1 计数器已运行提供触发；MOE 必须处于关断（无相电流）。
  * @param  samples 平均次数（建议 >= 256）
  * @retval HAL_OK 成功；HAL_TIMEOUT 等待触发超时（TIM1 未运行）
  */
HAL_StatusTypeDef BSP_ADC_MeasureOffsets(uint32_t samples)
{
    uint32_t i;
    uint32_t timeout;
    float sumU   = 0.0f;
    float sumV   = 0.0f;
    float sumBus = 0.0f;

    for (i = 0U; i < samples; i++)
    {
        /* 等待一次注入序列完成（20kHz 触发，50us 周期） */
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOS);
        timeout = 1000000U;
        while (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOS) == 0U)
        {
            if (--timeout == 0U)
            {
                return HAL_TIMEOUT;
            }
        }

        sumU   += (float)hadc1.Instance->JDR1;
        sumBus += (float)hadc1.Instance->JDR2;
        sumV   += (float)hadc2.Instance->JDR1;
    }

    offsetU   = sumU / (float)samples;
    offsetV   = sumV / (float)samples;
    offsetBus = sumBus / (float)samples;

    /* 偏置合理性检查：理论值 = 1.65V 基准对应约半量程 */
    if ((offsetU < BOARD_ADC_OFFSET_MIN) || (offsetU > BOARD_ADC_OFFSET_MAX) ||
        (offsetV < BOARD_ADC_OFFSET_MIN) || (offsetV > BOARD_ADC_OFFSET_MAX) ||
        (offsetBus < BOARD_ADC_OFFSET_MIN) || (offsetBus > BOARD_ADC_OFFSET_MAX))
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/**
  * @brief  注册电流环回调并使能 JEOS 中断（进入运行节拍）
  * @param  callback 电流环入口（ISR 上下文执行）
  * @retval None
  */
void BSP_ADC_StartFocIrq(ADC_FocCallbackTypeDef callback)
{
    focCallback = callback;

    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOS);
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOS);

    HAL_NVIC_SetPriority(ADC1_2_IRQn, BOARD_IRQPRIO_FOC, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

/**
  * @brief  读取电流通道零流偏置（监控/诊断显示用）
  * @param  offsU/offsV/offsBus 偏置输出 [LSB]
  * @retval None
  */
void BSP_ADC_GetOffsets(float *offsU, float *offsV, float *offsBus)
{
    *offsU   = offsetU;
    *offsV   = offsetV;
    *offsBus = offsetBus;
}

/**
  * @brief  写入电流通道零流偏置（调试注入预留，正常路径为上电实测）
  * @param  offsU/offsV/offsBus 偏置 [LSB]
  * @retval None
  */
void BSP_ADC_SetOffsets(float offsU, float offsV, float offsBus)
{
    offsetU   = offsU;
    offsetV   = offsV;
    offsetBus = offsBus;
}

/**
  * @brief  读取滤波后母线电压 [V]（后台/保护用）
  */
float BSP_ADC_GetVbus(void)
{
    return vbusFilt;
}

/**
  * @brief  读取 MOS 管温度 [摄氏度]（阻塞轮询规则组，后台调用）
  * @retval 温度 [C]；传感器开路/短路返回 BOARD_NTC_FAULT_TEMP
  */
float BSP_ADC_ReadMosTemp(void)
{
    float raw = BSP_ADC_ReadRegularOnce(&hadc1);

    return BSP_ADC_NtcToTemp((uint16_t)raw, BOARD_NTC_MOS_R25, BOARD_NTC_MOS_BETA);
}

/**
  * @brief  读取电机绕组温度 [摄氏度]（阻塞轮询规则组，后台调用）
  * @retval 温度 [C]；传感器开路/短路返回 BOARD_NTC_FAULT_TEMP
  */
float BSP_ADC_ReadMotorTemp(void)
{
    float raw = BSP_ADC_ReadRegularOnce(&hadc2);

    return BSP_ADC_NtcToTemp((uint16_t)raw, BOARD_NTC_MOT_R25, BOARD_NTC_MOT_BETA);
}

/**
  * @brief  ADC1_2 中断服务（stm32g4xx_it.c 用户区调用）
  * @note   JEOS 即电流环节拍：读取原始值、换算物理量、回调控制层。
  *         寄存器级操作，不经 HAL 状态机。
  */
void BSP_ADC_IRQHandler(void)
{
    ADC_TypeDef *adc1 = hadc1.Instance;
    ADC_TypeDef *adc2 = hadc2.Instance;

    if ((adc1->ISR & ADC_ISR_JEOS) != 0U)
    {
        ADC_FocMeasTypeDef meas;
        float vbusRaw;
        uint32_t rawU;
        uint32_t rawV;

        adc1->ISR = ADC_ISR_JEOS;   /* 写 1 清除 */

        rawU = adc1->JDR1;
        rawV = adc2->JDR1;

        /* 贴轨检测：原始值达到轨附近说明放大器/ADC 饱和，实际电流超出
           测量范围（此时换算值被钳位，纯阈值比较无法发现过流），
           上报控制层按过流处理。该检测同时兜底偏置不对称导致的
           单方向量程收缩问题 */
        meas.CurSat = ((rawU <= BOARD_ADC_RAIL_LOW) || (rawU >= BOARD_ADC_RAIL_HIGH) ||
                       (rawV <= BOARD_ADC_RAIL_LOW) || (rawV >= BOARD_ADC_RAIL_HIGH)) ? 1U : 0U;

        /* 电流换算：低侧 shunt 差分放大，偏置减除后按 LSB 折算 */
        meas.Iu   = ((float)rawU - offsetU)         * BOARD_CUR_LSB * BOARD_CUR_SIGN_U;
        meas.Ibus = ((float)adc1->JDR2 - offsetBus) * BOARD_CUR_LSB * BOARD_CUR_SIGN_BUS;
        meas.Iv   = ((float)rawV - offsetV)         * BOARD_CUR_LSB * BOARD_CUR_SIGN_V;

        /* 母线电压：分压折算 + 一阶低通（截止约 160Hz，见 BOARD_VBUS_LPF_ALPHA） */
        vbusRaw  = (float)adc1->JDR3 * BOARD_VBUS_LSB;
        vbusFilt += BOARD_VBUS_LPF_ALPHA * (vbusRaw - vbusFilt);
        meas.Vbus = vbusFilt;

        if (focCallback != NULL)
        {
            focCallback(&meas);
        }
    }
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  软件触发单次规则组转换（NTC 慢通道专用）
  * @note   - 规则序列/采样时间已在 BSP_ADC_Init 中一次性配好，此处只做
  *           启动-轮询-取值；单次模式下序列结束 ADSTART 自动清零，
  *           HAL 状态自动收回 READY，可反复调用；
  *         - 严禁调用 HAL_ADC_Stop：本 HAL 版本会连同注入组一起停止并
  *           关闭 ADC，导致硬件触发的电流环节拍永久丢失；
  *         - 注入组优先级高于规则组，触发冲突时硬件自动仲裁（规则转换
  *           被注入序列打断后自动恢复），本函数仅在后台低频调用，
  *           阻塞几微秒。
  * @param  hadc ADC 句柄（ADC1 = MOS NTC，ADC2 = 电机 NTC）
  * @retval 12bit 原始值；失败返回 0
  */
static float BSP_ADC_ReadRegularOnce(ADC_HandleTypeDef *hadc)
{
    float result = 0.0f;

    if (HAL_ADC_Start(hadc) == HAL_OK)
    {
        if (HAL_ADC_PollForConversion(hadc, 2U) == HAL_OK)
        {
            result = (float)HAL_ADC_GetValue(hadc);
        }
    }
    return result;
}

/**
  * @brief  NTC 原始值转温度
  * @note   电路拓扑：上拉电阻 BOARD_NTC_PULLUP 接 VCC，NTC 接 GND，
  *         ADC 参考与 VCC 同源，比值式测量与电源电压无关。
  *         beta 方程：T = 1 / (1/T25 + ln(R/R25)/B) - 273.15。
  * @param  raw  12bit 原始值
  * @param  r25  NTC 25 摄氏度阻值 [Ohm]
  * @param  beta NTC B 常数 [K]
  * @retval 温度 [摄氏度]；采样值异常（开路/短路）返回 BOARD_NTC_FAULT_TEMP
  */
static float BSP_ADC_NtcToTemp(uint16_t raw, float r25, float beta)
{
    float ratio;
    float rntc;
    float tempK;

    if ((raw < 8U) || (raw > 4088U))
    {
        /* 开路或短路：返回故障温度值交由保护层处理 */
        return BOARD_NTC_FAULT_TEMP;
    }

    ratio = (float)raw / 4096.0f;
    rntc  = BOARD_NTC_PULLUP * ratio / (1.0f - ratio);
    tempK = 1.0f / ((1.0f / 298.15f) + logf(rntc / r25) / beta);

    return tempK - 273.15f;
}
