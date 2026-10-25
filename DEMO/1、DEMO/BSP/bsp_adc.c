/**
  ******************************************************************************
  * @file    bsp_adc.c
  * @brief   ADC 采样 BSP（电流/母线电压注入组 + 温度规则组）
  * @note    采样架构：
  *          - 注入组由 TIM1 TRGO2（OC4，计数器峰值附近）硬件触发，20kHz：
  *            ADC1 注入序列 = 相电流(U) + 母线电流 + 母线电压，
  *            ADC2 注入序列 = 相电流(V)；两 ADC 同触发准同步采样；
  *          - ADC1 JEOS（注入序列转换完成）中断即电流环节拍；
  *          - NTC 温度走规则组，后台软件触发轮询（毫秒级）；
  *          - 电流换算：I = (raw - offset) * VREF / (4096 * 增益 * 采样电阻)，
  *            换算常数见 board_config.h 的 BOARD_CUR_LSB（增益 24）；
  *            偏置在上电封波状态下实测平均（运放偏置 + 1.65V 基准合并补偿）；
  *          - CubeMX 生成的 ADC 参数（软件触发/单通道）为占位值，本模块重配。
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
static volatile float vbusFilt = 0.0f;
static volatile uint32_t irqCount = 0U;
static volatile ADC_FocMeasTypeDef lastMeas = {0};
static ADC_FocCallbackTypeDef focCallback = NULL;

/* Private function prototypes -----------------------------------------------*/
static float BSP_ADC_NtcToTemp(uint16_t raw, float r25, float beta);
static float BSP_ADC_ReadRegularOnce(ADC_HandleTypeDef *hadc);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 ADC（启动运放、重配注入组/触发/校准）
  * @note   须在 MX_ADC1/2_Init 与 BSP_PWM_Init 之后调用（依赖 TIM1 触发源）。
  */
void BSP_ADC_Init(void)
{
    ADC_InjectionConfTypeDef jConfig = {0};

    /* ★关键修正★：必须先使能 “扫描模式” 标志，否则多通道注入序列会被退化成单通道！
     *
     * 原因（见 Drivers/.../stm32g4xx_hal_adc_ex.c 的 HAL_ADCEx_InjectedConfigChannel）：
     *    if ((hadc->Init.ScanConvMode == ADC_SCAN_DISABLE) ||
     *        (pConfigInjected->InjectedNbrOfConversion == 1U))
     *    {  // 单通道分支：丢弃 InjectedNbrOfConversion，JL 固定为 0，只写 JSQ1
     *    }
     *    else
     *    {  // 多通道分支：按 rank 逐个累积 JSQ1..JSQ4，由最后一次调用写入 JSQR
     *    }
     * CubeMX 生成的 adc.c 里 ScanConvMode = ADC_SCAN_DISABLE（默认），
     * 结果 ADC1 注入序列只有 rank1（U 相电流）被真正配置：
     *   JSQR 实测 = 0x000018A0 → JL=0、JSQ1=CH12、JSQ2=JSQ3=0，
     *   于是 JDR2(母线电流)、JDR3(母线电压) 永远为 0。
     * G4 的 “扫描模式” 并非硬件位，仅被 HAL 用作软件分支判断（见 HAL 注释
     * "Scan mode is not present by hardware on this device, but used by software"），
     * 因此这里直接置位结构体成员即可，无需重新 HAL_ADC_Init。
     * 规则组 NbrOfConversion 保持 1（单通道 NTC），不受影响。 */
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;

    /* 启动三路运放（MX 仅完成 Init；standalone 模式、外部反馈网络增益 24） */
    if (HAL_OPAMP_Start(&hopamp1) != HAL_OK) { Error_Handler(); }
    if (HAL_OPAMP_Start(&hopamp2) != HAL_OK) { Error_Handler(); }
    if (HAL_OPAMP_Start(&hopamp3) != HAL_OK) { Error_Handler(); }

    /* 单端自校准（内部失调），须在使能前执行 */
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) { Error_Handler(); }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) { Error_Handler(); }

    /* ---------------- ADC1 注入组：相电流 U + 母线电流 + 母线电压 ------------ */
    jConfig.InjectedSamplingTime          = BOARD_ADC_CUR_SMPTIME;
    jConfig.InjectedSingleDiff            = ADC_SINGLE_ENDED;
    jConfig.InjectedOffsetNumber          = ADC_OFFSET_NONE;
    jConfig.InjectedOffset                = 0U;
    jConfig.InjectedNbrOfConversion       = 3U;
    jConfig.InjectedDiscontinuousConvMode = DISABLE;
    jConfig.AutoInjectedConv              = DISABLE;
    jConfig.QueueInjectedContext          = DISABLE;
    jConfig.ExternalTrigInjecConv         = ADC_EXTERNALTRIGINJEC_T1_TRGO2;
    jConfig.ExternalTrigInjecConvEdge     = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    jConfig.InjecOversamplingMode         = DISABLE;

    jConfig.InjectedRank    = ADC_INJECTED_RANK_1;
    jConfig.InjectedChannel = BOARD_ADC1_JCH_CUR;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &jConfig) != HAL_OK) { Error_Handler(); }

    jConfig.InjectedRank    = ADC_INJECTED_RANK_2;
    jConfig.InjectedChannel = BOARD_ADC1_JCH_IBUS;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &jConfig) != HAL_OK) { Error_Handler(); }

    jConfig.InjectedRank         = ADC_INJECTED_RANK_3;
    jConfig.InjectedChannel      = BOARD_ADC1_JCH_VBUS;
    jConfig.InjectedSamplingTime = BOARD_ADC_VBUS_SMPTIME;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &jConfig) != HAL_OK) { Error_Handler(); }

    /* ---------------- ADC2 注入组：相电流 V --------------------------------- */
    jConfig.InjectedSamplingTime    = BOARD_ADC_CUR_SMPTIME;
    jConfig.InjectedNbrOfConversion = 1U;
    jConfig.InjectedRank            = ADC_INJECTED_RANK_1;
    jConfig.InjectedChannel         = BOARD_ADC2_JCH_CUR;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &jConfig) != HAL_OK) { Error_Handler(); }

    /* ---------------- 规则组：NTC 温度通道（须在注入组启动之前配好） -------- */
    {
        ADC_ChannelConfTypeDef chConfig = {0};

        chConfig.Rank         = ADC_REGULAR_RANK_1;
        chConfig.SamplingTime = BOARD_ADC_NTC_SMPTIME;
        chConfig.SingleDiff   = ADC_SINGLE_ENDED;
        chConfig.OffsetNumber = ADC_OFFSET_NONE;
        chConfig.Offset       = 0U;

        chConfig.Channel = BOARD_ADC1_RCH_MOSNTC;
        if (HAL_ADC_ConfigChannel(&hadc1, &chConfig) != HAL_OK) { Error_Handler(); }
        chConfig.Channel = BOARD_ADC2_RCH_MOTNTC;
        if (HAL_ADC_ConfigChannel(&hadc2, &chConfig) != HAL_OK) { Error_Handler(); }
    }

    /* 使能 ADC 并挂起注入组等待硬件触发 */
    if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK) { Error_Handler(); }
    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief  测量电流通道零流偏置（阻塞，封波状态下调用）
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

    if (samples == 0U)
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < samples; i++)
    {
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
  * @brief  读取电流通道零流偏置
  */
void BSP_ADC_GetOffsets(float *offsU, float *offsV, float *offsBus)
{
    *offsU   = offsetU;
    *offsV   = offsetV;
    *offsBus = offsetBus;
}

/**
  * @brief  写入电流通道零流偏置（调试注入预留）
  */
void BSP_ADC_SetOffsets(float offsU, float offsV, float offsBus)
{
    offsetU   = offsU;
    offsetV   = offsV;
    offsetBus = offsBus;
}

/**
  * @brief  读取滤波后母线电压 [V]
  */
float BSP_ADC_GetVbus(void)
{
    return vbusFilt;
}

/**
  * @brief  读取 MOS 管温度 [摄氏度]
  */
float BSP_ADC_ReadMosTemp(void)
{
    float raw = BSP_ADC_ReadRegularOnce(&hadc1);

    return BSP_ADC_NtcToTemp((uint16_t)raw, BOARD_NTC_MOS_R25, BOARD_NTC_MOS_BETA);
}

/**
  * @brief  读取电机绕组温度 [摄氏度]
  */
float BSP_ADC_ReadMotorTemp(void)
{
    float raw = BSP_ADC_ReadRegularOnce(&hadc2);

    return BSP_ADC_NtcToTemp((uint16_t)raw, BOARD_NTC_MOT_R25, BOARD_NTC_MOT_BETA);
}

/**
  * @brief  读取最近一次注入组换算结果（后台显示用）
  * @note   lastMeas 由 20kHz 的 JEOS ISR 更新，主循环拷贝期间可能被刷新，
  *         为避免多字段“撕裂”（电流与电压来自不同节拍），拷贝时短暂关中断。
  *         该临界区仅几十个周期，不影响电流环节拍。
  */
void BSP_ADC_GetLastMeas(ADC_FocMeasTypeDef *meas)
{
    if (meas != NULL)
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        *meas = lastMeas;
        if (primask == 0U)
        {
            __enable_irq();
        }
    }
}

/**
  * @brief  读取电流环中断计数
  */
uint32_t BSP_ADC_GetIrqCount(void)
{
    return irqCount;
}

/**
  * @brief  ADC1_2 中断服务（stm32g4xx_it.c 用户区调用）
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
        uint32_t rawIbus;
        uint32_t rawVbus;

        adc1->ISR = ADC_ISR_JEOS;   /* 写 1 清除 */

        /* 注入组读取顺序（与 BSP_ADC_Init 配置一致）：
           ADC1: rank1 = U 相电流(CH12) -> JDR1
                 rank2 = 母线电流(CH3)  -> JDR2
                 rank3 = 母线电压(CH11) -> JDR3
           ADC2: rank1 = V 相电流(CH3)  -> JDR1 */
        rawU    = adc1->JDR1;
        rawIbus = adc1->JDR2;
        rawVbus = adc1->JDR3;
        rawV    = adc2->JDR1;

        /* 保存原始值供上位机/状态打印诊断（判断是否贴轨、偏置是否正常） */
        meas.RawU    = (uint16_t)rawU;
        meas.RawIbus = (uint16_t)rawIbus;
        meas.RawVbus = (uint16_t)rawVbus;
        meas.RawV    = (uint16_t)rawV;

        /* 贴轨检测：原始值达到轨附近说明放大器/ADC 饱和，实际电流超出
           测量范围（此时换算值被钳位，纯阈值比较无法发现过流） */
        meas.CurSat = ((rawU <= BOARD_ADC_RAIL_LOW) || (rawU >= BOARD_ADC_RAIL_HIGH) ||
                       (rawV <= BOARD_ADC_RAIL_LOW) || (rawV >= BOARD_ADC_RAIL_HIGH) ||
                       (rawIbus <= BOARD_ADC_RAIL_LOW) || (rawIbus >= BOARD_ADC_RAIL_HIGH)) ? 1U : 0U;

        /* 电流换算：低侧 shunt 差分放大，偏置减除后按 LSB 折算 */
        meas.Iu   = ((float)rawU - offsetU)         * BOARD_CUR_LSB * BOARD_CUR_SIGN_U;
        meas.Iv   = ((float)rawV - offsetV)         * BOARD_CUR_LSB * BOARD_CUR_SIGN_V;
        meas.Iw   = -(meas.Iu + meas.Iv);   /* 三线制电机：Iw = -(Iu + Iv) */
        meas.Ibus = ((float)rawIbus - offsetBus)    * BOARD_CUR_LSB * BOARD_CUR_SIGN_BUS;

        /* 母线电压：分压折算 + 一阶低通（截止约 160Hz，见 BOARD_VBUS_LPF_ALPHA） */
        vbusRaw  = (float)rawVbus * BOARD_VBUS_LSB;
        vbusFilt += BOARD_VBUS_LPF_ALPHA * (vbusRaw - vbusFilt);
        meas.Vbus = vbusFilt;

        lastMeas = meas;
        irqCount++;

        if (focCallback != NULL)
        {
            focCallback(&meas);
        }
    }
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  软件触发单次规则组转换（NTC 慢通道专用）
  * @note   严禁调用 HAL_ADC_Stop：会连同注入组一起停止并关闭 ADC。
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
  * @brief  NTC 原始值转温度（上拉接 VCC + NTC 接 GND）
  */
static float BSP_ADC_NtcToTemp(uint16_t raw, float r25, float beta)
{
    float ratio;
    float rntc;
    float tempK;

    if ((raw < 8U) || (raw > 4088U))
    {
        return BOARD_NTC_FAULT_TEMP;
    }

    ratio = (float)raw / 4096.0f;
    rntc  = BOARD_NTC_PULLUP * ratio / (1.0f - ratio);
    tempK = 1.0f / ((1.0f / 298.15f) + logf(rntc / r25) / beta);

    return tempK - 273.15f;
}
