/**
  ******************************************************************************
  * @file    bsp_pwm.c
  * @brief   TIM1 三相互补 PWM BSP
  * @note    - 中心对齐模式，PWM 频率与死区由 board_config.h 决定；
  *          - OC4 通道（无引脚输出）在计数器峰值前产生 TRGO2 触发 ADC 注入组；
  *          - MOE 作为软件封波开关：Disable 后输出高阻，FD6288 内部下拉
  *            将栅极驱动输入拉低，三相全关（电机 freewheel）；
  *          - 占空比语义：高侧导通比例，CCR = duty * ARR；
  *          - CubeMX 生成的 TIM1 参数（边沿对齐 / ARR=65535 / 无死区）为占位值，
  *            本模块整体重配。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_pwm.h"
#include "board_config.h"
#include "tim.h"

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化三相 PWM（重配 TIM1，输出保持封波）
  * @note   须在 MX_TIM1_Init 之后调用。初始化完成后计数器持续运行
  *         （为 ADC 提供触发），但 MOE 关闭、输出高阻。
  */
void BSP_PWM_Init(void)
{
    TIM_OC_InitTypeDef             ocConfig = {0};
    TIM_MasterConfigTypeDef        masterConfig = {0};
    TIM_BreakDeadTimeConfigTypeDef bdtConfig = {0};

    /* 基础时基：中心对齐，PWM 频率 = fclk / (2*ARR) */
    htim1.Init.Prescaler         = 0U;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period            = BOARD_PWM_ARR;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0U;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    /* 三相输出通道：PWM1，CCR 预装载，空闲态低 */
    ocConfig.OCMode       = TIM_OCMODE_PWM1;
    ocConfig.Pulse        = 0U;
    ocConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
    ocConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    ocConfig.OCFastMode   = TIM_OCFAST_DISABLE;
    ocConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
    ocConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &ocConfig, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &ocConfig, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &ocConfig, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }

    /* OC4：ADC 注入组触发源。PWM2 模式 + CCR4 = ARR - 提前量 */
    ocConfig.OCMode = TIM_OCMODE_PWM2;
    ocConfig.Pulse  = BOARD_PWM_ARR - BOARD_PWM_ADC_TRIG_ADVANCE;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &ocConfig, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }

    /* TRGO2 -> ADC 注入组外部触发 */
    masterConfig.MasterOutputTrigger  = TIM_TRGO_RESET;
    masterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;
    masterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &masterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* 死区与刹车：未使用外部 BKIN */
    bdtConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    bdtConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    bdtConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    bdtConfig.DeadTime         = BOARD_PWM_DEADTIME_DTG;
    bdtConfig.BreakState       = TIM_BREAK_DISABLE;
    bdtConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    bdtConfig.BreakFilter      = 0U;
    bdtConfig.Break2State      = TIM_BREAK2_DISABLE;
    bdtConfig.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
    bdtConfig.Break2Filter     = 0U;
    bdtConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &bdtConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* 启动全部通道（HAL 会置位 MOE），随即封波保持安全态 */
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    /* 必须用无条件版本：普通 __HAL_TIM_MOE_DISABLE 在通道常开时恒为空操作 */
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);

    BSP_PWM_SetDutyZero();
}

/**
  * @brief  使能功率输出（解除封波）
  */
void BSP_PWM_Enable(void)
{
    BSP_PWM_SetDutyZero();
    __HAL_TIM_MOE_ENABLE(&htim1);
}

/**
  * @brief  封波（输出高阻，电机 freewheel）
  * @note   ISR 安全，故障保护路径可直接调用。
  */
void BSP_PWM_Disable(void)
{
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
}

/**
  * @brief  查询功率输出是否使能
  */
uint8_t BSP_PWM_IsEnabled(void)
{
    return ((htim1.Instance->BDTR & TIM_BDTR_MOE) != 0U) ? 1U : 0U;
}

/**
  * @brief  写入三相占空比
  * @param  dutyU/V/W 高侧导通占空比 [0, 1)
  */
void BSP_PWM_SetDuty(float dutyU, float dutyV, float dutyW)
{
    TIM_TypeDef *tim = htim1.Instance;

    tim->BOARD_PWM_CCR_U = (uint32_t)(dutyU * (float)BOARD_PWM_ARR);
    tim->BOARD_PWM_CCR_V = (uint32_t)(dutyV * (float)BOARD_PWM_ARR);
    tim->BOARD_PWM_CCR_W = (uint32_t)(dutyW * (float)BOARD_PWM_ARR);
}

/**
  * @brief  三相占空比清零（低侧全导通：自举预充/动态刹车态）
  */
void BSP_PWM_SetDutyZero(void)
{
    TIM_TypeDef *tim = htim1.Instance;

    tim->BOARD_PWM_CCR_U = 0U;
    tim->BOARD_PWM_CCR_V = 0U;
    tim->BOARD_PWM_CCR_W = 0U;
}
