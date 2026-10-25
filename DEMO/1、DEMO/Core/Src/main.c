/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : GIM4310 / GIM4305 关节电机控制器 外设自检例程
  ******************************************************************************
  * @attention
  *
  * 本工程为基于伺泰威 GIM4310 / GIM4305 关节电机控制器（STM32G431CBU6）的
  * 外设自检例程，用于上电后逐项验证，并以 VOFA+（JustFloat）为主要观测手段：
  *   - 电流采样（MCU 内部 OPAMP 差分放大，增益 24）
  *   - 母线电压采样
  *   - NTC 温度采样（MOS 管 / 电机绕组）
  *   - 磁编码器角度采样（MT6701，软件 SPI）
  *   - PWM 发波 / MOS 管开通关断
  *   - 串口 VOFA+ 上位机数据（USART3，921600，JustFloat，12 通道）
  *   - RGB 灯颜色控制（WS2812）
  *   - CAN 通讯（含 120R 终端电阻控制）
  *
  * ---------------------------------------------------------------------------
  * VOFA+ JustFloat 通道定义（波特率 921600，无校验，8 数据位，1 停止位）：
  *   ch0  Iu        U 相电流            [A]
  *   ch1  Iv        V 相电流            [A]
  *   ch2  Ibus      母线电流            [A]（低侧采样时刻≈0，仅诊断）
  *   ch3  Vbus      母线电压            [V]
  *   ch4  rotor     转子机械角（单圈）  [deg]
  *   ch5  elec      电角度              [deg]（极对数 14）
  *   ch6  dutyU     U 相高侧占空比      [0~1]
  *   ch7  dutyV     V 相高侧占空比      [0~1]
  *   ch8  dutyW     W 相高侧占空比      [0~1]
  *   ch9  Tmos      MOS 管温度          [C]
  *   ch10 Tmot      电机绕组温度        [C]
  *   ch11 canRxCnt  CAN 接收帧计数      [-]
  *   ch12 Iw        W 相电流（重构）    [A]（Iw = -(Iu+Iv)，三线制）
  *
  * 串口命令（普通终端或 VOFA 的 RAW 发送）：
  *   e  使能开环 PWM（旋转电压矢量）      d  封波
  *   a  循环切换开环幅值 3/6/9/15/25%      f  循环切换开环电频率 0.5/1/2/5Hz
  *   t  切换 CAN 120R 终端                s  立即发送一帧 CAN
  *   c  切换到下一种 RGB 颜色             r  重新标定 ADC 零流偏置
  *   v  切换 VOFA 数据流 / 文本状态模式    ?  立即打印一次状态
  *
  * 输出模式说明（重要）：
  *   - 默认 vofaStreamOn=1：只发送 JustFloat 二进制帧（500Hz），
  *     不输出任何周期性 ASCII 文本，保证 VOFA+ 解析稳定；
  *   - 发送 'v' 后关闭数据流并恢复 1Hz 文本状态打印，便于普通串口终端查看；
  *   - 发送 '?' 会立即插入一段文本，VOFA 波形会短暂跳变，属正常现象。
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "fdcan.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "board_config.h"
#include "bsp_clock.h"
#include "bsp_uart.h"
#include "bsp_vofa.h"
#include "bsp_pwm.h"
#include "bsp_adc.h"
#include "bsp_encoder.h"
#include "bsp_can.h"
#include "bsp_ws2812.h"
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
  * @brief CAN 接收信息（便于后台打印/诊断）
  */
typedef struct
{
    uint32_t lastId;        /*!< 最近一帧的标准帧 ID */
    uint8_t  lastData[8];   /*!< 最近一帧数据 */
    uint8_t  lastLen;       /*!< 最近一帧长度 */
    uint32_t count;         /*!< 累计接收帧数（回调内自增） */
} CanRxInfoTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST_PI                 (3.14159265358979f)
#define TEST_TWO_PI             (6.28318530717959f)

#define TEST_OPENLOOP_HZ_IDX0   (0.5f)   /*!< 开环电频率档位 0 */
#define TEST_OPENLOOP_HZ_IDX1   (1.0f)   /*!< 开环电频率档位 1 */
#define TEST_OPENLOOP_HZ_IDX2   (2.0f)   /*!< 开环电频率档位 2 */
#define TEST_OPENLOOP_HZ_IDX3   (5.0f)   /*!< 开环电频率档位 3 */
/* 开环幅值档位：幅值越大 -> 相电压越高 -> 电流越大，便于用 VOFA 观察电流采样。
   25% 对应相电压约 0.25×Vbus（12V 时约 3V），注意电机温升与电源限流。 */
#define TEST_OPENLOOP_AMP_NUM   (5U)     /*!< 幅值档位数量 */
#define TEST_OPENLOOP_AMP_IDX0  (0.03f)  /*!< 档位 0：3%  占空比 */
#define TEST_OPENLOOP_AMP_IDX1  (0.06f)  /*!< 档位 1：6% */
#define TEST_OPENLOOP_AMP_IDX2  (0.09f)  /*!< 档位 2：9% */
#define TEST_OPENLOOP_AMP_IDX3  (0.15f)  /*!< 档位 3：15% */
#define TEST_OPENLOOP_AMP_IDX4  (0.25f)  /*!< 档位 4：25%（最大） */

#define TEST_VOFA_PERIOD_MS     (2U)     /*!< VOFA 上报周期（500Hz） */
#define TEST_TEMP_PERIOD_MS     (100U)   /*!< NTC 温度轮询周期（10Hz） */
#define TEST_STATUS_PERIOD_MS   (1000U)  /*!< 串口状态打印周期（1Hz） */
#define TEST_CAN_PERIOD_MS      (200U)   /*!< CAN 周期发送周期（5Hz） */
#define TEST_LED_PERIOD_MS      (700U)   /*!< RGB 颜色切换周期 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static ENC_DataTypeDef    encData;       /*!< 编码器运行数据（单圈/多圈/状态） */
static ADC_FocMeasTypeDef adcMeas;       /*!< 最近一次电流/母线电压换算结果 */
static volatile CanRxInfoTypeDef canRx;  /*!< CAN 接收信息（ISR 更新） */

static uint8_t  openLoopOn = 0U;         /*!< 开环 PWM 测试使能 */
static uint8_t  vofaStreamOn = 1U;       /*!< VOFA 数据流使能（1=只发 JustFloat，0=发 1Hz 文本） */
static uint8_t  ledIndex   = 0U;         /*!< 当前灯色序号（0=绿） */
static uint8_t  ampIndex   = 1U;         /*!< 开环幅值档位（默认 6%） */
static uint8_t  freqIndex  = 1U;         /*!< 开环频率档位（默认 1Hz） */
static uint16_t canSeq     = 0U;         /*!< CAN 发送序号 */
static float    elecAngle  = 0.0f;       /*!< 开环电角度积分 [rad] */
static float    dutyU      = 0.0f;       /*!< 当前 U 相占空比（供 VOFA 观测） */
static float    dutyV      = 0.0f;       /*!< 当前 V 相占空比 */
static float    dutyW      = 0.0f;       /*!< 当前 W 相占空比 */
static float    tmosC      = 0.0f;       /*!< MOS 温度缓存 [C] */
static float    tmotC      = 0.0f;       /*!< 电机温度缓存 [C] */

static uint32_t lastVofaTick   = 0U;
static uint32_t lastTempTick   = 0U;
static uint32_t lastStatusTick = 0U;
static uint32_t lastCanTick    = 0U;
static uint32_t lastLedTick    = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void APP_Init(void);
static void APP_Loop(void);
static void APP_HandleCommand(uint8_t cmd);
static void APP_SetLedByIndex(uint8_t idx);
static void APP_CanRxCallback(uint32_t id, const uint8_t *data, uint8_t len);
static void APP_UpdateTemps(void);
static void APP_ReportVofa(void);
static void APP_PrintStatus(void);
static float APP_OpenLoopAmp(void);
static float APP_OpenLoopFreq(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  读取当前开环幅值档位对应的占空比幅值
  */
static float APP_OpenLoopAmp(void)
{
    switch (ampIndex)
    {
        case 0U:  return TEST_OPENLOOP_AMP_IDX0;
        case 2U:  return TEST_OPENLOOP_AMP_IDX2;
        case 3U:  return TEST_OPENLOOP_AMP_IDX3;
        case 4U:  return TEST_OPENLOOP_AMP_IDX4;
        default:  return TEST_OPENLOOP_AMP_IDX1;
    }
}

/**
  * @brief  读取当前开环电频率档位对应的频率 [Hz]
  */
static float APP_OpenLoopFreq(void)
{
    switch (freqIndex)
    {
        case 0U:  return TEST_OPENLOOP_HZ_IDX0;
        case 2U:  return TEST_OPENLOOP_HZ_IDX2;
        case 3U:  return TEST_OPENLOOP_HZ_IDX3;
        default:  return TEST_OPENLOOP_HZ_IDX1;
    }
}

/**
  * @brief  CAN 接收回调（ISR 上下文）：记录最近一帧
  * @note   仅做轻量拷贝，不在此处做任何阻塞或打印。
  */
static void APP_CanRxCallback(uint32_t id, const uint8_t *data, uint8_t len)
{
    uint8_t i;

    canRx.lastId  = id;
    canRx.lastLen = (len > 8U) ? 8U : len;
    for (i = 0U; i < canRx.lastLen; i++)
    {
        canRx.lastData[i] = data[i];
    }
    canRx.count++;
}

/**
  * @brief  设置 RGB 灯色（循环色表：绿/红/蓝/白/黄/青/紫/灭）
  * @note   初始化完成默认绿色，便于一眼确认固件已运行。
  */
static void APP_SetLedByIndex(uint8_t idx)
{
    switch (idx % 8U)
    {
        case 0U: BSP_WS2812_SetColor(0U,   255U, 0U);   break;  /* 绿（默认） */
        case 1U: BSP_WS2812_SetColor(255U, 0U,   0U);   break;  /* 红 */
        case 2U: BSP_WS2812_SetColor(0U,   0U,   255U); break;  /* 蓝 */
        case 3U: BSP_WS2812_SetColor(255U, 255U, 255U); break;  /* 白 */
        case 4U: BSP_WS2812_SetColor(255U, 128U, 0U);   break;  /* 黄 */
        case 5U: BSP_WS2812_SetColor(0U,   255U, 255U); break;  /* 青 */
        case 6U: BSP_WS2812_SetColor(128U, 0U,   255U); break;  /* 紫 */
        default: BSP_WS2812_SetColor(0U,   0U,   0U);   break;  /* 灭 */
    }
}

/**
  * @brief  串口命令处理
  * @param  cmd 接收到的单字节命令
  */
static void APP_HandleCommand(uint8_t cmd)
{
    switch (cmd)
    {
        case 'e':
        case 'E':
            openLoopOn = 1U;
            BSP_PWM_Enable();
            BSP_UART_Printf("[CMD] PWM open-loop ENABLED (amp=%u%%, f=%.1fHz)\r\n",
                            (unsigned)(APP_OpenLoopAmp() * 100.0f), (double)APP_OpenLoopFreq());
            break;
        case 'd':
        case 'D':
            openLoopOn = 0U;
            dutyU = dutyV = dutyW = 0.0f;
            BSP_PWM_Disable();
            BSP_UART_Printf("[CMD] PWM DISABLED (MOE=0)\r\n");
            break;
        case 'a':
        case 'A':
            ampIndex = (uint8_t)((ampIndex + 1U) % TEST_OPENLOOP_AMP_NUM);
            BSP_UART_Printf("[CMD] open-loop amplitude = %u%%\r\n",
                            (unsigned)(APP_OpenLoopAmp() * 100.0f));
            break;
        case 'f':
        case 'F':
            freqIndex = (uint8_t)((freqIndex + 1U) % 4U);
            BSP_UART_Printf("[CMD] open-loop freq = %.1f Hz\r\n", (double)APP_OpenLoopFreq());
            break;
        case 't':
        case 'T':
            BSP_CAN_SetTermination((uint8_t)(BSP_CAN_GetTermination() ? 0U : 1U));
            BSP_UART_Printf("[CMD] CAN 120R termination = %u\r\n", BSP_CAN_GetTermination());
            break;
        case 's':
        case 'S':
        {
            uint8_t payload[8];
            payload[0] = 0x55U;
            payload[1] = 0xAAU;
            payload[2] = (uint8_t)canSeq;
            payload[3] = (uint8_t)(canSeq >> 8);
            payload[4] = 0U;
            payload[5] = 0U;
            payload[6] = 0U;
            payload[7] = 0U;
            (void)BSP_CAN_Send(BOARD_CAN_DEFAULT_NODE_ID, payload, 8U);
            BSP_UART_Printf("[CMD] CAN frame sent\r\n");
            break;
        }
        case 'c':
        case 'C':
            ledIndex++;
            APP_SetLedByIndex(ledIndex);
            BSP_UART_Printf("[CMD] RGB index = %u\r\n", ledIndex);
            break;
        case 'r':
        case 'R':
            /* 零点标定必须在封波、无电流条件下进行，否则会把运行电流误当作偏置 */
            if (BSP_PWM_IsEnabled() != 0U)
            {
                BSP_UART_Printf("[CMD] ADC offset re-cal rejected: disable PWM first (send 'd')\r\n");
            }
            else if (BSP_ADC_MeasureOffsets(256U) == HAL_OK)
            {
                float offU;
                float offV;
                float offBus;
                BSP_ADC_GetOffsets(&offU, &offV, &offBus);
                BSP_UART_Printf("[CMD] ADC offsets re-calibrated: Iu=%.0f Iv=%.0f Ibus=%.0f\r\n",
                                (double)offU, (double)offV, (double)offBus);
            }
            else
            {
                BSP_UART_Printf("[CMD] ADC offset re-cal FAILED (check bridges off)\r\n");
            }
            break;
        case 'v':
        case 'V':
            vofaStreamOn = (uint8_t)((vofaStreamOn != 0U) ? 0U : 1U);
            if (vofaStreamOn == 0U)
            {
                /* 关闭数据流后才打印文本，避免污染 JustFloat 帧 */
                BSP_UART_Printf("[CMD] VOFA stream OFF, text status 1Hz ON\r\n");
            }
            break;
        case '?':
            APP_PrintStatus();
            break;
        default:
            break;
    }
}

/**
  * @brief  轮询读取两路 NTC 温度并缓存（低频调用，避免占用 500Hz 上报带宽）
  */
static void APP_UpdateTemps(void)
{
    tmosC = BSP_ADC_ReadMosTemp();
    tmotC = BSP_ADC_ReadMotorTemp();
}

/**
  * @brief  填充并发送一帧 VOFA+ JustFloat 数据（12 通道，见文件头注释）
  */
static void APP_ReportVofa(void)
{
    float rotorDeg;
    float elecDeg;

    /* 电流/母线电压：由 ADC 注入组 ISR 更新 */
    BSP_ADC_GetLastMeas(&adcMeas);

    /* 机械角（单圈）与电角度 */
    rotorDeg = (float)encData.Raw * 360.0f / (float)ENC_CPR;
    elecDeg  = fmodf(rotorDeg * 14.0f, 360.0f);   /* 极对数 14 */

    BSP_VOFA_SetChannel(0U,  adcMeas.Iu);
    BSP_VOFA_SetChannel(1U,  adcMeas.Iv);
    BSP_VOFA_SetChannel(2U,  adcMeas.Ibus);
    BSP_VOFA_SetChannel(3U,  adcMeas.Vbus);
    BSP_VOFA_SetChannel(4U,  rotorDeg);
    BSP_VOFA_SetChannel(5U,  elecDeg);
    BSP_VOFA_SetChannel(6U,  dutyU);
    BSP_VOFA_SetChannel(7U,  dutyV);
    BSP_VOFA_SetChannel(8U,  dutyW);
    BSP_VOFA_SetChannel(9U,  tmosC);
    BSP_VOFA_SetChannel(10U, tmotC);
    BSP_VOFA_SetChannel(11U, (float)BSP_CAN_GetRxCount());
    BSP_VOFA_SetChannel(12U, adcMeas.Iw);   /* W 相电流（由 Iu+Iv 重构） */

    BSP_VOFA_Send();
}

/**
  * @brief  串口打印一次完整状态（也用于排查初始化是否成功）
  */
static void APP_PrintStatus(void)
{
    float offU;
    float offV;
    float offBus;

    BSP_ADC_GetLastMeas(&adcMeas);
    BSP_ADC_GetOffsets(&offU, &offV, &offBus);

    BSP_UART_Printf("\r\n===== GIM4310/4305 DEMO STATUS =====\r\n");
    BSP_UART_Printf("clock        : %s\r\n", BSP_CLOCK_IsHse() ? "HSE 20MHz" : "HSI 16MHz");
    BSP_UART_Printf("adc offset   : Iu=%.0f Iv=%.0f Ibus=%.0f [LSB] (mid-scale=2048)\r\n",
                    (double)offU, (double)offV, (double)offBus);
    BSP_UART_Printf("adc raw      : U=%u IBUS=%u VBUS=%u V=%u [LSB]\r\n",
                    (unsigned)adcMeas.RawU, (unsigned)adcMeas.RawIbus,
                    (unsigned)adcMeas.RawVbus, (unsigned)adcMeas.RawV);
    BSP_UART_Printf("current      : Iu=%+.3f Iv=%+.3f Iw=%+.3f Ibus=%+.3f A (sat=%u)\r\n",
                    (double)adcMeas.Iu, (double)adcMeas.Iv, (double)adcMeas.Iw,
                    (double)adcMeas.Ibus, adcMeas.CurSat);
    BSP_UART_Printf("vbus         : %.2f V\r\n", (double)adcMeas.Vbus);

    /* 电流采样前端体检：零流偏置应约等于半量程(2048)。
       若明显贴近某一轨（<500 或 >3600），说明运放零流输出已饱和，
       常见原因是差分放大器的偏置网络阻值不匹配
       （要求 R40‖R41 = 反馈电阻 R39，实物 R40=R41=10k 时并联仅 5k，
        正确应约 47k~48k）。此时电流读数会恒≈0，需硬件整改。 */
    if ((offU < 500.0f) || (offU > 3600.0f) ||
        (offV < 500.0f) || (offV > 3600.0f))
    {
        BSP_UART_Printf("[WARN] CSA zero-current offset near rail -> "
                        "current sampling saturated; check bias resistors R40/R41 (should be ~47k)\r\n");
    }
    BSP_UART_Printf("temp         : MOS=%.1f C  MOTOR=%.1f C\r\n", (double)tmosC, (double)tmotC);
    BSP_UART_Printf("encoder      : raw=%u turns=%ld status=%d crc_err=%lu field_err=%lu\r\n",
                    encData.Raw, (long)encData.Turns, (int)encData.Status,
                    (unsigned long)encData.CrcErrCnt, (unsigned long)encData.FieldErrCnt);
    BSP_UART_Printf("pwm          : %s  duties=%.2f/%.2f/%.2f\r\n",
                    BSP_PWM_IsEnabled() ? "ENABLED" : "disabled",
                    (double)dutyU, (double)dutyV, (double)dutyW);
    BSP_UART_Printf("can          : rx=%lu err=%lu term=%u lastId=0x%03lX\r\n",
                    (unsigned long)BSP_CAN_GetRxCount(), (unsigned long)BSP_CAN_GetErrCount(),
                    BSP_CAN_GetTermination(), (unsigned long)canRx.lastId);
    BSP_UART_Printf("foc irq cnt  : %lu\r\n", (unsigned long)BSP_ADC_GetIrqCount());
    BSP_UART_Printf("cmd: e/d/a/f/t/s/c/r/v/?  (v=VOFA stream on/off)\r\n");
}

/**
  * @brief  应用初始化：按依赖顺序初始化各外设 BSP
  * @note   CubeMX 的 MX_*_Init 已完成外设时钟/引脚，本函数负责“板级真实参数”重配。
  */
static void APP_Init(void)
{
    /* 1) 时钟切换到板载 20MHz 晶振（提升 CAN 位时序精度；失败自动回退 HSI） */
    (void)BSP_CLOCK_SwitchToHse();

    /* 2) 调试串口 921600 + VOFA 数据帧 */
    BSP_UART_Init();
    BSP_VOFA_Init();
    BSP_UART_Printf("\r\n\r\n*** GIM4310/4305 peripheral self-test DEMO ***\r\n");
    BSP_UART_Printf("SYSCLK=%lu Hz, source=%s\r\n",
                    (unsigned long)HAL_RCC_GetHCLKFreq(),
                    BSP_CLOCK_IsHse() ? "HSE" : "HSI");
    /* 串口自检信息：正常时 BRR 应与 170MHz/921600 匹配（约 0x0B87）。
       若此处乱码，说明终端波特率不是 921600。 */
    BSP_UART_Printf("USART3: %lu bps, BRR=0x%04lX, JustFloat 12ch\r\n",
                    (unsigned long)huart3.Init.BaudRate,
                    (unsigned long)(huart3.Instance->BRR & 0xFFFFU));
    BSP_UART_Printf(">>> VOFA streaming ON (text muted). Send 'v' to switch to text mode.\r\n");

    /* 3) PWM：先让 TIM1 计数器运行（为 ADC 提供 TRGO2 触发），输出保持封波 */
    BSP_PWM_Init();

    /* 4) ADC + 内部运放电流采样 + NTC/母线电压；随后标定零流偏置 */
    BSP_ADC_Init();
    {
        /* 无论返回成功与否都把实测偏置打印出来（便于诊断前端电路） */
        uint8_t offOk = (BSP_ADC_MeasureOffsets(256U) == HAL_OK) ? 1U : 0U;
        float offU;
        float offV;
        float offBus;
        BSP_ADC_GetOffsets(&offU, &offV, &offBus);
        BSP_UART_Printf("ADC zero offset: Iu=%.0f Iv=%.0f Ibus=%.0f [LSB] (ok=%u, mid-scale=2048)\r\n",
                        (double)offU, (double)offV, (double)offBus, offOk);
        if (offOk == 0U)
        {
            BSP_UART_Printf("[WARN] offset out of range -> current front-end likely saturated.\r\n");
        }
    }

    /* 5) 磁编码器（软件 SPI）；MT6701 上电最大 32ms 就绪，故延时后预载 */
    BSP_ENC_Init();
    HAL_Delay(35U);
    BSP_ENC_Preload(&encData);

    /* 6) CAN：默认 1Mbps 且接入 120R 终端 */
    BSP_CAN_RegisterRxCallback(APP_CanRxCallback);
    BSP_CAN_Init(CAN_BAUD_1M, BOARD_CAN_DEFAULT_NODE_ID);

    /* 7) WS2812 RGB，初始化完成亮绿色 */
    BSP_WS2812_Init();
    ledIndex = 0U;
    APP_SetLedByIndex(ledIndex);
    BSP_WS2812_Poll();

    /* 8) 使能 ADC JEOS 中断节拍（本自检只做数据采集，回调传 NULL） */
    BSP_ADC_StartFocIrq(NULL);

    /* 9) 首次温度采样 */
    APP_UpdateTemps();

    BSP_UART_Printf("init done. cmd: e=enablePWM d=disable a=amp f=freq r=recal v=vofa/term ?=status\r\n");
}

/**
  * @brief  应用主循环（前后台调度）
  * @note   各任务节拍：命令/编码器每轮；VOFA 2ms；温度 100ms；CAN 200ms；
  *         RGB 700ms；状态打印 1s；开环 PWM 每轮（按实际 dt 积分）。
  */
static void APP_Loop(void)
{
    static uint32_t lastLoopTick = 0U;
    uint32_t now = HAL_GetTick();
    int32_t  ch;

    /* ---- 串口命令（非阻塞取完所有已收字节） ---- */
    while ((ch = BSP_UART_ReadByte()) >= 0)
    {
        APP_HandleCommand((uint8_t)ch);
    }

    /* ---- 编码器：每轮读取一次，供 VOFA/状态显示共用 ---- */
    {
        ENC_DataTypeDef local = encData;
        (void)BSP_ENC_Read(&local);
        encData = local;
    }

    /* ---- NTC 温度：100ms ---- */
    if ((now - lastTempTick) >= TEST_TEMP_PERIOD_MS)
    {
        lastTempTick = now;
        APP_UpdateTemps();
    }

    /* ---- RGB 颜色循环：700ms（开环演示时保持不变以便观察） ---- */
    if ((now - lastLedTick) >= TEST_LED_PERIOD_MS)
    {
        lastLedTick = now;
        if (openLoopOn == 0U)
        {
            ledIndex++;
            APP_SetLedByIndex(ledIndex);
        }
    }
    BSP_WS2812_Poll();

    /* ---- 开环 PWM 旋转电压矢量（验证 PWM 发波 / MOS 开通关断） ----
       生成三相 120° 相位差的正弦占空比，使能 MOE 后可在示波器/相电压上
       观察到旋转磁场；VOFA 的 ch6~ch8 同步显示三相占空比。 */
    if (openLoopOn != 0U)
    {
        float dt = (float)(now - lastLoopTick) / 1000.0f;   /* 主循环实际步长 [s] */
        float amp = APP_OpenLoopAmp();
        float freq = APP_OpenLoopFreq();

        if (dt > 0.05f)
        {
            dt = 0.05f;                                     /* 限幅，避免偶发长阻塞导致角度跳变 */
        }

        elecAngle += TEST_TWO_PI * freq * dt;
        if (elecAngle >= TEST_TWO_PI)
        {
            elecAngle -= TEST_TWO_PI;
        }

        dutyU = 0.5f + amp * sinf(elecAngle);
        dutyV = 0.5f + amp * sinf(elecAngle - (2.0f * TEST_PI / 3.0f));
        dutyW = 0.5f + amp * sinf(elecAngle + (2.0f * TEST_PI / 3.0f));
        BSP_PWM_SetDuty(dutyU, dutyV, dutyW);
    }

    /* ---- CAN 周期发送测试帧（ID=1，帧头 0x55 0xAA + 序号 + 电流环计数低字节） ---- */
    if ((now - lastCanTick) >= TEST_CAN_PERIOD_MS)
    {
        uint8_t payload[8];
        lastCanTick = now;
        payload[0] = 0x55U;
        payload[1] = 0xAAU;
        payload[2] = (uint8_t)canSeq;
        payload[3] = (uint8_t)(canSeq >> 8);
        payload[4] = (uint8_t)(BSP_ADC_GetIrqCount() & 0xFFU);
        payload[5] = 0U;
        payload[6] = 0U;
        payload[7] = 0U;
        (void)BSP_CAN_Send(BOARD_CAN_DEFAULT_NODE_ID, payload, 8U);
        canSeq++;
    }

    /* ---- VOFA+ 上报：2ms（500Hz，仅在数据流开启时） ---- */
    if ((vofaStreamOn != 0U) && ((now - lastVofaTick) >= TEST_VOFA_PERIOD_MS))
    {
        lastVofaTick = now;
        APP_ReportVofa();
    }

    /* ---- 串口文本状态：1s（仅在数据流关闭时，避免污染 JustFloat 帧） ---- */
    if ((vofaStreamOn == 0U) && ((now - lastStatusTick) >= TEST_STATUS_PERIOD_MS))
    {
        lastStatusTick = now;
        APP_PrintStatus();
    }

    lastLoopTick = now;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* 显式设置向量表基址（双保险）：
     CubeMX 默认不设置 SCB->VTOR，依赖“地址 0 的启动别名”。本板 PB8=BOOT0
     与 USART3_RX 共用引脚，若上电瞬间 BOOT0 为高电平，地址 0 会别名到 ROM，
     导致所有中断向量取自系统 Bootloader。此处固定指向 Flash，保证正确。 */
  SCB->VTOR = FLASH_BASE;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_FDCAN1_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  APP_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    APP_Loop();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 异常时先封波，输出高阻，避免功率管误动作 */
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
