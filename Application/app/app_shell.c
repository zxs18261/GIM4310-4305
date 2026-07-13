/**
  ******************************************************************************
  * @file    app_shell.c
  * @brief   调试 shell 模块（USART3，行式命令）
  * @note    - 后台轮询解析，支持退格编辑与命令回显；
  *          - 命令表见 help 输出；参数读写与 CAN 配置通道共用同一套索引；
  *          - 所有输出走非阻塞环形缓冲，不影响控制环。
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
#include "app_shell.h"
#include "app_params.h"
#include "app_can_protocol.h"
#include "app_monitor.h"
#include "motor_ctrl.h"
#include "bsp_uart.h"
#include "board_config.h"
#include <string.h>
#include <stdlib.h>

/* Private defines -----------------------------------------------------------*/
#define SHELL_LINE_MAX          (96U)
#define SHELL_ARGS_MAX          (4U)

/* Private types -------------------------------------------------------------*/

/**
  * @brief 参数名 -> 配置索引映射表项
  */
typedef struct
{
    const char *Name;
    uint8_t     Index;
} SHELL_ParamMapTypeDef;

/* Private constants ---------------------------------------------------------*/
static const SHELL_ParamMapTypeDef paramMap[] =
{
    { "id",      CANCFG_P_NODEID      },
    { "baud",    CANCFG_P_CANBAUD     },
    { "term",    CANCFG_P_CANTERM     },
    { "timeout", CANCFG_P_TIMEOUT_MS  },
    { "kp",      CANCFG_P_SPEED_KP    },
    { "ki",      CANCFG_P_SPEED_KI    },
    { "poskp",   CANCFG_P_POS_KP      },
    { "curmax",  CANCFG_P_CUR_MAX     },
    { "spdmax",  CANCFG_P_SPEED_MAX   },
    { "bright",  CANCFG_P_LED_BRIGHT  },
    { "offset",  CANCFG_P_ELEC_OFFSET },
    { "pp",      CANCFG_P_POLE_PAIRS  },
    { "encinv",  CANCFG_P_ENC_INVERT  },
    { "caldone", CANCFG_P_CALIBRATED  },
};

/* Private variables ---------------------------------------------------------*/
static char     lineBuf[SHELL_LINE_MAX];
static uint32_t lineLen = 0U;

/* Private function prototypes -----------------------------------------------*/
static void APP_SHELL_Execute(char *line);
static void APP_SHELL_Help(void);
static void APP_SHELL_GetSet(uint32_t argc, char **argv);
static void APP_SHELL_Boot0(uint32_t argc, char **argv);
static const SHELL_ParamMapTypeDef *APP_SHELL_FindParam(const char *name);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 shell
  */
void APP_SHELL_Init(void)
{
    lineLen = 0U;
}

/**
  * @brief  shell 轮询（后台调用）
  */
void APP_SHELL_Poll(void)
{
    int32_t c;

    while ((c = BSP_UART_ReadByte()) >= 0)
    {
        char ch = (char)c;

        if ((ch == '\r') || (ch == '\n'))
        {
            BSP_UART_Printf("\r\n");
            if (lineLen > 0U)
            {
                lineBuf[lineLen] = '\0';
                APP_SHELL_Execute(lineBuf);
                lineLen = 0U;
            }
            BSP_UART_Printf("> ");
        }
        else if ((ch == '\b') || (ch == 0x7F))
        {
            if (lineLen > 0U)
            {
                lineLen--;
                BSP_UART_Printf("\b \b");
            }
        }
        else if ((ch >= ' ') && (lineLen < (SHELL_LINE_MAX - 1U)))
        {
            lineBuf[lineLen++] = ch;
            BSP_UART_Write((const uint8_t *)&ch, 1U);   /* 回显 */
        }
        else
        {
            /* 控制字符或缓冲满：丢弃 */
        }
    }
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  解析并执行一行命令
  */
static void APP_SHELL_Execute(char *line)
{
    char    *argv[SHELL_ARGS_MAX] = {0};
    uint32_t argc = 0U;
    char    *tok  = strtok(line, " \t");

    while ((tok != NULL) && (argc < SHELL_ARGS_MAX))
    {
        argv[argc++] = tok;
        tok = strtok(NULL, " \t");
    }
    if (argc == 0U)
    {
        return;
    }

    if (strcmp(argv[0], "help") == 0)
    {
        APP_SHELL_Help();
    }
    else if (strcmp(argv[0], "status") == 0)
    {
        APP_MON_PrintStatus();
    }
    else if (strcmp(argv[0], "info") == 0)
    {
        APP_MON_PrintBanner();
    }
    else if (strcmp(argv[0], "en") == 0)
    {
        MC_ModeTypeDef mode = MC_MODE_TORQUE;
        uint8_t valid = 0U;

        if (argc >= 2U)
        {
            valid = 1U;
            if      (strcmp(argv[1], "open")   == 0) { mode = MC_MODE_OPENLOOP; }
            else if (strcmp(argv[1], "torque") == 0) { mode = MC_MODE_TORQUE;   }
            else if (strcmp(argv[1], "speed")  == 0) { mode = MC_MODE_SPEED;    }
            else if (strcmp(argv[1], "pos")    == 0) { mode = MC_MODE_POSITION; }
            else if (strcmp(argv[1], "mit")    == 0) { mode = MC_MODE_MIT;      }
            else { valid = 0U; }
        }
        if (valid == 0U)
        {
            BSP_UART_Printf("usage: en <open|torque|speed|pos|mit>\r\n");
        }
        else if ((MC_SetMode(mode) != HAL_OK) || (MC_Enable() != HAL_OK))
        {
            BSP_UART_Printf("enable failed (fault? not calibrated?)\r\n");
        }
        else
        {
            APP_CAN_SetControlled(0U);      /* shell 发起的使能不受 CAN 超时约束 */
            BSP_UART_Printf("enabled\r\n");
        }
    }
    else if (strcmp(argv[0], "dis") == 0)
    {
        MC_Disable();
        (void)MC_SetMode(MC_MODE_DISABLED);
        BSP_UART_Printf("disabled\r\n");
    }
    else if (strcmp(argv[0], "cal") == 0)
    {
        if (MC_StartCalibration() == HAL_OK)
        {
            BSP_UART_Printf("calibration started (motor will rotate!)\r\n");
        }
        else
        {
            BSP_UART_Printf("cal start failed\r\n");
        }
    }
    else if (strcmp(argv[0], "zero") == 0)
    {
        MC_SetZeroHere();
        BSP_UART_Printf("zero set\r\n");
    }
    else if (strcmp(argv[0], "clear") == 0)
    {
        (void)MC_ClearFault();
        BSP_UART_Printf("fault cleared\r\n");
    }
    else if (strcmp(argv[0], "iq") == 0)
    {
        if (argc >= 2U)
        {
            MC_SetIqRef(strtof(argv[1], NULL));
            BSP_UART_Printf("iq ref = %s A\r\n", argv[1]);
        }
    }
    else if (strcmp(argv[0], "torque") == 0)
    {
        if (argc >= 2U)
        {
            MC_SetTorque(strtof(argv[1], NULL));
            BSP_UART_Printf("torque ref = %s Nm\r\n", argv[1]);
        }
    }
    else if (strcmp(argv[0], "speed") == 0)
    {
        if (argc >= 2U)
        {
            MC_SetSpeedRef(strtof(argv[1], NULL));
            BSP_UART_Printf("speed ref = %s rad/s\r\n", argv[1]);
        }
    }
    else if (strcmp(argv[0], "pos") == 0)
    {
        if (argc >= 2U)
        {
            MC_SetPosRef(strtof(argv[1], NULL));
            BSP_UART_Printf("pos ref = %s rad\r\n", argv[1]);
        }
    }
    else if (strcmp(argv[0], "open") == 0)
    {
        if (argc >= 4U)
        {
            MC_SetOpenLoop(strtof(argv[1], NULL), strtof(argv[2], NULL), strtof(argv[3], NULL));
            BSP_UART_Printf("openloop vd/vq/we set\r\n");
        }
        else
        {
            BSP_UART_Printf("usage: open <vd> <vq> <we_erad_s>\r\n");
        }
    }
    else if ((strcmp(argv[0], "get") == 0) || (strcmp(argv[0], "set") == 0))
    {
        APP_SHELL_GetSet(argc, argv);
    }
    else if (strcmp(argv[0], "save") == 0)
    {
        APP_PARAMS_RequestSave();
        BSP_UART_Printf("save requested (executes when disabled)\r\n");
    }
    else if (strcmp(argv[0], "defaults") == 0)
    {
        if (g_Mc.Enabled == 0U)
        {
            APP_PARAMS_LoadDefaults();
            MC_ApplyPidParams();
            APP_CAN_RequestReinit();    /* NodeId/波特率/终端恢复默认后重配 CAN */
            BSP_UART_Printf("defaults loaded (not saved)\r\n");
        }
        else
        {
            BSP_UART_Printf("disable first\r\n");
        }
    }
    else if (strcmp(argv[0], "log") == 0)
    {
        if ((argc >= 2U) && (strcmp(argv[1], "on") == 0))
        {
            uint32_t hz = (argc >= 3U) ? (uint32_t)atoi(argv[2]) : 20U;
            APP_MON_SetLog(1U, hz);
        }
        else
        {
            APP_MON_SetLog(0U, 20U);
            BSP_UART_Printf("log off\r\n");
        }
    }
    else if (strcmp(argv[0], "boot0") == 0)
    {
        APP_SHELL_Boot0(argc, argv);
    }
    else if (strcmp(argv[0], "reboot") == 0)
    {
        BSP_UART_Printf("rebooting...\r\n");
        BSP_UART_Flush();
        NVIC_SystemReset();
    }
    else
    {
        BSP_UART_Printf("unknown cmd '%s', try 'help'\r\n", argv[0]);
    }
}

/**
  * @brief  help 输出
  */
static void APP_SHELL_Help(void)
{
    BSP_UART_Printf(
        "commands:\r\n"
        "  status / info            state snapshot / banner\r\n"
        "  en <open|torque|speed|pos|mit>   set mode + enable\r\n"
        "  dis                      disable output\r\n"
        "  cal                      run pre-position calibration\r\n"
        "  zero                     set current position as zero\r\n"
        "  clear                    clear latched fault\r\n"
        "  iq/torque/speed/pos <v>  target value (A/Nm/rad_s/rad)\r\n"
        "  open <vd> <vq> <we>      openloop voltage vector\r\n"
        "  get [name] / set <name> <v>   params: id baud term timeout kp ki\r\n"
        "                           poskp curmax spdmax bright offset pp encinv caldone\r\n"
        "  save / defaults          persist / restore params\r\n"
        "  log on [hz] / log off    periodic CSV telemetry\r\n"
        "  boot0 [fix confirm]      check/program nSWBOOT0 option byte\r\n"
        "  reboot                   system reset\r\n");
}

/**
  * @brief  get/set 命令处理
  */
static void APP_SHELL_GetSet(uint32_t argc, char **argv)
{
    const SHELL_ParamMapTypeDef *map;
    uint8_t ok;
    float   value;
    uint32_t i;

    if (strcmp(argv[0], "get") == 0)
    {
        if (argc < 2U)
        {
            /* 全量列出 */
            for (i = 0U; i < (sizeof(paramMap) / sizeof(paramMap[0])); i++)
            {
                value = APP_CAN_GetParamByIndex(paramMap[i].Index, &ok);
                BSP_UART_Printf("  %-8s = %.4f\r\n", paramMap[i].Name, value);
            }
            return;
        }
        map = APP_SHELL_FindParam(argv[1]);
        if (map == NULL)
        {
            BSP_UART_Printf("unknown param\r\n");
            return;
        }
        value = APP_CAN_GetParamByIndex(map->Index, &ok);
        BSP_UART_Printf("%s = %.4f\r\n", map->Name, value);
    }
    else
    {
        if (argc < 3U)
        {
            BSP_UART_Printf("usage: set <name> <value>\r\n");
            return;
        }
        map = APP_SHELL_FindParam(argv[1]);
        if (map == NULL)
        {
            BSP_UART_Printf("unknown param\r\n");
            return;
        }
        if (APP_CAN_SetParamByIndex(map->Index, strtof(argv[2], NULL)) != 0U)
        {
            BSP_UART_Printf("%s set (use 'save' to persist)\r\n", map->Name);
        }
        else
        {
            BSP_UART_Printf("rejected (readonly or out of range)\r\n");
        }
    }
}

/**
  * @brief  boot0 命令：检查/编程 nSWBOOT0 选项字节
  * @note   本板 USART3_RX 与 BOOT0 复用（PB8）：外部串口若在复位时拉高
  *         该脚会误入系统 bootloader。nSWBOOT0=0 + nBOOT0=1 固定从主 Flash
  *         启动。编程后触发 OBL launch（等效复位）。
  */
static void APP_SHELL_Boot0(uint32_t argc, char **argv)
{
    uint32_t optr = FLASH->OPTR;

    if ((argc >= 3U) && (strcmp(argv[1], "fix") == 0) && (strcmp(argv[2], "confirm") == 0))
    {
        FLASH_OBProgramInitTypeDef ob = {0};

        if (g_Mc.Enabled != 0U)
        {
            BSP_UART_Printf("disable motor first\r\n");
            return;
        }

        ob.OptionType = OPTIONBYTE_USER;
        ob.USERType   = OB_USER_nSWBOOT0 | OB_USER_nBOOT0;
        ob.USERConfig = OB_BOOT0_FROM_OB | OB_nBOOT0_SET;   /* BOOT0 取选项位，且=1 -> 主 Flash */

        (void)HAL_FLASH_Unlock();
        (void)HAL_FLASH_OB_Unlock();
        if (HAL_FLASHEx_OBProgram(&ob) == HAL_OK)
        {
            BSP_UART_Printf("OB programmed, launching (reset)...\r\n");
            BSP_UART_Flush();
            (void)HAL_FLASH_OB_Launch();    /* 触发复位，不再返回 */
        }
        BSP_UART_Printf("OB program failed\r\n");
        (void)HAL_FLASH_OB_Lock();
        (void)HAL_FLASH_Lock();
    }
    else
    {
        BSP_UART_Printf("OPTR = 0x%08lX, nSWBOOT0 = %u %s\r\n",
                        (unsigned long)optr,
                        (unsigned int)((optr & FLASH_OPTR_nSWBOOT0) ? 1U : 0U),
                        (optr & FLASH_OPTR_nSWBOOT0) ?
                            "(BOOT0 pin ACTIVE - uart high at reset enters bootloader! use 'boot0 fix confirm')" :
                            "(boot fixed to main flash, ok)");
    }
}

/**
  * @brief  参数名查表
  */
static const SHELL_ParamMapTypeDef *APP_SHELL_FindParam(const char *name)
{
    uint32_t i;

    for (i = 0U; i < (sizeof(paramMap) / sizeof(paramMap[0])); i++)
    {
        if (strcmp(name, paramMap[i].Name) == 0)
        {
            return &paramMap[i];
        }
    }
    return NULL;
}
