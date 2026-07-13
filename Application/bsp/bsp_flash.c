/**
  ******************************************************************************
  * @file    bsp_flash.c
  * @brief   片内 Flash 参数持久化 BSP
  * @note    - 使用 STM32G431CB 最后一页（page 63，0x0801F800，2KB）存储参数；
  *          - 记录头含 magic + 长度 + CRC32（硬件 CRC 外设），上电校验；
  *          - 页擦除约 20ms 且期间总线取指 stall：调用方必须保证电机已封波
  *            （擦写期间电流环 ISR 无法执行），本模块内部关中断保护编程序列。
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
#include "bsp_flash.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define PARAM_PAGE_INDEX        (63U)                       /*!< 参数页页号 */
#define PARAM_PAGE_ADDR         (0x0801F800UL)              /*!< 参数页首地址 */
#define PARAM_PAGE_SIZE         (2048U)                     /*!< 页大小 */
#define PARAM_MAGIC             (0x47494D34UL)              /*!< 'GIM4' */

/**
  * @brief 参数记录头（16 字节对齐到双字）
  */
typedef struct
{
    uint32_t Magic;         /*!< 固定标识 PARAM_MAGIC */
    uint32_t Size;          /*!< 参数体字节数 */
    uint32_t Crc;           /*!< 参数体 CRC32（硬件 CRC 默认配置） */
    uint32_t Reserved;      /*!< 保留（双字对齐填充） */
} FLASH_ParamHeaderTypeDef;

/* Private function prototypes -----------------------------------------------*/
static uint32_t BSP_FLASH_Crc32(const uint8_t *data, uint32_t size);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  从参数页加载参数
  * @param  buf  目标缓冲
  * @param  size 参数体大小（须与保存时一致）
  * @retval HAL_OK 校验通过；HAL_ERROR 无有效记录（首次上电或已损坏）
  */
HAL_StatusTypeDef BSP_FLASH_LoadParams(void *buf, uint32_t size)
{
    const FLASH_ParamHeaderTypeDef *hdr = (const FLASH_ParamHeaderTypeDef *)PARAM_PAGE_ADDR;
    const uint8_t *body = (const uint8_t *)(PARAM_PAGE_ADDR + sizeof(FLASH_ParamHeaderTypeDef));

    if ((hdr->Magic != PARAM_MAGIC) || (hdr->Size != size))
    {
        return HAL_ERROR;
    }
    if (BSP_FLASH_Crc32(body, size) != hdr->Crc)
    {
        return HAL_ERROR;
    }

    memcpy(buf, body, size);
    return HAL_OK;
}

/**
  * @brief  保存参数到参数页（擦除 + 双字编程）
  * @note   仅允许在电机封波、后台上下文调用；内部全程关中断。
  * @param  buf  参数体
  * @param  size 参数体大小，须 <= PARAM_PAGE_SIZE - 头大小
  * @retval HAL 状态
  */
HAL_StatusTypeDef BSP_FLASH_SaveParams(const void *buf, uint32_t size)
{
    FLASH_EraseInitTypeDef  erase;
    FLASH_ParamHeaderTypeDef hdr;
    uint32_t pageErr;
    uint32_t primask;
    HAL_StatusTypeDef status;
    uint64_t dword;
    uint32_t addr;
    uint32_t i;
    /* 头 + 体拼接的双字流缓冲：逐 8 字节读取源，尾部补 0xFF */
    const uint8_t *src;
    uint32_t total;

    if (size > (PARAM_PAGE_SIZE - sizeof(FLASH_ParamHeaderTypeDef)))
    {
        return HAL_ERROR;
    }

    hdr.Magic    = PARAM_MAGIC;
    hdr.Size     = size;
    hdr.Crc      = BSP_FLASH_Crc32((const uint8_t *)buf, size);
    hdr.Reserved = 0xFFFFFFFFUL;

    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
    {
        return status;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = PARAM_PAGE_INDEX;
    erase.NbPages   = 1U;
    status = HAL_FLASHEx_Erase(&erase, &pageErr);

    if (status == HAL_OK)
    {
        /* 先写头 */
        addr = PARAM_PAGE_ADDR;
        src  = (const uint8_t *)&hdr;
        for (i = 0U; (i < sizeof(hdr)) && (status == HAL_OK); i += 8U)
        {
            memcpy(&dword, &src[i], 8U);
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i, dword);
        }

        /* 再写参数体（尾部不足 8 字节补 0xFF） */
        addr = PARAM_PAGE_ADDR + sizeof(hdr);
        src  = (const uint8_t *)buf;
        total = size;
        for (i = 0U; (i < total) && (status == HAL_OK); i += 8U)
        {
            if ((total - i) >= 8U)
            {
                memcpy(&dword, &src[i], 8U);
            }
            else
            {
                dword = 0xFFFFFFFFFFFFFFFFULL;
                memcpy(&dword, &src[i], total - i);
            }
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i, dword);
        }
    }

    if (primask == 0U)
    {
        __enable_irq();
    }

    (void)HAL_FLASH_Lock();

    /* 回读校验 */
    if (status == HAL_OK)
    {
        const uint8_t *body = (const uint8_t *)(PARAM_PAGE_ADDR + sizeof(hdr));
        if (BSP_FLASH_Crc32(body, size) != hdr.Crc)
        {
            status = HAL_ERROR;
        }
    }
    return status;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  硬件 CRC32 计算（默认多项式 0x4C11DB7，全字复位初值）
  * @note   按字节流喂入（CRC 外设配置 8bit 输入），与保存/加载两侧一致即可。
  */
static uint32_t BSP_FLASH_Crc32(const uint8_t *data, uint32_t size)
{
    uint32_t i;

    __HAL_RCC_CRC_CLK_ENABLE();

    /* 复位 CRC 计算单元；默认 32bit 多项式、无反转。
       8bit 输入由对 DR 的字节宽度写访问实现（CR 中无输入位宽配置位） */
    CRC->CR = CRC_CR_RESET;
    CRC->CR = 0U;

    for (i = 0U; i < size; i++)
    {
        *(volatile uint8_t *)&CRC->DR = data[i];
    }
    return CRC->DR;
}
