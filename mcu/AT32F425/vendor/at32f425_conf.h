/**
  **************************************************************************
  * @file     at32f425_conf.h
  * @brief    at32f425 config header file (trimmed for ESCape32 Stage A bring-up)
  **************************************************************************
  *
  * Copyright (c) 2025, Artery Technology, All rights reserved.
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

#ifndef __AT32F425_CONF_H
#define __AT32F425_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Stage A default: HICK-based clock path (no external crystal assumed).
 * HEXT_VALUE is defined here only because at32f425_crm.h references it;
 * it is NOT used by the Stage A clock config (see stage_a_main.c), since
 * the presence of an HEXT crystal on the real PCB has not been confirmed.
 * If the board is later confirmed to have an HEXT crystal, switch the
 * clock source in stage_a_main.c and correct this value accordingly.
 */
#if !defined  HEXT_VALUE
#define HEXT_VALUE               ((uint32_t)8000000)
#endif

#define HEXT_STARTUP_TIMEOUT             ((uint16_t)0x3000)
#define HICK_VALUE                       ((uint32_t)8000000)
#define LEXT_VALUE                       ((uint32_t)32768)

/* module define: Stage A/B/C need CRM + GPIO + FLASH + TMR + ADC + DMA ----- */
#define CRM_MODULE_ENABLED
#define GPIO_MODULE_ENABLED
#define FLASH_MODULE_ENABLED
#define TMR_MODULE_ENABLED
#define MISC_MODULE_ENABLED
#define ADC_MODULE_ENABLED
#define DMA_MODULE_ENABLED

#ifdef CRM_MODULE_ENABLED
#include "at32f425_crm.h"
#endif
#ifdef GPIO_MODULE_ENABLED
#include "at32f425_gpio.h"
#endif
#ifdef FLASH_MODULE_ENABLED
#include "at32f425_flash.h"
#endif
#ifdef TMR_MODULE_ENABLED
#include "at32f425_tmr.h"
#endif
#ifdef MISC_MODULE_ENABLED
#include "at32f425_misc.h"
#endif
#ifdef ADC_MODULE_ENABLED
#include "at32f425_adc.h"
#endif
#ifdef DMA_MODULE_ENABLED
#include "at32f425_dma.h"
#endif

#ifdef __cplusplus
}
#endif

#endif
