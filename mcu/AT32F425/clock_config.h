#pragma once

/*
 * Shared 96MHz clock bring-up, used by both the Stage A and Stage B
 * bring-up harnesses (stage_a_main.c / stage_b_main.c). Extracted out
 * of stage_a_main.c verbatim (no logic change) so Stage B can reuse
 * the exact, already hardware-verified clock sequence instead of
 * duplicating or re-deriving it. See stage_a_main.c's "BUG FIX LOG"
 * comment for why CRM_PLL_MULT_24 (not _4) is required here.
 */

#include "vendor/at32f425.h"

void clock_config_96mhz(void);

/* Diagnostics, captured right after clock_config_96mhz() returns.
 * Register/field names are exactly as vendored from Artery's own
 * at32f425_crm.h / system_at32f425.c -- none are guessed. */
extern volatile uint32_t stage_crm_ctrl;   /* CRM->ctrl */
extern volatile uint32_t stage_crm_cfg;    /* CRM->cfg */
extern volatile uint32_t stage_crm_pll;    /* CRM->pll */
extern volatile uint32_t stage_sysclk_status;     /* crm_sysclk_switch_status_get(), expect CRM_SCLK_PLL(2) */
extern volatile uint32_t stage_pll_mult_decoded;  /* expect 24 */

void clock_capture_diagnostics(void);
