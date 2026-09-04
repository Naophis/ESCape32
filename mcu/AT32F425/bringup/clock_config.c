#include "clock_config.h"

/*
 * Clock plan (see porting-plan Section 18-B):
 *   HICK_VALUE/2 = 4 MHz PLL entry --x24 PLL--> 96 MHz
 * This path does NOT require an HEXT crystal. Official Artery BSP
 * examples default to HEXT x12 instead; we default to HICK here
 * because the presence of an HEXT crystal on this board has not
 * been confirmed (existing AT32F421 ESCape32 targets are crystal-less).
 * If HEXT is confirmed present, switch AT32F425_CLOCK_SOURCE_HEXT to 1
 * below and set HEXT_VALUE in vendor/at32f425_conf.h to the real
 * crystal frequency.
 */
#define AT32F425_CLOCK_SOURCE_HEXT 0

volatile uint32_t stage_crm_ctrl;
volatile uint32_t stage_crm_cfg;
volatile uint32_t stage_crm_pll;
volatile uint32_t stage_sysclk_status;
volatile uint32_t stage_pll_mult_decoded;

void clock_config_96mhz(void)
{
  crm_reset();
  flash_psr_set(FLASH_WAIT_CYCLE_2); /* required for 65~96MHz per datasheet Table 17 note */

#if AT32F425_CLOCK_SOURCE_HEXT
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
  while (crm_hext_stable_wait() == ERROR);
  crm_pll_config(CRM_PLL_SOURCE_HEXT, CRM_PLL_MULT_12); /* HEXT(8MHz) x12 = 96MHz */
#else
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);
  while (crm_flag_get(CRM_HICK_STABLE_FLAG) != SET);
  crm_pll_config(CRM_PLL_SOURCE_HICK, CRM_PLL_MULT_24); /* (HICK_VALUE/2=4MHz) x24 = 96MHz */
#endif

  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
  while (crm_flag_get(CRM_PLL_STABLE_FLAG) != SET);

  crm_ahb_div_set(CRM_AHB_DIV_1);
  crm_apb2_div_set(CRM_APB2_DIV_1);
  crm_apb1_div_set(CRM_APB1_DIV_1);

  crm_sysclk_switch(CRM_SCLK_PLL);
  while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL);

  system_core_clock_update();
}

void clock_capture_diagnostics(void)
{
  stage_crm_ctrl = CRM->ctrl;
  stage_crm_cfg = CRM->cfg;
  stage_crm_pll = CRM->pll;
  stage_sysclk_status = (uint32_t)crm_sysclk_switch_status_get();

  /* Reproduce system_core_clock_update()'s pll_mult decode so it can be
   * inspected directly, independent of system_core_clock itself. */
  {
    uint32_t pll_mult_l = CRM->cfg_bit.pllmult_l;
    uint32_t pll_mult_h = CRM->cfg_bit.pllmult_h;
    uint32_t pll_mult;
    if ((pll_mult_h != 0U) || (pll_mult_l == 15U))
      pll_mult = pll_mult_l + (16U * pll_mult_h) + 1U;
    else
      pll_mult = pll_mult_l + 2U;
    stage_pll_mult_decoded = pll_mult;
  }
}
