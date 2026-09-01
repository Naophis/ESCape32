/*
 * ESCape32 AT32F425 port -- Stage A bring-up (CPU only).
 *
 * Scope (per porting-plan Section 15 "Stage A"): prove that
 *   - the AT32F425K8U7-4 boots from flash,
 *   - the clock reaches 96 MHz,
 *   - SWD stays attachable,
 *   - main() is reached with no HardFault,
 * WITHOUT touching TIM1 / MP6540HA / BEMF control (Stage B+).
 *
 * This file deliberately does NOT go through the shared ESCape32
 * src/main.c control loop: that loop configures TIM1 (the MP6540HA
 * gate driver timer) and calls compctl(), which has no meaning on
 * AT32F425 (no analog comparator -- see porting-plan Section 18-G/H).
 * Wiring Stage A into that shared loop would require a compctl()
 * stub, which was intentionally deferred until the Stage D BEMF
 * design is in place. Stage A is therefore a standalone program.
 *
 * GPIO policy for this stage: no pin is driven by default. Per the
 * AT32F425 datasheet (Table 5 note), all GPIOs default to floating
 * input on reset; Stage A only performs a read of GPIOA (a
 * side-effect-free bus access) to prove the AHB2 GPIO domain is
 * clocked and reachable. The one optional exception is the CLKOUT
 * probe below (off by default, see STAGE_A_ENABLE_CLKOUT).
 *
 * Liveness is proven via SWD by inspecting `stage_a_heartbeat` (free
 * running counter) and `stage_a_clock_ok` (set once system_core_clock
 * is confirmed == 96000000) in a debugger.
 *
 * ---------------------------------------------------------------------
 * BUG FIX LOG (first bring-up attempt measured system_core_clock ==
 * 16,000,000 instead of 96,000,000; heartbeat was running fine, so
 * startup/linker/main-loop were never in question -- only the PLL
 * multiplier was wrong). Root cause, confirmed against the exact
 * vendored source (not re-derived from memory):
 *
 *   vendor/system_at32f425.c, system_core_clock_update(), the
 *   CRM_SCLK_PLL + pll_clock_source==0x00 (CRM_PLL_SOURCE_HICK) branch:
 *
 *       /' hick divided by 2 selected as pll clock entry '/
 *       system_core_clock = (HICK_VALUE >> 1) * pll_mult;
 *
 *   HICK_VALUE is 8,000,000 (vendor/at32f425_conf.h -- Artery's own
 *   constant, unmodified). So the PLL entry frequency for
 *   CRM_PLL_SOURCE_HICK is HICK_VALUE/2 = 4,000,000 Hz, NOT
 *   "48MHz oscillator /2 = 24MHz" as first assumed when this file was
 *   written. (The 48MHz figure in the datasheet's Clocks section
 *   describes the raw HICK oscillator used only by the separate
 *   hick_to_sclk direct-SCLK bypass path -- see the CRM_SCLK_HICK
 *   branch a few lines above the one quoted -- not the PLL entry tap.
 *   This 4MHz PLL-entry convention is the same one AT32F421 uses,
 *   mcu/AT32F421/config.c: "PLLMUL=011101(x30)" from HSI/2=4MHz.)
 *
 *   The previous code used CRM_PLL_MULT_4 (x4), giving 4MHz x4 = 16MHz
 *   -- exactly the measured value, and self-consistent with the
 *   readback (pll_mult decoded from CRM->cfg was indeed 4). This means
 *   the PLL itself DID lock (the code physically cannot reach the
 *   heartbeat loop otherwise -- see clock_config_96mhz(), both
 *   "while (crm_flag_get(CRM_PLL_STABLE_FLAG) != SET);" and
 *   "while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL);" are
 *   blocking waits with no timeout), and SCLK genuinely switched to
 *   PLL. The bug was purely the multiplier choice for a correctly
 *   locked, correctly source-switched PLL: 4MHz needs CRM_PLL_MULT_24
 *   (x24 = 96MHz), not CRM_PLL_MULT_4. No AT32F421 register was ever
 *   poked directly here (this file only calls AT32F425's own vendored
 *   crm_*() driver functions) -- the error was in the multiplier
 *   constant passed to crm_pll_config(), not in a copied register
 *   layout.
 * ---------------------------------------------------------------------
 */

#include "vendor/at32f425.h"

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

/*
 * Optional real-clock probe: CRM_CLKOUT_SCLK / DIV_16 on PA8, GPIO_MUX_0.
 * Confirmed against two official Artery examples that both do exactly
 * this (project/at_start_f425/examples/crm/sclk_switch/src/main.c and
 * .../crm/clock_failure_detection/src/main.c: gpio_pin_mux_config(GPIOA,
 * GPIO_PINS_SOURCE8, GPIO_MUX_0) + crm_clock_out_set(...)) -- not guessed.
 *
 * DISABLED BY DEFAULT: PA8 is the pin this port plans to use for
 * TMR1_CH1 (MP6540HA HSA) from Stage B onward. Only enable this with
 * MP6540HA definitely unpowered / nSLEEP held LOW, verify SCLK/16 =
 * 6.000 MHz on a scope at PA8, and then set this back to 0 before
 * starting Stage B (TIM1 bring-up needs PA8 for its real function).
 */
#define STAGE_A_ENABLE_CLKOUT 0

volatile uint32_t stage_a_heartbeat;
volatile int stage_a_clock_ok;

/* CRM diagnostics, captured right after clock_config_96mhz() returns.
 * Register/field names below are exactly as vendored from Artery's own
 * at32f425_crm.h / system_at32f425.c -- none are guessed. */
volatile uint32_t stage_a_crm_ctrl;   /* CRM->ctrl: hicken/hickstbl/pllen/pllstbl etc. */
volatile uint32_t stage_a_crm_cfg;    /* CRM->cfg: sclksel/sclksts/ahbdiv/apb1div/apb2div/pllrcs/pllmult_l/pllmult_h */
volatile uint32_t stage_a_crm_pll;    /* CRM->pll: pllms/pllns/pllfr/pllcfgen (fractional-N path, unused here) */
volatile uint32_t stage_a_sysclk_status; /* crm_sysclk_switch_status_get() result: expect CRM_SCLK_PLL (2) */
volatile uint32_t stage_a_pll_mult_decoded; /* pll_mult reconstructed the same way system_core_clock_update() does */

/*
 * vendor/startup_at32f425.s calls __libc_init_array (newlib), which
 * expects crti.o/crtn.o-provided _init/_fini. Those are pulled in by
 * gcc's normal startfiles, which -nostartfiles (used for every
 * ESCape32 target, see top-level CMakeLists.txt) deliberately
 * suppresses. Provide no-op stubs instead of pulling in gcc's startfiles.
 */
void _init(void) {}
void _fini(void) {}

static void clock_config_96mhz(void)
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

#if STAGE_A_ENABLE_CLKOUT
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE8, GPIO_MUX_0);
  crm_clkout_div_set(CRM_CLKOUT_DIV_16);
  crm_clock_out_set(CRM_CLKOUT_SCLK); /* expect 96MHz/16 = 6.000MHz on PA8 */
#endif
}

static void capture_crm_diagnostics(void)
{
  stage_a_crm_ctrl = CRM->ctrl;
  stage_a_crm_cfg = CRM->cfg;
  stage_a_crm_pll = CRM->pll;
  stage_a_sysclk_status = (uint32_t)crm_sysclk_switch_status_get();

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
    stage_a_pll_mult_decoded = pll_mult;
  }
}

int main(void)
{
  clock_config_96mhz();
  capture_crm_diagnostics();
  stage_a_clock_ok = (system_core_clock == 96000000U);

  /* Prove the GPIO/AHB2 bus is reachable without driving any pin
   * (unless STAGE_A_ENABLE_CLKOUT is on, see above). */
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  (void)GPIOA->idt;

  for (;;) {
    ++stage_a_heartbeat;
  }
}
