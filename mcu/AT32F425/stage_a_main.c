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
 * GPIO policy for this stage: no pin is driven. Per the AT32F425
 * datasheet (Table 5 note), all GPIOs default to floating input on
 * reset; since the real schematic (which pins reach MP6540HA gates)
 * has not been confirmed yet, Stage A only performs a read of GPIOA
 * (a side-effect-free bus access) to prove the AHB2 GPIO domain is
 * clocked and reachable. No ODR/MUX writes are performed here.
 *
 * Liveness is proven via SWD by inspecting `stage_a_heartbeat` (free
 * running counter) and `stage_a_clock_ok` (set once system_core_clock
 * is confirmed == 96000000) in a debugger, rather than by toggling an
 * physical LED/GPIO whose function on this board is not yet confirmed.
 */

#include "vendor/at32f425.h"

/*
 * Clock plan (see porting-plan Section 18-B):
 *   HICK (48 MHz) --/2--> 24 MHz --x4 PLL--> 96 MHz
 * This path does NOT require an HEXT crystal. Official Artery BSP
 * examples default to HEXT x12 instead; we default to HICK here
 * because the presence of an HEXT crystal on this board has not
 * been confirmed (existing AT32F421 ESCape32 targets are crystal-less).
 * If HEXT is confirmed present, switch AT32F425_CLOCK_SOURCE_HEXT to 1
 * below and set HEXT_VALUE in vendor/at32f425_conf.h to the real
 * crystal frequency.
 */
#define AT32F425_CLOCK_SOURCE_HEXT 0

volatile uint32_t stage_a_heartbeat;
volatile int stage_a_clock_ok;

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
  crm_pll_config(CRM_PLL_SOURCE_HICK, CRM_PLL_MULT_4); /* (HICK/2=24MHz) x4 = 96MHz */
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

int main(void)
{
  clock_config_96mhz();
  stage_a_clock_ok = (system_core_clock == 96000000U);

  /* Prove the GPIO/AHB2 bus is reachable without driving any pin. */
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  (void)GPIOA->idt;

  for (;;) {
    ++stage_a_heartbeat;
  }
}
