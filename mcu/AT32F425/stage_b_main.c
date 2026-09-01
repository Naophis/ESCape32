/*
 * ESCape32 AT32F425 port -- Stage B bring-up (TIM1 6-step complementary
 * PWM, MP6540HA gate driver still UNPOWERED / nSLEEP held LOW).
 *
 * Scope (porting-plan Section 15 "Stage B"): drive a static (non-
 * commutating) 50% duty complementary PWM on all three TMR1 channel
 * pairs, with dead-time inserted, and verify on a scope:
 *   - PWM frequency
 *   - polarity (active high on the complementary output too, i.e.
 *     CxNP is NOT inverted -- matches MP6540HA's HSx/LSx = CHx/CHxN
 *     direct mapping, no PWM_ENABLE-style HS/EN scheme)
 *   - complementary behavior (CH1 and CH1C never both high)
 *   - dead time between CH1 falling and CH1C rising (and vice versa)
 *   - all-off state (tmr_output_enable(TMR1, FALSE) forces every gate low)
 *
 * MP6540HA MUST remain unpowered (nSLEEP low / VM disconnected) for
 * this entire stage. No commutation, no BEMF, no ADC: `sync` never
 * needs to reach anything here, there is no `nextstep()` involved.
 *
 * Pin plan (QFN32 package, AT32F425K8U7-4), all cross-checked against
 * DS_AT32F425_V2.03_EN.pdf Table 5 (pin definitions) -- see the
 * porting-plan Section 18 report for the derivation of the MUX index
 * (datasheet IOMUX column order == GPIO_MUX_N index, verified against
 * 4 independent official Artery examples for PA8/PA9/PA10/CLKOUT):
 *
 *   TMR1_CH1  -> PA8  (GPIO_MUX_2) -> MP6540HA HSA
 *   TMR1_CH1C -> PA7  (GPIO_MUX_3) -> MP6540HA LSA
 *   TMR1_CH2  -> PA9  (GPIO_MUX_2) -> MP6540HA HSB
 *   TMR1_CH2C -> PB0  (GPIO_MUX_1) -> MP6540HA LSB
 *   TMR1_CH3  -> PA10 (GPIO_MUX_2) -> MP6540HA HSC
 *   TMR1_CH3C -> PB1  (GPIO_MUX_2) -> MP6540HA LSC
 *
 * PA7/PB0/PB1's MUX indices (3/1/2) are NOT independently confirmed by
 * an official example (only PA8/PA9/PA10 = MUX_2 are). If one of them
 * is wrong, the affected pin will simply stay at its floating-input
 * reset state and show NO signal on the scope (safe failure mode, not
 * a live-but-wrong signal) -- that is exactly what this stage's scope
 * checklist is for.
 */

#include "clock_config.h"

volatile uint32_t stage_b_heartbeat;
volatile int stage_b_pwm_configured;
volatile uint32_t stage_b_tmr1_ctrl1;
volatile uint32_t stage_b_tmr1_cctrl; /* CCER-equivalent: CxE/CxCE/CxP/CxCP */
volatile uint32_t stage_b_tmr1_brk;   /* BDTR-equivalent: dtc/brken/aoen/moen */
volatile uint32_t stage_b_tmr1_pr;    /* ARR */

void _init(void) {}
void _fini(void) {}

/*
 * PWM test frequency: 24kHz (ESCape32's own freq_min default), safe
 * and easy to read on a scope. arr = CLK/24000 - 1.
 */
#define STAGE_B_PWM_ARR (96000000u / 24000u - 1u)

/*
 * Dead time: linear DTG region (DEAD_TIME<128 -> TIM_DTG==DEAD_TIME,
 * see src/defs.h). 53 counts @96MHz ~= 552ns, matching the ~550ns
 * F421 targets get from DEAD_TIME=66 @120MHz. THIS IS A STARTING
 * VALUE ONLY -- confirm the real dead time on a scope (HSx/LSx) and
 * adjust against MP6540HA's actual switching characteristics before
 * trusting it under load.
 */
#define STAGE_B_DEAD_TIME 53u

static void tim1_pwm_gpio_config(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;

  gpio_init_struct.gpio_pins = GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10;
  gpio_init(GPIOA, &gpio_init_struct);

  gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1;
  gpio_init(GPIOB, &gpio_init_struct);

  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE8, GPIO_MUX_2);  /* CH1  */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_3);  /* CH1C */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_2);  /* CH2  */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE0, GPIO_MUX_1);  /* CH2C */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_2); /* CH3  */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE1, GPIO_MUX_2);  /* CH3C */
}

static void tim1_pwm_config(void)
{
  tmr_output_config_type oc;
  tmr_brkdt_config_type brkdt;
  uint16_t ccr = (STAGE_B_PWM_ARR + 1u) / 2u; /* 50% duty */

  tmr_base_init(TMR1, (uint16_t)STAGE_B_PWM_ARR, 0);
  tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);

  tmr_output_default_para_init(&oc);
  oc.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  oc.oc_output_state = TRUE;
  oc.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  oc.oc_idle_state = FALSE; /* idle = LOW: safe state, MP6540HA HSx=0 */
  oc.occ_output_state = TRUE;
  oc.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  oc.occ_idle_state = FALSE; /* idle = LOW: safe state, MP6540HA LSx=0 */

  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &oc);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, ccr);
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_2, &oc);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, ccr);
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &oc);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, ccr);

  tmr_brkdt_default_para_init(&brkdt);
  brkdt.brk_enable = FALSE; /* BRK pin (PA6) not wired/confirmed yet */
  brkdt.auto_output_enable = TRUE;
  brkdt.deadtime = STAGE_B_DEAD_TIME;
  brkdt.fcsodis_state = TRUE;
  brkdt.fcsoen_state = TRUE;
  brkdt.brk_polarity = TMR_BRK_INPUT_ACTIVE_HIGH;
  brkdt.wp_level = TMR_WP_OFF;
  tmr_brkdt_config(TMR1, &brkdt);

  tmr_channel_buffer_enable(TMR1, TRUE);
  /* Commit the preloaded CCMR/CCER/BRK values (CCPC-style: needs an
   * update+hall/COM software event, same pattern ESCape32's own
   * laststep()/nextstep() use as TIM_EGR_UG|TIM_EGR_COMG on F421). */
  tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG | TMR_HALL_SWTRIG);

  tmr_output_enable(TMR1, TRUE);
  tmr_counter_enable(TMR1, TRUE);
}

static void capture_tmr1_diagnostics(void)
{
  stage_b_tmr1_ctrl1 = TMR1->ctrl1;
  stage_b_tmr1_cctrl = TMR1->cctrl;
  stage_b_tmr1_brk = TMR1->brk;
  stage_b_tmr1_pr = TMR1->pr;
}

int main(void)
{
  clock_config_96mhz();

  tim1_pwm_gpio_config();
  tim1_pwm_config();
  capture_tmr1_diagnostics();
  stage_b_pwm_configured = (system_core_clock == 96000000U);

  for (;;) {
    ++stage_b_heartbeat;
  }
}
