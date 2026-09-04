/*
 * ESCape32 AT32F425 port -- Stage E25: TMR3(TRGO) -> TMR1(TRGI/COM)
 * event path, standalone register-level verification ONLY. NOT a
 * motor-control test -- TMR1's MOE/output stage is never enabled, no
 * GPIO is routed to TIM1 alternate function, no PWM channel is
 * configured at all. Motor drive stays fully off throughout.
 *
 * WHY: upstream ESCape32's commutation-timing mechanism relies on
 * TIM1_CR2 = TIM_CR2_CCPC | TIM_CR2_CCUS | TIM_CR2_MMS_COMPARE_PULSE
 * (CCPC = shadow/preload enable, CCUS = "a TRGI event also commits the
 * shadow registers", i.e. an automatic COM event driven by IFTIM's
 * delayed pulse arriving via TIM1's internal trigger input) together
 * with TIM1_SMCR = TIM_SMCR_TS_ITR2 (select IFTIM as that internal
 * trigger source). Cross-checking Artery's OWN vendor driver against
 * this: the bit at the SAME position STM32 calls CCUS (ctrl2 bit2) is
 * named `ccfs` in Artery's header and implemented by
 * tmr_hall_select() -- literally "select Hall sensor mode", not
 * obviously the same "commit-shadow-registers-on-trigger" function
 * STM32's CCUS performs. However: TMR1_BRK_OVF_TRG_HALL_IRQn / the
 * matching TMR1_BRK_OVF_TRG_HALL_IRQHandler weak symbol (vendored
 * startup_at32f425.s) is the SAME 4-way combined vector STM32 calls
 * TIM1_BRK_UP_TRG_COM -- i.e. Artery appears to have renamed "COM" to
 * "HALL" throughout this peripheral's naming (matching this project's
 * own prior finding that TMR_HALL_SWTRIG, used successfully as the
 * software commutation-commit trigger since Stage E2, occupies
 * exactly the bit position STM32 calls COMG). This file tests,
 * empirically rather than by further name-reading, whether the
 * HARDWARE version of that same mechanism (a real TRGI event, not a
 * software COMG/HALLSWTRG) also produces a working automatic
 * shadow-commit + interrupt -- and if so, which of TMR1's four
 * internal-trigger-select values (IS0-IS3, sub_tmr_input_sel_type)
 * actually carries TMR3's TRGO signal on this specific silicon.
 *
 * Method: TMR3 is configured with tmr_primary_mode_select(TMR3,
 * TMR_PRIMARY_SEL_RESET) (STM32's MMS=Reset -- TRGO pulses on every
 * UG/counter-reset event), so a plain tmr_event_sw_trigger(TMR3,
 * TMR_OVERFLOW_SWTRIG) produces a TRGO pulse at a precisely known
 * instant (recorded via TMR2, this project's standard microsecond
 * timestamp). TMR1 is configured with CCPC enabled (tmr_channel_
 * buffer_enable(), the same call already proven correct throughout
 * Stage E2-E23's software-COM commutation), the bit-in-question set
 * (tmr_hall_select(TMR1, TRUE)) exactly as upstream's CCUS write
 * would be attempted, TMR_HALL_INT enabled, and MOE explicitly left
 * OFF (tmr_output_enable(TMR1, FALSE) -- never called TRUE at all in
 * this file). For each of TMR_SUB_INPUT_SEL_IS0..IS3 in turn
 * (tmr_trigger_input_select(TMR1, ...)), 8 trials each fire a TMR3
 * TRGO pulse and record: whether TMR1's combined BRK/OVF/TRG/HALL ISR
 * fired at all, which of ists.hallif (TMR_HALL_FLAG) and ists.trgif
 * (bit6, no vendor macro exists for it so read directly) were set,
 * the TRGO-generation timestamp, and the ISR-entry timestamp.
 *
 * Verdict per instruction: if one IS value produces a 1:1 TRGO-to-
 * COM-event correspondence across all 8 trials, that IS is the
 * correct upstream-equivalent trigger source and the CCUS-equivalent
 * automatic-commit path IS usable on this part (adopt that IS,
 * proceed with the upstream-style CR2/SMCR wiring in the real
 * backend). If NO IS value ever produces a hardware COM event, the
 * STM32-CCUS-dependent path is NOT used on F425; the real backend
 * instead has IFTIM's own one-shot-complete interrupt directly call
 * tmr_event_sw_trigger(TMR1, TMR_HALL_SWTRIG) in software (exactly the
 * mechanism already proven throughout Stage E2-E23). No further IS
 * sweeping or bit-meaning guessing is done beyond this file, per
 * instruction.
 */

#include "clock_config.h"

#define NUM_IS_CANDIDATES 4u
#define NUM_TRIALS 8u
#define TRIAL_SPACING_US 500u

static const sub_tmr_input_sel_type is_candidates[NUM_IS_CANDIDATES] = {
  TMR_SUB_INPUT_SEL_IS0, TMR_SUB_INPUT_SEL_IS1, TMR_SUB_INPUT_SEL_IS2, TMR_SUB_INPUT_SEL_IS3
};

typedef struct {
  uint32_t trgo_timestamp_us;
  uint32_t com_isr_timestamp_us; /* 0 if it never fired */
  uint32_t isr_fired;            /* 0/1 */
  uint32_t hallif_seen;          /* TMR_HALL_FLAG, read inside the ISR */
  uint32_t trgif_seen;           /* ists bit6, no vendor macro -- read directly, inside the ISR */
} stage_e25_trial_t;

stage_e25_trial_t stage_e25_results[NUM_IS_CANDIDATES][NUM_TRIALS];
volatile uint32_t stage_e25_com_event_count;
volatile uint32_t stage_e25_last_isr_timestamp_us;
volatile uint32_t stage_e25_last_hallif;
volatile uint32_t stage_e25_last_trgif;
volatile uint32_t stage_e25_heartbeat;
volatile int stage_e25_done;

void _init(void) {}
void _fini(void) {}

static void timestamp_timer_config(void)
{
  crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
  tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
  tmr_base_init(TMR2, 0xFFFFFFFFu, (uint32_t)(system_core_clock / 1000000u) - 1u);
  tmr_counter_enable(TMR2, TRUE);
}

static void delay_us(uint32_t us)
{
  while (us) {
    uint16_t chunk = (us > 60000u) ? 60000u : (uint16_t)us;
    uint16_t start = (uint16_t)TMR2->cval;
    while ((uint16_t)((uint16_t)TMR2->cval - start) < chunk);
    us -= chunk;
  }
}

/* Motor safety: all six gate pins held plain-GPIO-low throughout --
 * this file never touches TIM1's AF routing or output stage at all. */
static void gate_pins_force_off(void)
{
  gpio_init_type g;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

  gpio_bits_reset(GPIOA, GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10);
  gpio_bits_reset(GPIOB, GPIO_PINS_0 | GPIO_PINS_1);

  gpio_default_para_init(&g);
  g.gpio_mode = GPIO_MODE_OUTPUT;
  g.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  g.gpio_pull = GPIO_PULL_NONE;
  g.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;

  g.gpio_pins = GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10;
  gpio_init(GPIOA, &g);
  g.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1;
  gpio_init(GPIOB, &g);

  gpio_bits_reset(GPIOA, GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10);
  gpio_bits_reset(GPIOB, GPIO_PINS_0 | GPIO_PINS_1);
}

/*
 * TMR1: CCPC enabled (tmr_channel_buffer_enable, already proven
 * correct throughout Stage E2-E23), the bit-in-question set
 * (tmr_hall_select -- ctrl2 bit2, upstream's CCUS write target),
 * TMR_HALL_INT enabled. MOE is NEVER enabled in this file -- no
 * tmr_output_enable(TMR1, TRUE) call exists anywhere below. No output
 * channel is configured at all.
 */
static void tmr1_test_init(void)
{
  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

  tmr_base_init(TMR1, 999u, 0); /* ARR arbitrary -- no PWM output involved */
  tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);

  tmr_channel_buffer_enable(TMR1, TRUE); /* CCPC-equivalent */
  tmr_hall_select(TMR1, TRUE);           /* the bit in question -- ctrl2 bit2 */

  nvic_irq_enable(TMR1_BRK_OVF_TRG_HALL_IRQn, 1, 0);
  tmr_interrupt_enable(TMR1, TMR_HALL_INT, TRUE);

  tmr_output_enable(TMR1, FALSE); /* explicit: MOE stays off throughout this file */
  tmr_counter_enable(TMR1, TRUE);
}

/*
 * TMR3: primary(trigger-output) mode = RESET, so a software UG event
 * (tmr_event_sw_trigger(TMR3, TMR_OVERFLOW_SWTRIG)) produces a TRGO
 * pulse at a precisely known instant -- our controlled stimulus.
 */
static void tmr3_trgo_source_init(void)
{
  crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
  tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
  tmr_base_init(TMR3, 0xFFFFu, 95u); /* free-running, 1us/tick -- period value irrelevant here */
  tmr_primary_mode_select(TMR3, TMR_PRIMARY_SEL_RESET);
  tmr_counter_enable(TMR3, TRUE);
}

void TMR1_BRK_OVF_TRG_HALL_IRQHandler(void)
{
  uint32_t now_us = TMR2->cval;
  int fired = 0;

  if (tmr_flag_get(TMR1, TMR_HALL_FLAG) != RESET) {
    tmr_flag_clear(TMR1, TMR_HALL_FLAG);
    stage_e25_last_hallif = 1;
    fired = 1;
  } else {
    stage_e25_last_hallif = 0;
  }

  /* ists bit6 (trgif) -- no vendor macro exists for this flag, read
   * directly. Clearing follows the same "write 0 to the bit" pattern
   * every other tmr_flag_clear() case uses on this part. */
  if (TMR1->ists & (1u << 6)) {
    TMR1->ists &= ~(1u << 6);
    stage_e25_last_trgif = 1;
    fired = 1;
  } else {
    stage_e25_last_trgif = 0;
  }

  if (tmr_flag_get(TMR1, TMR_OVF_FLAG) != RESET) tmr_flag_clear(TMR1, TMR_OVF_FLAG);
  if (tmr_flag_get(TMR1, TMR_BRK_FLAG) != RESET) tmr_flag_clear(TMR1, TMR_BRK_FLAG);

  if (fired) {
    stage_e25_last_isr_timestamp_us = now_us;
    stage_e25_com_event_count++;
  }
}

int main(void)
{
  clock_config_96mhz();

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  gate_pins_force_off();
  timestamp_timer_config();
  tmr1_test_init();
  tmr3_trgo_source_init();

  for (uint32_t is = 0; is < NUM_IS_CANDIDATES; is++) {
    tmr_trigger_input_select(TMR1, is_candidates[is]);

    for (uint32_t trial = 0; trial < NUM_TRIALS; trial++) {
      delay_us(TRIAL_SPACING_US);

      uint32_t before_count = stage_e25_com_event_count;
      uint32_t trgo_ts = TMR2->cval;

      tmr_event_sw_trigger(TMR3, TMR_OVERFLOW_SWTRIG);

      delay_us(50u); /* let the ISR run */

      stage_e25_trial_t *r = &stage_e25_results[is][trial];
      r->trgo_timestamp_us = trgo_ts;
      r->isr_fired = (stage_e25_com_event_count != before_count) ? 1u : 0u;
      r->com_isr_timestamp_us = r->isr_fired ? stage_e25_last_isr_timestamp_us : 0u;
      r->hallif_seen = stage_e25_last_hallif;
      r->trgif_seen = stage_e25_last_trgif;
    }
  }

  stage_e25_done = 1;

  for (;;) {
    ++stage_e25_heartbeat;
  }
}
