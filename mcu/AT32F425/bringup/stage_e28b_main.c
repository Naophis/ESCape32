/*
 * ESCape32 AT32F425 port -- Stage E28B: retest TMR2 as a genuine 32-bit
 * counter, this time actually enabling Plus Mode (PMEN) via
 * tmr_32_bit_function_enable() -- Stage E28 omitted this call entirely
 * and found TMR2->pr read back as 0x0000FFFF after a
 * tmr_base_init(TMR2, 0xFFFFFFFFu, ...) call, which looked like
 * conclusive proof TMR2 is hardware-16-bit. Per instruction, that
 * conclusion is wrong: per the official datasheet, TMR2 IS a 32-bit
 * timer, but only counts as 32-bit once CTRL1's PMEN bit is set (via
 * tmr_32_bit_function_enable(TMR2, TRUE)); left at its default (0), it
 * behaves exactly like a 16-bit timer -- which is exactly the
 * (misleading) symptom Stage E28 saw, because it never called this
 * function at all.
 *
 * Standalone, no motor involvement whatsoever (no TIM1/PWM/ADC/DMA),
 * same as Stage E27/E28. This file specifically checks, in order,
 * every item requested:
 *
 *   1. TMR2 reset + clock enable (clean state before configuring).
 *   2. tmr_32_bit_function_enable(TMR2, TRUE).
 *   3. Read back CTRL1.PMEN to confirm it actually latched as 1.
 *   4. tmr_base_init(TMR2, 0xFFFFFFFFu, 47u) (500ns/tick, matching
 *      IFTIM_XRES=0).
 *   5. Read back PR and confirm it is 0xFFFFFFFF this time (not
 *      truncated to 0xFFFF).
 *   6. Sample CVAL repeatedls while it counts past 0xFFFF, confirming
 *      it continues 0xFFFF -> 0x10000 -> 0x10001 -> ... rather than
 *      wrapping back to 0.
 *   7. Output-compare (CH3) scheduling at 70000/80000/140000 ticks
 *      (35/40/70ms) -- all three exceed a single 16-bit period
 *      (65536 ticks) and are exactly the ones Stage E28 got wrong by
 *      whole multiples of 65536.
 *   8. Software capture (CH1, EGR_CC1G-equivalent, Stage E24-validated
 *      mechanism) correctly holding a captured CCR1 value above
 *      65535.
 *
 * If all pass, IFTIM formally moves to TMR2 in mcu/AT32F425/config.h/
 * config.c/artery_hal.c, TMR3 becomes the dedicated 2us break-before-
 * make scheduler (no longer IFTIM), and the abandoned TMR3 software-
 * overflow-extension approach (Stage E27) is not carried into the
 * real target (it never was -- see prior confirmation). Motor spin
 * stays paused until this passes.
 */

#include "clock_config.h"

#define IFTIM_TICK_NS 500u
#define TICKS_PER_MS (1000000u / IFTIM_TICK_NS) /* 2000 */

static const uint32_t trial_delay_ms[] = {5, 20, 35, 40, 70};
#define NUM_TRIALS (sizeof(trial_delay_ms) / sizeof(trial_delay_ms[0]))

#define NUM_CVAL_SAMPLES 10

typedef struct {
  uint32_t pmen_before;     /* CTRL1.PMEN before enabling -- expect 0 */
  uint32_t pmen_after;      /* CTRL1.PMEN after tmr_32_bit_function_enable(TRUE) -- expect 1 */
  uint32_t pr_readback;     /* TMR2->pr after tmr_base_init(..., 0xFFFFFFFFu, ...) -- expect 0xFFFFFFFF this time */
  uint32_t cval_samples[NUM_CVAL_SAMPLES]; /* CVAL sampled every ~4ms starting just before the 0xFFFF boundary */
} stage_e28b_setup_result_t;

typedef struct {
  uint32_t delay_ms_requested;
  uint32_t ccr1_before;
  uint32_t ccr1_after;
  uint32_t elapsed_ticks;
  uint32_t expected_ticks;
  int32_t diff_ticks;
} stage_e28b_capture_trial_t;

typedef struct {
  uint32_t target_ticks;
  uint32_t cnt_at_flag_set;
  int32_t diff_ticks;
  uint32_t flag_seen;
} stage_e28b_oc_trial_t;

stage_e28b_setup_result_t stage_e28b_setup;
stage_e28b_capture_trial_t stage_e28b_capture_results[NUM_TRIALS];
stage_e28b_oc_trial_t stage_e28b_oc_results[NUM_TRIALS];
volatile uint32_t stage_e28b_heartbeat;
volatile int stage_e28b_done;

void _init(void) {}
void _fini(void) {}

static void timestamp_timer_config(void)
{
  crm_periph_clock_enable(CRM_TMR7_PERIPH_CLOCK, TRUE);
  tmr_cnt_dir_set(TMR7, TMR_COUNT_UP);
  tmr_base_init(TMR7, 0xFFFFFFFFu, (uint32_t)(system_core_clock / 1000000u) - 1u);
  tmr_counter_enable(TMR7, TRUE);
}

static void delay_us(uint32_t us)
{
  uint32_t start = TMR7->cval;
  while ((uint32_t)(TMR7->cval - start) < us);
}

static uint32_t tmr2_capture(void)
{
  uint32_t ccr;
  __disable_irq();
  TMR2->swevt_bit.c1swtr = TRUE;
  ccr = TMR2->c1dt;
  __enable_irq();
  return ccr;
}

int main(void)
{
  clock_config_96mhz();

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  timestamp_timer_config();

  /* --- 1. Reset + clock enable --- */
  crm_periph_reset(CRM_TMR2_PERIPH_RESET, TRUE);
  crm_periph_reset(CRM_TMR2_PERIPH_RESET, FALSE);
  crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);

  stage_e28b_setup.pmen_before = TMR2->ctrl1_bit.pmen;

  /* --- 2/3. Enable Plus Mode (32-bit), read back --- */
  tmr_32_bit_function_enable(TMR2, TRUE);
  stage_e28b_setup.pmen_after = TMR2->ctrl1_bit.pmen;

  tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);

  /* --- 4/5. base_init, read back PR --- */
  tmr_base_init(TMR2, 0xFFFFFFFFu, 47u); /* 96MHz/48 = 2MHz, 500ns/tick */
  stage_e28b_setup.pr_readback = TMR2->pr;

  TMR2->cctrl_bit.c1en = FALSE;
  TMR2->cm1_input_bit.c1c = 0x1; /* CC1S = 01, IC1 <- TI1 direct mapping */
  TMR2->cctrl_bit.c1en = TRUE;

  {
    tmr_output_config_type oc;
    tmr_output_default_para_init(&oc);
    oc.oc_mode = TMR_OUTPUT_CONTROL_OFF;
    oc.oc_output_state = FALSE;
    tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_3, &oc);
  }

  tmr_event_sw_trigger(TMR2, TMR_OVERFLOW_SWTRIG);
  tmr_counter_enable(TMR2, TRUE);

  /* --- 6. CVAL progression across the 0xFFFF boundary --- */
  {
    /* Run CNT up close to the boundary first (fast-forward via UG is
     * not usable here since we specifically want to WATCH it cross
     * 0xFFFF, not skip past it) -- just wait long enough from a fresh
     * UG=0 start, sampling every ~4ms; 0xFFFF ticks @ 500ns = 32.768ms,
     * so by sample ~8-9 we should be past it if PMEN is really working. */
    tmr_event_sw_trigger(TMR2, TMR_OVERFLOW_SWTRIG); /* UG -- CNT starts at 0 */
    for (uint32_t i = 0; i < NUM_CVAL_SAMPLES; i++) {
      delay_us(4000u);
      stage_e28b_setup.cval_samples[i] = TMR2->cval;
    }
  }

  /* --- 7/8. Capture accuracy + output-compare scheduling, same 5 delays --- */
  for (uint32_t i = 0; i < NUM_TRIALS; i++) {
    stage_e28b_capture_trial_t *r = &stage_e28b_capture_results[i];
    r->delay_ms_requested = trial_delay_ms[i];
    r->ccr1_before = tmr2_capture();

    delay_us(trial_delay_ms[i] * 1000u);

    r->ccr1_after = tmr2_capture();
    r->elapsed_ticks = r->ccr1_after - r->ccr1_before;
    r->expected_ticks = trial_delay_ms[i] * TICKS_PER_MS;
    r->diff_ticks = (int32_t)(r->elapsed_ticks - r->expected_ticks);

    delay_us(1000u);
  }

  for (uint32_t i = 0; i < NUM_TRIALS; i++) {
    stage_e28b_oc_trial_t *r = &stage_e28b_oc_results[i];
    r->target_ticks = trial_delay_ms[i] * TICKS_PER_MS;

    __disable_irq();
    tmr_event_sw_trigger(TMR2, TMR_OVERFLOW_SWTRIG); /* UG */
    TMR2->c3dt = r->target_ticks;
    tmr_flag_clear(TMR2, TMR_C3_FLAG);
    __enable_irq();

    uint32_t poll_start = TMR7->cval;
    uint32_t timeout_us = trial_delay_ms[i] * 1000u + 5000u;
    r->flag_seen = 0;
    while ((uint32_t)(TMR7->cval - poll_start) < timeout_us) {
      if (tmr_flag_get(TMR2, TMR_C3_FLAG) != RESET) {
        r->cnt_at_flag_set = TMR2->cval;
        r->flag_seen = 1;
        tmr_flag_clear(TMR2, TMR_C3_FLAG);
        break;
      }
    }
    r->diff_ticks = r->flag_seen ? (int32_t)(r->cnt_at_flag_set - r->target_ticks) : 0x7fffffff;

    delay_us(1000u);
  }

  stage_e28b_done = 1;

  for (;;) {
    ++stage_e28b_heartbeat;
  }
}
