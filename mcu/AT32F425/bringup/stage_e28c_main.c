/*
 * ESCape32 AT32F425 port -- Stage E28C: two focused, motor-off
 * diagnostics following Stage E28B (TMR2 Plus Mode/32-bit confirmed
 * working for CVAL progression and PR/PMEN, but CH3 output-compare
 * failed to fire at 80000/140000 ticks and CH1 software capture
 * produced inconsistent values):
 *
 *   PART 1 -- CH3 output-compare failure isolation. For target =
 *   70000 (known-good in E28B), 80000, and 140000 ticks: snapshot
 *   C3DT readback, CVAL, PR, the C3 compare flag, and C3's interrupt-
 *   enable bit immediately after arming, then poll to either the flag
 *   firing or a timeout, recording CVAL at whichever happens.
 *   Distinguishes three hypotheses:
 *     - C3DT correctly holds 80000/140000 and CVAL visibly passes
 *       that value, but the flag never sets -> a Plus-Mode compare-
 *       EVENT generation problem (register content is fine, the
 *       comparator/flag-generation logic isn't working across that
 *       range).
 *     - C3DT itself reads back truncated/wrong right after being
 *       written -> a 32-bit CCR3 register-access problem.
 *     - CVAL at timeout hasn't even reached the target yet -> this
 *       test harness's own timeout is simply too short, not a timer
 *       problem at all.
 *
 *   PART 2 -- validates the DESIGN REPLACEMENT for CH1 software
 *   capture: since every real "ZC" in the ADC-ZC backend already
 *   happens as a software event (dma1_channel1_isr's schmitt-trigger
 *   confirm), there is no need for a hardware/software TIMER CAPTURE
 *   at all -- the confirm handler can simply read TMR2->cval directly
 *   at that instant. This part synthesizes "ZC" events at 5/20/35/40/
 *   70ms intervals via plain CVAL reads (no CH1/EGR_CC1G/swevt
 *   involved anywhere) and checks the plain 32-bit subtraction
 *   between consecutive reads reconstructs the interval to within a
 *   few ticks -- exactly the same check Part 1's CVAL progression
 *   already implied should work, confirmed end-to-end here as the
 *   actual intended usage pattern.
 *
 * No motor involvement whatsoever (no TIM1/PWM/ADC/DMA). Per
 * instruction, CH1 software-capture's Plus-Mode-specific behavior is
 * NOT investigated further after this file -- if Part 2 passes, the
 * real ADC-ZC backend abandons the capture-register path entirely in
 * favor of direct CVAL reads, sidestepping whatever Part 1/prior CH1
 * anomaly exists rather than debugging it further. upstream's own
 * accept condition/timing math is not touched anywhere.
 */

#include "clock_config.h"

#define IFTIM_TICK_NS 500u
#define TICKS_PER_MS (1000000u / IFTIM_TICK_NS) /* 2000 */

static const uint32_t oc_targets[] = {70000, 80000, 140000};
#define NUM_OC_TARGETS (sizeof(oc_targets) / sizeof(oc_targets[0]))

static const uint32_t trial_delay_ms[] = {5, 20, 35, 40, 70};
#define NUM_TRIALS (sizeof(trial_delay_ms) / sizeof(trial_delay_ms[0]))

typedef struct {
  uint32_t target_ticks;
  uint32_t c3dt_readback;     /* TMR2->c3dt read back immediately after being written */
  uint32_t cval_at_arm;       /* TMR2->cval at the same moment */
  uint32_t pr_at_arm;         /* TMR2->pr at the same moment -- confirm still 0xFFFFFFFF */
  uint32_t c3_flag_at_arm;    /* Compare flag immediately after arming -- expect 0 (just cleared) */
  uint32_t c3ien_at_arm;      /* DIER.C3IEN -- expect 0, we poll the flag directly, no interrupt used */
  uint32_t flag_seen;         /* 0/1 -- did the compare flag set before the poll timeout */
  uint32_t cval_at_flag_or_timeout; /* CVAL read at whichever of those happened */
  int32_t diff_ticks;         /* cval_at_flag_or_timeout - target_ticks, only meaningful if flag_seen */
} stage_e28c_oc_diag_t;

typedef struct {
  uint32_t delay_ms_requested;
  uint32_t cval_before;   /* Direct TMR2->cval read -- the "just before this synthetic ZC" value */
  uint32_t cval_after;    /* Direct TMR2->cval read at the "current synthetic ZC" */
  uint32_t elapsed_ticks; /* Plain 32-bit subtraction, no capture register involved */
  uint32_t expected_ticks;
  int32_t diff_ticks;
} stage_e28c_cval_trial_t;

stage_e28c_oc_diag_t stage_e28c_oc_results[NUM_OC_TARGETS];
stage_e28c_cval_trial_t stage_e28c_cval_results[NUM_TRIALS];
volatile uint32_t stage_e28c_heartbeat;
volatile int stage_e28c_done;

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

static void tmr2_32bit_init(void)
{
  crm_periph_reset(CRM_TMR2_PERIPH_RESET, TRUE);
  crm_periph_reset(CRM_TMR2_PERIPH_RESET, FALSE);
  crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);

  tmr_32_bit_function_enable(TMR2, TRUE); /* PMEN -- Stage E28B-validated fix */
  tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
  tmr_base_init(TMR2, 0xFFFFFFFFu, 47u); /* 96MHz/48 = 2MHz, 500ns/tick */

  {
    tmr_output_config_type oc;
    tmr_output_default_para_init(&oc);
    oc.oc_mode = TMR_OUTPUT_CONTROL_OFF;
    oc.oc_output_state = FALSE;
    tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_3, &oc);
  }

  tmr_event_sw_trigger(TMR2, TMR_OVERFLOW_SWTRIG);
  tmr_counter_enable(TMR2, TRUE);
}

int main(void)
{
  clock_config_96mhz();

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  timestamp_timer_config();
  tmr2_32bit_init();

  /* --- Part 1: CH3 output-compare failure isolation --- */
  for (uint32_t i = 0; i < NUM_OC_TARGETS; i++) {
    stage_e28c_oc_diag_t *r = &stage_e28c_oc_results[i];
    r->target_ticks = oc_targets[i];

    __disable_irq();
    tmr_event_sw_trigger(TMR2, TMR_OVERFLOW_SWTRIG); /* UG -- CNT starts at 0 */
    TMR2->c3dt = r->target_ticks;
    tmr_flag_clear(TMR2, TMR_C3_FLAG);
    r->c3dt_readback = TMR2->c3dt;
    r->cval_at_arm = TMR2->cval;
    r->pr_at_arm = TMR2->pr;
    r->c3_flag_at_arm = (tmr_flag_get(TMR2, TMR_C3_FLAG) != RESET) ? 1u : 0u;
    r->c3ien_at_arm = TMR2->iden_bit.c3ien;
    __enable_irq();

    uint32_t poll_start = TMR7->cval;
    uint32_t timeout_us = (r->target_ticks / TICKS_PER_MS) * 1000u + 20000u; /* generous margin */
    r->flag_seen = 0;
    for (;;) {
      if (tmr_flag_get(TMR2, TMR_C3_FLAG) != RESET) {
        r->flag_seen = 1;
        r->cval_at_flag_or_timeout = TMR2->cval;
        tmr_flag_clear(TMR2, TMR_C3_FLAG);
        break;
      }
      if ((uint32_t)(TMR7->cval - poll_start) >= timeout_us) {
        r->cval_at_flag_or_timeout = TMR2->cval; /* Timeout -- record where CVAL actually ended up */
        break;
      }
    }
    r->diff_ticks = (int32_t)(r->cval_at_flag_or_timeout - r->target_ticks);

    delay_us(2000u);
  }

  /* --- Part 2: direct CVAL read as the "ZC timestamp", no capture register at all --- */
  for (uint32_t i = 0; i < NUM_TRIALS; i++) {
    stage_e28c_cval_trial_t *r = &stage_e28c_cval_results[i];
    r->delay_ms_requested = trial_delay_ms[i];

    __disable_irq();
    r->cval_before = TMR2->cval;
    __enable_irq();

    delay_us(trial_delay_ms[i] * 1000u);

    __disable_irq();
    r->cval_after = TMR2->cval;
    __enable_irq();

    r->elapsed_ticks = r->cval_after - r->cval_before;
    r->expected_ticks = trial_delay_ms[i] * TICKS_PER_MS;
    r->diff_ticks = (int32_t)(r->elapsed_ticks - r->expected_ticks);

    delay_us(1000u);
  }

  stage_e28c_done = 1;

  for (;;) {
    ++stage_e28c_heartbeat;
  }
}
