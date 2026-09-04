/*
 * ESCape32 AT32F425 port -- Stage E28: verify TMR2, configured as a
 * WIDE (32-bit) free-running timer, as the candidate replacement for
 * TMR3's role as upstream's IFTIM -- specifically to eliminate the
 * 16-bit-wrap accounting failure Stage E27 found (35ms/70ms trials
 * landed on a bogus, identical raw value of ARR-3 with no overflow
 * detected at all; per instruction that overflow-extension approach is
 * abandoned rather than debugged further).
 *
 * New role split (per instruction, not yet applied to config.c -- this
 * file is the prerequisite verification only):
 *   TMR1  : 6PWM (unchanged)
 *   TMR2  : upstream IFTIM / ZC capture / timing advance (32-bit) -- NEW
 *   TMR3  : 2us break-before-make scheduler (was IFTIM; freed up by TMR2 taking over)
 *   TMR7  : BENCH_TEST elapsed-time watchdog (unchanged)
 *   TMR15 : IOTIM/DSHOT (unchanged)
 *
 * This file is standalone and touches NOTHING else -- no TIM1/PWM, no
 * ADC/DMA, no GPIO output, no motor involvement whatsoever. It
 * verifies three things TMR2 needs to provide correctly as IFTIM,
 * mirroring exactly what upstream's iftim_isr()/nextstep() (src/
 * main.c, unmodified) assume about IFTIM's register behavior:
 *
 *   1. Free-running capture, 500ns/tick (matching IFTIM_XRES=0):
 *      CH1 configured as genuine input-capture (same IFTIM_ICM1/2/3
 *      convention as any other backend), captured only via software
 *      (EGR_CC1G / c1swtr, Stage E24-validated equivalence to a real
 *      edge), at 5/20/35/40/70ms synthetic intervals -- since TMR2's
 *      ARR here is the FULL 32-bit range (0xFFFFFFFF), none of these
 *      delays can wrap at all (32-bit @ 500ns/tick wraps only after
 *      ~35.8 minutes), so this checks plain, direct accuracy with NO
 *      overflow/epoch bookkeeping needed -- the entire class of bug
 *      Stage E27 hit structurally cannot occur here.
 *   2. UG reset semantics: upstream's iftim_isr() (accept path) issues
 *      TIM_EGR(IFTIM)=TIM_EGR_UG once it accepts a capture, and from
 *      then on relies on IFTIM_ICR at the NEXT capture directly
 *      representing "elapsed since that UG". This trial explicitly
 *      issues UG (EGR_OVERFLOW_SWTRIG, same bit position as UG -- see
 *      this project's own confirmed EGR/swevt bit-position audit),
 *      confirms CNT reads back near 0 immediately after, then times a
 *      known interval and confirms the capture correctly reports
 *      "time since the UG", not "time since epoch/boot".
 *   3. Output-compare scheduling: CH3 (matching IFTIM_OCR=CCR3, the
 *      register nextstep()/iftim_isr() write as the commutation
 *      target) configured for compare-match (no physical output pin
 *      needed -- the C3F flag sets purely from the internal
 *      comparator, independent of any output enable). After a UG
 *      reset, CCR3 is set to a known target tick count; this checks
 *      that the compare flag fires at (or extremely close to) that
 *      absolute CNT value -- the exact mechanism the future TMR3
 *      2us-scheduler will depend on to read a *correct* IFTIM_OCR/
 *      CCR3 value from TMR2 and schedule its own gate-off/commit
 *      relative to it.
 *
 * Pass criteria (per instruction): ALL FIVE delays (5/20/35/40/70ms)
 * must reconstruct to within a few ticks of the expected value, with
 * NO wrap-accounting involved at all. If confirmed, IFTIM formally
 * moves to TMR2 in mcu/AT32F425/config.h/config.c/artery_hal.c and
 * TMR3 is repurposed for the 2us scheduler; motor spin stays paused
 * until this passes.
 */

#include "clock_config.h"

#define IFTIM_TICK_NS 500u /* 500ns/tick, matches IFTIM_XRES=0 exactly */
#define TICKS_PER_MS (1000000u / IFTIM_TICK_NS) /* 2000 */

static const uint32_t trial_delay_ms[] = {5, 20, 35, 40, 70};
#define NUM_TRIALS (sizeof(trial_delay_ms) / sizeof(trial_delay_ms[0]))

typedef struct {
  uint32_t delay_ms_requested;
  uint32_t cnt_before;      /* TMR2_CNT read immediately before the "previous ZC" capture trigger */
  uint32_t ccr1_before;     /* TMR2_CCR1 latched by the "previous ZC" capture (== upstream's IFTIM_ICR) */
  uint32_t cnt_after;       /* TMR2_CNT read immediately before the "current ZC" capture trigger */
  uint32_t ccr1_after;      /* TMR2_CCR1 latched by the "current ZC" capture */
  uint32_t elapsed_ticks;   /* ccr1_after - ccr1_before, plain 32-bit subtraction, no epoch logic */
  uint32_t expected_ticks;  /* delay_ms_requested * TICKS_PER_MS */
  int32_t diff_ticks;       /* elapsed_ticks - expected_ticks, signed */
} stage_e28_capture_trial_t;

typedef struct {
  uint32_t cnt_immediately_after_ug; /* Should read very close to 0 */
  uint32_t delay_ms_requested;
  uint32_t ccr1_captured;   /* Should closely match expected_ticks, since counting started at 0 */
  uint32_t expected_ticks;
  int32_t diff_ticks;
} stage_e28_ug_trial_t;

typedef struct {
  uint32_t target_ticks;      /* CCR3 programmed value, relative to a UG reset */
  uint32_t cnt_at_flag_set;   /* TMR2_CNT read as soon as the polling loop observes C3F set */
  int32_t diff_ticks;         /* cnt_at_flag_set - target_ticks */
  uint32_t flag_seen;         /* 0/1 -- did C3F actually set within the poll timeout */
} stage_e28_oc_trial_t;

stage_e28_capture_trial_t stage_e28_capture_results[NUM_TRIALS];
stage_e28_ug_trial_t stage_e28_ug_results[NUM_TRIALS];
stage_e28_oc_trial_t stage_e28_oc_results[NUM_TRIALS];
volatile uint32_t stage_e28_heartbeat;
volatile int stage_e28_done;

void _init(void) {}
void _fini(void) {}

/* Independent reference clock for the requested delays -- TMR7 here
 * (not TMR2, which is under test), same validated 1MHz/1us/32-bit-ARR
 * pattern this project has used since Stage E14/E24/E27. */
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

/* TMR2 under test: free-running, 32-bit ARR, 500ns/tick. CH1 input-
 * capture (IFTIM_ICM convention). CH3 output-compare, no physical pin
 * routed -- only the internal C3F flag is used. */
static void tmr2_test_init(void)
{
  crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);

  tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
  tmr_base_init(TMR2, 0xFFFFFFFFu, 47u); /* 96MHz/48 = 2MHz, 500ns/tick -- (CLK_MHZ>>(XRES+1))-1 = 47 */

  TMR2->cctrl_bit.c1en = FALSE;
  TMR2->cm1_input_bit.c1c = 0x1; /* CC1S = 01, IC1 <- TI1 direct mapping */
  TMR2->cctrl_bit.c1en = TRUE;

  {
    tmr_output_config_type oc;
    tmr_output_default_para_init(&oc);
    oc.oc_mode = TMR_OUTPUT_CONTROL_OFF; /* No physical output needed -- only the C3F compare flag is used */
    oc.oc_output_state = FALSE;
    tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_3, &oc);
  }

  tmr_event_sw_trigger(TMR2, TMR_OVERFLOW_SWTRIG); /* commit config, established project pattern */
  tmr_counter_enable(TMR2, TRUE);
}

static uint32_t tmr2_capture(void)
{
  uint32_t ccr;
  __disable_irq();
  TMR2->swevt_bit.c1swtr = TRUE; /* EGR_CC1G equivalent -- Stage E24-validated software capture */
  ccr = TMR2->c1dt;
  __enable_irq();
  return ccr;
}

int main(void)
{
  clock_config_96mhz();

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  timestamp_timer_config();
  tmr2_test_init();

  /* --- Part 1: free-running capture accuracy, no UG in between --- */
  for (uint32_t i = 0; i < NUM_TRIALS; i++) {
    stage_e28_capture_trial_t *r = &stage_e28_capture_results[i];
    r->delay_ms_requested = trial_delay_ms[i];
    r->cnt_before = TMR2->cval;
    r->ccr1_before = tmr2_capture();

    delay_us(trial_delay_ms[i] * 1000u);

    r->cnt_after = TMR2->cval;
    r->ccr1_after = tmr2_capture();

    r->elapsed_ticks = r->ccr1_after - r->ccr1_before;
    r->expected_ticks = trial_delay_ms[i] * TICKS_PER_MS;
    r->diff_ticks = (int32_t)(r->elapsed_ticks - r->expected_ticks);

    delay_us(1000u);
  }

  /* --- Part 2: UG reset semantics, one fresh UG per trial --- */
  for (uint32_t i = 0; i < NUM_TRIALS; i++) {
    stage_e28_ug_trial_t *r = &stage_e28_ug_results[i];
    r->delay_ms_requested = trial_delay_ms[i];

    __disable_irq();
    tmr_event_sw_trigger(TMR2, TMR_OVERFLOW_SWTRIG); /* UG -- matches upstream iftim_isr()'s accept-path reset */
    r->cnt_immediately_after_ug = TMR2->cval;
    __enable_irq();

    delay_us(trial_delay_ms[i] * 1000u);

    r->ccr1_captured = tmr2_capture();
    r->expected_ticks = trial_delay_ms[i] * TICKS_PER_MS;
    r->diff_ticks = (int32_t)(r->ccr1_captured - r->expected_ticks);

    delay_us(1000u);
  }

  /* --- Part 3: output-compare scheduling, target relative to a fresh UG --- */
  for (uint32_t i = 0; i < NUM_TRIALS; i++) {
    stage_e28_oc_trial_t *r = &stage_e28_oc_results[i];
    r->target_ticks = trial_delay_ms[i] * TICKS_PER_MS;

    __disable_irq();
    tmr_event_sw_trigger(TMR2, TMR_OVERFLOW_SWTRIG); /* UG */
    TMR2->c3dt = r->target_ticks;
    tmr_flag_clear(TMR2, TMR_C3_FLAG);
    __enable_irq();

    uint32_t poll_start = TMR7->cval;
    uint32_t timeout_us = trial_delay_ms[i] * 1000u + 5000u; /* generous margin */
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

  stage_e28_done = 1;

  for (;;) {
    ++stage_e28_heartbeat;
  }
}
