/*
 * ESCape32 AT32F425 port -- Stage E27: verify a 16-bit-capture +
 * software-overflow-extension mechanism produces a CORRECT 32-bit
 * "elapsed since previous ZC" value across TMR3's 32.767ms wrap
 * (ARR=65535 @ 500ns/tick, exactly matching the REAL MOUSEF425
 * target's IFTIM configuration -- upstream's own literal ARR
 * computation, `(1<<(IFTIM_XRES+16))-1`, forces this 16-bit-range
 * regardless of TMR3's underlying counter width).
 *
 * WHY: the first BENCH_TEST run (real motor, 15% duty, 1400+ ADC-ZC
 * confirms) found iftim_isr() rejecting every single confirm via
 * `t < ival>>1`, with a per-confirm diagnostic log showing `t`
 * (=IFTIM_ICR=TMR3_CCR1) values in the low hundreds of ticks while
 * `ival` never moved off its 10000-tick startup default. A full
 * TMR3-write audit (config.c, done with the motor stationary) found
 * NO code -- ours or upstream's -- resets TMR3's counter once per
 * commutation; upstream's iftim_isr() only issues TIM_EGR(IFTIM)=
 * TIM_EGR_UG on ACCEPT (never on reject), so before the first accept,
 * TMR3 free-runs and wraps on its own ARR=65535 period (32.767ms) --
 * meaning a captured `t` measures "time since the last incidental
 * 32.767ms wrap", not "time since the previous ZC candidate", exactly
 * the discrepancy suspected. Per instruction, motor spin is paused
 * until this is understood -- this file tests ONLY the TMR3 capture/
 * overflow mechanism itself, with NO ADC, NO DMA, NO TMR1/PWM, NO
 * motor involvement whatsoever, using entirely synthetic (software-
 * triggered) "ZC" events at controlled, independently-timed intervals
 * that deliberately straddle 0, 1, and 2 ARR wraps:
 *
 *   5ms, 20ms, 30ms  -- within a single 32.767ms period (no wrap)
 *   35ms, 40ms       -- straddle exactly ONE wrap
 *   70ms             -- straddles TWO wraps
 *
 * Mechanism under test (intended to generalize directly into the real
 * ADC-ZC backend's confirm path, config.c, once validated here):
 *   - TMR3 CH1 in genuine input-capture mode (CCMR1 CC1S=IN_TI1,
 *     CCER CC1E -- the same production fix already applied to
 *     config.c's init(), previously missing entirely), captured only
 *     via software (EGR_CC1G), matching Stage E24's validated
 *     equivalence to a real hardware edge.
 *   - A software overflow counter, incremented on TMR3's own UIF
 *     (i.e. every 65536-tick wrap), independent of and NOT
 *     interfering with the capture itself.
 *   - wide_capture(): atomically (IRQs off) triggers a capture, reads
 *     CCR1, and combines it with the current overflow count -- with a
 *     magnitude-based epoch-disambiguation heuristic for the case
 *     where a wrap lands exactly between the overflow counter being
 *     read and the capture being latched (see its own comment).
 *   - elapsed = wide_capture() - previous wide_capture() -- a plain
 *     32-bit subtraction, correct across any number of wraps as long
 *     as the true elapsed time doesn't exceed a 32-bit tick count
 *     (jscale: ~35.8 minutes at 500ns/tick -- far beyond any real
 *     concern here).
 *
 * This file does NOT touch upstream's iftim_isr()/ival/accept
 * condition in any way (it isn't even linked in) -- it only proves
 * whether the OVERFLOW-EXTENSION mechanism itself is correct, as the
 * necessary prerequisite before deciding how to feed a correct
 * "elapsed since previous ZC" value into the real integration.
 */

#include "clock_config.h"

#define IFTIM_TICK_NS 500u /* 2MHz @ IFTIM_XRES=0, matches config.h/the real target exactly */
#define TICKS_PER_MS (1000000u / IFTIM_TICK_NS) /* 2000 */
#define IFTIM_ARR 65535u /* matches upstream's (1<<(IFTIM_XRES+16))-1 exactly */
#define IFTIM_PERIOD_TICKS (IFTIM_ARR + 1u) /* 65536 -- one full wrap */

static const uint32_t trial_delay_ms[] = {5, 20, 30, 35, 40, 70};
#define NUM_TRIALS (sizeof(trial_delay_ms) / sizeof(trial_delay_ms[0]))

typedef struct {
  uint32_t delay_ms_requested;
  uint32_t raw_cnt_before;      /* TMR3_CNT read just before the "previous ZC" capture trigger */
  uint32_t raw_ccr_before;      /* TMR3_CCR1 latched by the "previous ZC" capture */
  uint32_t of_count_before;     /* overflow count at the "previous ZC" capture */
  uint32_t raw_cnt_after;       /* TMR3_CNT read just before the "current ZC" capture trigger */
  uint32_t raw_ccr_after;       /* TMR3_CCR1 latched by the "current ZC" capture */
  uint32_t of_count_after;      /* overflow count at the "current ZC" capture */
  uint32_t overflow_delta;      /* of_count_after - of_count_before -- how many wraps this trial straddled */
  uint32_t wide_before;         /* 32-bit extended timestamp, previous ZC */
  uint32_t wide_after;          /* 32-bit extended timestamp, current ZC */
  uint32_t elapsed_ticks;       /* wide_after - wide_before */
  uint32_t expected_ticks;      /* delay_ms_requested * TICKS_PER_MS */
  int32_t diff_ticks;           /* elapsed_ticks - expected_ticks, signed */
} stage_e27_trial_t;

stage_e27_trial_t stage_e27_results[NUM_TRIALS];
volatile uint32_t stage_e27_of_count;
volatile uint32_t stage_e27_heartbeat;
volatile int stage_e27_done;

void _init(void) {}
void _fini(void) {}

/* Independent reference clock for the requested delays -- unrelated to
 * TMR3, same validated pattern as Stage E14/E24 (1MHz/1us, 32-bit ARR). */
static void timestamp_timer_config(void)
{
  crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
  tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
  tmr_base_init(TMR2, 0xFFFFFFFFu, (uint32_t)(system_core_clock / 1000000u) - 1u);
  tmr_counter_enable(TMR2, TRUE);
}

static void delay_us(uint32_t us)
{
  uint32_t start = TMR2->cval;
  while ((uint32_t)(TMR2->cval - start) < us);
}

/* TMR3 (IFTIM in the real target): CH1 input-capture mode (production
 * fix), ARR=65535 @ 500ns/tick -- deliberately matching the real
 * MOUSEF425 target's actual configuration, NOT Stage E24's wide
 * 32-bit ARR, since the wrap behavior itself is exactly what's under
 * test here. */
static void tmr3_test_init(void)
{
  crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
  nvic_irq_enable(TMR3_GLOBAL_IRQn, 1, 0);

  tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
  tmr_base_init(TMR3, (uint16_t)IFTIM_ARR, 47u); /* 96MHz/48 = 2MHz, 500ns/tick -- (CLK_MHZ>>(XRES+1))-1 = 47 */

  TMR3->cctrl_bit.c1en = FALSE;
  TMR3->cm1_input_bit.c1c = 0x1; /* CC1S = 01, IC1 <- TI1 direct mapping (upstream's IFTIM_ICM1/2/3 convention) */
  TMR3->cctrl_bit.c1en = TRUE;

  tmr_interrupt_enable(TMR3, TMR_OVF_INT, TRUE); /* overflow-extension counter -- see TMR3_GLOBAL_IRQHandler */
  tmr_event_sw_trigger(TMR3, TMR_OVERFLOW_SWTRIG); /* commit config, established project pattern */

  tmr_counter_enable(TMR3, TRUE);
}

void TMR3_GLOBAL_IRQHandler(void)
{
  if (tmr_flag_get(TMR3, TMR_OVF_FLAG) != RESET) {
    tmr_flag_clear(TMR3, TMR_OVF_FLAG);
    stage_e27_of_count++;
  }
  /* CH1's own capture flag is never enabled as an interrupt here --
   * this test only needs the LATCHED VALUE (TMR3->c1dt), read
   * synchronously right after triggering, never via an async ISR. */
}

/* Atomically trigger a software capture and read back CCR1 + the
 * overflow count, resolving the one genuine race: an overflow whose
 * flag has been set by hardware but not yet drained into
 * stage_e27_of_count by the ISR (interrupts are off here, so the ISR
 * cannot run until this function returns). If UIF is pending AND the
 * just-captured CCR1 is small (< half the ARR range), the capture
 * landed AFTER the wrap that this pending UIF represents, so it
 * belongs to the NEXT epoch (of_count+1); if CCR1 is large, the
 * capture landed BEFORE that pending wrap and belongs to the current
 * epoch (of_count as read). */
static uint32_t wide_capture(uint32_t *raw_cnt_out, uint32_t *raw_ccr_out, uint32_t *of_count_out)
{
  uint32_t of, ccr, cnt;
  __disable_irq();
  cnt = TMR3->cval;
  of = stage_e27_of_count;
  TMR3->swevt_bit.c1swtr = TRUE; /* EGR_CC1G equivalent -- Stage E24-validated software capture */
  ccr = TMR3->c1dt;
  if (TMR3->ists_bit.ovfif && ccr < (IFTIM_ARR / 2u)) of++;
  __enable_irq();
  if (raw_cnt_out) *raw_cnt_out = cnt;
  if (raw_ccr_out) *raw_ccr_out = ccr;
  if (of_count_out) *of_count_out = of;
  return of * IFTIM_PERIOD_TICKS + ccr;
}

int main(void)
{
  clock_config_96mhz();

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  timestamp_timer_config();
  tmr3_test_init();

  for (uint32_t i = 0; i < NUM_TRIALS; i++) {
    stage_e27_trial_t *r = &stage_e27_results[i];
    r->delay_ms_requested = trial_delay_ms[i];

    r->wide_before = wide_capture(&r->raw_cnt_before, &r->raw_ccr_before, &r->of_count_before);

    delay_us(trial_delay_ms[i] * 1000u);

    r->wide_after = wide_capture(&r->raw_cnt_after, &r->raw_ccr_after, &r->of_count_after);

    r->overflow_delta = r->of_count_after - r->of_count_before;
    r->elapsed_ticks = r->wide_after - r->wide_before;
    r->expected_ticks = trial_delay_ms[i] * TICKS_PER_MS;
    r->diff_ticks = (int32_t)(r->elapsed_ticks - r->expected_ticks);

    delay_us(1000u); /* Settle between trials -- not timing-critical */
  }

  stage_e27_done = 1;

  for (;;) {
    ++stage_e27_heartbeat;
  }
}
