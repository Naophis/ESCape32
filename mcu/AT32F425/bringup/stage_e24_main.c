/*
 * ESCape32 AT32F425 port -- Stage E24: minimal, isolated verification
 * that a TMR software channel-capture event (TMR_Cx_SWTRIG via
 * tmr_event_sw_trigger()) actually latches the timer's CURRENT counter
 * value into that channel's capture register (CxDT) AND fires the
 * SAME capture interrupt path a real hardware edge would use -- NOT a
 * separate/different mechanism.
 *
 * WHY: per the upstream/AT32F425 integration plan, the ADC-based BEMF
 * zero-cross backend is meant to feed upstream's iftim_isr() (the
 * shared, unmodified commutation-timing core in src/main.c) by
 * synthesizing a software capture event on IFTIM the instant our own
 * ADC arm/confirm state machine confirms a zero-cross -- instead of
 * relying on a real hardware comparator-driven edge, which this part
 * does not have. iftim_isr() reads IFTIM's capture register directly
 * (IFTIM_ICR) assuming it holds a real capture value and that the
 * interrupt it is servicing is the genuine capture-compare interrupt.
 * This file is the prerequisite hardware confirmation, requested
 * explicitly before any integration code is written: if software
 * capture does NOT behave identically to hardware capture on this
 * part, iftim_isr() cannot be reused unmodified and a different
 * synthesis mechanism is needed.
 *
 * This file is deliberately standalone and unrelated to the BEMF/
 * commutation stack -- no ADC, no DMA, no TMR1 PWM, no motor
 * involvement at all. It uses TMR3 (already proven present and usable
 * on this exact part throughout Stage E14-E23) purely as a scratch
 * timer for this one test:
 *
 *   - TMR3 configured free-running, 1us/tick (div=95, 96MHz/96=1MHz),
 *     32-bit ARR (TMR3's pr register is confirmed 32-bit on this part,
 *     per Stage E20's cval-progression register dump).
 *   - TMR3 CH1 configured as a genuine input-capture channel
 *     (tmr_input_channel_init(), TMR_CC_CHANNEL_MAPPED_DIRECT i.e.
 *     IC1<-TI1, rising edge, no filter, divider=1) -- exactly the
 *     configuration upstream's IFTIM_ICM1/2/3 macros use on other
 *     targets. No real signal is wired to this channel's pin for this
 *     test; TI1's actual level is irrelevant because only the
 *     SOFTWARE trigger path is exercised.
 *   - CH1's capture interrupt (TMR_C1_INT) is enabled, serviced by the
 *     same TMR3_GLOBAL_IRQHandler vector this project has used
 *     throughout Stage E14-E23 for TMR3's overflow interrupt --
 *     itself confirming CH1 shares that combined vector, same as
 *     upstream's IFTIM/iftim_isr() convention on every existing
 *     backend.
 *
 * Main loop (not real-time critical -- this is a one-shot bench test,
 * not the real integration) runs NUM_TRIALS iterations, each: record
 * TMR3->cval immediately before the trigger (cnt_before), call
 * tmr_event_sw_trigger(TMR3, TMR_C1_SWTRIG), then a short settle delay,
 * then record whether the capture ISR fired (event counter delta) and
 * what it captured (CH1DT / TMR3->c1dt, read inside the ISR itself,
 * not re-read afterward) into stage_e24_results[].
 *
 * Pass criteria: for every trial, isr_fired becomes true (the ISR
 * actually ran) and ccr1_captured is close to cnt_before (within a few
 * ticks of latency, not some fixed/stale/zero value) -- i.e. the
 * software trigger behaves exactly like a real capture edge would at
 * that instant. If confirmed, upstream's iftim_isr() can be reused
 * completely unmodified in the real integration, driven by our own
 * software capture instead of a hardware comparator edge.
 */

#include "clock_config.h"

#define NUM_TRIALS 8u
#define TRIAL_SPACING_US 500u

typedef struct {
  uint32_t cnt_before;     /* TMR3->cval read immediately before the software trigger */
  uint32_t ccr1_captured;  /* TMR3->c1dt as read INSIDE the capture ISR */
  uint32_t isr_fired;      /* 0/1 -- did the capture ISR actually run for this trial */
  int32_t delta;           /* ccr1_captured - cnt_before, signed */
} stage_e24_result_t;

stage_e24_result_t stage_e24_results[NUM_TRIALS];
volatile uint32_t stage_e24_capture_event_count;
volatile uint32_t stage_e24_last_ccr1;
volatile uint32_t stage_e24_heartbeat;
volatile int stage_e24_done;

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

/*
 * TMR3: free-running, 1us/tick, CH1 as a genuine input-capture channel
 * (same shape upstream's IFTIM_ICM1/2/3 macros configure on other
 * targets), driven ONLY by software capture for this test.
 */
static void tmr3_capture_test_init(void)
{
  tmr_input_config_type ic;

  crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
  nvic_irq_enable(TMR3_GLOBAL_IRQn, 1, 0);

  tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
  tmr_base_init(TMR3, 0xFFFFFFFFu, 95u); /* 96MHz/96 = 1MHz, 1us/tick */

  tmr_input_default_para_init(&ic);
  ic.input_channel_select = TMR_SELECT_CHANNEL_1;
  ic.input_polarity_select = TMR_INPUT_RISING_EDGE;
  ic.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT; /* IC1 <- TI1, same as upstream's IFTIM convention */
  ic.input_filter_value = 0;
  tmr_input_channel_init(TMR3, &ic, TMR_CHANNEL_INPUT_DIV_1);

  tmr_interrupt_enable(TMR3, TMR_C1_INT, TRUE);
  tmr_event_sw_trigger(TMR3, TMR_OVERFLOW_SWTRIG); /* commit config, matches this project's established pattern */

  tmr_counter_enable(TMR3, TRUE);
}

void TMR3_GLOBAL_IRQHandler(void)
{
  if (tmr_flag_get(TMR3, TMR_C1_FLAG) != RESET) {
    tmr_flag_clear(TMR3, TMR_C1_FLAG);
    stage_e24_last_ccr1 = TMR3->c1dt;
    stage_e24_capture_event_count++;
  }
  if (tmr_flag_get(TMR3, TMR_OVF_FLAG) != RESET) {
    tmr_flag_clear(TMR3, TMR_OVF_FLAG);
  }
}

int main(void)
{
  clock_config_96mhz();

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  timestamp_timer_config();
  tmr3_capture_test_init();

  for (uint32_t i = 0; i < NUM_TRIALS; i++) {
    delay_us(TRIAL_SPACING_US);

    uint32_t before_count = stage_e24_capture_event_count;
    uint32_t cnt_before = TMR3->cval;

    tmr_event_sw_trigger(TMR3, TMR_C1_SWTRIG);

    delay_us(50u); /* let the capture ISR run */

    stage_e24_result_t *r = &stage_e24_results[i];
    r->cnt_before = cnt_before;
    r->isr_fired = (stage_e24_capture_event_count != before_count) ? 1u : 0u;
    r->ccr1_captured = stage_e24_last_ccr1;
    r->delta = (int32_t)(r->ccr1_captured - r->cnt_before);
  }

  stage_e24_done = 1;

  for (;;) {
    ++stage_e24_heartbeat;
  }
}
