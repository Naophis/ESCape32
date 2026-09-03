/*
 * ESCape32 AT32F425 port -- Stage E16: FIRST closed-loop BEMF-driven
 * commutation handover. Everything upstream of this file is now
 * trusted on real hardware: TMR1 CH1-3 6-PWM (MUX_2 fix confirmed 6/6
 * correct regardless of configuration order, Stage E15D), CH4
 * PWM_MODE_B ADC trigger at ~1.2us (Stage E10), 15k/3k divider,
 * diff=floating_adc-positive_adc/2 threshold, the UNARMED->ARMED->
 * CONFIRM Schmitt-trigger ZC state machine, and the TMR2 16-bit-wrap-
 * safe delay measurement (all Stage E15, re-validated on the MUX-fixed
 * board: 9/9 buckets from 15ms down to 3ms/step came back with ZC
 * confirmed on essentially every sector, ADC saturation=0, dma_error=0,
 * and a clean, speed-independent expected_dir[6] = {RISING, FALLING,
 * RISING, FALLING, RISING, FALLING} for steps 0-5, now adopted as
 * final). NONE of that is changed in this file.
 *
 * WHAT THIS FILE ADDS: alignment -> open-loop ramp (15,13,11,9,7ms,
 * 40 steps each, identical mechanism to Stage E2/E6/E14/E15) -> hold
 * at 6ms/step (the fastest speed Stage E15 validated without stall;
 * open-loop forced acceleration to 3ms/step was shown by the user to
 * desync the real motor, so 6ms is this file's target, not 3ms) ->
 * once 6 CONSECUTIVE sectors at that hold rate confirm ZC in the
 * expected_dir[] direction, HANDOVER to closed-loop commutation.
 *
 * Closed-loop mechanism: on every direction-correct confirmed ZC, the
 * already-computed zc_delay_us (time from the last commutation edge to
 * this zero-cross, i.e. the ~30 electrical-degree interval) is reused
 * as-is for the delay from THIS zero-cross to the NEXT commutation --
 * the classic symmetric "30 degrees after ZC" scheme, deliberately not
 * doing anything more sophisticated (no filtering/prediction) for this
 * first bring-up. That delay is converted to TMR3 ticks (10us/tick,
 * same TMR3 unit Stage E15's ramp already uses) and TMR3 is reprogrammed
 * as a ONE-SHOT: paused, counter reset to 0, pr set to the new delay,
 * and restarted -- so the SAME TMR3-overflow-driven apply_step()
 * advance mechanism Stage E2 onward has always used now fires at a
 * ZC-derived instant instead of a fixed ramp period. No new commutation
 * mechanism was introduced -- only what reprograms TMR3's period, and
 * when.
 *
 * TMR3's role while MODE_CLOSED_LOOP is ALWAYS one of exactly two,
 * tracked by an explicit tmr3_purpose_t state (see its declaration
 * below) so a ZC-wait timeout can never masquerade as -- or silently
 * trigger -- a normal commutation:
 *   - TMR3_PURPOSE_TIMEOUT: armed immediately after every apply_step()
 *     (arm_timeout_watchdog()), waiting for this sector's ZC. An
 *     overflow in this state is PURELY a timeout (zc_timeout_count++)
 *     -- it does not call apply_step() and does not advance the
 *     sector; the watchdog is simply re-armed (or the motor stopped,
 *     see below) and commutation stays where it is until a real ZC
 *     arrives.
 *   - TMR3_PURPOSE_COMMUTATION: armed ONLY by a direction-correct
 *     confirmed ZC (schedule_next_commutation()), for exactly the
 *     measured 30-degree-equivalent delay. Its overflow IS the real
 *     scheduled commutation -- apply_step() runs, then TMR3 is
 *     immediately re-armed back to TMR3_PURPOSE_TIMEOUT for the new
 *     sector.
 *
 * Fault handling (kept deliberately simple, per instruction -- no
 * acceleration control or auto-restart in this first version):
 *   - A confirmed ZC in the WRONG direction (mismatch vs expected_dir)
 *     is counted (polarity_error_count) but does NOT reprogram TMR3 --
 *     the running timeout watchdog is left untouched, so a suspect
 *     reading is never acted on.
 *   - A timeout watchdog overflow (no confirmed ZC at all this sector)
 *     is counted (zc_timeout_count) and the watchdog is simply
 *     re-armed for another window -- see TMR3_PURPOSE_TIMEOUT above;
 *     it never drives a commutation by itself.
 *   - Either fault increments a shared consecutive-fault counter,
 *     reset to 0 on the next direction-correct confirm. After
 *     FAULT_STOP_THRESHOLD consecutive faults, the motor is stopped
 *     (all six gates forced low) immediately -- no automatic restart,
 *     per instruction.
 *   - The open-loop hold phase (before handover) also has a bound,
 *     MAX_STEPS_AT_TARGET: if 6 consecutive correct sectors are never
 *     achieved within that many steps at 6ms/step, the run stops
 *     safely with handover_success left at 0.
 *
 * Diagnostics (global, not per-bucket -- this file measures ONE run):
 * handover_success, handover_step_index, closed_loop_step_count,
 * zc_count, zc_timeout_count, polarity_error_count, sector_period_us
 * min/max/sum (closed-loop only), scheduled_delay_us min/max/sum
 * (closed-loop, direction-correct events only), final_step.
 */

#include "clock_config.h"

#define PHASE_A_CHANNEL ADC_CHANNEL_0 /* PA0 */
#define PHASE_B_CHANNEL ADC_CHANNEL_4 /* PA4 */
#define PHASE_C_CHANNEL ADC_CHANNEL_5 /* PA5 */

#define PWM_ARR (96000000u / 24000u - 1u) /* 24kHz, unchanged */
#define DEAD_TIME_COUNT 53u /* unverified placeholder, see porting-plan Section 18-C/H */

#define OPEN_LOOP_DUTY_PERCENT 15u /* fixed for this whole run -- no closed-loop accel control yet */
#define ALIGN_DURATION_MS 300u
#define TIM1_TICKS_PER_US 96u
#define ADC_TRIGGER_OFFSET_TICKS 115u /* ~1.2us -- unchanged from Stage E10/E14/E15 */

#define COMMUTATION_BLANK_SCANS 2u
#define ZC_CONFIRM_COUNT 3
#define ZC_DEADBAND 8

#define TMR3_TICK_US 10u
/* Open-loop ramp: 15,13,11,9,7 ms/step, 40 steps each -- identical
 * mechanism/values to Stage E2/E6/E14/E15's ramp floor. */
static const uint32_t ramp_period_ticks[] = {1500, 1300, 1100, 900, 700};
#define NUM_RAMP_BUCKETS (sizeof(ramp_period_ticks) / sizeof(ramp_period_ticks[0]))
#define STEPS_PER_RAMP_BUCKET 40u

#define HOLD_PERIOD_TICKS 600u /* 6ms/step -- fastest step Stage E15 validated without real-motor stall */
#define MAX_STEPS_AT_TARGET 200u /* safety cap while waiting for handover at the hold rate */
#define HANDOVER_CONSECUTIVE_REQUIRED 6u

#define FAULT_STOP_THRESHOLD 3u /* consecutive zc_timeout/polarity_error events before all-low stop */

typedef enum { PH_A = 0, PH_B = 1, PH_C = 2 } phase_t;

typedef struct {
  phase_t pos, neg, floating;
} step_t;

/* Unchanged drive pattern, Stage E2 onward. */
static const step_t steps[6] = {
  {PH_A, PH_B, PH_C},
  {PH_A, PH_C, PH_B},
  {PH_B, PH_C, PH_A},
  {PH_B, PH_A, PH_C},
  {PH_C, PH_A, PH_B},
  {PH_C, PH_B, PH_A},
};

/* Adopted per Stage E15's MUX-fixed, 9-bucket (15-3ms) confirmation:
 * speed-independent, RISING/FALLING alternating exactly as listed.
 * 1=RISING, 2=FALLING (matches zc_filter_update()'s return convention). */
static const int expected_dir[6] = {1, 2, 1, 2, 1, 2};

static const tmr_channel_select_type phase_channel[3] = {
  TMR_SELECT_CHANNEL_1, TMR_SELECT_CHANNEL_2, TMR_SELECT_CHANNEL_3
};

static volatile uint16_t adc_buf[3];

typedef enum { MODE_RAMP = 0, MODE_HOLD = 1, MODE_CLOSED_LOOP = 2, MODE_STOPPED = 3 } run_mode_t;

/*
 * While MODE_CLOSED_LOOP, TMR3 always has exactly one of these two
 * purposes armed -- never both, never ambiguous:
 *   TMR3_PURPOSE_TIMEOUT: armed immediately after every apply_step(),
 *     waiting for the NEXT sector's ZC. If TMR3 overflows in this
 *     state, no ZC arrived in time -- that is purely a timeout event
 *     (zc_timeout_count++); it NEVER calls apply_step()/advances
 *     commutation by itself.
 *   TMR3_PURPOSE_COMMUTATION: armed only by a direction-correct
 *     confirmed ZC (schedule_next_commutation()), for exactly the
 *     measured 30-degree-equivalent delay. If TMR3 overflows in this
 *     state, that IS the scheduled commutation -- apply_step() runs,
 *     then TMR3 is immediately re-armed back to TMR3_PURPOSE_TIMEOUT
 *     for the new sector.
 * This is the only thing that changed from the first E16 draft: a
 * timeout no longer silently drives an ordinary commutation on stale
 * timing -- it is counted and re-armed as a timeout, nothing else.
 */
typedef enum { TMR3_PURPOSE_TIMEOUT = 0, TMR3_PURPOSE_COMMUTATION = 1 } tmr3_purpose_t;
volatile tmr3_purpose_t stage_e16_tmr3_purpose;

volatile run_mode_t stage_e16_mode;
volatile uint32_t stage_e16_step_index;
volatile uint32_t stage_e16_step_count;
volatile uint32_t stage_e16_consecutive_correct; /* live, during MODE_HOLD, toward handover */
volatile uint32_t stage_e16_at_target_step_count;

volatile int stage_e16_floating_phase, stage_e16_positive_phase, stage_e16_negative_phase;
volatile uint16_t stage_e16_adc_a, stage_e16_adc_b, stage_e16_adc_c;
volatile int32_t stage_e16_floating_diff;
volatile uint32_t stage_e16_step_start_us;
volatile uint32_t stage_e16_current_step_period_us;

/* --- diagnostics requested for this stage --- */
volatile int stage_e16_handover_success;
volatile uint32_t stage_e16_handover_step_index;
volatile uint32_t stage_e16_closed_loop_step_count;
volatile uint32_t stage_e16_zc_count;
volatile uint32_t stage_e16_zc_timeout_count;
volatile uint32_t stage_e16_polarity_error_count;
volatile uint32_t stage_e16_sector_period_us_min, stage_e16_sector_period_us_max, stage_e16_sector_period_us_sum;
volatile uint32_t stage_e16_scheduled_delay_us_min, stage_e16_scheduled_delay_us_max, stage_e16_scheduled_delay_us_sum;
volatile uint32_t stage_e16_final_step;

volatile uint32_t stage_e16_heartbeat;
volatile int stage_e16_running;
volatile int stage_e16_aligning;
volatile uint32_t stage_e16_dma_error;
volatile uint32_t stage_e16_dma_fdt_count;
volatile uint32_t stage_e16_adc_conversion_count;
volatile uint32_t stage_e16_tmr1_ch4_event_count;

static uint32_t consecutive_fault_count;
static uint32_t last_scheduled_delay_us; /* most recent good ZC-measured delay, seeds the timeout window */

#define ZC_TIMEOUT_MULTIPLIER 3u /* watchdog window = this many times the last known good delay */

void _init(void) {}
void _fini(void) {}

typedef struct {
  int confirmed_sign;
  int confirm_run;
} zc_filter_t;

typedef enum { ZC_UNARMED = 0, ZC_ARMED_RISING = 1, ZC_ARMED_FALLING = 2 } zc_arm_state_t;

static volatile zc_filter_t zc_active;
static volatile zc_arm_state_t zc_arm_state;
static volatile int zc_locked;
static volatile uint32_t blank_scans_remaining;

/* Identical to Stage E15's zc_filter_update() -- not touched. */
static int zc_filter_update(volatile zc_filter_t *f, int32_t diff)
{
  int qualifies, new_sign;

  if (f->confirmed_sign <= 0) {
    new_sign = 1;
    qualifies = (diff >= ZC_DEADBAND);
  } else {
    new_sign = -1;
    qualifies = (diff <= -ZC_DEADBAND);
  }

  if (qualifies) {
    if (f->confirm_run < ZC_CONFIRM_COUNT) f->confirm_run++;
  } else {
    f->confirm_run = 0;
  }

  if (f->confirm_run >= ZC_CONFIRM_COUNT) {
    f->confirm_run = 0;
    int prev = f->confirmed_sign;
    f->confirmed_sign = new_sign;
    if (prev != new_sign) return new_sign > 0 ? 1 : 2;
  }
  return 0;
}

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

static void tim1_pins_to_af(void)
{
  gpio_init_type g;

  gpio_default_para_init(&g);
  g.gpio_mode = GPIO_MODE_MUX;
  g.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  g.gpio_pull = GPIO_PULL_NONE;
  g.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;

  g.gpio_pins = GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10;
  gpio_init(GPIOA, &g);
  g.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1;
  gpio_init(GPIOB, &g);

  /* MUX_2 for all six pins -- Stage E15D confirmed fix. */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE8, GPIO_MUX_2);  /* CH1  */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_2);  /* CH1C */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_2);  /* CH2  */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE0, GPIO_MUX_2);  /* CH2C */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_2); /* CH3  */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE1, GPIO_MUX_2);  /* CH3C */
}

static void tim1_init(void)
{
  tmr_brkdt_config_type brkdt;

  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

  tmr_base_init(TMR1, (uint16_t)PWM_ARR, 0);
  tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);

  tmr_brkdt_default_para_init(&brkdt);
  brkdt.brk_enable = FALSE;
  brkdt.auto_output_enable = TRUE;
  brkdt.deadtime = DEAD_TIME_COUNT;
  brkdt.fcsodis_state = TRUE;
  brkdt.fcsoen_state = TRUE;
  brkdt.brk_polarity = TMR_BRK_INPUT_ACTIVE_HIGH;
  brkdt.wp_level = TMR_WP_OFF;
  tmr_brkdt_config(TMR1, &brkdt);

  tmr_channel_buffer_enable(TMR1, TRUE);
}

static uint16_t g_duty_ccr;

/* Verbatim from Stage E10/E14/E15 -- CH4 trigger mechanism untouched. */
static void tim1_adc_trigger_config(void)
{
  tmr_output_config_type oc;

  tmr_output_default_para_init(&oc);
  oc.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
  oc.oc_output_state = TRUE;
  oc.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  oc.oc_idle_state = FALSE;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &oc);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, ADC_TRIGGER_OFFSET_TICKS);
  tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_4, TRUE);
  tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG | TMR_HALL_SWTRIG);

  nvic_irq_enable(TMR1_CH_IRQn, 2, 0);
  tmr_interrupt_enable(TMR1, TMR_C4_INT, TRUE);
}

void TMR1_CH_IRQHandler(void)
{
  if (tmr_flag_get(TMR1, TMR_C4_FLAG) != RESET) {
    tmr_flag_clear(TMR1, TMR_C4_FLAG);
    stage_e16_tmr1_ch4_event_count++;
  }
}

/* Identical to Stage E15's apply_step() (drive pattern unchanged),
 * plus resetting the per-sector timeout-tracking flags. */
static void apply_step(int idx, uint16_t duty_ccr)
{
  const step_t *s = &steps[idx];
  tmr_output_config_type oc;

  for (int p = 0; p < 3; p++) {
    tmr_output_default_para_init(&oc);
    oc.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    oc.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;

    if (p == s->pos) {
      oc.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
      oc.oc_output_state = TRUE;
      oc.oc_idle_state = FALSE;
      oc.occ_output_state = TRUE;
      oc.occ_idle_state = FALSE;
      tmr_output_channel_config(TMR1, phase_channel[p], &oc);
      tmr_channel_value_set(TMR1, phase_channel[p], duty_ccr);
    } else if (p == s->neg) {
      oc.oc_mode = TMR_OUTPUT_CONTROL_FORCE_LOW;
      oc.oc_output_state = TRUE;
      oc.oc_idle_state = FALSE;
      oc.occ_output_state = TRUE;
      oc.occ_idle_state = FALSE;
      tmr_output_channel_config(TMR1, phase_channel[p], &oc);
    } else {
      oc.oc_mode = TMR_OUTPUT_CONTROL_FORCE_LOW;
      oc.oc_output_state = FALSE;
      oc.oc_idle_state = FALSE;
      oc.occ_output_state = FALSE;
      oc.occ_idle_state = FALSE;
      tmr_output_channel_config(TMR1, phase_channel[p], &oc);
    }
  }

  tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG | TMR_HALL_SWTRIG);

  stage_e16_step_index = (uint32_t)idx;
  stage_e16_floating_phase = (int)s->floating;
  stage_e16_positive_phase = (int)s->pos;
  stage_e16_negative_phase = (int)s->neg;

  zc_arm_state = ZC_UNARMED;
  zc_locked = 0;
  blank_scans_remaining = COMMUTATION_BLANK_SCANS;
  stage_e16_step_start_us = TMR2->cval;
}

static void stop_and_force_off(void)
{
  tmr_output_enable(TMR1, FALSE);
  tmr_counter_enable(TMR1, FALSE);
  tmr_counter_enable(TMR3, FALSE);
  gate_pins_force_off();
  stage_e16_running = 0;
  stage_e16_mode = MODE_STOPPED;
  stage_e16_final_step = stage_e16_step_index;
}

static void tmr3_arm_oneshot(uint32_t us)
{
  uint32_t ticks = us / TMR3_TICK_US;
  if (ticks < 1u) ticks = 1u;

  tmr_counter_enable(TMR3, FALSE);
  tmr_counter_value_set(TMR3, 0);
  TMR3->pr = ticks - 1u;
  tmr_event_sw_trigger(TMR3, TMR_OVERFLOW_SWTRIG);
  tmr_counter_enable(TMR3, TRUE);

  stage_e16_current_step_period_us = ticks * TMR3_TICK_US;
}

/*
 * Armed ONLY by a direction-correct confirmed ZC, for exactly the
 * measured delay -- the next TMR3 overflow IS the real, scheduled
 * commutation (see tmr3_purpose_t comment above).
 */
static void schedule_next_commutation(uint32_t delay_us)
{
  tmr3_arm_oneshot(delay_us);
  stage_e16_tmr3_purpose = TMR3_PURPOSE_COMMUTATION;
  last_scheduled_delay_us = delay_us;
}

/*
 * Armed right after every apply_step() while MODE_CLOSED_LOOP, as a
 * pure watchdog for "no ZC arrived in time" -- its overflow NEVER
 * calls apply_step() (see tmr3_purpose_t comment above). Window is a
 * generous multiple of the last known-good delay so normal jitter
 * doesn't false-trigger it.
 */
static void arm_timeout_watchdog(void)
{
  uint32_t base_us = last_scheduled_delay_us ? last_scheduled_delay_us
                                              : (HOLD_PERIOD_TICKS * TMR3_TICK_US);
  tmr3_arm_oneshot(base_us * ZC_TIMEOUT_MULTIPLIER);
  stage_e16_tmr3_purpose = TMR3_PURPOSE_TIMEOUT;
}

static void check_fault_stop(void)
{
  if (consecutive_fault_count >= FAULT_STOP_THRESHOLD) {
    stop_and_force_off();
  }
}

static uint32_t bucket_step_count;

static void start_ramp_bucket(uint32_t idx)
{
  uint32_t period_ticks = ramp_period_ticks[idx];
  stage_e16_current_step_period_us = period_ticks * TMR3_TICK_US;
  bucket_step_count = 0;
  TMR3->pr = period_ticks - 1u;
}

static void enter_hold(void)
{
  stage_e16_mode = MODE_HOLD;
  stage_e16_current_step_period_us = HOLD_PERIOD_TICKS * TMR3_TICK_US;
  stage_e16_at_target_step_count = 0;
  stage_e16_consecutive_correct = 0;
  TMR3->pr = HOLD_PERIOD_TICKS - 1u;
}

static uint32_t ramp_bucket_index;

void TMR3_GLOBAL_IRQHandler(void)
{
  if (tmr_flag_get(TMR3, TMR_OVF_FLAG) == RESET) return;
  tmr_flag_clear(TMR3, TMR_OVF_FLAG);

  if (!stage_e16_running) return;

  if (stage_e16_mode == MODE_RAMP) {
    int next = (int)((stage_e16_step_index + 1u) % 6u);
    apply_step(next, g_duty_ccr);
    stage_e16_step_count++;
    bucket_step_count++;

    if (bucket_step_count >= STEPS_PER_RAMP_BUCKET) {
      ramp_bucket_index++;
      if (ramp_bucket_index >= NUM_RAMP_BUCKETS) {
        enter_hold();
      } else {
        start_ramp_bucket(ramp_bucket_index);
      }
    }
    return;
  }

  if (stage_e16_mode == MODE_HOLD) {
    int next = (int)((stage_e16_step_index + 1u) % 6u);
    apply_step(next, g_duty_ccr);
    stage_e16_step_count++;
    stage_e16_at_target_step_count++;

    if (stage_e16_at_target_step_count >= MAX_STEPS_AT_TARGET) {
      /* Handover never achieved within the safety cap -- stop, leave
       * handover_success at 0. */
      stop_and_force_off();
    }
    return;
  }

  if (stage_e16_mode == MODE_CLOSED_LOOP) {
    if (stage_e16_tmr3_purpose == TMR3_PURPOSE_TIMEOUT) {
      /*
       * Pure watchdog firing: no ZC arrived in time. NOT a
       * commutation -- apply_step() is NOT called, step_index/
       * step_count do not advance. Count the fault and keep waiting
       * (re-arm another timeout window) unless the fault threshold
       * stops the motor.
       */
      stage_e16_zc_timeout_count++;
      consecutive_fault_count++;
      check_fault_stop();
      if (stage_e16_running) arm_timeout_watchdog();
      return;
    }

    /* TMR3_PURPOSE_COMMUTATION: this overflow IS the ZC-scheduled
     * commutation. */
    uint32_t prev_step_start = stage_e16_step_start_us;
    int next = (int)((stage_e16_step_index + 1u) % 6u);

    apply_step(next, g_duty_ccr);
    stage_e16_step_count++;
    stage_e16_closed_loop_step_count++;

    uint32_t now_us = stage_e16_step_start_us; /* just set by apply_step() */
    uint32_t period_us = (uint32_t)(uint16_t)(now_us - prev_step_start);
    if (period_us < stage_e16_sector_period_us_min) stage_e16_sector_period_us_min = period_us;
    if (period_us > stage_e16_sector_period_us_max) stage_e16_sector_period_us_max = period_us;
    stage_e16_sector_period_us_sum += period_us;

    /* Immediately go back to waiting for THIS new sector's ZC -- TMR3
     * must not carry over its commutation purpose. */
    arm_timeout_watchdog();
    return;
  }
}

static void step_timer_init(uint32_t first_period_ticks)
{
  crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
  nvic_irq_enable(TMR3_GLOBAL_IRQn, 1, 0);

  tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
  tmr_base_init(TMR3, first_period_ticks - 1u, 960u - 1u); /* 10us tick, unchanged from Stage E15 */
  tmr_interrupt_enable(TMR3, TMR_OVF_INT, TRUE);
}

static void adc_gpio_config(void)
{
  gpio_init_type g;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&g);
  g.gpio_mode = GPIO_MODE_ANALOG;
  g.gpio_pins = GPIO_PINS_0 | GPIO_PINS_4 | GPIO_PINS_5;
  gpio_init(GPIOA, &g);
}

static void dma_config(void)
{
  dma_init_type d;

  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
  nvic_irq_enable(DMA1_Channel1_IRQn, 0, 0);
  dma_reset(DMA1_CHANNEL1);

  dma_flexible_config(DMA1, FLEX_CHANNEL1, DMA_FLEXIBLE_ADC1);

  dma_default_para_init(&d);
  d.buffer_size = 3;
  d.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  d.memory_base_addr = (uint32_t)adc_buf;
  d.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  d.memory_inc_enable = TRUE;
  d.peripheral_base_addr = (uint32_t)&(ADC1->odt);
  d.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  d.peripheral_inc_enable = FALSE;
  d.priority = DMA_PRIORITY_HIGH;
  d.loop_mode_enable = TRUE;
  dma_init(DMA1_CHANNEL1, &d);

  dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_DTERR_INT, TRUE);
}

static void adc_config(void)
{
  adc_base_config_type b;

  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
  adc_reset(ADC1);
  crm_adc_clock_div_set(CRM_ADC_DIV_4);

  adc_base_default_para_init(&b);
  b.sequence_mode = TRUE;
  b.repeat_mode = FALSE;
  b.data_align = ADC_RIGHT_ALIGNMENT;
  b.ordinary_channel_length = 3;
  adc_base_config(ADC1, &b);

  adc_ordinary_channel_set(ADC1, PHASE_A_CHANNEL, 1, ADC_SAMPLETIME_13_5);
  adc_ordinary_channel_set(ADC1, PHASE_B_CHANNEL, 2, ADC_SAMPLETIME_13_5);
  adc_ordinary_channel_set(ADC1, PHASE_C_CHANNEL, 3, ADC_SAMPLETIME_13_5);

  adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_TMR1CH4, TRUE);
  adc_dma_mode_enable(ADC1, TRUE);

  nvic_irq_enable(ADC1_IRQn, 2, 0);
  adc_interrupt_enable(ADC1, ADC_CCE_INT, TRUE);

  adc_enable(ADC1, TRUE);

  adc_calibration_init(ADC1);
  while (adc_calibration_init_status_get(ADC1));
  adc_calibration_start(ADC1);
  while (adc_calibration_status_get(ADC1));
}

void ADC1_IRQHandler(void)
{
  if (adc_interrupt_flag_get(ADC1, ADC_CCE_FLAG) != RESET) {
    adc_flag_clear(ADC1, ADC_CCE_FLAG);
    stage_e16_adc_conversion_count++;
  }
}

void DMA1_Channel1_IRQHandler(void)
{
  if (dma_interrupt_flag_get(DMA1_FDT1_FLAG) != RESET) {
    dma_flag_clear(DMA1_FDT1_FLAG);
    stage_e16_dma_fdt_count++;

    uint16_t a = adc_buf[0], b = adc_buf[1], c = adc_buf[2];
    uint16_t vals[3];
    vals[0] = a; vals[1] = b; vals[2] = c;

    stage_e16_adc_a = a;
    stage_e16_adc_b = b;
    stage_e16_adc_c = c;

    if (stage_e16_running && !stage_e16_aligning) {
      uint16_t v = vals[stage_e16_floating_phase];
      uint16_t pos_v = vals[stage_e16_positive_phase];

      int32_t diff = (int32_t)v - (int32_t)pos_v / 2;
      stage_e16_floating_diff = diff;

      if (blank_scans_remaining) {
        blank_scans_remaining--;
      } else if (!zc_locked) {
        int confirmed_dir = 0;

        if (zc_arm_state == ZC_UNARMED) {
          if (diff <= -ZC_DEADBAND) {
            zc_arm_state = ZC_ARMED_RISING;
            zc_active.confirmed_sign = -1;
            zc_active.confirm_run = 0;
          } else if (diff >= ZC_DEADBAND) {
            zc_arm_state = ZC_ARMED_FALLING;
            zc_active.confirmed_sign = 1;
            zc_active.confirm_run = 0;
          }
        } else {
          int r = zc_filter_update(&zc_active, diff);
          if (zc_arm_state == ZC_ARMED_RISING && r == 1) confirmed_dir = 1;
          else if (zc_arm_state == ZC_ARMED_FALLING && r == 2) confirmed_dir = 2;
        }

        if (confirmed_dir) {
          zc_locked = 1;

          uint32_t now_us = TMR2->cval;
          uint32_t delay_us = (uint32_t)(uint16_t)(now_us - stage_e16_step_start_us);
          int matched = (confirmed_dir == expected_dir[stage_e16_step_index]);

          if (stage_e16_mode == MODE_RAMP || stage_e16_mode == MODE_HOLD) {
            if (matched) {
              stage_e16_consecutive_correct++;
            } else {
              stage_e16_consecutive_correct = 0;
            }

            if (stage_e16_mode == MODE_HOLD &&
                stage_e16_consecutive_correct >= HANDOVER_CONSECUTIVE_REQUIRED) {
              /* HANDOVER -- schedule_next_commutation() below arms
               * TMR3_PURPOSE_COMMUTATION for the first closed-loop
               * step; TMR3_GLOBAL_IRQHandler re-arms the timeout
               * watchdog immediately after that step applies. */
              stage_e16_mode = MODE_CLOSED_LOOP;
              stage_e16_handover_success = 1;
              stage_e16_handover_step_index = stage_e16_step_index;
              consecutive_fault_count = 0;

              stage_e16_sector_period_us_min = 0xffffffffu;
              stage_e16_sector_period_us_max = 0;
              stage_e16_sector_period_us_sum = 0;
              stage_e16_scheduled_delay_us_min = 0xffffffffu;
              stage_e16_scheduled_delay_us_max = 0;
              stage_e16_scheduled_delay_us_sum = 0;

              schedule_next_commutation(delay_us);
              stage_e16_scheduled_delay_us_min = delay_us;
              stage_e16_scheduled_delay_us_max = delay_us;
              stage_e16_scheduled_delay_us_sum = delay_us;
            }
          } else if (stage_e16_mode == MODE_CLOSED_LOOP) {
            stage_e16_zc_count++;

            if (matched) {
              consecutive_fault_count = 0;

              if (delay_us < stage_e16_scheduled_delay_us_min) stage_e16_scheduled_delay_us_min = delay_us;
              if (delay_us > stage_e16_scheduled_delay_us_max) stage_e16_scheduled_delay_us_max = delay_us;
              stage_e16_scheduled_delay_us_sum += delay_us;

              /* Supersede the timeout watchdog: this real ZC arms the
               * actual scheduled commutation instead. */
              schedule_next_commutation(delay_us);
            } else {
              stage_e16_polarity_error_count++;
              consecutive_fault_count++;
              check_fault_stop();
              /* leave the running timeout watchdog untouched -- we do
               * not act on a suspect (wrong-direction) reading */
            }
          }
        }
      }
    }
  }
  if (dma_interrupt_flag_get(DMA1_DTERR1_FLAG) != RESET) {
    dma_flag_clear(DMA1_DTERR1_FLAG);
    stage_e16_dma_error++;
  }
}

/*
 * TMR2 wraps at 16 bits (see zc_delay_us's own mod-2^16 handling in
 * DMA1_Channel1_IRQHandler). A single mod-2^16 comparison is only
 * valid for waits under 65536us -- ALIGN_DURATION_MS*1000 = 300000us
 * exceeds that, so a bare (uint16_t)(now-start) < us busy-wait can
 * never terminate (the wrapped difference maxes out at 65535, which
 * is always < 300000). Fixed by splitting any wait into <=60000us
 * chunks, each safely inside one 16-bit window.
 */
static void delay_us(uint32_t us)
{
  while (us) {
    uint16_t chunk = (us > 60000u) ? 60000u : (uint16_t)us;
    uint16_t start = (uint16_t)TMR2->cval;
    while ((uint16_t)((uint16_t)TMR2->cval - start) < chunk);
    us -= chunk;
  }
}

static void timestamp_timer_config(void)
{
  crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
  tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
  tmr_base_init(TMR2, 0xFFFFFFFFu, (uint32_t)(system_core_clock / 1000000u) - 1u);
  tmr_counter_enable(TMR2, TRUE);
}

int main(void)
{
  clock_config_96mhz();

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  gate_pins_force_off();

  timestamp_timer_config();
  adc_gpio_config();
  dma_config();

  tim1_init();
  g_duty_ccr = (uint16_t)((PWM_ARR + 1u) * OPEN_LOOP_DUTY_PERCENT / 100u);
  tim1_adc_trigger_config();

  adc_config();
  dma_channel_enable(DMA1_CHANNEL1, TRUE);

  tim1_pins_to_af();
  stage_e16_aligning = 1;
  stage_e16_running = 1;
  stage_e16_mode = MODE_RAMP;
  stage_e16_step_index = 0;
  apply_step(0, g_duty_ccr);
  tmr_output_enable(TMR1, TRUE);
  tmr_counter_enable(TMR1, TRUE);
  delay_us(ALIGN_DURATION_MS * 1000u);
  stage_e16_aligning = 0;

  ramp_bucket_index = 0;
  start_ramp_bucket(0);
  stage_e16_step_count = 1;
  step_timer_init(ramp_period_ticks[0]);
  tmr_counter_enable(TMR3, TRUE);

  for (;;) {
    ++stage_e16_heartbeat;
  }
}
