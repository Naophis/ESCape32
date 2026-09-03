/*
 * ESCape32 AT32F425 port -- Stage E15: ZC-detection logic redesign,
 * following Stage E14's real-hardware result.
 *
 * WHY: Stage E14 (CH4 PWM_MODE_B @ ~1.2us offset, 15k/3k divider,
 * positive_adc/2 threshold) came back clean on every INFRASTRUCTURE
 * metric -- ADC saturation=0, dma_error=0, 279/280 sectors decided --
 * so the trigger/timing/divider/threshold chain fixed in Stages E9-E14
 * is NOT in question here and is kept verbatim (ADC_TRIGGER_OFFSET_TICKS
 * =115 i.e. ~1.2us, CH4 PWM_MODE_B, 15k/3k divider, positive_adc/2
 * threshold -- none of these change in this file). But the ZC
 * CLASSIFICATION data itself was physically implausible:
 *   - dir_hist was nowhere near the alternating rising/falling pattern
 *     6-step commutation requires (S0/S3/S4/S5 were 100% one direction,
 *     S1/S2 were a near-even mix -- not the clean per-sector alternation
 *     real BEMF crossing a fixed sector's floating phase should produce).
 *   - diff_min/max (~-800/+1540) stayed essentially IDENTICAL from
 *     15ms/step down to 5ms/step, where real BEMF amplitude should
 *     scale with rotor speed.
 * Both point at the same conclusion: Stage E14's zc_rise/zc_fall
 * filters were confirming on the COMMUTATION TRANSIENT (the step
 * edge's own switching disturbance decaying through the RC/winding
 * network) rather than the actual BEMF zero-crossing, because they
 * started evaluating diff immediately after only a short, fixed
 * POST_COMMUTATION_BLANK_SCANS=2 blank with no requirement that the
 * signal actually settle to a stable baseline before arming.
 *
 * This file changes ONLY the ZC classification state machine, per
 * explicit instruction -- no expected_dir table is generated here,
 * dir_hist is not used to seed anything, and none of the ADC trigger/
 * divider/threshold/6-step-drive-pattern infrastructure changes:
 *
 * (1) Commutation blanking is unchanged in spirit but named/kept
 *     explicit: COMMUTATION_BLANK_SCANS=2 non-blanked ADC scans (each
 *     scan = one 24kHz PWM cycle = ~41.7us) are discarded right after
 *     every apply_step(), before ANY classification logic runs.
 *
 * (2) NEW two-phase state machine per sector, replacing the old
 *     "race two independent filters from the first non-blanked
 *     sample" design:
 *       UNARMED  -- after blanking ends, just watch diff. The sector
 *                   only becomes ARMED once diff clears the Schmitt
 *                   deadband on ONE side: diff <= -ZC_DEADBAND arms
 *                   for a RISING crossing only (the only direction
 *                   physically possible from that baseline); diff >=
 *                   +ZC_DEADBAND arms for FALLING only. While
 *                   |diff| < ZC_DEADBAND the sector stays UNARMED
 *                   (this is what a transient's own decay through the
 *                   deadband looks like -- it no longer gets treated
 *                   as evidence of anything).
 *       ARMED    -- exactly ONE zc_filter_t (not a race between two)
 *                   is now live, seeded from the observed baseline, so
 *                   it can only confirm the crossing consistent with
 *                   that baseline -- it takes an actual transition
 *                   across the OPPOSITE threshold, held for
 *                   ZC_CONFIRM_COUNT consecutive scans, to confirm.
 *     This directly targets the transient-detection failure mode: a
 *     switching transient that merely blips past the deadband and
 *     decays back cannot arm-then-immediately-confirm in the old
 *     design's single-filter-race sense, because arming and
 *     confirming are now separated into two observably distinct
 *     phases with a real amplitude requirement on both sides.
 *
 * (3) zc_delay_us (TMR2 timestamp difference between apply_step() and
 *     the ARMED->confirmed transition) and its ratio to the sector's
 *     own period are now recorded per event. A real BEMF crossing
 *     should land roughly mid-sector regardless of speed; a
 *     transient-driven false confirm clusters near sector start
 *     instead. stage_e15_zc_delay_hist[10] bins the ratio into
 *     [0.0-0.1) .. [0.9-1.0] globally across the whole run (not
 *     per-bucket -- the ratio itself is the speed-normalized
 *     quantity), plus per-bucket zc_delay_us min/max/sum(->mean) for
 *     a direct, un-normalized sanity check.
 *
 * (4) The open-loop ramp is extended well past Stage E2/E6's floor of
 *     5ms/step: bucket_period_ms now continues 4, 3, 2, 1.5, 1.0,
 *     0.75, 0.5 (represented internally as bucket_period_ticks in
 *     10us units so the sub-millisecond entries are exact integers --
 *     TMR3's prescaler is reconfigured to a 10us tick, still a plain
 *     linear extension of Stage E2/E6's TMR3-overflow-driven
 *     commutation, nothing structural changed there). There is no
 *     automatic stall/desync detector -- per instruction, this file
 *     just runs the full extended ramp and reports per-bucket data;
 *     wherever the motor stops following will show up in the returned
 *     numbers (and/or be visible to the user directly) rather than
 *     being auto-detected and stopped early.
 *
 * steps[] pos/neg/floating assignment, gate_pins_force_off(), DMA/ADC
 * init pattern, TMR1 CH1-3 PWM config, and CH4's trigger mechanism are
 * all unchanged from Stage E14.
 */

#include "clock_config.h"

#define PHASE_A_CHANNEL ADC_CHANNEL_0 /* PA0 */
#define PHASE_B_CHANNEL ADC_CHANNEL_4 /* PA4 */
#define PHASE_C_CHANNEL ADC_CHANNEL_5 /* PA5 */

#define PWM_ARR (96000000u / 24000u - 1u) /* 24kHz, unchanged */
#define DEAD_TIME_COUNT 53u /* unverified placeholder, see porting-plan Section 18-C/H */

#define OPEN_LOOP_DUTY_PERCENT 15u /* raised from 8 -- matches Stage E7-E13's ON-time window sizing */
#define ALIGN_DURATION_MS 300u

/*
 * Extended past Stage E2/E6's 5ms floor, in 10us TICKS (not ms) so the
 * sub-millisecond entries (1.5/0.75ms etc.) are exact integers. TMR3's
 * prescaler is set for a 10us tick (div=959 -> 96MHz/960=100kHz), a
 * plain linear extension of the same TMR3-overflow-driven commutation
 * Stage E2/E6 always used -- nothing structural changed.
 */
static const uint32_t bucket_period_ticks[] = {
  1500, 1300, 1100, 900, 700, 600, 500, /* 15,13,11,9,7,6,5 ms -- unchanged floor */
  400, 300, 200, 150, 100, 75, 50       /* 4,3,2,1.5,1.0,0.75,0.5 ms -- NEW */
};
#define NUM_BUCKETS (sizeof(bucket_period_ticks) / sizeof(bucket_period_ticks[0]))
#define STEPS_PER_BUCKET 40u
#define TMR3_TICK_US 10u

#define ZC_CONFIRM_COUNT 3
#define ZC_DEADBAND 8 /* unchanged -- still the Stage D/E placeholder */

#define TIM1_TICKS_PER_US 96u
#define COMMUTATION_BLANK_SCANS 2u /* non-blanked-scan count discarded right after every apply_step(), before classification runs at all */
#define ADC_TRIGGER_OFFSET_TICKS 115u /* ~1.2us -- unchanged from Stage E14; CH4 rising edge (Mode B) = ADC trigger instant */

/* Sector ZC state machine (per-sector, reset in apply_step()) */
typedef enum { ZC_UNARMED = 0, ZC_ARMED_RISING = 1, ZC_ARMED_FALLING = 2 } zc_arm_state_t;

#define ZC_DELAY_HIST_BINS 10u

#define SAT_LOW_THRESHOLD 50u
#define SAT_HIGH_THRESHOLD 4045u

typedef enum { PH_A = 0, PH_B = 1, PH_C = 2 } phase_t;

typedef struct {
  phase_t pos, neg, floating;
} step_t;

/*
 * Same pos/neg/floating assignments as Stage E2-E6 (unchanged: the
 * open-loop drive pattern itself was never in question). No
 * expected_dir field anymore -- Stage E5/E6's expected_dir values were
 * derived on top of the broken CH4-trigger + broken-divider sensing
 * chain and are discarded outright per instruction, not reused as a
 * prior. This file records which direction (rising/falling) each
 * sector's zero-cross actually confirms as (stage_e15_dir_hist[]),
 * with NO notion of correct/wrong yet -- that table must be re-derived
 * from this run's real data, same as Stage E5 originally did.
 */
static const step_t steps[6] = {
  {PH_A, PH_B, PH_C},
  {PH_A, PH_C, PH_B},
  {PH_B, PH_C, PH_A},
  {PH_B, PH_A, PH_C},
  {PH_C, PH_A, PH_B},
  {PH_C, PH_B, PH_A},
};

static const tmr_channel_select_type phase_channel[3] = {
  TMR_SELECT_CHANNEL_1, TMR_SELECT_CHANNEL_2, TMR_SELECT_CHANNEL_3
};

static volatile uint16_t adc_buf[3];

typedef struct {
  uint32_t period_us;
  uint32_t step_count;
  uint32_t zc_rising;
  uint32_t zc_falling;
  uint32_t zc_undecided; /* step_count - zc_rising - zc_falling; sectors with no confirmed event at all */
  int32_t diff_min, diff_max;
  uint32_t sat_low_count;
  uint32_t sat_high_count;
  uint16_t adc_a, adc_b, adc_c;
  uint16_t floating_adc_last, positive_adc_last, negative_adc_last; /* role-tagged */
  uint32_t dma_error;
  /* --- new in Stage E15: when within the sector a confirmed ZC lands --- */
  uint32_t zc_delay_us_min, zc_delay_us_max;
  uint32_t zc_delay_us_sum; /* / (zc_rising+zc_falling) for mean */
} stage_e15_bucket_t;

stage_e15_bucket_t stage_e15_results[NUM_BUCKETS];
volatile uint32_t stage_e15_bucket_index;

/* Accumulated across the WHOLE run (all buckets), indexed by step_idx
 * (0-5). Still recorded as a diagnostic, same as Stage E14, but NOT
 * used to seed an expected_dir table in this file -- per instruction,
 * that derivation waits until the ZC-classification redesign itself
 * (this file) is validated against real BEMF timing (zc_delay), not
 * transient timing. */
typedef struct { uint32_t rising, falling; } stage_e15_dir_hist_entry_t;
stage_e15_dir_hist_entry_t stage_e15_dir_hist[6];

/* Ratio-of-sector-period histogram for confirmed ZC delay, accumulated
 * globally across the whole run (bin i = [i/10, (i+1)/10) of
 * zc_delay_us/period_us). A real BEMF crossing should cluster near the
 * middle bins regardless of speed; a transient-driven false confirm
 * clusters near bin 0. */
volatile uint32_t stage_e15_zc_delay_hist[ZC_DELAY_HIST_BINS];

volatile uint32_t stage_e15_heartbeat;
volatile int stage_e15_running;
volatile int stage_e15_aligning;
volatile uint32_t stage_e15_current_step_period_us;
volatile uint32_t stage_e15_step_index;
volatile uint32_t stage_e15_step_count;
volatile int stage_e15_floating_phase;
volatile int stage_e15_positive_phase;
volatile int stage_e15_negative_phase;
volatile uint16_t stage_e15_adc_a, stage_e15_adc_b, stage_e15_adc_c;
volatile uint16_t stage_e15_floating_adc, stage_e15_positive_adc, stage_e15_negative_adc; /* role-tagged live values */
volatile int32_t stage_e15_floating_diff; /* floating_adc - positive_adc/2 */
volatile int32_t stage_e15_floating_diff_min, stage_e15_floating_diff_max;
volatile uint32_t stage_e15_zc_rising_count;  /* current bucket, live */
volatile uint32_t stage_e15_zc_falling_count; /* current bucket, live */
volatile uint32_t stage_e15_sat_low_count;
volatile uint32_t stage_e15_sat_high_count;
volatile uint32_t stage_e15_dma_error;
volatile int stage_e15_zc_locked; /* set the instant a confirmed ZC is recorded for this sector */

/* --- new in Stage E15: two-phase arm/confirm state, per sector --- */
volatile zc_arm_state_t stage_e15_zc_arm_state;
volatile uint32_t stage_e15_step_start_us; /* TMR2->cval at apply_step() */
volatile uint32_t stage_e15_zc_delay_us_last; /* last confirmed event's delay, live */

volatile uint32_t stage_e15_tmr1_ch4_event_count;
volatile uint32_t stage_e15_adc_conversion_count;
volatile uint32_t stage_e15_dma_fdt_count;
volatile uint32_t stage_e15_tim1_c4dt;

void _init(void) {}
void _fini(void) {}

typedef struct {
  int confirmed_sign;
  int confirm_run;
} zc_filter_t;

/*
 * ONE filter, live only while ARMED (Stage E15's two-phase redesign --
 * see file header (2)). Unlike Stage E6-E14's always-racing pair, this
 * filter is seeded fresh the instant the sector arms, so it can only
 * ever confirm the crossing consistent with the observed baseline.
 */
static volatile zc_filter_t zc_active;
static volatile uint32_t blank_scans_remaining;

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

  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE8, GPIO_MUX_2);  /* CH1  */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_3);  /* CH1C */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_2);  /* CH2  */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE0, GPIO_MUX_1);  /* CH2C */
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

/*
 * CH4 config copied verbatim from Stage E10's fixed/confirmed
 * mechanism (PWM_MODE_B so the rising edge -- the ADC's actual
 * external-trigger event -- occurs AT CCR4, not at Stage E6's assumed-
 * but-never-actually-honored duty_ccr+blank point). CCR4 is a FIXED
 * ADC_TRIGGER_OFFSET_TICKS (~1.2us) instead of a duty-relative
 * formula, placing the trigger early inside the ON window per the
 * user's 1.0-1.5us instruction.
 */
static void tim1_adc_trigger_config(void)
{
  tmr_output_config_type oc;
  uint16_t ccr4 = ADC_TRIGGER_OFFSET_TICKS;

  tmr_output_default_para_init(&oc);
  oc.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B; /* Stage E10 fix, unchanged */
  oc.oc_output_state = TRUE; /* required for cctrl.c4en -- Stage E3's fix */
  oc.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  oc.oc_idle_state = FALSE;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &oc);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, ccr4);
  tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_4, TRUE);
  tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG | TMR_HALL_SWTRIG);

  nvic_irq_enable(TMR1_CH_IRQn, 2, 0);
  tmr_interrupt_enable(TMR1, TMR_C4_INT, TRUE);

  stage_e15_tim1_c4dt = ccr4;
}

void TMR1_CH_IRQHandler(void)
{
  if (tmr_flag_get(TMR1, TMR_C4_FLAG) != RESET) {
    tmr_flag_clear(TMR1, TMR_C4_FLAG);
    stage_e15_tmr1_ch4_event_count++;
  }
}

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

  stage_e15_step_index = (uint32_t)idx;
  stage_e15_floating_phase = (int)s->floating;
  stage_e15_positive_phase = (int)s->pos;
  stage_e15_negative_phase = (int)s->neg;

  /* Two-phase state machine reset: UNARMED until the post-blanking
   * baseline clears the deadband on one side (see DMA IRQ). No filter
   * is seeded here -- zc_active is only initialized once arming
   * actually happens. */
  stage_e15_zc_arm_state = ZC_UNARMED;
  stage_e15_zc_locked = 0;
  blank_scans_remaining = COMMUTATION_BLANK_SCANS;
  stage_e15_step_start_us = TMR2->cval;
}

static void stop_and_force_off(void)
{
  tmr_output_enable(TMR1, FALSE);
  tmr_counter_enable(TMR1, FALSE);
  tmr_counter_enable(TMR3, FALSE);
  gate_pins_force_off();
  stage_e15_running = 0;
}

static uint32_t bucket_zc_delay_us_min, bucket_zc_delay_us_max, bucket_zc_delay_us_sum;

static void reset_bucket_accumulator(void)
{
  stage_e15_zc_rising_count = 0;
  stage_e15_zc_falling_count = 0;
  stage_e15_floating_diff_min = 0x7fffffff;
  stage_e15_floating_diff_max = -0x7fffffff - 1;
  stage_e15_sat_low_count = 0;
  stage_e15_sat_high_count = 0;
  bucket_zc_delay_us_min = 0xffffffffu;
  bucket_zc_delay_us_max = 0;
  bucket_zc_delay_us_sum = 0;
}

static uint32_t bucket_step_count;

static void snapshot_bucket(uint32_t idx)
{
  stage_e15_bucket_t *r = &stage_e15_results[idx];
  r->period_us = stage_e15_current_step_period_us;
  r->step_count = bucket_step_count;
  r->zc_rising = stage_e15_zc_rising_count;
  r->zc_falling = stage_e15_zc_falling_count;
  r->zc_undecided = bucket_step_count - stage_e15_zc_rising_count - stage_e15_zc_falling_count;
  r->diff_min = stage_e15_floating_diff_min;
  r->diff_max = stage_e15_floating_diff_max;
  r->sat_low_count = stage_e15_sat_low_count;
  r->sat_high_count = stage_e15_sat_high_count;
  r->adc_a = stage_e15_adc_a;
  r->adc_b = stage_e15_adc_b;
  r->adc_c = stage_e15_adc_c;
  r->floating_adc_last = stage_e15_floating_adc;
  r->positive_adc_last = stage_e15_positive_adc;
  r->negative_adc_last = stage_e15_negative_adc;
  r->dma_error = stage_e15_dma_error;
  r->zc_delay_us_min = (bucket_zc_delay_us_min == 0xffffffffu) ? 0 : bucket_zc_delay_us_min;
  r->zc_delay_us_max = bucket_zc_delay_us_max;
  r->zc_delay_us_sum = bucket_zc_delay_us_sum;
}

static void start_bucket(uint32_t idx)
{
  uint32_t period_ticks = bucket_period_ticks[idx];
  stage_e15_current_step_period_us = period_ticks * TMR3_TICK_US;
  bucket_step_count = 0;
  reset_bucket_accumulator();
  TMR3->pr = period_ticks - 1u;
}

void TMR3_GLOBAL_IRQHandler(void)
{
  if (tmr_flag_get(TMR3, TMR_OVF_FLAG) == RESET) return;
  tmr_flag_clear(TMR3, TMR_OVF_FLAG);

  if (!stage_e15_running) return;

  int next = (int)((stage_e15_step_index + 1u) % 6u);
  apply_step(next, g_duty_ccr);
  stage_e15_step_count++;
  bucket_step_count++;

  if (bucket_step_count >= STEPS_PER_BUCKET) {
    snapshot_bucket(stage_e15_bucket_index);
    stage_e15_bucket_index++;
    if (stage_e15_bucket_index >= NUM_BUCKETS) {
      stop_and_force_off();
      return;
    }
    start_bucket(stage_e15_bucket_index);
  }
}

/*
 * Prescaler changed from Stage E2/E6's 1kHz(1ms) tick to 100kHz(10us)
 * tick (div=959 -> 96MHz/960=100kHz) so the extended ramp's sub-
 * millisecond entries (1.5/0.75/0.5ms = 150/75/50 ticks) are exact
 * integers. Still the same TMR3-overflow-driven commutation structure.
 */
static void step_timer_init(uint32_t first_period_ticks)
{
  crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
  nvic_irq_enable(TMR3_GLOBAL_IRQn, 1, 0);

  tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
  tmr_base_init(TMR3, first_period_ticks - 1u, 960u - 1u);
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
  b.repeat_mode = FALSE; /* Stage E3 fix, unchanged */
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
    stage_e15_adc_conversion_count++;
  }
}

void DMA1_Channel1_IRQHandler(void)
{
  if (dma_interrupt_flag_get(DMA1_FDT1_FLAG) != RESET) {
    dma_flag_clear(DMA1_FDT1_FLAG);
    stage_e15_dma_fdt_count++;

    uint16_t a = adc_buf[0], b = adc_buf[1], c = adc_buf[2];
    uint16_t vals[3];
    vals[0] = a; vals[1] = b; vals[2] = c;

    stage_e15_adc_a = a;
    stage_e15_adc_b = b;
    stage_e15_adc_c = c;

    if (stage_e15_running && !stage_e15_aligning) {
      uint16_t v = vals[stage_e15_floating_phase];
      uint16_t pos_v = vals[stage_e15_positive_phase];
      stage_e15_floating_adc = v;
      stage_e15_positive_adc = pos_v;
      stage_e15_negative_adc = vals[stage_e15_negative_phase];

      /*
       * THRESHOLD CHANGE: positive_adc/2 instead of the old 3-channel
       * average. positive_adc is now a consistent, non-saturated
       * ON-time reading (post-divider-fix), so half of it approximates
       * the divided-domain half-bus point BEMF crosses -- see file
       * header.
       */
      int32_t diff = (int32_t)v - (int32_t)pos_v / 2;
      stage_e15_floating_diff = diff;

      if (blank_scans_remaining) {
        blank_scans_remaining--;
      } else {
        if (v <= SAT_LOW_THRESHOLD) stage_e15_sat_low_count++;
        if (v >= SAT_HIGH_THRESHOLD) stage_e15_sat_high_count++;

        if (diff < stage_e15_floating_diff_min) stage_e15_floating_diff_min = diff;
        if (diff > stage_e15_floating_diff_max) stage_e15_floating_diff_max = diff;

        /*
         * Two-phase arm/confirm (Stage E15 redesign -- see file header
         * (2)). UNARMED: just watch diff until it clears the deadband
         * on ONE side, which both establishes the baseline and seeds
         * the single active filter for the ONLY physically-consistent
         * crossing direction. ARMED_*: zc_active can now only confirm
         * that one direction, on an actual transition across the
         * opposite threshold held for ZC_CONFIRM_COUNT scans.
         */
        if (!stage_e15_zc_locked) {
          if (stage_e15_zc_arm_state == ZC_UNARMED) {
            if (diff <= -ZC_DEADBAND) {
              stage_e15_zc_arm_state = ZC_ARMED_RISING;
              zc_active.confirmed_sign = -1; /* armed: only a rising (r==1) confirm is possible */
              zc_active.confirm_run = 0;
            } else if (diff >= ZC_DEADBAND) {
              stage_e15_zc_arm_state = ZC_ARMED_FALLING;
              zc_active.confirmed_sign = 1; /* armed: only a falling (r==2) confirm is possible */
              zc_active.confirm_run = 0;
            }
            /* else: baseline still ambiguous (inside the deadband) -- stay UNARMED */
          } else {
            int r = zc_filter_update(&zc_active, diff);
            int confirmed_dir = 0;
            if (stage_e15_zc_arm_state == ZC_ARMED_RISING && r == 1) confirmed_dir = 1;
            else if (stage_e15_zc_arm_state == ZC_ARMED_FALLING && r == 2) confirmed_dir = 2;

            if (confirmed_dir) {
              stage_e15_zc_locked = 1;

              uint32_t now_us = TMR2->cval;
              uint32_t delay_us = now_us - stage_e15_step_start_us; /* unsigned subtraction, wrap-safe */
              stage_e15_zc_delay_us_last = delay_us;
              if (delay_us < bucket_zc_delay_us_min) bucket_zc_delay_us_min = delay_us;
              if (delay_us > bucket_zc_delay_us_max) bucket_zc_delay_us_max = delay_us;
              bucket_zc_delay_us_sum += delay_us;

              uint32_t period_us = stage_e15_current_step_period_us;
              uint32_t bin = period_us ? (uint32_t)(((uint64_t)delay_us * ZC_DELAY_HIST_BINS) / period_us) : 0;
              if (bin >= ZC_DELAY_HIST_BINS) bin = ZC_DELAY_HIST_BINS - 1;
              stage_e15_zc_delay_hist[bin]++;

              if (confirmed_dir == 1) {
                stage_e15_zc_rising_count++;
                stage_e15_dir_hist[stage_e15_step_index].rising++;
              } else {
                stage_e15_zc_falling_count++;
                stage_e15_dir_hist[stage_e15_step_index].falling++;
              }
            }
          }
        }
      }
    }
  }
  if (dma_interrupt_flag_get(DMA1_DTERR1_FLAG) != RESET) {
    dma_flag_clear(DMA1_DTERR1_FLAG);
    stage_e15_dma_error++;
  }
}

static void delay_us(uint32_t us)
{
  uint32_t start = TMR2->cval;
  while ((uint32_t)(TMR2->cval - start) < us);
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
  stage_e15_aligning = 1;
  stage_e15_running = 1;
  stage_e15_step_index = 0;
  apply_step(0, g_duty_ccr);
  tmr_output_enable(TMR1, TRUE);
  tmr_counter_enable(TMR1, TRUE);
  delay_us(ALIGN_DURATION_MS * 1000u);
  stage_e15_aligning = 0;

  stage_e15_bucket_index = 0;
  start_bucket(0);
  stage_e15_step_count = 1;
  step_timer_init(bucket_period_ticks[0]);
  tmr_counter_enable(TMR3, TRUE);

  for (;;) {
    ++stage_e15_heartbeat;
  }
}
