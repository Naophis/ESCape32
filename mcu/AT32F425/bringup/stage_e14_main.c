/*
 * ESCape32 AT32F425 port -- Stage E14: first real closed-chain BEMF/ZC
 * re-evaluation after TWO independent root causes were fixed since the
 * last time open-loop ZC quality was measured (Stage E2/E5/E6, all of
 * which are now considered INVALID and superseded, not just stale):
 *
 *   (1) FIRMWARE: TMR1_CH4's PWM_MODE_A->PWM_MODE_B fix (Stage E9/E10)
 *       -- the ADC's timer-channel external trigger fires on CH4's
 *       RISING edge, which in up-count PWM_MODE_A is fixed at CNT=0
 *       regardless of CCR4. Every ADC-triggered sample taken in Stages
 *       E3-E8 (including all of Stage E6's "OFF-time" ZC data) was
 *       actually triggering at a near-fixed point close to CNT=0, not
 *       at the programmed BLANK_AFTER_EDGE_US offset -- so Stage E6's
 *       zc_correct/zc_wrong numbers were never measuring what they
 *       claimed to.
 *   (2) HARDWARE: the BEMF phase-voltage divider was physically
 *       assembled as 15k/15k instead of the intended 15k/3k, which the
 *       user has now corrected on the real board. Stage E10's retest
 *       confirmed the fix: PWM ON-time positive-role ADC now reads a
 *       consistent ~2300 counts, negative/floating-role ADC ~20 counts
 *       during ON-time, and PA8's falling edge lines up with the
 *       programmed 15%-duty/6.25us window -- all self-consistent for
 *       the first time in this project.
 *
 * Because of (1) and (2) together, EVERY prior open-loop ZC-quality
 * number in this project (Stage E2's ~50/50 split, Stage E4-E6's
 * correct/wrong tables, the E5 "polarity inversion" A/B test) was
 * collected on top of a broken trigger AND a broken divider ratio, and
 * per explicit instruction must be discarded outright -- not reused,
 * not treated as a prior for this file's expected_dir table. The
 * steps[] pos/neg/floating assignment (which phase drives/floats each
 * sector) is kept from Stage E2-E6 since the OPEN-LOOP DRIVE PATTERN
 * itself was never in question -- only expected_dir (which ZC edge
 * direction "correct" means) is reset to a neutral placeholder here
 * and must be re-derived from THIS file's real data, the same way
 * Stage E5 derived it originally, since the old evidence no longer
 * applies to a working sensing chain.
 *
 * Sampling redesign, ON-time instead of OFF-time:
 *   - TMR1 CH4 config is copied verbatim from Stage E10 (PWM_MODE_B,
 *     oc_output_state=TRUE for cctrl.c4en, ACTIVE_HIGH) -- the fixed,
 *     confirmed-working trigger mechanism, not re-derived.
 *   - CCR4 is set to a FIXED ADC_TRIGGER_OFFSET_TICKS (~1.2us into the
 *     PWM cycle, within the user-specified 1.0-1.5us window) instead
 *     of Stage E6's duty_ccr+BLANK_AFTER_EDGE_US formula (which placed
 *     the trigger in what was assumed to be OFF-time, but which never
 *     actually landed there because of bug (1) above). With duty=15%
 *     (6.25us ON window) and offset=1.2us, the 3-channel ADC scan
 *     (~3.25us) finishes around 4.45us, leaving ~1.8us margin before
 *     the falling edge -- same timing budget Stage E7/E8 computed.
 *   - OPEN_LOOP_DUTY_PERCENT raised 8->15 to match the ON-time window
 *     size this sampling design needs (same value Stage E7-E13 used).
 *   - ADC ordinary sequence stays FIXED to physical A/B/C channels
 *     (PHASE_A/B/C_CHANNEL, ranks 1/2/3, exactly as Stage E2-E6 always
 *     did) -- roles (floating/positive/negative) are resolved in
 *     SOFTWARE via vals[floating_phase]/vals[positive_phase]/
 *     vals[negative_phase], set once per apply_step(). This avoids any
 *     runtime OSQ3 rewrite entirely (sidestepping the whole class of
 *     "reconfigured ADC sequence while still externally triggered"
 *     hazard Stage E12 had to fix for its channel-switch diagnostic --
 *     there is no per-step channel switch here at all).
 *   - Zero-cross THRESHOLD changed from the old 3-channel average
 *     "neutral" to positive_adc/2 (per prior explicit permission to
 *     try this): diff = floating_adc - positive_adc/2. With positive
 *     now a real, consistent ~2300-count ON-time reading (post-divider
 *     fix), positive/2 approximates the divided-domain half-bus point
 *     BEMF crosses through -- the classical sensorless reference,
 *     unlike the old 3-average which mixed in two RAIL-saturated
 *     values from OFF-time sampling.
 *
 * Everything else -- steps[] pos/neg/floating table, Schmitt-trigger
 * zc_filter_update() (deadband/confirm-count/shared-lock design from
 * Stage E6), alignment+ramp state machine (TMR3, 300ms align,
 * {15,13,11,9,7,6,5}ms/40-steps-per-bucket), gate_pins_force_off(),
 * DMA/ADC init pattern -- is structurally unchanged from Stage E6.
 *
 * expected_dir in steps[] below is a PLACEHOLDER (all 0 = "unknown/
 * neutral", meaning zc_filter_update()'s output is recorded but not
 * yet classified correct/wrong) until this file's own data determines
 * which direction is correct for THIS corrected sensing chain -- see
 * stage_e14_dir_hist[] (rising-vs-falling confirmation counts per
 * step, tracked regardless of any expected_dir) below, which is enough
 * data to derive the real table afterward, exactly as Stage E5 did
 * the first time.
 */

#include "clock_config.h"

#define PHASE_A_CHANNEL ADC_CHANNEL_0 /* PA0 */
#define PHASE_B_CHANNEL ADC_CHANNEL_4 /* PA4 */
#define PHASE_C_CHANNEL ADC_CHANNEL_5 /* PA5 */

#define PWM_ARR (96000000u / 24000u - 1u) /* 24kHz, unchanged */
#define DEAD_TIME_COUNT 53u /* unverified placeholder, see porting-plan Section 18-C/H */

#define OPEN_LOOP_DUTY_PERCENT 15u /* raised from 8 -- matches Stage E7-E13's ON-time window sizing */
#define ALIGN_DURATION_MS 300u

static const uint32_t bucket_period_ms[] = {15, 13, 11, 9, 7, 6, 5}; /* unchanged */
#define NUM_BUCKETS (sizeof(bucket_period_ms) / sizeof(bucket_period_ms[0]))
#define STEPS_PER_BUCKET 40u

#define ZC_CONFIRM_COUNT 3
#define ZC_DEADBAND 8 /* unchanged -- still the Stage D/E placeholder */

#define TIM1_TICKS_PER_US 96u
#define POST_COMMUTATION_BLANK_SCANS 2u /* unchanged -- guards the first couple of scans right after each commutation edge */
#define ADC_TRIGGER_OFFSET_TICKS 115u /* ~1.2us -- within the user-specified 1.0-1.5us window; CH4 rising edge (Mode B) = ADC trigger instant */

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
 * sector's zero-cross actually confirms as (stage_e14_dir_hist[]),
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
} stage_e14_bucket_t;

stage_e14_bucket_t stage_e14_results[NUM_BUCKETS];
volatile uint32_t stage_e14_bucket_index;

/* Accumulated across the WHOLE run (all buckets), indexed by step_idx
 * (0-5) -- this is the data to derive a real expected_dir table from:
 * for each sector, whichever of rising/falling dominates is that
 * sector's true ZC direction. */
typedef struct { uint32_t rising, falling; } stage_e14_dir_hist_entry_t;
stage_e14_dir_hist_entry_t stage_e14_dir_hist[6];

volatile uint32_t stage_e14_heartbeat;
volatile int stage_e14_running;
volatile int stage_e14_aligning;
volatile uint32_t stage_e14_current_step_period_us;
volatile uint32_t stage_e14_step_index;
volatile uint32_t stage_e14_step_count;
volatile int stage_e14_floating_phase;
volatile int stage_e14_positive_phase;
volatile int stage_e14_negative_phase;
volatile uint16_t stage_e14_adc_a, stage_e14_adc_b, stage_e14_adc_c;
volatile uint16_t stage_e14_floating_adc, stage_e14_positive_adc, stage_e14_negative_adc; /* role-tagged live values */
volatile int32_t stage_e14_floating_diff; /* floating_adc - positive_adc/2 */
volatile int32_t stage_e14_floating_diff_min, stage_e14_floating_diff_max;
volatile uint32_t stage_e14_zc_rising_count;  /* current bucket, live */
volatile uint32_t stage_e14_zc_falling_count; /* current bucket, live */
volatile uint32_t stage_e14_sat_low_count;
volatile uint32_t stage_e14_sat_high_count;
volatile uint32_t stage_e14_dma_error;
volatile int stage_e14_zc_locked; /* shared lock: set the instant EITHER filter confirms anything */

volatile uint32_t stage_e14_tmr1_ch4_event_count;
volatile uint32_t stage_e14_adc_conversion_count;
volatile uint32_t stage_e14_dma_fdt_count;
volatile uint32_t stage_e14_tim1_c4dt;

void _init(void) {}
void _fini(void) {}

typedef struct {
  int confirmed_sign;
  int confirm_run;
} zc_filter_t;

/* Symmetric, unlike Stage E6's expect/anti: zc_rise is armed to catch
 * a rising crossing, zc_fall a falling one -- whichever confirms first
 * in a sector wins (shared lock below), with no notion of which one
 * was "expected". */
static volatile zc_filter_t zc_rise, zc_fall;
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

  stage_e14_tim1_c4dt = ccr4;
}

void TMR1_CH_IRQHandler(void)
{
  if (tmr_flag_get(TMR1, TMR_C4_FLAG) != RESET) {
    tmr_flag_clear(TMR1, TMR_C4_FLAG);
    stage_e14_tmr1_ch4_event_count++;
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

  stage_e14_step_index = (uint32_t)idx;
  stage_e14_floating_phase = (int)s->floating;
  stage_e14_positive_phase = (int)s->pos;
  stage_e14_negative_phase = (int)s->neg;

  /* Symmetric arming: zc_rise only ever detects rising, zc_fall only
   * ever detects falling (see zc_filter_update()'s confirmed_sign<=0
   * branch selection) -- no assumption about which one is "correct"
   * for this sector. */
  zc_rise.confirmed_sign = -1;
  zc_rise.confirm_run = 0;
  zc_fall.confirmed_sign = 1;
  zc_fall.confirm_run = 0;
  stage_e14_zc_locked = 0;
  blank_scans_remaining = POST_COMMUTATION_BLANK_SCANS;
}

static void stop_and_force_off(void)
{
  tmr_output_enable(TMR1, FALSE);
  tmr_counter_enable(TMR1, FALSE);
  tmr_counter_enable(TMR3, FALSE);
  gate_pins_force_off();
  stage_e14_running = 0;
}

static void reset_bucket_accumulator(void)
{
  stage_e14_zc_rising_count = 0;
  stage_e14_zc_falling_count = 0;
  stage_e14_floating_diff_min = 0x7fffffff;
  stage_e14_floating_diff_max = -0x7fffffff - 1;
  stage_e14_sat_low_count = 0;
  stage_e14_sat_high_count = 0;
}

static uint32_t bucket_step_count;

static void snapshot_bucket(uint32_t idx)
{
  stage_e14_bucket_t *r = &stage_e14_results[idx];
  r->period_us = stage_e14_current_step_period_us;
  r->step_count = bucket_step_count;
  r->zc_rising = stage_e14_zc_rising_count;
  r->zc_falling = stage_e14_zc_falling_count;
  r->zc_undecided = bucket_step_count - stage_e14_zc_rising_count - stage_e14_zc_falling_count;
  r->diff_min = stage_e14_floating_diff_min;
  r->diff_max = stage_e14_floating_diff_max;
  r->sat_low_count = stage_e14_sat_low_count;
  r->sat_high_count = stage_e14_sat_high_count;
  r->adc_a = stage_e14_adc_a;
  r->adc_b = stage_e14_adc_b;
  r->adc_c = stage_e14_adc_c;
  r->floating_adc_last = stage_e14_floating_adc;
  r->positive_adc_last = stage_e14_positive_adc;
  r->negative_adc_last = stage_e14_negative_adc;
  r->dma_error = stage_e14_dma_error;
}

static void start_bucket(uint32_t idx)
{
  uint32_t period_ms = bucket_period_ms[idx];
  stage_e14_current_step_period_us = period_ms * 1000u;
  bucket_step_count = 0;
  reset_bucket_accumulator();
  TMR3->pr = period_ms - 1u;
}

void TMR3_GLOBAL_IRQHandler(void)
{
  if (tmr_flag_get(TMR3, TMR_OVF_FLAG) == RESET) return;
  tmr_flag_clear(TMR3, TMR_OVF_FLAG);

  if (!stage_e14_running) return;

  int next = (int)((stage_e14_step_index + 1u) % 6u);
  apply_step(next, g_duty_ccr);
  stage_e14_step_count++;
  bucket_step_count++;

  if (bucket_step_count >= STEPS_PER_BUCKET) {
    snapshot_bucket(stage_e14_bucket_index);
    stage_e14_bucket_index++;
    if (stage_e14_bucket_index >= NUM_BUCKETS) {
      stop_and_force_off();
      return;
    }
    start_bucket(stage_e14_bucket_index);
  }
}

static void step_timer_init(uint32_t first_period_ms)
{
  crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
  nvic_irq_enable(TMR3_GLOBAL_IRQn, 1, 0);

  tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
  tmr_base_init(TMR3, first_period_ms - 1u, 96000u - 1u);
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
    stage_e14_adc_conversion_count++;
  }
}

void DMA1_Channel1_IRQHandler(void)
{
  if (dma_interrupt_flag_get(DMA1_FDT1_FLAG) != RESET) {
    dma_flag_clear(DMA1_FDT1_FLAG);
    stage_e14_dma_fdt_count++;

    uint16_t a = adc_buf[0], b = adc_buf[1], c = adc_buf[2];
    uint16_t vals[3];
    vals[0] = a; vals[1] = b; vals[2] = c;

    stage_e14_adc_a = a;
    stage_e14_adc_b = b;
    stage_e14_adc_c = c;

    if (stage_e14_running && !stage_e14_aligning) {
      uint16_t v = vals[stage_e14_floating_phase];
      uint16_t pos_v = vals[stage_e14_positive_phase];
      stage_e14_floating_adc = v;
      stage_e14_positive_adc = pos_v;
      stage_e14_negative_adc = vals[stage_e14_negative_phase];

      /*
       * THRESHOLD CHANGE: positive_adc/2 instead of the old 3-channel
       * average. positive_adc is now a consistent, non-saturated
       * ON-time reading (post-divider-fix), so half of it approximates
       * the divided-domain half-bus point BEMF crosses -- see file
       * header.
       */
      int32_t diff = (int32_t)v - (int32_t)pos_v / 2;
      stage_e14_floating_diff = diff;

      if (blank_scans_remaining) {
        blank_scans_remaining--;
      } else {
        if (v <= SAT_LOW_THRESHOLD) stage_e14_sat_low_count++;
        if (v >= SAT_HIGH_THRESHOLD) stage_e14_sat_high_count++;

        if (diff < stage_e14_floating_diff_min) stage_e14_floating_diff_min = diff;
        if (diff > stage_e14_floating_diff_max) stage_e14_floating_diff_max = diff;

        /* Shared lock (Stage E6's counting fix, kept): at most one of
         * rising/falling is ever recorded per sector. */
        if (!stage_e14_zc_locked) {
          int r = zc_filter_update(&zc_rise, diff);
          if (r == 1) {
            stage_e14_zc_locked = 1;
            stage_e14_zc_rising_count++;
            stage_e14_dir_hist[stage_e14_step_index].rising++;
          } else {
            int f = zc_filter_update(&zc_fall, diff);
            if (f == 2) {
              stage_e14_zc_locked = 1;
              stage_e14_zc_falling_count++;
              stage_e14_dir_hist[stage_e14_step_index].falling++;
            }
          }
        }
      }
    }
  }
  if (dma_interrupt_flag_get(DMA1_DTERR1_FLAG) != RESET) {
    dma_flag_clear(DMA1_DTERR1_FLAG);
    stage_e14_dma_error++;
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
  stage_e14_aligning = 1;
  stage_e14_running = 1;
  stage_e14_step_index = 0;
  apply_step(0, g_duty_ccr);
  tmr_output_enable(TMR1, TRUE);
  tmr_counter_enable(TMR1, TRUE);
  delay_us(ALIGN_DURATION_MS * 1000u);
  stage_e14_aligning = 0;

  stage_e14_bucket_index = 0;
  start_bucket(0);
  stage_e14_step_count = 1;
  step_timer_init(bucket_period_ms[0]);
  tmr_counter_enable(TMR3, TRUE);

  for (;;) {
    ++stage_e14_heartbeat;
  }
}
