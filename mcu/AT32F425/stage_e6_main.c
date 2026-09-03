/*
 * ESCape32 AT32F425 port -- Stage E6: (1) permanent expected-direction
 * table fix, evidence-based from Stage E5 (inverted polarity: 183
 * correct / 102 wrong vs Stage E4 blank=5us's 128 correct / 186 wrong
 * -- a clean, consistent flip across all 7 speed buckets); (2) a
 * zc_correct/zc_wrong COUNTING BUG fix; (3) role-tagged ADC diagnostics
 * to empirically pin down which role (floating/positive/negative) is
 * actually the one hitting the ADC rails, instead of guessing from an
 * unverified physical model. ADC trigger configuration, DMA, and the
 * open-loop 6-step/alignment/ramp state machine are UNCHANGED from
 * Stage E5, per explicit instruction.
 *
 * (1) steps[] expected_dir is now PERMANENTLY the inverted assignment
 *     Stage E5 tested via ZC_POLARITY_INVERT -- that flag is gone,
 *     the table itself carries the corrected values now. Reasoning
 *     recorded here: the previous table was derived from a generic
 *     "standard alternating rising/falling" textbook pattern with no
 *     way to know a priori which absolute direction corresponds to
 *     "rising" vs "falling" for THIS winding-to-ADC-pin mapping and
 *     THIS rotation direction -- Stage E5's A/B test is the actual
 *     evidence that resolves that ambiguity, not a re-derivation from
 *     first principles.
 *
 * (2) COUNTING BUG (why zc_correct+zc_wrong could exceed step_count,
 *     e.g. 43 > 40 at 7ms and 5ms in the E5 data): zc_anti (the
 *     "wrong" diagnostic filter) was being updated on EVERY non-
 *     blanked scan for the WHOLE sector, with no lock of its own --
 *     only zc_expect ("correct") had a per-sector lock
 *     (stage_e5_zc_locked). So a single sector could rack up multiple
 *     "wrong" counts (if the anti-direction condition flickered
 *     confirmed more than once before the sector ended) IN ADDITION
 *     to a "correct" count, breaking the "at most one classification
 *     per sector" invariant the user specified.
 *     FIX: both filters now share ONE per-sector lock
 *     (stage_e6_zc_locked, set the instant EITHER filter confirms
 *     anything -- correct OR wrong). Once locked, neither filter is
 *     updated again until the next apply_step() resets everything.
 *     This makes zc_correct_count and zc_wrong_count mutually
 *     exclusive PER SECTOR by construction: for any bucket,
 *     zc_correct + zc_wrong + zc_undecided == step_count, always
 *     (zc_undecided = sectors where neither filter ever confirmed
 *     anything -- also newly tracked and stored per bucket).
 *
 * (3) Role-tagged ADC diagnostics: the previous adc_a/b/c snapshot
 *     fields don't say which of the three was playing which role
 *     (positive/negative/floating) at that instant, so "adc_b=4095"
 *     couldn't be attributed to a role. This file additionally
 *     records floating_adc_last/positive_adc_last/negative_adc_last
 *     per bucket (the corresponding raw code from the LAST non-
 *     blanked scan of that bucket), so the persistent near-4095/near-0
 *     readings can be attributed to a role instead of an arbitrary
 *     phase letter -- needed before deciding how to redesign sampling
 *     (per-role behavior, not per-letter, is what the next redesign
 *     step needs to target). No sampling-method change is made in
 *     this file -- still full 3-channel OFF-time scan, same trigger
 *     timing (BLANK_AFTER_EDGE_US=5us) as Stage E5.
 */

#include "clock_config.h"

#define PHASE_A_CHANNEL ADC_CHANNEL_0 /* PA0 */
#define PHASE_B_CHANNEL ADC_CHANNEL_4 /* PA4 */
#define PHASE_C_CHANNEL ADC_CHANNEL_5 /* PA5 */

#define PWM_ARR (96000000u / 24000u - 1u) /* 24kHz, unchanged */
#define DEAD_TIME_COUNT 53u /* unverified placeholder, see porting-plan Section 18-C/H */

#define OPEN_LOOP_DUTY_PERCENT 8u /* held constant, unchanged */
#define ALIGN_DURATION_MS 300u

static const uint32_t bucket_period_ms[] = {15, 13, 11, 9, 7, 6, 5}; /* unchanged */
#define NUM_BUCKETS (sizeof(bucket_period_ms) / sizeof(bucket_period_ms[0]))
#define STEPS_PER_BUCKET 40u

#define ZC_CONFIRM_COUNT 3
#define ZC_DEADBAND 8 /* unchanged -- still the Stage D/E placeholder */

#define TIM1_TICKS_PER_US 96u
#define POST_COMMUTATION_BLANK_SCANS 2u /* unchanged */
#define BLANK_AFTER_EDGE_US 5u /* unchanged from Stage E5 */

#define SAT_LOW_THRESHOLD 50u
#define SAT_HIGH_THRESHOLD 4045u

typedef enum { PH_A = 0, PH_B = 1, PH_C = 2 } phase_t;

typedef struct {
  phase_t pos, neg, floating;
  int expected_dir; /* 1=rising, 2=falling -- PERMANENTLY corrected per Stage E5's evidence */
} step_t;

/*
 * Same pos/neg/floating assignments as Stage E2-E5 (unchanged: the
 * open-loop drive pattern itself was never in question). Only
 * expected_dir is flipped relative to the original table, per Stage
 * E5's confirmed result.
 */
static const step_t steps[6] = {
  {PH_A, PH_B, PH_C, 2},
  {PH_A, PH_C, PH_B, 1},
  {PH_B, PH_C, PH_A, 2},
  {PH_B, PH_A, PH_C, 1},
  {PH_C, PH_A, PH_B, 2},
  {PH_C, PH_B, PH_A, 1},
};

static const tmr_channel_select_type phase_channel[3] = {
  TMR_SELECT_CHANNEL_1, TMR_SELECT_CHANNEL_2, TMR_SELECT_CHANNEL_3
};

static volatile uint16_t adc_buf[3];

typedef struct {
  uint32_t period_us;
  uint32_t step_count;
  uint32_t zc_correct;
  uint32_t zc_wrong;
  uint32_t zc_undecided; /* step_count - zc_correct - zc_wrong; sectors with no confirmed event at all */
  int32_t diff_min, diff_max;
  uint32_t sat_low_count;
  uint32_t sat_high_count;
  uint16_t adc_a, adc_b, adc_c;
  uint16_t floating_adc_last, positive_adc_last, negative_adc_last; /* role-tagged, NEW */
  uint32_t dma_error;
} stage_e6_bucket_t;

stage_e6_bucket_t stage_e6_results[NUM_BUCKETS];
volatile uint32_t stage_e6_bucket_index;

volatile uint32_t stage_e6_heartbeat;
volatile int stage_e6_running;
volatile int stage_e6_aligning;
volatile uint32_t stage_e6_current_step_period_us;
volatile uint32_t stage_e6_step_index;
volatile uint32_t stage_e6_step_count;
volatile int stage_e6_floating_phase;
volatile int stage_e6_positive_phase;
volatile int stage_e6_negative_phase;
volatile int stage_e6_expected_dir;
volatile uint16_t stage_e6_adc_a, stage_e6_adc_b, stage_e6_adc_c;
volatile uint16_t stage_e6_floating_adc, stage_e6_positive_adc, stage_e6_negative_adc; /* NEW: role-tagged live values */
volatile int32_t stage_e6_neutral;
volatile int32_t stage_e6_floating_diff;
volatile int32_t stage_e6_floating_diff_min, stage_e6_floating_diff_max;
volatile uint32_t stage_e6_zc_correct_count; /* current bucket, live */
volatile uint32_t stage_e6_zc_wrong_count;   /* current bucket, live */
volatile uint32_t stage_e6_sat_low_count;
volatile uint32_t stage_e6_sat_high_count;
volatile uint32_t stage_e6_dma_error;
volatile int stage_e6_zc_locked; /* shared lock: set the instant EITHER filter confirms anything */

volatile uint32_t stage_e6_tmr1_ch4_event_count;
volatile uint32_t stage_e6_adc_conversion_count;
volatile uint32_t stage_e6_dma_fdt_count;
volatile uint32_t stage_e6_tim1_c4dt;

void _init(void) {}
void _fini(void) {}

typedef struct {
  int confirmed_sign;
  int confirm_run;
} zc_filter_t;

static volatile zc_filter_t zc_expect, zc_anti;
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

static void tim1_adc_trigger_config(void)
{
  tmr_output_config_type oc;
  uint16_t ccr4 = (uint16_t)(g_duty_ccr + BLANK_AFTER_EDGE_US * TIM1_TICKS_PER_US);

  tmr_output_default_para_init(&oc);
  oc.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  oc.oc_output_state = TRUE; /* required for cctrl.c4en -- Stage E3's fix */
  oc.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  oc.oc_idle_state = FALSE;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &oc);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, ccr4);
  tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_4, TRUE);
  tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG | TMR_HALL_SWTRIG);

  nvic_irq_enable(TMR1_CH_IRQn, 2, 0);
  tmr_interrupt_enable(TMR1, TMR_C4_INT, TRUE);

  stage_e6_tim1_c4dt = ccr4;
}

void TMR1_CH_IRQHandler(void)
{
  if (tmr_flag_get(TMR1, TMR_C4_FLAG) != RESET) {
    tmr_flag_clear(TMR1, TMR_C4_FLAG);
    stage_e6_tmr1_ch4_event_count++;
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

  stage_e6_step_index = (uint32_t)idx;
  stage_e6_floating_phase = (int)s->floating;
  stage_e6_positive_phase = (int)s->pos;
  stage_e6_negative_phase = (int)s->neg;
  stage_e6_expected_dir = s->expected_dir;

  zc_expect.confirmed_sign = (s->expected_dir == 1) ? -1 : 1;
  zc_expect.confirm_run = 0;
  zc_anti.confirmed_sign = (s->expected_dir == 1) ? 1 : -1;
  zc_anti.confirm_run = 0;
  stage_e6_zc_locked = 0;
  blank_scans_remaining = POST_COMMUTATION_BLANK_SCANS;
}

static void stop_and_force_off(void)
{
  tmr_output_enable(TMR1, FALSE);
  tmr_counter_enable(TMR1, FALSE);
  tmr_counter_enable(TMR3, FALSE);
  gate_pins_force_off();
  stage_e6_running = 0;
}

static void reset_bucket_accumulator(void)
{
  stage_e6_zc_correct_count = 0;
  stage_e6_zc_wrong_count = 0;
  stage_e6_floating_diff_min = 0x7fffffff;
  stage_e6_floating_diff_max = -0x7fffffff - 1;
  stage_e6_sat_low_count = 0;
  stage_e6_sat_high_count = 0;
}

static uint32_t bucket_step_count;

static void snapshot_bucket(uint32_t idx)
{
  stage_e6_bucket_t *r = &stage_e6_results[idx];
  r->period_us = stage_e6_current_step_period_us;
  r->step_count = bucket_step_count;
  r->zc_correct = stage_e6_zc_correct_count;
  r->zc_wrong = stage_e6_zc_wrong_count;
  r->zc_undecided = bucket_step_count - stage_e6_zc_correct_count - stage_e6_zc_wrong_count;
  r->diff_min = stage_e6_floating_diff_min;
  r->diff_max = stage_e6_floating_diff_max;
  r->sat_low_count = stage_e6_sat_low_count;
  r->sat_high_count = stage_e6_sat_high_count;
  r->adc_a = stage_e6_adc_a;
  r->adc_b = stage_e6_adc_b;
  r->adc_c = stage_e6_adc_c;
  r->floating_adc_last = stage_e6_floating_adc;
  r->positive_adc_last = stage_e6_positive_adc;
  r->negative_adc_last = stage_e6_negative_adc;
  r->dma_error = stage_e6_dma_error;
}

static void start_bucket(uint32_t idx)
{
  uint32_t period_ms = bucket_period_ms[idx];
  stage_e6_current_step_period_us = period_ms * 1000u;
  bucket_step_count = 0;
  reset_bucket_accumulator();
  TMR3->pr = period_ms - 1u;
}

void TMR3_GLOBAL_IRQHandler(void)
{
  if (tmr_flag_get(TMR3, TMR_OVF_FLAG) == RESET) return;
  tmr_flag_clear(TMR3, TMR_OVF_FLAG);

  if (!stage_e6_running) return;

  int next = (int)((stage_e6_step_index + 1u) % 6u);
  apply_step(next, g_duty_ccr);
  stage_e6_step_count++;
  bucket_step_count++;

  if (bucket_step_count >= STEPS_PER_BUCKET) {
    snapshot_bucket(stage_e6_bucket_index);
    stage_e6_bucket_index++;
    if (stage_e6_bucket_index >= NUM_BUCKETS) {
      stop_and_force_off();
      return;
    }
    start_bucket(stage_e6_bucket_index);
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
    stage_e6_adc_conversion_count++;
  }
}

void DMA1_Channel1_IRQHandler(void)
{
  if (dma_interrupt_flag_get(DMA1_FDT1_FLAG) != RESET) {
    dma_flag_clear(DMA1_FDT1_FLAG);
    stage_e6_dma_fdt_count++;

    uint16_t a = adc_buf[0], b = adc_buf[1], c = adc_buf[2];
    int32_t neutral = ((int32_t)a + (int32_t)b + (int32_t)c) / 3;
    uint16_t vals[3];
    vals[0] = a; vals[1] = b; vals[2] = c;

    stage_e6_adc_a = a;
    stage_e6_adc_b = b;
    stage_e6_adc_c = c;
    stage_e6_neutral = neutral;

    if (stage_e6_running && !stage_e6_aligning) {
      uint16_t v = vals[stage_e6_floating_phase];
      stage_e6_floating_adc = v;
      stage_e6_positive_adc = vals[stage_e6_positive_phase];
      stage_e6_negative_adc = vals[stage_e6_negative_phase];

      int32_t diff = (int32_t)v - neutral;
      stage_e6_floating_diff = diff;

      if (blank_scans_remaining) {
        blank_scans_remaining--;
      } else {
        if (v <= SAT_LOW_THRESHOLD) stage_e6_sat_low_count++;
        if (v >= SAT_HIGH_THRESHOLD) stage_e6_sat_high_count++;

        if (diff < stage_e6_floating_diff_min) stage_e6_floating_diff_min = diff;
        if (diff > stage_e6_floating_diff_max) stage_e6_floating_diff_max = diff;

        /*
         * COUNTING FIX: both filters now gated by the SAME lock, set
         * the instant either one confirms anything. This makes
         * correct/wrong mutually exclusive per sector -- see file
         * header (2).
         */
        if (!stage_e6_zc_locked) {
          int r = zc_filter_update(&zc_expect, diff);
          if (r == stage_e6_expected_dir) {
            stage_e6_zc_locked = 1;
            stage_e6_zc_correct_count++;
          } else {
            int ra = zc_filter_update(&zc_anti, diff);
            if (ra != 0) {
              stage_e6_zc_locked = 1;
              stage_e6_zc_wrong_count++;
            }
          }
        }
      }
    }
  }
  if (dma_interrupt_flag_get(DMA1_DTERR1_FLAG) != RESET) {
    dma_flag_clear(DMA1_DTERR1_FLAG);
    stage_e6_dma_error++;
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
  stage_e6_aligning = 1;
  stage_e6_running = 1;
  stage_e6_step_index = 0;
  apply_step(0, g_duty_ccr);
  tmr_output_enable(TMR1, TRUE);
  tmr_counter_enable(TMR1, TRUE);
  delay_us(ALIGN_DURATION_MS * 1000u);
  stage_e6_aligning = 0;

  stage_e6_bucket_index = 0;
  start_bucket(0);
  stage_e6_step_count = 1;
  step_timer_init(bucket_period_ms[0]);
  tmr_counter_enable(TMR3, TRUE);

  for (;;) {
    ++stage_e6_heartbeat;
  }
}
