/*
 * ESCape32 AT32F425 port -- Stage E5: expected-polarity-inversion A/B
 * test. Per the user's explicit instruction after Stage E4's results
 * (blank sweep 2/5/8us showed no meaningful change in saturation
 * counts or the correct/wrong ratio -- switching-transient timing is
 * NOT the dominant factor here): this file changes ONLY
 * ZC_POLARITY_INVERT (fixed to 1) and fixes BLANK_AFTER_EDGE_US=5us
 * (mid-point of the E4 sweep, no sweep here). Sampling timing, ADC
 * trigger configuration, DMA setup, and the open-loop 6-step/
 * alignment/ramp state machine are otherwise BYTE-FOR-BYTE identical
 * to Stage E4 (with the sweep machinery removed since only one blank
 * value is used) -- nothing else was touched, per instruction.
 *
 * Purpose: find out whether steps[]'s expected_dir assignment is
 * simply backwards for this motor's actual winding/rotation-direction
 * combination. The PRODUCTION `steps[]` table itself is NOT modified
 * here (same table, byte-for-byte, as Stage E2/E3/E4) -- only which
 * direction the filters are armed to accept is flipped, exactly like
 * Stage E4's ZC_POLARITY_INVERT mechanism, just fixed to 1 instead of
 * offered as a togglable default-0 option.
 *
 * How to read the result: compare this file's stage_e5_results[7]
 * zc_correct/zc_wrong against Stage E4's blank_us=5 rows (results[7..13]
 * in that run, i.e. the second group of 7). If correct/wrong roughly
 * SWAP (this file's correct >> wrong where E4 had wrong >> correct),
 * the expected_dir assignment in steps[] is backwards for this motor
 * and should be corrected (with this evidence as the reason, not a
 * guess). If the ratio does NOT improve here either, the expected-
 * direction table is not the (or not the only) problem, and the next
 * step is to reconsider the OFF-time sampling approach itself, per the
 * user's own stated fallback plan -- not attempted in this file.
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

#define BLANK_AFTER_EDGE_US 5u /* fixed, mid-point of Stage E4's sweep; no sweep in this file */

#define SAT_LOW_THRESHOLD 50u
#define SAT_HIGH_THRESHOLD 4045u

/* THE ONLY logic change from Stage E4: fixed to 1 instead of a
 * togglable default-0 diagnostic option. steps[] below is untouched. */
#define ZC_POLARITY_INVERT 1

typedef enum { PH_A = 0, PH_B = 1, PH_C = 2 } phase_t;

typedef struct {
  phase_t pos, neg, floating;
  int expected_dir; /* 1=rising, 2=falling -- SAME table as Stage E2/E3/E4, unmodified */
} step_t;

static const step_t steps[6] = {
  {PH_A, PH_B, PH_C, 1},
  {PH_A, PH_C, PH_B, 2},
  {PH_B, PH_C, PH_A, 1},
  {PH_B, PH_A, PH_C, 2},
  {PH_C, PH_A, PH_B, 1},
  {PH_C, PH_B, PH_A, 2},
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
  int32_t diff_min, diff_max;
  uint32_t sat_low_count;
  uint32_t sat_high_count;
  uint16_t adc_a, adc_b, adc_c;
  uint32_t dma_error;
} stage_e5_bucket_t;

stage_e5_bucket_t stage_e5_results[NUM_BUCKETS];
volatile uint32_t stage_e5_bucket_index;

volatile uint32_t stage_e5_heartbeat;
volatile int stage_e5_running;
volatile int stage_e5_aligning;
volatile uint32_t stage_e5_current_step_period_us;
volatile uint32_t stage_e5_step_index;
volatile uint32_t stage_e5_step_count;
volatile int stage_e5_floating_phase;
volatile int stage_e5_expected_dir;           /* real table value, never inverted */
volatile int stage_e5_effective_expected_dir; /* what the filter is actually armed for (inverted here) */
volatile uint16_t stage_e5_adc_a, stage_e5_adc_b, stage_e5_adc_c;
volatile int32_t stage_e5_neutral;
volatile int32_t stage_e5_floating_diff;
volatile int32_t stage_e5_floating_diff_min, stage_e5_floating_diff_max;
volatile uint32_t stage_e5_zc_correct_count;
volatile uint32_t stage_e5_zc_wrong_count;
volatile uint32_t stage_e5_sat_low_count;
volatile uint32_t stage_e5_sat_high_count;
volatile uint32_t stage_e5_dma_error;
volatile int stage_e5_zc_locked;
volatile uint32_t stage_e5_tim1_c4dt;

volatile uint32_t stage_e5_tmr1_ch4_event_count;
volatile uint32_t stage_e5_adc_conversion_count;
volatile uint32_t stage_e5_dma_fdt_count;

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

  stage_e5_tim1_c4dt = ccr4;
}

void TMR1_CH_IRQHandler(void)
{
  if (tmr_flag_get(TMR1, TMR_C4_FLAG) != RESET) {
    tmr_flag_clear(TMR1, TMR_C4_FLAG);
    stage_e5_tmr1_ch4_event_count++;
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

  stage_e5_step_index = (uint32_t)idx;
  stage_e5_floating_phase = (int)s->floating;
  stage_e5_expected_dir = s->expected_dir; /* always the real table value */

#if ZC_POLARITY_INVERT
  int effective_dir = (s->expected_dir == 1) ? 2 : 1;
#else
  int effective_dir = s->expected_dir;
#endif
  stage_e5_effective_expected_dir = effective_dir;

  zc_expect.confirmed_sign = (effective_dir == 1) ? -1 : 1;
  zc_expect.confirm_run = 0;
  zc_anti.confirmed_sign = (effective_dir == 1) ? 1 : -1;
  zc_anti.confirm_run = 0;
  stage_e5_zc_locked = 0;
  blank_scans_remaining = POST_COMMUTATION_BLANK_SCANS;
}

static void stop_and_force_off(void)
{
  tmr_output_enable(TMR1, FALSE);
  tmr_counter_enable(TMR1, FALSE);
  tmr_counter_enable(TMR3, FALSE);
  gate_pins_force_off();
  stage_e5_running = 0;
}

static void reset_bucket_accumulator(void)
{
  stage_e5_zc_correct_count = 0;
  stage_e5_zc_wrong_count = 0;
  stage_e5_floating_diff_min = 0x7fffffff;
  stage_e5_floating_diff_max = -0x7fffffff - 1;
  stage_e5_sat_low_count = 0;
  stage_e5_sat_high_count = 0;
}

static uint32_t bucket_step_count;

static void snapshot_bucket(uint32_t idx)
{
  stage_e5_bucket_t *r = &stage_e5_results[idx];
  r->period_us = stage_e5_current_step_period_us;
  r->step_count = bucket_step_count;
  r->zc_correct = stage_e5_zc_correct_count;
  r->zc_wrong = stage_e5_zc_wrong_count;
  r->diff_min = stage_e5_floating_diff_min;
  r->diff_max = stage_e5_floating_diff_max;
  r->sat_low_count = stage_e5_sat_low_count;
  r->sat_high_count = stage_e5_sat_high_count;
  r->adc_a = stage_e5_adc_a;
  r->adc_b = stage_e5_adc_b;
  r->adc_c = stage_e5_adc_c;
  r->dma_error = stage_e5_dma_error;
}

static void start_bucket(uint32_t idx)
{
  uint32_t period_ms = bucket_period_ms[idx];
  stage_e5_current_step_period_us = period_ms * 1000u;
  bucket_step_count = 0;
  reset_bucket_accumulator();
  TMR3->pr = period_ms - 1u;
}

void TMR3_GLOBAL_IRQHandler(void)
{
  if (tmr_flag_get(TMR3, TMR_OVF_FLAG) == RESET) return;
  tmr_flag_clear(TMR3, TMR_OVF_FLAG);

  if (!stage_e5_running) return;

  int next = (int)((stage_e5_step_index + 1u) % 6u);
  apply_step(next, g_duty_ccr);
  stage_e5_step_count++;
  bucket_step_count++;

  if (bucket_step_count >= STEPS_PER_BUCKET) {
    snapshot_bucket(stage_e5_bucket_index);
    stage_e5_bucket_index++;
    if (stage_e5_bucket_index >= NUM_BUCKETS) {
      stop_and_force_off();
      return;
    }
    start_bucket(stage_e5_bucket_index);
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
    stage_e5_adc_conversion_count++;
  }
}

void DMA1_Channel1_IRQHandler(void)
{
  if (dma_interrupt_flag_get(DMA1_FDT1_FLAG) != RESET) {
    dma_flag_clear(DMA1_FDT1_FLAG);
    stage_e5_dma_fdt_count++;

    uint16_t a = adc_buf[0], b = adc_buf[1], c = adc_buf[2];
    int32_t neutral = ((int32_t)a + (int32_t)b + (int32_t)c) / 3;

    stage_e5_adc_a = a;
    stage_e5_adc_b = b;
    stage_e5_adc_c = c;
    stage_e5_neutral = neutral;

    if (stage_e5_running && !stage_e5_aligning) {
      uint16_t v = (stage_e5_floating_phase == 0) ? a : (stage_e5_floating_phase == 1) ? b : c;
      int32_t diff = (int32_t)v - neutral;
      stage_e5_floating_diff = diff;

      if (blank_scans_remaining) {
        blank_scans_remaining--;
      } else {
        if (v <= SAT_LOW_THRESHOLD) stage_e5_sat_low_count++;
        if (v >= SAT_HIGH_THRESHOLD) stage_e5_sat_high_count++;

        if (diff < stage_e5_floating_diff_min) stage_e5_floating_diff_min = diff;
        if (diff > stage_e5_floating_diff_max) stage_e5_floating_diff_max = diff;

        if (!stage_e5_zc_locked) {
          int r = zc_filter_update(&zc_expect, diff);
          if (r == stage_e5_effective_expected_dir) {
            stage_e5_zc_locked = 1;
            stage_e5_zc_correct_count++;
          }
        }
        int ra = zc_filter_update(&zc_anti, diff);
        if (ra != 0) stage_e5_zc_wrong_count++;
      }
    }
  }
  if (dma_interrupt_flag_get(DMA1_DTERR1_FLAG) != RESET) {
    dma_flag_clear(DMA1_DTERR1_FLAG);
    stage_e5_dma_error++;
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
  stage_e5_aligning = 1;
  stage_e5_running = 1;
  stage_e5_step_index = 0;
  apply_step(0, g_duty_ccr);
  tmr_output_enable(TMR1, TRUE);
  tmr_counter_enable(TMR1, TRUE);
  delay_us(ALIGN_DURATION_MS * 1000u);
  stage_e5_aligning = 0;

  stage_e5_bucket_index = 0;
  start_bucket(0);
  stage_e5_step_count = 1;
  step_timer_init(bucket_period_ms[0]);
  tmr_counter_enable(TMR3, TRUE);

  for (;;) {
    ++stage_e5_heartbeat;
  }
}
