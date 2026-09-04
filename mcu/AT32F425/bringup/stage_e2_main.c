/*
 * ESCape32 AT32F425 port -- Stage E2: rotor alignment + open-loop
 * step-period acceleration ramp (15ms/step -> 5ms/step), still no
 * BEMF-driven commutation timing. Builds directly on Stage E, which
 * confirmed on real hardware: motor spins, 60/60 steps completed,
 * dma_error=0, zc_correct=177 vs zc_wrong=153 at a fixed 15ms/step
 * (roughly 50/50, i.e. BEMF too weak/noisy at that speed to trust for
 * closed-loop -- expected, per the user, at such a low speed).
 *
 * Sequence implemented here (matches the classic sensorless-ESC
 * open-loop startup structure the user asked for):
 *   1. rotor alignment: hold step 0's phase pattern statically (no
 *      advancing) for ALIGN_DURATION_MS so the rotor settles into a
 *      known position before stepping starts.
 *   2. low-speed open-loop start at bucket_period_ms[0] (15ms/step).
 *   3. step period is shortened in discrete steps (buckets) down to
 *      bucket_period_ms[NUM_BUCKETS-1] (5ms/step); STEPS_PER_BUCKET
 *      steps are run at each period before moving to the next.
 *   4. BEMF amplitude (floating-phase diff min/max) and zero-cross
 *      correct/wrong counts are accumulated per bucket and snapshotted
 *      into stage_e2_results[] when each bucket completes, so the
 *      whole ramp's history can be read back over OpenOCD in one shot
 *      after the run finishes (no need to catch it live).
 *   5. NOT implemented here on purpose: handover to BEMF-driven
 *      commutation. This file only proves the open-loop acceleration
 *      itself is stable and characterizes how correct/wrong ZC ratio
 *      changes with speed. Closed-loop handover is later work, gated
 *      on this ramp's results.
 *
 * Duty is held CONSTANT at OPEN_LOOP_DUTY_PERCENT for the whole ramp
 * (not requested to change it) so step period is the only variable
 * changing between buckets -- keeps the correct/wrong-vs-speed
 * comparison the user wants clean.
 *
 * Everything else (6-step table, per-phase drive reasoning, GPIO
 * safe-state sequencing, ISR weight) is unchanged from stage_e_main.c
 * -- see that file's header comment for the detailed rationale.
 */

#include "clock_config.h"

#define PHASE_A_CHANNEL ADC_CHANNEL_0 /* PA0 */
#define PHASE_B_CHANNEL ADC_CHANNEL_4 /* PA4 */
#define PHASE_C_CHANNEL ADC_CHANNEL_5 /* PA5 */

#define PWM_ARR (96000000u / 24000u - 1u) /* 24kHz, same as Stage B/E */
#define DEAD_TIME_COUNT 53u /* unverified placeholder, see porting-plan Section 18-C/H */

#define OPEN_LOOP_DUTY_PERCENT 8u /* held constant through the whole ramp */
#define ALIGN_DURATION_MS 300u

/* 15ms -> 5ms in 7 discrete steps, STEPS_PER_BUCKET steps run at each
 * before moving to the next. Tune freely -- this is the first, most
 * conservative attempt. */
static const uint32_t bucket_period_ms[] = {15, 13, 11, 9, 7, 6, 5};
#define NUM_BUCKETS (sizeof(bucket_period_ms) / sizeof(bucket_period_ms[0]))
#define STEPS_PER_BUCKET 40u /* ~6.7 electrical revolutions per bucket */

#define ZC_CONFIRM_COUNT 3
#define ZC_DEADBAND 8 /* PLACEHOLDER, same as Stage D/E -- re-tune from real data */

typedef enum { PH_A = 0, PH_B = 1, PH_C = 2 } phase_t;

typedef struct {
  phase_t pos, neg, floating;
  int expected_dir; /* 1=rising, 2=falling */
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
  uint16_t adc_a, adc_b, adc_c; /* last sample observed in this bucket */
  uint32_t dma_error;           /* cumulative dma_error at end of this bucket */
} stage_e2_bucket_t;

stage_e2_bucket_t stage_e2_results[NUM_BUCKETS];
volatile uint32_t stage_e2_bucket_index; /* bucket currently running (or NUM_BUCKETS when done) */

volatile uint32_t stage_e2_heartbeat;
volatile int stage_e2_running;
volatile int stage_e2_aligning;
volatile uint32_t stage_e2_current_step_period_us;
volatile uint32_t stage_e2_step_index;
volatile uint32_t stage_e2_step_count; /* cumulative across the whole ramp */
volatile int stage_e2_floating_phase;
volatile int stage_e2_expected_dir;
volatile uint16_t stage_e2_adc_a, stage_e2_adc_b, stage_e2_adc_c;
volatile int32_t stage_e2_neutral;
volatile int32_t stage_e2_floating_diff;
volatile int32_t stage_e2_floating_diff_min, stage_e2_floating_diff_max; /* current bucket, live */
volatile uint32_t stage_e2_zc_correct_count; /* current bucket, live */
volatile uint32_t stage_e2_zc_wrong_count;   /* current bucket, live */
volatile uint32_t stage_e2_dma_error;        /* cumulative */

void _init(void) {}
void _fini(void) {}

typedef struct {
  int confirmed_sign;
  int confirm_run;
} zc_filter_t;

static volatile zc_filter_t zc;

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

  stage_e2_step_index = (uint32_t)idx;
  stage_e2_floating_phase = (int)s->floating;
  stage_e2_expected_dir = s->expected_dir;

  zc.confirmed_sign = (s->expected_dir == 1) ? -1 : 1;
  zc.confirm_run = 0;
}

static void stop_and_force_off(void)
{
  tmr_output_enable(TMR1, FALSE);
  tmr_counter_enable(TMR1, FALSE);
  tmr_counter_enable(TMR3, FALSE);
  gate_pins_force_off();
  stage_e2_running = 0;
}

static void reset_bucket_accumulator(void)
{
  stage_e2_zc_correct_count = 0;
  stage_e2_zc_wrong_count = 0;
  stage_e2_floating_diff_min = 0x7fffffff;
  stage_e2_floating_diff_max = -0x7fffffff - 1;
}

static uint32_t bucket_step_count;
static uint16_t g_duty_ccr;

static void snapshot_bucket(uint32_t idx)
{
  stage_e2_bucket_t *r = &stage_e2_results[idx];
  r->period_us = stage_e2_current_step_period_us;
  r->step_count = bucket_step_count;
  r->zc_correct = stage_e2_zc_correct_count;
  r->zc_wrong = stage_e2_zc_wrong_count;
  r->diff_min = stage_e2_floating_diff_min;
  r->diff_max = stage_e2_floating_diff_max;
  r->adc_a = stage_e2_adc_a;
  r->adc_b = stage_e2_adc_b;
  r->adc_c = stage_e2_adc_c;
  r->dma_error = stage_e2_dma_error;
}

static void start_bucket(uint32_t idx)
{
  uint32_t period_ms = bucket_period_ms[idx];
  stage_e2_current_step_period_us = period_ms * 1000u;
  bucket_step_count = 0;
  reset_bucket_accumulator();
  TMR3->pr = period_ms - 1u; /* direct ARR update; takes effect from the next reload */
}

void TMR3_GLOBAL_IRQHandler(void)
{
  if (tmr_flag_get(TMR3, TMR_OVF_FLAG) == RESET) return;
  tmr_flag_clear(TMR3, TMR_OVF_FLAG);

  if (!stage_e2_running) return;

  int next = (int)((stage_e2_step_index + 1u) % 6u);
  apply_step(next, g_duty_ccr);
  stage_e2_step_count++;
  bucket_step_count++;

  if (bucket_step_count >= STEPS_PER_BUCKET) {
    snapshot_bucket(stage_e2_bucket_index);
    stage_e2_bucket_index++;
    if (stage_e2_bucket_index >= NUM_BUCKETS) {
      stop_and_force_off();
      return;
    }
    start_bucket(stage_e2_bucket_index);
  }
}

static void step_timer_init(uint32_t first_period_ms)
{
  crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
  nvic_irq_enable(TMR3_GLOBAL_IRQn, 1, 0);

  tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
  tmr_base_init(TMR3, first_period_ms - 1u, 96000u - 1u); /* 96MHz/96000=1kHz -> 1ms tick */
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
  b.repeat_mode = TRUE;
  b.data_align = ADC_RIGHT_ALIGNMENT;
  b.ordinary_channel_length = 3;
  adc_base_config(ADC1, &b);

  adc_ordinary_channel_set(ADC1, PHASE_A_CHANNEL, 1, ADC_SAMPLETIME_13_5);
  adc_ordinary_channel_set(ADC1, PHASE_B_CHANNEL, 2, ADC_SAMPLETIME_13_5);
  adc_ordinary_channel_set(ADC1, PHASE_C_CHANNEL, 3, ADC_SAMPLETIME_13_5);

  adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);
  adc_dma_mode_enable(ADC1, TRUE);

  adc_enable(ADC1, TRUE);

  adc_calibration_init(ADC1);
  while (adc_calibration_init_status_get(ADC1));
  adc_calibration_start(ADC1);
  while (adc_calibration_status_get(ADC1));
}

void DMA1_Channel1_IRQHandler(void)
{
  if (dma_interrupt_flag_get(DMA1_FDT1_FLAG) != RESET) {
    dma_flag_clear(DMA1_FDT1_FLAG);

    uint16_t a = adc_buf[0], b = adc_buf[1], c = adc_buf[2];
    int32_t neutral = ((int32_t)a + (int32_t)b + (int32_t)c) / 3;

    stage_e2_adc_a = a;
    stage_e2_adc_b = b;
    stage_e2_adc_c = c;
    stage_e2_neutral = neutral;

    if (stage_e2_running && !stage_e2_aligning) {
      int32_t v = (stage_e2_floating_phase == 0) ? a : (stage_e2_floating_phase == 1) ? b : c;
      int32_t diff = v - neutral;
      stage_e2_floating_diff = diff;
      if (diff < stage_e2_floating_diff_min) stage_e2_floating_diff_min = diff;
      if (diff > stage_e2_floating_diff_max) stage_e2_floating_diff_max = diff;

      int r = zc_filter_update(&zc, diff);
      if (r == stage_e2_expected_dir) stage_e2_zc_correct_count++;
      else if (r != 0) stage_e2_zc_wrong_count++;
    }
  }
  if (dma_interrupt_flag_get(DMA1_DTERR1_FLAG) != RESET) {
    dma_flag_clear(DMA1_DTERR1_FLAG);
    stage_e2_dma_error++;
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

  /* 1) Safe state first. */
  gate_pins_force_off();

  /* 2) BEMF-phase ADC, running throughout. */
  timestamp_timer_config();
  adc_gpio_config();
  dma_config();
  adc_config();
  dma_channel_enable(DMA1_CHANNEL1, TRUE);
  adc_ordinary_software_trigger_enable(ADC1, TRUE);

  /* 3) TIM1 configured, pins still plain GPIO LOW -- nothing driven yet. */
  tim1_init();
  g_duty_ccr = (uint16_t)((PWM_ARR + 1u) * OPEN_LOOP_DUTY_PERCENT / 100u);

  /* 4) Switch to TIM1 AF and hold step 0 statically for rotor alignment. */
  tim1_pins_to_af();
  stage_e2_aligning = 1;
  stage_e2_running = 1;
  stage_e2_step_index = 0;
  apply_step(0, g_duty_ccr);
  tmr_output_enable(TMR1, TRUE);
  tmr_counter_enable(TMR1, TRUE);
  delay_us(ALIGN_DURATION_MS * 1000u);
  stage_e2_aligning = 0;

  /* 5) Start the open-loop acceleration ramp. */
  stage_e2_bucket_index = 0;
  start_bucket(0);
  stage_e2_step_count = 1;
  step_timer_init(bucket_period_ms[0]);
  tmr_counter_enable(TMR3, TRUE);

  for (;;) {
    ++stage_e2_heartbeat;
  }
}
