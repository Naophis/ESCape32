/*
 * ESCape32 AT32F425 port -- Stage E8: PWM timing characterization
 * (no oscilloscope available, so the ADC itself is used as the probe).
 *
 * WHY: Stage E7's positive_adc/negative_adc readings were wildly
 * inconsistent with the expected ON-time plateau (15k/3k divider @3S
 * should give positive~=2300-2700, negative~=0): observed positive
 * ranged 101..4095 and negative up to 1322 across buckets, and
 * diff_min/max pinned at -2047/+2048 in every bucket. Per the user's
 * explicit instruction, this does NOT yet mean "ON-time sampling is
 * wrong" -- it could equally mean TMR1_CH4's trigger point isn't
 * landing where the CCR4-vs-CNT arithmetic assumes it does. This file
 * settles that empirically instead of assuming either way.
 *
 * WHAT THIS FILE DOES (and does NOT do):
 *   1. Commutation is FROZEN at a single sector: apply_step(0, ...) is
 *      called exactly once at startup and NEVER again. No TMR3 step
 *      timer runs in this file at all -- positive/negative/floating
 *      role assignments (and TIM1 CH1-3's PWM mode/polarity/
 *      complementary configuration) are constant for the whole run.
 *      Per the fixed steps[0] = {pos=A, neg=B, floating=C}:
 *        - Phase A (positive): TMR1 CH1, PWM_MODE_A, oc_polarity=
 *          ACTIVE_HIGH, complementary (CH1C) enabled ACTIVE_HIGH ->
 *          configured as standard damped/complementary PWM, CCR=
 *          duty_ccr. On the CONFIGURED convention (PWM_MODE_A active
 *          while CNT<CCR, per STM32-compatible OCxM=110 semantics
 *          this port's TIM1 register layout was confirmed bit-for-bit
 *          compatible with -- see porting-plan Section 18), HSA should
 *          be high for CNT in [0, ccr) and low (LSA high, dead-time
 *          inserted) for CNT in [ccr, ARR]. Per instruction (8), this
 *          is a CONFIGURATION fact, not asserted as the measured
 *          truth -- the sweep below is what actually answers "is CNT
 *          in [0,ccr) really when HSA is high".
 *        - Phase B (negative): TMR1 CH2, main forced LOW (HSB
 *          constant 0), complementary (CH2C) enabled ACTIVE_HIGH ->
 *          configured as LSB constant-on, HSB constant-off, for the
 *          WHOLE period (not just part of it).
 *        - Phase C (floating): TMR1 CH3/CH3C both disabled
 *          (oc_output_state=occ_output_state=FALSE), idle-state LOW ->
 *          configured as both FETs off for the whole period.
 *   2. duty=15%, PWM=24kHz (ccr=600, ARR=3999) -- same as Stage E7.
 *   3. TMR1_CH4's trigger offset (CCR4, i.e. ticks after CNT wraps to
 *      0) is swept across offset_half_us[] = 0.5,1,1.5,...,6,8,10,15,
 *      20,30,40 us (18 points), covering nearly the whole 41.67us
 *      period, not just the assumed ON-time window -- exactly per
 *      instruction. Updated via the same direct, non-buffered
 *      TMR1->c4dt write Stage E4 established (safe: doesn't touch
 *      CH1-3's CCPC-buffered state).
 *   4. 128 PWM cycles are sampled at each offset (SAMPLES_PER_OFFSET),
 *      accumulating min/max/sum for positive_adc, negative_adc,
 *      floating_adc, plus a per-channel near-rail saturation count,
 *      into stage_e8_results[offset_index] -- read the whole array
 *      after the run auto-stops.
 *   5. ADC/DMA setup (channels, ADC12_ORDINARY_TRIG_TMR1CH4,
 *      repeat_mode=FALSE, DMA circular 3-halfword transfer) is REUSED
 *      unmodified from Stage E7 -- only the ordinary-sequence channel
 *      assignment is programmed ONCE (for the frozen role mapping)
 *      instead of every step, since there is no "every step" here.
 *   6. NO zero-cross filter/logic at all in this file -- pure voltage/
 *      timing characterization, nothing that resembles a ZC decision.
 *   7-9. See stage_e8_results[]; interpretation (which offset actually
 *      lands positive~=2300-2700/negative~=0, and what that says about
 *      HS-on timing) is left to the data, not asserted here.
 *
 * The motor does NOT spin in this file (commutation frozen at one
 * sector, same as the alignment hold in earlier stages) -- ~100ms
 * total runtime (18 offsets x 128 cycles x ~41.7us), low average
 * current at 15% duty. Safe, brief, stationary hold.
 */

#include "clock_config.h"

static const int16_t phase_adc_channel[3] = {
  (int16_t)ADC_CHANNEL_0, /* PA0 = A */
  (int16_t)ADC_CHANNEL_4, /* PA4 = B */
  (int16_t)ADC_CHANNEL_5, /* PA5 = C */
};

#define PWM_ARR (96000000u / 24000u - 1u) /* 24kHz, unchanged */
#define DEAD_TIME_COUNT 53u /* unverified placeholder, see porting-plan Section 18-C/H */

#define OPEN_LOOP_DUTY_PERCENT 15u /* unchanged from Stage E7 */
#define TIM1_TICKS_PER_US 96u

#define SAT_LOW_THRESHOLD 50u
#define SAT_HIGH_THRESHOLD 4045u

/* Offsets in HALF-microseconds (so 0.5us steps are exact integers):
 * 0.5,1.0,1.5,2.0,2.5,3.0,3.5,4.0,4.5,5.0,5.5,6.0,8,10,15,20,30,40 us */
static const uint16_t offset_half_us[] = {
  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 16, 20, 30, 40, 60, 80
};
#define NUM_OFFSETS (sizeof(offset_half_us) / sizeof(offset_half_us[0]))
#define SAMPLES_PER_OFFSET 128u

typedef enum { PH_A = 0, PH_B = 1, PH_C = 2 } phase_t;

typedef struct {
  phase_t pos, neg, floating;
} fixed_step_t;

/* Same steps[0] as Stage E2-E7's table (pos=A, neg=B, floating=C);
 * frozen here, never advanced. */
static const fixed_step_t fixed_step = {PH_A, PH_B, PH_C};

static const tmr_channel_select_type phase_channel[3] = {
  TMR_SELECT_CHANNEL_1, TMR_SELECT_CHANNEL_2, TMR_SELECT_CHANNEL_3
};

static volatile uint16_t adc_buf[3]; /* [0]=floating, [1]=positive, [2]=negative */

typedef struct {
  float offset_us;
  uint16_t offset_ticks; /* raw CCR4 value used, for cross-checking */
  uint16_t sample_count;
  uint16_t positive_min, positive_max;
  uint16_t negative_min, negative_max;
  uint16_t floating_min, floating_max;
  uint32_t positive_sum, negative_sum, floating_sum; /* mean = sum/sample_count */
  uint32_t positive_sat_low, positive_sat_high;
  uint32_t negative_sat_low, negative_sat_high;
  uint32_t floating_sat_low, floating_sat_high;
  uint32_t dma_error;
} stage_e8_offset_result_t;

stage_e8_offset_result_t stage_e8_results[NUM_OFFSETS];
volatile uint32_t stage_e8_offset_index;

volatile uint32_t stage_e8_heartbeat;
volatile int stage_e8_running;
volatile uint32_t stage_e8_sample_index; /* within current offset, live */
volatile uint16_t stage_e8_floating_adc, stage_e8_positive_adc, stage_e8_negative_adc; /* live */
volatile uint32_t stage_e8_tmr1_ch4_event_count;
volatile uint32_t stage_e8_adc_conversion_count;
volatile uint32_t stage_e8_dma_fdt_count;
volatile uint32_t stage_e8_dma_error;
volatile uint32_t stage_e8_tim1_c4dt; /* live CCR4 */

void _init(void) {}
void _fini(void) {}

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

static void tim1_adc_trigger_config(uint16_t first_offset_ticks)
{
  tmr_output_config_type oc;

  tmr_output_default_para_init(&oc);
  oc.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  oc.oc_output_state = TRUE; /* required for cctrl.c4en -- Stage E3's fix */
  oc.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  oc.oc_idle_state = FALSE;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &oc);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, first_offset_ticks);
  tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_4, TRUE);
  tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG | TMR_HALL_SWTRIG);

  nvic_irq_enable(TMR1_CH_IRQn, 2, 0);
  tmr_interrupt_enable(TMR1, TMR_C4_INT, TRUE);

  stage_e8_tim1_c4dt = first_offset_ticks;
}

/* Direct, non-preloaded CCR4 write only -- see Stage E4's
 * update_adc_trigger_ccr4() rationale: does not touch CH1-3's CCPC-
 * buffered state, so it cannot disturb the frozen commutation. */
static void update_adc_trigger_offset(uint16_t ticks)
{
  TMR1->c4dt = ticks;
  stage_e8_tim1_c4dt = ticks;
}

void TMR1_CH_IRQHandler(void)
{
  if (tmr_flag_get(TMR1, TMR_C4_FLAG) != RESET) {
    tmr_flag_clear(TMR1, TMR_C4_FLAG);
    stage_e8_tmr1_ch4_event_count++;
  }
}

/* Called exactly once, at startup. Configures TIM1 CH1-3 for the
 * frozen sector and the ADC's ordinary sequence for its role mapping.
 * Never called again -- no commutation advance in this file. */
static void apply_fixed_step(uint16_t duty_ccr)
{
  const fixed_step_t *s = &fixed_step;
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

  adc_ordinary_channel_set(ADC1, (adc_channel_select_type)phase_adc_channel[s->floating], 1, ADC_SAMPLETIME_13_5);
  adc_ordinary_channel_set(ADC1, (adc_channel_select_type)phase_adc_channel[s->pos], 2, ADC_SAMPLETIME_13_5);
  adc_ordinary_channel_set(ADC1, (adc_channel_select_type)phase_adc_channel[s->neg], 3, ADC_SAMPLETIME_13_5);
}

static void stop_and_force_off(void)
{
  tmr_output_enable(TMR1, FALSE);
  tmr_counter_enable(TMR1, FALSE);
  gate_pins_force_off();
  stage_e8_running = 0;
}

static void reset_offset_accumulator(void)
{
  stage_e8_sample_index = 0;
}

static uint16_t pos_min, pos_max, neg_min, neg_max, flo_min, flo_max;
static uint32_t pos_sum, neg_sum, flo_sum;
static uint32_t pos_sat_lo, pos_sat_hi, neg_sat_lo, neg_sat_hi, flo_sat_lo, flo_sat_hi;

static void start_offset(uint32_t idx)
{
  reset_offset_accumulator();
  pos_min = flo_min = neg_min = 0xffff;
  pos_max = flo_max = neg_max = 0;
  pos_sum = neg_sum = flo_sum = 0;
  pos_sat_lo = pos_sat_hi = neg_sat_lo = neg_sat_hi = flo_sat_lo = flo_sat_hi = 0;
  uint16_t ticks = (uint16_t)(offset_half_us[idx] * (TIM1_TICKS_PER_US / 2u));
  update_adc_trigger_offset(ticks);
}

static void snapshot_offset(uint32_t idx)
{
  stage_e8_offset_result_t *r = &stage_e8_results[idx];
  r->offset_us = offset_half_us[idx] / 2.0f;
  r->offset_ticks = (uint16_t)(offset_half_us[idx] * (TIM1_TICKS_PER_US / 2u));
  r->sample_count = (uint16_t)stage_e8_sample_index;
  r->positive_min = pos_min; r->positive_max = pos_max;
  r->negative_min = neg_min; r->negative_max = neg_max;
  r->floating_min = flo_min; r->floating_max = flo_max;
  r->positive_sum = pos_sum; r->negative_sum = neg_sum; r->floating_sum = flo_sum;
  r->positive_sat_low = pos_sat_lo; r->positive_sat_high = pos_sat_hi;
  r->negative_sat_low = neg_sat_lo; r->negative_sat_high = neg_sat_hi;
  r->floating_sat_low = flo_sat_lo; r->floating_sat_high = flo_sat_hi;
  r->dma_error = stage_e8_dma_error;
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

  /* Real channel assignment done once by apply_fixed_step(). */
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
    stage_e8_adc_conversion_count++;
  }
}

void DMA1_Channel1_IRQHandler(void)
{
  if (dma_interrupt_flag_get(DMA1_FDT1_FLAG) != RESET) {
    dma_flag_clear(DMA1_FDT1_FLAG);
    stage_e8_dma_fdt_count++;

    uint16_t floating_v = adc_buf[0];
    uint16_t positive_v = adc_buf[1];
    uint16_t negative_v = adc_buf[2];

    stage_e8_floating_adc = floating_v;
    stage_e8_positive_adc = positive_v;
    stage_e8_negative_adc = negative_v;

    if (stage_e8_running) {
      if (positive_v < pos_min) pos_min = positive_v;
      if (positive_v > pos_max) pos_max = positive_v;
      pos_sum += positive_v;
      if (positive_v <= SAT_LOW_THRESHOLD) pos_sat_lo++;
      if (positive_v >= SAT_HIGH_THRESHOLD) pos_sat_hi++;

      if (negative_v < neg_min) neg_min = negative_v;
      if (negative_v > neg_max) neg_max = negative_v;
      neg_sum += negative_v;
      if (negative_v <= SAT_LOW_THRESHOLD) neg_sat_lo++;
      if (negative_v >= SAT_HIGH_THRESHOLD) neg_sat_hi++;

      if (floating_v < flo_min) flo_min = floating_v;
      if (floating_v > flo_max) flo_max = floating_v;
      flo_sum += floating_v;
      if (floating_v <= SAT_LOW_THRESHOLD) flo_sat_lo++;
      if (floating_v >= SAT_HIGH_THRESHOLD) flo_sat_hi++;

      stage_e8_sample_index++;
      if (stage_e8_sample_index >= SAMPLES_PER_OFFSET) {
        snapshot_offset(stage_e8_offset_index);
        stage_e8_offset_index++;
        if (stage_e8_offset_index >= NUM_OFFSETS) {
          stop_and_force_off();
        } else {
          start_offset(stage_e8_offset_index);
        }
      }
    }
  }
  if (dma_interrupt_flag_get(DMA1_DTERR1_FLAG) != RESET) {
    dma_flag_clear(DMA1_DTERR1_FLAG);
    stage_e8_dma_error++;
  }
}

int main(void)
{
  clock_config_96mhz();

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  gate_pins_force_off();

  adc_gpio_config();
  dma_config();

  tim1_init();
  g_duty_ccr = (uint16_t)((PWM_ARR + 1u) * OPEN_LOOP_DUTY_PERCENT / 100u);

  uint16_t first_ticks = (uint16_t)(offset_half_us[0] * (TIM1_TICKS_PER_US / 2u));
  tim1_adc_trigger_config(first_ticks);

  adc_config();
  dma_channel_enable(DMA1_CHANNEL1, TRUE);

  tim1_pins_to_af();
  apply_fixed_step(g_duty_ccr); /* once, forever -- no further commutation */
  tmr_output_enable(TMR1, TRUE);
  tmr_counter_enable(TMR1, TRUE);

  stage_e8_offset_index = 0;
  start_offset(0);
  stage_e8_running = 1;

  for (;;) {
    ++stage_e8_heartbeat;
  }
}
