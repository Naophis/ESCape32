/*
 * ESCape32 AT32F425 port -- Stage E12: verify the ADC ordinary-sequence
 * channel-select register (OSQ3) actually changes per pass, using a
 * halt-reconfigure-restart channel switch instead of Stage E11's
 * live rewrite while the ADC was still externally triggered.
 *
 * WHY: Stage E11 ran the same 3-pass single-channel sweep (PA0/PA4/PA5)
 * that this file also runs, but its real-hardware result was that all
 * three passes read nearly IDENTICAL waveforms at every offset (e.g.
 * offset=1us: PA0 mean~2887, PA4 mean~2946, PA5 mean~2938; offset=2us:
 * all three ~0-22) -- physically impossible for a fixed sector with
 * pos=A(HS-driven)/neg=B(LS-on)/floating=C, since A should sit near
 * VBUS/6 during HS-on while B sits near GND. That points at the ADC
 * channel switch between passes (Stage E11's switch_pass_channel(),
 * a bare adc_ordinary_channel_set() call issued from DMA-IRQ context
 * while the ADC was still running under external TMR1CH4 trigger)
 * simply not taking effect -- either OSQ3 never actually changed, or
 * it changed but the ADC kept converting a stale/latched channel
 * selection because the sequence register was rewritten out from
 * under a live triggered conversion.
 *
 * This file does NOT change PWM, commutation, or ADC-trigger timing
 * from Stage E10/E11 (TMR1 CH1-3 fixed sector, CH4 PWM_MODE_B trigger
 * mechanism, offset sweep list -- all untouched). It changes ONLY the
 * pass-switch procedure and adds direct register-level proof:
 *
 * (1) switch_pass_channel_safe() now HALTS the ADC/DMA pipeline before
 *     touching OSQ3: disables the TMR1CH4 external-trigger enable bit
 *     (adc_ordinary_conversion_trigger_set(..., FALSE)), disables the
 *     DMA channel, THEN calls adc_ordinary_channel_set() to rewrite
 *     OSQ3/SPT2 while nothing is converting, THEN resets the DMA
 *     transfer count (dma_data_number_set()) and clears any stale
 *     ADC_CCE flag, and only THEN re-enables DMA and the external
 *     trigger. TMR1 itself (CH1-3 PWM, CH4 trigger position/mode) is
 *     never touched by this procedure -- only the ADC's own trigger-
 *     enable bit and the DMA channel-enable bit are gated.
 *
 * (2) stage_e12_switch_diag[pass] records, immediately after each
 *     switch: expected_channel (what this file asked for), osq3_raw
 *     (ADC1->osq3 read back right after the write), and spt2_raw
 *     (ADC1->spt2 read back at the same time, since
 *     adc_ordinary_channel_set() also rewrites the per-channel
 *     sample-time field there). Per the vendor driver source
 *     (adc_ordinary_channel_set() in at32f425_adc.c), with
 *     adc_sequence=1 (< 7) the channel number is written into OSQ3's
 *     osn1 field, bits[4:0] -- so osq3_raw & 0x1F must read back
 *     exactly 0 (pass0/PA0/ADC_CHANNEL_0), 4 (pass1/PA4/ADC_CHANNEL_4),
 *     and 5 (pass2/PA5/ADC_CHANNEL_5) if the write itself is landing
 *     correctly. This is checked directly, not re-derived from
 *     conversion results.
 *
 * (3) adc_ordinary_channel_set()'s signature/argument order was
 *     re-read from the vendor header (not assumed): (adc_type *adc_x,
 *     adc_channel_select_type adc_channel, uint8_t adc_sequence,
 *     adc_sampletime_select_type adc_sampletime) -- this file's calls
 *     (adc_ordinary_channel_set(ADC1, pass_adc_channel[pass], 1,
 *     ADC_SAMPLETIME_13_5), same as Stage E11's) match that order,
 *     with adc_sequence=1 (1-based, as the driver source's "- 1" shift
 *     confirms) for the single-channel-length-1 sequence used here.
 *
 * Until stage_e12_switch_diag confirms OSQ3 actually reads back 0/4/5
 * across the three passes, no conclusion should be drawn about ZC or
 * A/B/C-to-pin/role mapping from the resulting waveform data -- this
 * file's job is only to prove (or disprove) that the channel select
 * itself is taking effect.
 */

#include "clock_config.h"

#define PWM_ARR (96000000u / 24000u - 1u) /* 24kHz, unchanged */
#define DEAD_TIME_COUNT 53u /* unverified placeholder, see porting-plan Section 18-C/H */

#define OPEN_LOOP_DUTY_PERCENT 15u /* unchanged from Stage E7-E10 */
#define TIM1_TICKS_PER_US 96u

#define SAT_LOW_THRESHOLD 50u
#define SAT_HIGH_THRESHOLD 4045u

/* offsets in us, *2 to keep integer ticks exact (96 ticks/us -> 48 ticks per 0.5us) */
static const uint16_t offset_half_us[] = {
  1, 2, 4, 6, 8, 10, 11, 12, 13, 14, 16, 20
};
#define NUM_OFFSETS (sizeof(offset_half_us) / sizeof(offset_half_us[0]))
#define SAMPLES_PER_OFFSET 128u

typedef enum { PH_A = 0, PH_B = 1, PH_C = 2 } phase_t;

typedef struct {
  phase_t pos, neg, floating;
} fixed_step_t;

/* Unchanged from Stage E10: A=positive(HS-driven), B=negative(LS-on), C=floating. */
static const fixed_step_t fixed_step = {PH_A, PH_B, PH_C};

static const tmr_channel_select_type phase_channel[3] = {
  TMR_SELECT_CHANNEL_1, TMR_SELECT_CHANNEL_2, TMR_SELECT_CHANNEL_3
};

/* One pass per physical BEMF pin -- NOT per PWM role. Pass order is
 * fixed: 0=PA0/ADC_CHANNEL_0, 1=PA4/ADC_CHANNEL_4, 2=PA5/ADC_CHANNEL_5.
 */
static const int16_t pass_adc_channel[3] = {
  (int16_t)ADC_CHANNEL_0, /* pass 0: PA0 */
  (int16_t)ADC_CHANNEL_4, /* pass 1: PA4 */
  (int16_t)ADC_CHANNEL_5, /* pass 2: PA5 */
};
#define NUM_PASSES 3

static volatile uint16_t adc_buf[1];

typedef struct {
  float offset_us;
  uint16_t offset_ticks;
  uint16_t sample_count;
  uint16_t v_min, v_max;
  uint32_t v_sum;
  uint32_t sat_low, sat_high;
  uint32_t dma_error;
  uint16_t ch4_cval_min, ch4_cval_max, ch4_cval_last;
  uint16_t dma_cval_min, dma_cval_max, dma_cval_last;
  uint32_t pa8_high_count; /* HSA level at CH4 IRQ instant, out of sample_count */
  uint32_t pa7_high_count; /* LSA(complementary) level, same instant -- see note below */
} stage_e12_offset_result_t;

/* [pass][offset]. pass 0/1/2 = PA0/PA4/PA5. */
stage_e12_offset_result_t stage_e12_results[NUM_PASSES][NUM_OFFSETS];
volatile uint32_t stage_e12_pass_index;
volatile uint32_t stage_e12_offset_index;

typedef struct {
  int16_t expected_channel;
  uint32_t osq3_raw;
  uint32_t spt2_raw;
} stage_e12_switch_diag_t;

/* Captured immediately after each channel switch (register readback,
 * not inferred from conversion data). */
stage_e12_switch_diag_t stage_e12_switch_diag[NUM_PASSES];

volatile uint32_t stage_e12_heartbeat;
volatile int stage_e12_running;
volatile uint32_t stage_e12_sample_index;
volatile uint32_t stage_e12_tmr1_ch4_event_count;
volatile uint32_t stage_e12_adc_conversion_count;
volatile uint32_t stage_e12_dma_fdt_count;
volatile uint32_t stage_e12_dma_error;

/* CH1 register dump, captured once (fixed sector, never changes). */
volatile uint32_t stage_e12_tmr1_c1dt;
volatile uint32_t stage_e12_tmr1_cm1;
volatile uint32_t stage_e12_tmr1_cctrl;
volatile uint32_t stage_e12_tmr1_brk;
volatile uint32_t stage_e12_tmr1_ctrl1;

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

/* Verbatim from Stage E10 -- CH4 trigger mechanism untouched. */
static void tim1_adc_trigger_config(uint16_t first_offset_ticks)
{
  tmr_output_config_type oc;

  tmr_output_default_para_init(&oc);
  oc.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B; /* Stage E10 fix, unchanged */
  oc.oc_output_state = TRUE; /* required for cctrl.c4en -- Stage E3's fix */
  oc.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  oc.oc_idle_state = FALSE;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &oc);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, first_offset_ticks);
  tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_4, TRUE);
  tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG | TMR_HALL_SWTRIG);

  nvic_irq_enable(TMR1_CH_IRQn, 2, 0);
  tmr_interrupt_enable(TMR1, TMR_C4_INT, TRUE);
}

static void update_adc_trigger_offset(uint16_t ticks)
{
  TMR1->c4dt = ticks;
}

static uint16_t ch4_cval_min, ch4_cval_max, ch4_cval_last;
static uint32_t pa8_high_count, pa7_high_count;

void TMR1_CH_IRQHandler(void)
{
  if (tmr_flag_get(TMR1, TMR_C4_FLAG) != RESET) {
    tmr_flag_clear(TMR1, TMR_C4_FLAG);
    stage_e12_tmr1_ch4_event_count++;

    if (stage_e12_running) {
      uint32_t cval = TMR1->cval;
      if ((uint16_t)cval < ch4_cval_min) ch4_cval_min = (uint16_t)cval;
      if ((uint16_t)cval > ch4_cval_max) ch4_cval_max = (uint16_t)cval;
      ch4_cval_last = (uint16_t)cval;

      /*
       * NOTE on pa7/pa8 diagnostic re-check (Stage E12 point 8): PA8 is
       * TMR1 CH1's pin (HSA, non-inverted PWM_MODE_A). PA7 is CH1's
       * COMPLEMENTARY pin (CH1C/LSA) -- with occ_polarity ACTIVE_HIGH
       * and hardware dead-time insertion, PA7 should be the logical
       * inverse of PA8 MINUS two DEAD_TIME_COUNT-wide gaps per cycle
       * (both low briefly around each edge), never simultaneously high
       * for any extended fraction of samples. If pa7_high_count and
       * pa8_high_count are both large at the SAME offset (as Stage
       * E10's raw log showed at a couple of offsets), that itself is a
       * signal worth cross-checking against dead-time expectations --
       * still measured here exactly as in Stage E10 (same registers,
       * same IRQ instant), no mechanism change, so this pass's A/B/C
       * pin data can be correlated against it directly.
       */
      uint32_t idt = GPIOA->idt;
      if (idt & (1u << 8)) pa8_high_count++; /* PA8 = HSA */
      if (idt & (1u << 7)) pa7_high_count++; /* PA7 = LSA complementary */
    }
  }
}

/* Called exactly once, at startup. Verbatim from Stage E10 -- fixed
 * sector (pos=A/CH1, neg=B/CH2, floating=C/CH3) untouched. */
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

  /* CH1 register dump, once (this sector never changes). */
  stage_e12_tmr1_c1dt = TMR1->c1dt;
  stage_e12_tmr1_cm1 = TMR1->cm1;
  stage_e12_tmr1_cctrl = TMR1->cctrl;
  stage_e12_tmr1_brk = TMR1->brk;
  stage_e12_tmr1_ctrl1 = TMR1->ctrl1;
}

static void stop_and_force_off(void)
{
  tmr_output_enable(TMR1, FALSE);
  tmr_counter_enable(TMR1, FALSE);
  gate_pins_force_off();
  stage_e12_running = 0;
}

static uint16_t v_min, v_max;
static uint32_t v_sum;
static uint32_t v_sat_lo, v_sat_hi;
static uint16_t dma_cval_min, dma_cval_max, dma_cval_last;

static void start_offset(uint32_t pass, uint32_t idx)
{
  stage_e12_sample_index = 0;
  v_min = 0xffff;
  v_max = 0;
  v_sum = 0;
  v_sat_lo = v_sat_hi = 0;
  ch4_cval_min = dma_cval_min = 0xffff;
  ch4_cval_max = dma_cval_max = 0;
  pa8_high_count = pa7_high_count = 0;
  uint16_t ticks = (uint16_t)(offset_half_us[idx] * (TIM1_TICKS_PER_US / 2u));
  update_adc_trigger_offset(ticks);
  (void)pass;
}

static void snapshot_offset(uint32_t pass, uint32_t idx)
{
  stage_e12_offset_result_t *r = &stage_e12_results[pass][idx];
  r->offset_us = offset_half_us[idx] / 2.0f;
  r->offset_ticks = (uint16_t)(offset_half_us[idx] * (TIM1_TICKS_PER_US / 2u));
  r->sample_count = (uint16_t)stage_e12_sample_index;
  r->v_min = v_min; r->v_max = v_max;
  r->v_sum = v_sum;
  r->sat_low = v_sat_lo; r->sat_high = v_sat_hi;
  r->dma_error = stage_e12_dma_error;
  r->ch4_cval_min = ch4_cval_min; r->ch4_cval_max = ch4_cval_max; r->ch4_cval_last = ch4_cval_last;
  r->dma_cval_min = dma_cval_min; r->dma_cval_max = dma_cval_max; r->dma_cval_last = dma_cval_last;
  r->pa8_high_count = pa8_high_count;
  r->pa7_high_count = pa7_high_count;
}

/*
 * Switch the single ADC ordinary channel to the next pass's pin.
 *
 * Stage E12 fix vs Stage E11: halt the ADC/DMA pipeline before touching
 * the sequence register, instead of rewriting OSQ3 while the ADC was
 * still live under external TMR1CH4 trigger. TMR1 itself (PWM, CH4
 * trigger position/mode) is NOT touched here -- only the ADC's own
 * external-trigger-enable bit and the DMA channel-enable bit are
 * gated off/on around the OSQ3/SPT2 rewrite.
 */
static void switch_pass_channel(uint32_t pass)
{
  /* 1) stop the external trigger from starting new conversions */
  adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_TMR1CH4, FALSE);
  /* 2) stop DMA from servicing any conversion still in flight */
  dma_channel_enable(DMA1_CHANNEL1, FALSE);

  /* 3) rewrite the sequence/sample-time registers while nothing is
   *    converting */
  adc_ordinary_channel_set(ADC1, (adc_channel_select_type)pass_adc_channel[pass], 1, ADC_SAMPLETIME_13_5);

  /* 4) register-level readback, immediately, before anything else can
   *    touch these registers */
  stage_e12_switch_diag[pass].expected_channel = pass_adc_channel[pass];
  stage_e12_switch_diag[pass].osq3_raw = ADC1->osq3;
  stage_e12_switch_diag[pass].spt2_raw = ADC1->spt2;

  /* 5) reset the DMA transfer count and clear any stale conversion
   *    flag left over from before the halt */
  dma_data_number_set(DMA1_CHANNEL1, 1);
  adc_flag_clear(ADC1, ADC_CCE_FLAG);

  /* 6) resume: DMA first, then the external trigger */
  dma_channel_enable(DMA1_CHANNEL1, TRUE);
  adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_TMR1CH4, TRUE);
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
  d.buffer_size = 1;
  d.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  d.memory_base_addr = (uint32_t)adc_buf;
  d.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  d.memory_inc_enable = FALSE;
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
  b.ordinary_channel_length = 1;
  adc_base_config(ADC1, &b);

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
    stage_e12_adc_conversion_count++;
  }
}

void DMA1_Channel1_IRQHandler(void)
{
  if (dma_interrupt_flag_get(DMA1_FDT1_FLAG) != RESET) {
    dma_flag_clear(DMA1_FDT1_FLAG);
    stage_e12_dma_fdt_count++;

    uint32_t cval = TMR1->cval;
    uint16_t v = adc_buf[0];

    if (stage_e12_running) {
      if ((uint16_t)cval < dma_cval_min) dma_cval_min = (uint16_t)cval;
      if ((uint16_t)cval > dma_cval_max) dma_cval_max = (uint16_t)cval;
      dma_cval_last = (uint16_t)cval;

      if (v < v_min) v_min = v;
      if (v > v_max) v_max = v;
      v_sum += v;
      if (v <= SAT_LOW_THRESHOLD) v_sat_lo++;
      if (v >= SAT_HIGH_THRESHOLD) v_sat_hi++;

      stage_e12_sample_index++;
      if (stage_e12_sample_index >= SAMPLES_PER_OFFSET) {
        snapshot_offset(stage_e12_pass_index, stage_e12_offset_index);
        stage_e12_offset_index++;
        if (stage_e12_offset_index >= NUM_OFFSETS) {
          stage_e12_pass_index++;
          if (stage_e12_pass_index >= NUM_PASSES) {
            stop_and_force_off();
          } else {
            stage_e12_offset_index = 0;
            switch_pass_channel(stage_e12_pass_index);
            start_offset(stage_e12_pass_index, 0);
          }
        } else {
          start_offset(stage_e12_pass_index, stage_e12_offset_index);
        }
      }
    }
  }
  if (dma_interrupt_flag_get(DMA1_DTERR1_FLAG) != RESET) {
    dma_flag_clear(DMA1_DTERR1_FLAG);
    stage_e12_dma_error++;
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
  switch_pass_channel(0); /* pass 0 = PA0/ADC_CHANNEL_0 */
  dma_channel_enable(DMA1_CHANNEL1, TRUE);

  tim1_pins_to_af();
  apply_fixed_step(g_duty_ccr); /* once, forever -- no further commutation */
  tmr_output_enable(TMR1, TRUE);
  tmr_counter_enable(TMR1, TRUE);

  stage_e12_pass_index = 0;
  stage_e12_offset_index = 0;
  start_offset(0, 0);
  stage_e12_running = 1;

  for (;;) {
    ++stage_e12_heartbeat;
  }
}
