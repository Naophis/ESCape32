/*
 * ESCape32 AT32F425 port -- Stage E10: re-run of Stage E9's ground-
 * truth sweep with ONE change: TMR1_CH4 (the ADC-trigger-only channel)
 * switched from PWM_MODE_A to PWM_MODE_B. CH1/CH2/CH3 (real PWM
 * outputs), the 6-step table, ADC sequence, DMA setup, and ZC logic
 * are all untouched -- see tim1_adc_trigger_config() for the one
 * changed line and its rationale.
 *
 * WHY: Stage E9 conclusively separated the two hypotheses it was built
 * to test: ch4_cval (read at the CC4 compare-match IRQ) tracked
 * offset_ticks correctly at every offset (e.g. 15us->cval 1507 vs
 * offset_ticks 1440; 40us->cval 3907 vs offset_ticks 3840) -- so CCR4
 * itself was updating correctly. But dma_cval (read at DMA transfer-
 * complete) stayed pinned at ~417 for EVERY offset from 1us to 40us.
 * That is the signature of the ADC's actual external-trigger event
 * being tied to CH4's RISING edge, not the CC4 compare-match flag: in
 * up-count PWM_MODE_A, the rising edge is fixed at CNT=0 every single
 * cycle (only the falling edge tracks CCR4) -- so the ADC was always
 * triggering at a fixed ~0-tick point regardless of the programmed
 * offset, while the CPU-visible CC4 interrupt (which DOES fire at the
 * CCR4 match / falling edge) gave the false impression the offset was
 * being honored. Root cause of every "OFF-time"/"ON-time" sampling
 * stage's inconsistent readings back through Stage E3.
 *
 * This file is the re-test: with CH4 in PWM_MODE_B, its rising edge
 * should now occur AT CCR4 instead of always at CNT=0, so dma_cval
 * should finally move with offset (dma_cval increasing by roughly
 * 96 ticks per +1us step, plus the fixed ADC-conversion+latency
 * offset), instead of staying pinned near 417 as it did in Stage E9.
 *
 * Otherwise identical to Stage E9/E8 -- adds the same six diagnostics
 * to distinguish:
 *   (A) CH4/ADC sampling position is correct, but the actual HS-on
 *       duty is ~85% instead of the configured 15% (a PWM_MODE_A /
 *       polarity / complementary-polarity definition problem), vs.
 *   (B) the real PWM output is fine, but TMR1_CH4's trigger position
 *       (or the ADC sampling instant it produces) isn't landing where
 *       the CCR4 arithmetic assumes -- a trigger-timing problem.
 *
 * Stage E8's data is the reason this file exists: positive_adc's mean
 * sat at ~2250 (out of a ~2700 high plateau) essentially FLAT across
 * every offset from 1us to 40us -- and 2250/2700 = 83%, suspiciously
 * close to 100%-15%=85%. That is exactly consistent with hypothesis
 * (A). But it is equally consistent with (B) if the true trigger
 * instant is not actually moving with CCR4 for some reason (e.g. a
 * stale/uncommitted CCR4, or sampling landing at a fixed point
 * elsewhere in the cycle regardless of the programmed offset) -- pure
 * code review already re-confirmed the driver source maps
 * PWM_MODE_A(0x06)+ACTIVE_HIGH(polarity bit=0, non-inverted) the same
 * way STM32's OC1M=110/CC1P=0 does (active while CNT<CCR), which
 * argues for (B), but that re-derivation is exactly the kind of
 * "should be right" reasoning the user is right not to trust blindly
 * -- see instruction 6. This file adds the concrete measurements to
 * decide instead of re-deriving further.
 *
 * ADDED (nothing else changed from Stage E8):
 *
 * (1)(2) TMR1->cval captured in BOTH interrupt handlers now:
 *     - TMR1_CH_IRQHandler (the CH4 compare-match event itself) reads
 *       TMR1->cval immediately -- ch4_cval_min/max/last per offset.
 *       If CH4 is truly firing where CCR4 says, this should track
 *       offset_ticks closely (modulo a few cycles of IRQ latency).
 *     - DMA1_Channel1_IRQHandler (transfer-complete, ~3.25us after the
 *       ADC started converting) ALSO reads TMR1->cval -- dma_cval_min/
 *       max/last per offset. (dma_cval - ch4_cval) should be roughly
 *       the ADC's own conversion latency in ticks (~3.25us*96=312
 *       ticks) if the whole chain is behaving as designed.
 *
 * (3) TMR1 CH1's real registers, captured ONCE (fixed sector, these
 *     never change across the sweep): stage_e10_tmr1_c1dt (CCR1),
 *     stage_e10_tmr1_cm1 (CH1/CH2 mode raw), stage_e10_tmr1_cctrl
 *     (CH1 enable/polarity raw, same register CH2-4 also live in),
 *     stage_e10_tmr1_brk (dead-time/MOE), stage_e10_tmr1_ctrl1 (CEN/
 *     direction).
 *
 * (4) Source-level derivation (not re-asserted as measured fact):
 *     at32f425_tmr.c's tmr_output_channel_config(), read directly
 *     (not from memory) for this file:
 *       channel_index = (oc_polarity << chx_offset); cctrl |= channel_index;
 *     with chx_offset=(tmr_channel*2)+1 -- for CH1 (tmr_channel=0)
 *     that is cctrl bit 1 = c1p. TMR_OUTPUT_ACTIVE_HIGH=0x00, so
 *     oc_polarity=ACTIVE_HIGH writes c1p=0 (non-inverted: OC1 pin =
 *     OC1REF directly, no inversion). Separately,
 *       cm1_output_bit.c1octrl = oc_mode;
 *     with TMR_OUTPUT_CONTROL_PWM_MODE_A=0x06 written verbatim into
 *     c1octrl. This numeric encoding (000=frozen,...,100=force low,
 *     101=force high,110=PWM mode1,111=PWM mode2) matches the
 *     STM32-compatible OC1M field this port's TIM1 was already
 *     confirmed bit-for-bit compatible with (porting-plan Section 18),
 *     where OC1M=110 means "OC1REF high while CNT<CCR1, low
 *     otherwise". Combined with c1p=0 (non-inverted): the
 *     CONFIGURATION says PA8 (HSA, CH1's pin) should read HIGH for
 *     CNT in [0,ccr) and LOW for CNT in [ccr,ARR]. This is a
 *     configuration-derived claim; (5) below checks it against the
 *     actual pad level.
 *
 * (5) GPIOA->idt sampled inside TMR1_CH_IRQHandler (i.e. at the CH4
 *     compare-match instant, +~150-300ns of IRQ entry/synchronizer
 *     latency), masked to bit8 (PA8/HSA) and bit7 (PA7/LSA) --
 *     pa8_high_count/pa7_high_count per offset, out of 128 samples.
 *     PA7/PA8 are configured GPIO_MODE_MUX (not GPIO_MODE_ANALOG), so
 *     their digital input path stays enabled and IDT reflects the
 *     real pad logic level independent of which peripheral currently
 *     drives the pin -- standard ARM/AT32 GPIO behavior (the input
 *     buffer is only disabled in analog mode to prevent leakage; MUX
 *     mode does not disable it). This is the most direct ground truth
 *     available without a scope.
 *
 * (6) The A-vs-B question is answered by cross-referencing (1)(2)(5):
 *     if pa8_high_count/128 tracks the SAME ~83% figure regardless of
 *     offset while ch4_cval tracks offset_ticks correctly, that is
 *     hypothesis (A) (real duty wrong). If ch4_cval does NOT track
 *     offset_ticks (stays fixed, or jumps unpredictably), that is
 *     hypothesis (B) (trigger timing broken) regardless of what PA8
 *     shows.
 *
 * Frozen-sector / non-spinning E8 methodology is otherwise unchanged;
 * ZC/filter/commutation logic is not present in this file at all.
 */

#include "clock_config.h"

static const int16_t phase_adc_channel[3] = {
  (int16_t)ADC_CHANNEL_0, /* PA0 = A */
  (int16_t)ADC_CHANNEL_4, /* PA4 = B */
  (int16_t)ADC_CHANNEL_5, /* PA5 = C */
};

#define PWM_ARR (96000000u / 24000u - 1u) /* 24kHz, unchanged */
#define DEAD_TIME_COUNT 53u /* unverified placeholder, see porting-plan Section 18-C/H */

#define OPEN_LOOP_DUTY_PERCENT 15u /* unchanged from Stage E7/E8 */
#define TIM1_TICKS_PER_US 96u

#define SAT_LOW_THRESHOLD 50u
#define SAT_HIGH_THRESHOLD 4045u

static const uint16_t offset_half_us[] = {
  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 16, 20, 30, 40, 60, 80
};
#define NUM_OFFSETS (sizeof(offset_half_us) / sizeof(offset_half_us[0]))
#define SAMPLES_PER_OFFSET 128u

typedef enum { PH_A = 0, PH_B = 1, PH_C = 2 } phase_t;

typedef struct {
  phase_t pos, neg, floating;
} fixed_step_t;

static const fixed_step_t fixed_step = {PH_A, PH_B, PH_C};

static const tmr_channel_select_type phase_channel[3] = {
  TMR_SELECT_CHANNEL_1, TMR_SELECT_CHANNEL_2, TMR_SELECT_CHANNEL_3
};

static volatile uint16_t adc_buf[3]; /* [0]=floating, [1]=positive, [2]=negative */

typedef struct {
  float offset_us;
  uint16_t offset_ticks;
  uint16_t sample_count;
  uint16_t positive_min, positive_max;
  uint16_t negative_min, negative_max;
  uint16_t floating_min, floating_max;
  uint32_t positive_sum, negative_sum, floating_sum;
  uint32_t positive_sat_low, positive_sat_high;
  uint32_t negative_sat_low, negative_sat_high;
  uint32_t floating_sat_low, floating_sat_high;
  uint32_t dma_error;
  /* --- new in Stage E10 --- */
  uint16_t ch4_cval_min, ch4_cval_max, ch4_cval_last;
  uint16_t dma_cval_min, dma_cval_max, dma_cval_last;
  uint32_t pa8_high_count; /* out of sample_count, at CH4 IRQ instant */
  uint32_t pa7_high_count;
} stage_e10_offset_result_t;

stage_e10_offset_result_t stage_e10_results[NUM_OFFSETS];
volatile uint32_t stage_e10_offset_index;

volatile uint32_t stage_e10_heartbeat;
volatile int stage_e10_running;
volatile uint32_t stage_e10_sample_index;
volatile uint16_t stage_e10_floating_adc, stage_e10_positive_adc, stage_e10_negative_adc;
volatile uint32_t stage_e10_tmr1_ch4_event_count;
volatile uint32_t stage_e10_adc_conversion_count;
volatile uint32_t stage_e10_dma_fdt_count;
volatile uint32_t stage_e10_dma_error;
volatile uint32_t stage_e10_tim1_c4dt;

/* CH1 register dump, captured once (fixed sector). */
volatile uint32_t stage_e10_tmr1_c1dt;
volatile uint32_t stage_e10_tmr1_cm1;
volatile uint32_t stage_e10_tmr1_cctrl;
volatile uint32_t stage_e10_tmr1_brk;
volatile uint32_t stage_e10_tmr1_ctrl1;

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
  /*
   * STAGE E10 FIX (CH4 ONLY -- CH1/CH2/CH3 in apply_fixed_step() below
   * are untouched): PWM_MODE_A -> PWM_MODE_B, per the user's diagnosis
   * of Stage E9's data. In up-count PWM_MODE_A (what every earlier
   * ADC-trigger stage used), CVAL<CCR4 is HIGH and CVAL>=CCR4 is LOW --
   * i.e. the RISING edge is fixed at CNT=0 every cycle, and only the
   * FALLING edge moves with CCR4. AT32F425's ADC external trigger from
   * a timer channel fires on that channel's RISING edge -- so every
   * previous sweep was actually triggering at a fixed CNT~=0 point
   * regardless of CCR4, while the CC4 compare-match INTERRUPT (which
   * fires on the CCR4 match itself, i.e. the falling edge) correctly
   * tracked the programmed offset -- exactly explaining Stage E9's
   * data: ch4_cval tracked offset_ticks, but dma_cval stayed pinned
   * near 417 (~0 + ADC conversion + latency) for every offset.
   * PWM_MODE_B inverts this (CVAL<CCR4 LOW, CVAL>=CCR4 HIGH), so the
   * rising edge -- and therefore the ADC's actual trigger instant --
   * now occurs AT CCR4, letting this sweep actually move it.
   */
  oc.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
  oc.oc_output_state = TRUE; /* required for cctrl.c4en -- Stage E3's fix */
  oc.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  oc.oc_idle_state = FALSE;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &oc);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, first_offset_ticks);
  tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_4, TRUE);
  tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG | TMR_HALL_SWTRIG);

  nvic_irq_enable(TMR1_CH_IRQn, 2, 0);
  tmr_interrupt_enable(TMR1, TMR_C4_INT, TRUE);

  stage_e10_tim1_c4dt = first_offset_ticks;
}

static void update_adc_trigger_offset(uint16_t ticks)
{
  TMR1->c4dt = ticks;
  stage_e10_tim1_c4dt = ticks;
}

static uint16_t ch4_cval_min, ch4_cval_max, ch4_cval_last;
static uint32_t pa8_high_count, pa7_high_count;

void TMR1_CH_IRQHandler(void)
{
  if (tmr_flag_get(TMR1, TMR_C4_FLAG) != RESET) {
    tmr_flag_clear(TMR1, TMR_C4_FLAG);
    stage_e10_tmr1_ch4_event_count++;

    if (stage_e10_running) {
      uint32_t cval = TMR1->cval;
      if ((uint16_t)cval < ch4_cval_min) ch4_cval_min = (uint16_t)cval;
      if ((uint16_t)cval > ch4_cval_max) ch4_cval_max = (uint16_t)cval;
      ch4_cval_last = (uint16_t)cval;

      uint32_t idt = GPIOA->idt;
      if (idt & (1u << 8)) pa8_high_count++; /* PA8 = HSA */
      if (idt & (1u << 7)) pa7_high_count++; /* PA7 = LSA */
    }
  }
}

/* Called exactly once, at startup. */
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

  /* CH1 register dump, once (this sector never changes). */
  stage_e10_tmr1_c1dt = TMR1->c1dt;
  stage_e10_tmr1_cm1 = TMR1->cm1;
  stage_e10_tmr1_cctrl = TMR1->cctrl;
  stage_e10_tmr1_brk = TMR1->brk;
  stage_e10_tmr1_ctrl1 = TMR1->ctrl1;
}

static void stop_and_force_off(void)
{
  tmr_output_enable(TMR1, FALSE);
  tmr_counter_enable(TMR1, FALSE);
  gate_pins_force_off();
  stage_e10_running = 0;
}

static void reset_offset_accumulator(void)
{
  stage_e10_sample_index = 0;
}

static uint16_t pos_min, pos_max, neg_min, neg_max, flo_min, flo_max;
static uint32_t pos_sum, neg_sum, flo_sum;
static uint32_t pos_sat_lo, pos_sat_hi, neg_sat_lo, neg_sat_hi, flo_sat_lo, flo_sat_hi;
static uint16_t dma_cval_min, dma_cval_max, dma_cval_last;

static void start_offset(uint32_t idx)
{
  reset_offset_accumulator();
  pos_min = flo_min = neg_min = 0xffff;
  pos_max = flo_max = neg_max = 0;
  pos_sum = neg_sum = flo_sum = 0;
  pos_sat_lo = pos_sat_hi = neg_sat_lo = neg_sat_hi = flo_sat_lo = flo_sat_hi = 0;
  ch4_cval_min = dma_cval_min = 0xffff;
  ch4_cval_max = dma_cval_max = 0;
  pa8_high_count = pa7_high_count = 0;
  uint16_t ticks = (uint16_t)(offset_half_us[idx] * (TIM1_TICKS_PER_US / 2u));
  update_adc_trigger_offset(ticks);
}

static void snapshot_offset(uint32_t idx)
{
  stage_e10_offset_result_t *r = &stage_e10_results[idx];
  r->offset_us = offset_half_us[idx] / 2.0f;
  r->offset_ticks = (uint16_t)(offset_half_us[idx] * (TIM1_TICKS_PER_US / 2u));
  r->sample_count = (uint16_t)stage_e10_sample_index;
  r->positive_min = pos_min; r->positive_max = pos_max;
  r->negative_min = neg_min; r->negative_max = neg_max;
  r->floating_min = flo_min; r->floating_max = flo_max;
  r->positive_sum = pos_sum; r->negative_sum = neg_sum; r->floating_sum = flo_sum;
  r->positive_sat_low = pos_sat_lo; r->positive_sat_high = pos_sat_hi;
  r->negative_sat_low = neg_sat_lo; r->negative_sat_high = neg_sat_hi;
  r->floating_sat_low = flo_sat_lo; r->floating_sat_high = flo_sat_hi;
  r->dma_error = stage_e10_dma_error;
  r->ch4_cval_min = ch4_cval_min; r->ch4_cval_max = ch4_cval_max; r->ch4_cval_last = ch4_cval_last;
  r->dma_cval_min = dma_cval_min; r->dma_cval_max = dma_cval_max; r->dma_cval_last = dma_cval_last;
  r->pa8_high_count = pa8_high_count;
  r->pa7_high_count = pa7_high_count;
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
    stage_e10_adc_conversion_count++;
  }
}

void DMA1_Channel1_IRQHandler(void)
{
  if (dma_interrupt_flag_get(DMA1_FDT1_FLAG) != RESET) {
    dma_flag_clear(DMA1_FDT1_FLAG);
    stage_e10_dma_fdt_count++;

    uint32_t cval = TMR1->cval; /* as close to the transfer-complete instant as possible */

    uint16_t floating_v = adc_buf[0];
    uint16_t positive_v = adc_buf[1];
    uint16_t negative_v = adc_buf[2];

    stage_e10_floating_adc = floating_v;
    stage_e10_positive_adc = positive_v;
    stage_e10_negative_adc = negative_v;

    if (stage_e10_running) {
      if ((uint16_t)cval < dma_cval_min) dma_cval_min = (uint16_t)cval;
      if ((uint16_t)cval > dma_cval_max) dma_cval_max = (uint16_t)cval;
      dma_cval_last = (uint16_t)cval;

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

      stage_e10_sample_index++;
      if (stage_e10_sample_index >= SAMPLES_PER_OFFSET) {
        snapshot_offset(stage_e10_offset_index);
        stage_e10_offset_index++;
        if (stage_e10_offset_index >= NUM_OFFSETS) {
          stop_and_force_off();
        } else {
          start_offset(stage_e10_offset_index);
        }
      }
    }
  }
  if (dma_interrupt_flag_get(DMA1_DTERR1_FLAG) != RESET) {
    dma_flag_clear(DMA1_DTERR1_FLAG);
    stage_e10_dma_error++;
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

  stage_e10_offset_index = 0;
  start_offset(0);
  stage_e10_running = 1;

  for (;;) {
    ++stage_e10_heartbeat;
  }
}
