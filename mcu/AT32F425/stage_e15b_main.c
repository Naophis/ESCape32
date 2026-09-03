/*
 * ESCape32 AT32F425 port -- Stage E15B: per-step (fixed, non-
 * commutating) ground truth of the intended positive/negative/
 * floating phase assignment against real ADC and real gate-pin
 * levels, for EACH of the 6 commutation steps individually.
 *
 * WHY: Stage E15's real-hardware result was that all 141 confirmed ZC
 * events across the whole run landed in steps 1 and 2 ONLY -- steps
 * 0/3/4/5 produced ZERO confirmed events despite armed_count showing
 * arming succeeded in essentially every sector. That rules out "not
 * enough BEMF amplitude" (arming already requires clearing the
 * deadband) and instead points at a STRUCTURAL problem specific to 4
 * of the 6 steps: something about those steps' positive/negative/
 * floating assignment, or the resulting gate drive, or the resulting
 * ADC readings, does not let diff cross to the opposite threshold the
 * way steps 1/2 do.
 *
 * This file isolates exactly that, one step at a time, with NO
 * commutation ramp and NO ZC logic at all -- motor should be
 * physically disconnected per instruction. TMR1 six-PWM/CH4-trigger/
 * duty/ADC-trigger-offset are copied verbatim from Stage E14/E15
 * (PWM_MODE_B, ADC_TRIGGER_OFFSET_TICKS=115 i.e. ~1.2us, 15% duty) --
 * none of that infrastructure is in question, only the per-step
 * pos/neg/floating table and its real electrical result.
 *
 * For each of the 6 steps in turn: apply_step(idx) is called ONCE
 * (exactly as Stage E14/E15's open-loop commutation would at that
 * point in the sequence, just without ever advancing to the next
 * step automatically), then 128 ADC scans are collected via the same
 * TMR1CH4-triggered 3-channel DMA sequence Stage E14/E15 use (fixed
 * physical ordinary sequence A/B/C, ranks 1/2/3 -- no per-step OSQ3
 * rewrite, same as Stage E14/E15). Per step this file records:
 *   - the INTENDED pos/neg/floating phase assignment (from steps[]),
 *   - physical A/B/C ADC min/max/mean over the 128 samples,
 *   - all SIX gate pins' GPIOx->idt level (PA7/PA8/PA9/PA10/PB0/PB1),
 *     sampled at the TMR1_CH4 compare-match instant (same technique
 *     Stage E10-E12 used), as a high-count out of the sample count.
 *
 * Expected result, if the intended assignment and the resulting drive
 * are both correct, for EVERY one of the 6 steps (not just some):
 *   - the ADC channel physically wired to the INTENDED positive phase
 *     reads ~2200-2500 during the ON window,
 *   - the ADC channel wired to the INTENDED negative phase reads ~0,
 *   - the ADC channel wired to the INTENDED floating phase reads
 *     ~20-50,
 *   - the positive phase's HS gate reads high or LS-gate low for
 *     nearly all 128 samples), the negative phase's LS gate reads
 *     high (HS low) for nearly all 128 samples, and the floating
 *     phase's HS and LS gates both read low for nearly all 128
 *     samples.
 * Per explicit instruction: no return to BEMF/ZC/closed-loop work
 * (Stage E16) until all 6 steps satisfy this.
 */

#include "clock_config.h"

#define PHASE_A_CHANNEL ADC_CHANNEL_0 /* PA0 */
#define PHASE_B_CHANNEL ADC_CHANNEL_4 /* PA4 */
#define PHASE_C_CHANNEL ADC_CHANNEL_5 /* PA5 */

#define PWM_ARR (96000000u / 24000u - 1u) /* 24kHz, unchanged */
#define DEAD_TIME_COUNT 53u /* unverified placeholder, see porting-plan Section 18-C/H */

#define OPEN_LOOP_DUTY_PERCENT 15u /* unchanged from Stage E14/E15 */
#define TIM1_TICKS_PER_US 96u
#define ADC_TRIGGER_OFFSET_TICKS 115u /* ~1.2us -- unchanged from Stage E10/E14/E15 */

#define COMMUTATION_BLANK_SCANS 2u /* discarded right after apply_step(), same as Stage E15 */
#define SAMPLES_PER_STEP 128u

#define SAT_LOW_THRESHOLD 50u
#define SAT_HIGH_THRESHOLD 4045u

typedef enum { PH_A = 0, PH_B = 1, PH_C = 2 } phase_t;

typedef struct {
  phase_t pos, neg, floating;
} step_t;

/* Identical to Stage E2-E15's drive pattern -- not touched here. */
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

static volatile uint16_t adc_buf[3]; /* [0]=A(PA0) [1]=B(PA4) [2]=C(PA5), fixed physical order */

typedef struct {
  int pos, neg, floating; /* intended phase indices, PH_A/B/C */
  uint16_t sample_count;
  uint16_t a_min, a_max; uint32_t a_sum;
  uint16_t b_min, b_max; uint32_t b_sum;
  uint16_t c_min, c_max; uint32_t c_sum;
  uint32_t hs_a_high, ls_a_high;
  uint32_t hs_b_high, ls_b_high;
  uint32_t hs_c_high, ls_c_high; /* all out of sample_count, at CH4 IRQ instant */
  uint32_t dma_error;
} stage_e15b_step_result_t;

stage_e15b_step_result_t stage_e15b_results[6];
volatile uint32_t stage_e15b_step_index;

volatile uint32_t stage_e15b_heartbeat;
volatile int stage_e15b_running;
volatile uint32_t stage_e15b_sample_index;
volatile uint32_t stage_e15b_dma_fdt_count;
volatile uint32_t stage_e15b_dma_error;
volatile uint32_t stage_e15b_adc_conversion_count;
volatile uint32_t stage_e15b_tmr1_ch4_event_count;

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

  /* PA7/PB0/PB1 corrected to MUX_2 per AT32F425 official GPIO IOMUX
   * table (user-confirmed) -- the previous MUX_3/MUX_1 values were
   * never independently verified and were wrong (Stage E15B/E15C). */
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
  oc.oc_output_state = TRUE; /* required for cctrl.c4en -- Stage E3's fix */
  oc.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  oc.oc_idle_state = FALSE;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &oc);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, ADC_TRIGGER_OFFSET_TICKS);
  tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_4, TRUE);
  tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG | TMR_HALL_SWTRIG);

  nvic_irq_enable(TMR1_CH_IRQn, 2, 0);
  tmr_interrupt_enable(TMR1, TMR_C4_INT, TRUE);
}

static uint32_t hs_a_high, ls_a_high, hs_b_high, ls_b_high, hs_c_high, ls_c_high;
static volatile uint32_t blank_scans_remaining;

void TMR1_CH_IRQHandler(void)
{
  if (tmr_flag_get(TMR1, TMR_C4_FLAG) != RESET) {
    tmr_flag_clear(TMR1, TMR_C4_FLAG);
    stage_e15b_tmr1_ch4_event_count++;

    if (stage_e15b_running && !blank_scans_remaining) {
      uint32_t idt_a = GPIOA->idt;
      uint32_t idt_b = GPIOB->idt;

      if (idt_a & (1u << 8)) hs_a_high++; /* PA8 = HS_A */
      if (idt_a & (1u << 7)) ls_a_high++; /* PA7 = LS_A */
      if (idt_a & (1u << 9)) hs_b_high++; /* PA9 = HS_B */
      if (idt_b & (1u << 0)) ls_b_high++; /* PB0 = LS_B */
      if (idt_a & (1u << 10)) hs_c_high++; /* PA10 = HS_C */
      if (idt_b & (1u << 1)) ls_c_high++; /* PB1 = LS_C */
    }
  }
}

/* Identical to Stage E14/E15's apply_step(), minus the ZC-state reset
 * (there is no ZC logic in this file at all). */
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
}

static void stop_and_force_off(void)
{
  tmr_output_enable(TMR1, FALSE);
  tmr_counter_enable(TMR1, FALSE);
  gate_pins_force_off();
  stage_e15b_running = 0;
}

static uint16_t a_min, a_max, b_min, b_max, c_min, c_max;
static uint32_t a_sum, b_sum, c_sum;

static void start_step(uint32_t idx)
{
  stage_e15b_sample_index = 0;
  a_min = b_min = c_min = 0xffff;
  a_max = b_max = c_max = 0;
  a_sum = b_sum = c_sum = 0;
  hs_a_high = ls_a_high = hs_b_high = ls_b_high = hs_c_high = ls_c_high = 0;
  blank_scans_remaining = COMMUTATION_BLANK_SCANS;

  apply_step((int)idx, g_duty_ccr);
}

static void snapshot_step(uint32_t idx)
{
  stage_e15b_step_result_t *r = &stage_e15b_results[idx];
  const step_t *s = &steps[idx];
  r->pos = (int)s->pos;
  r->neg = (int)s->neg;
  r->floating = (int)s->floating;
  r->sample_count = (uint16_t)stage_e15b_sample_index;
  r->a_min = a_min; r->a_max = a_max; r->a_sum = a_sum;
  r->b_min = b_min; r->b_max = b_max; r->b_sum = b_sum;
  r->c_min = c_min; r->c_max = c_max; r->c_sum = c_sum;
  r->hs_a_high = hs_a_high; r->ls_a_high = ls_a_high;
  r->hs_b_high = hs_b_high; r->ls_b_high = ls_b_high;
  r->hs_c_high = hs_c_high; r->ls_c_high = ls_c_high;
  r->dma_error = stage_e15b_dma_error;
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
    stage_e15b_adc_conversion_count++;
  }
}

void DMA1_Channel1_IRQHandler(void)
{
  if (dma_interrupt_flag_get(DMA1_FDT1_FLAG) != RESET) {
    dma_flag_clear(DMA1_FDT1_FLAG);
    stage_e15b_dma_fdt_count++;

    uint16_t a = adc_buf[0], b = adc_buf[1], c = adc_buf[2];

    if (stage_e15b_running) {
      if (blank_scans_remaining) {
        blank_scans_remaining--;
      } else {
        if (a < a_min) a_min = a;
        if (a > a_max) a_max = a;
        a_sum += a;
        if (b < b_min) b_min = b;
        if (b > b_max) b_max = b;
        b_sum += b;
        if (c < c_min) c_min = c;
        if (c > c_max) c_max = c;
        c_sum += c;

        stage_e15b_sample_index++;
        if (stage_e15b_sample_index >= SAMPLES_PER_STEP) {
          snapshot_step(stage_e15b_step_index);
          stage_e15b_step_index++;
          if (stage_e15b_step_index >= 6u) {
            stop_and_force_off();
          } else {
            start_step(stage_e15b_step_index);
          }
        }
      }
    }
  }
  if (dma_interrupt_flag_get(DMA1_DTERR1_FLAG) != RESET) {
    dma_flag_clear(DMA1_DTERR1_FLAG);
    stage_e15b_dma_error++;
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
  tim1_adc_trigger_config();

  adc_config();
  dma_channel_enable(DMA1_CHANNEL1, TRUE);

  tim1_pins_to_af();

  stage_e15b_step_index = 0;
  stage_e15b_running = 1;
  start_step(0);

  tmr_output_enable(TMR1, TRUE);
  tmr_counter_enable(TMR1, TRUE);

  for (;;) {
    ++stage_e15b_heartbeat;
  }
}
