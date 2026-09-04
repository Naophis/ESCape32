/*
 * ESCape32 AT32F425 port -- Stage D "active" sub-stage (PWM-synchronized
 * 3-phase ADC + software virtual-neutral + zero-cross candidate
 * detection), for use ONCE the passive BEMF test in stage_d_main.c has
 * been validated and MP6540HA is about to start actually driving the
 * motor.
 *
 * NOT currently the active Stage D target (see mcu/AT32F425/flash.sh --
 * it builds stage_d_main.c, the passive test, by default). This file
 * is kept here so the PWM-trigger design work isn't thrown away, but
 * per the user's explicit sequencing it comes AFTER the passive test
 * passes, not before. Build/flash it explicitly if you want it:
 *   cmake --build build --target MOUSEF425_STAGE_D_ACTIVE
 *
 * MP6540HA still unpowered / nSLEEP low when bringing this up: TIM1
 * runs a static 50% test duty (same as Stage B) purely to give
 * TMR1_CH4 something periodic to anchor the ADC trigger to. No
 * commutation, no BEMF processing tied into the real 6-step sequence
 * yet -- that integration (replacing compctl(), wiring into shared
 * src/main.c via the bemf_zero_cross() abstraction, dynamic per-duty
 * CCR4 retargeting) is separate follow-up work, not part of this file.
 *
 * What THIS file proves:
 *   1. TMR1_CH4 (internal only -- no GPIO, not brought out on this
 *      QFN32 package's pin table anyway) can trigger ADC1's ordinary
 *      sequence via ADC12_ORDINARY_TRIG_TMR1CH4, landing the 3-channel
 *      VA/VB/VC scan at a chosen, edge-free point in the PWM cycle
 *      instead of free-running.
 *   2. The software "virtual neutral" = (VA+VB+VC)/3 and per-phase
 *      differences (VA-neutral etc.) compute correctly and self-
 *      compensate for the ~110-code common ADC offset the user found
 *      during Stage C bring-up (per their explicit instruction: judge
 *      by difference-from-reference, not raw absolute code).
 *   3. A consecutive-sample + confirm-count filter on those
 *      differences behaves sanely (exposed as a per-phase crossing
 *      counter).
 *
 * ADC pin/channel mapping (user-confirmed real schematic, Stage C):
 *   PA0 = VA = ADC1_IN0 (ADC_CHANNEL_0)
 *   PA4 = VB = ADC1_IN4 (ADC_CHANNEL_4)
 *   PA5 = VC = ADC1_IN5 (ADC_CHANNEL_5)
 *   No dedicated virtual-neutral pin -- synthesized in software.
 *
 * ADC trigger point: TMR1_CH4 in PWM_MODE_A, output disabled
 * (oc_output_state=FALSE -- internal event only), CCR4 = midpoint of
 * the OFF-time window [ccr, arr] for the current (static, 50%) test
 * duty. Confirmed against project/at_start_f425/examples/adc/
 * tmr_trigger_automatic_preempted (same tmr_output_channel_config()+
 * ADC12_ORDINARY_TRIG_TMR1CH4 combination, just at PWM-rate instead of
 * their 1Hz demo).
 *
 * IMPORTANT OPEN ISSUE (not resolved by this file, flagged for the
 * next iteration once real waveform/RPM data exists): a single ADC
 * trigger per PWM period tops out at the PWM frequency (24kHz here,
 * ESCape32's variable-frequency ramp goes up to freq_max at high
 * eRPM). At the ~600k eRPM implied by a 100k mechanical RPM / 12-pole
 * target, a 16.67us electrical sector needs several samples to
 * usefully apply the consecutive-sample filter this port's own
 * timing-budget analysis called for; one sample per (possibly still
 * only tens-of-kHz) PWM period may not be enough. Two ways to close
 * that gap later: (a) add a second internal trigger channel firing in
 * the ON-time quiet window too (roughly doubling the effective rate),
 * or (b) move to continuous/free-running sampling with a software
 * blanking window around the known switching edges instead of a
 * single hardware-gated trigger point. Also needs dynamic CCR4
 * retargeting whenever duty (ccr) changes, once wired into the real
 * control loop -- this file only handles the fixed 50% test case.
 */

#include "clock_config.h"

#define PHASE_A_CHANNEL ADC_CHANNEL_0 /* PA0 */
#define PHASE_B_CHANNEL ADC_CHANNEL_4 /* PA4 */
#define PHASE_C_CHANNEL ADC_CHANNEL_5 /* PA5 */

#define PWM_ARR (96000000u / 24000u - 1u) /* 24kHz, same as Stage B */
#define DEAD_TIME_COUNT 53u

#define ZC_CONFIRM_COUNT 3 /* consecutive scans required to confirm a crossing */

static volatile uint16_t adc_buf[3]; /* DMA target: VA, VB, VC (sequence 1,2,3) */

volatile uint32_t stage_d_heartbeat;
volatile uint32_t stage_d_scan_count;
volatile uint32_t stage_d_dma_error;
volatile uint16_t stage_d_adc_a, stage_d_adc_b, stage_d_adc_c;
volatile int32_t stage_d_neutral;
volatile int32_t stage_d_diff_a, stage_d_diff_b, stage_d_diff_c;
volatile uint32_t stage_d_zc_count_a, stage_d_zc_count_b, stage_d_zc_count_c;
volatile uint32_t stage_d_tmr1_pr;
volatile uint32_t stage_d_tmr1_c4dt; /* CCR4, the ADC trigger point */

void _init(void) {}
void _fini(void) {}

typedef struct {
  int candidate_sign;
  int confirmed_sign;
  int confirm_run;
} zc_filter_t;

static zc_filter_t zc_a, zc_b, zc_c;

static void zc_filter_update(zc_filter_t *f, int32_t diff, volatile uint32_t *count)
{
  int sign = (diff >= 0) ? 1 : -1;
  if (sign != f->candidate_sign) {
    f->candidate_sign = sign;
    f->confirm_run = 1;
  } else if (f->confirm_run < ZC_CONFIRM_COUNT) {
    f->confirm_run++;
  }
  if (f->confirm_run == ZC_CONFIRM_COUNT && f->confirmed_sign != sign) {
    f->confirmed_sign = sign;
    (*count)++;
  }
}

static void tim1_pwm_gpio_config(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;

  gpio_init_struct.gpio_pins = GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10;
  gpio_init(GPIOA, &gpio_init_struct);
  gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1;
  gpio_init(GPIOB, &gpio_init_struct);

  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE8, GPIO_MUX_2);  /* CH1  */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_3);  /* CH1C */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_2);  /* CH2  */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE0, GPIO_MUX_1);  /* CH2C */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_2); /* CH3  */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE1, GPIO_MUX_2);  /* CH3C */
}

static void tim1_pwm_config(void)
{
  tmr_output_config_type oc;
  tmr_brkdt_config_type brkdt;
  uint16_t ccr = (PWM_ARR + 1u) / 2u;      /* 50% test duty, same as Stage B */
  uint16_t ccr4 = (uint16_t)((ccr + PWM_ARR) / 2u); /* midpoint of the OFF-time window */

  tmr_base_init(TMR1, (uint16_t)PWM_ARR, 0);
  tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);

  tmr_output_default_para_init(&oc);
  oc.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  oc.oc_output_state = TRUE;
  oc.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  oc.oc_idle_state = FALSE;
  oc.occ_output_state = TRUE;
  oc.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  oc.occ_idle_state = FALSE;

  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &oc);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, ccr);
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_2, &oc);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, ccr);
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &oc);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, ccr);

  /* CH4: internal ADC-trigger anchor only. Not brought out on this
   * QFN32 package's TMR1 pin set (only CH1/CH1C/CH2/CH2C/CH3/CH3C are),
   * and oc_output_state=FALSE regardless so nothing is driven even if
   * it were. */
  tmr_output_default_para_init(&oc);
  oc.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  oc.oc_output_state = FALSE;
  oc.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  oc.oc_idle_state = FALSE;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &oc);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, ccr4);

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
  tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG | TMR_HALL_SWTRIG);

  tmr_output_enable(TMR1, TRUE);
  tmr_counter_enable(TMR1, TRUE);

  stage_d_tmr1_pr = TMR1->pr;
  stage_d_tmr1_c4dt = TMR1->c4dt;
}

static void adc_gpio_config(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_4 | GPIO_PINS_5;
  gpio_init(GPIOA, &gpio_init_struct);
}

static void dma_config(void)
{
  dma_init_type dma_init_struct;

  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
  nvic_irq_enable(DMA1_Channel1_IRQn, 0, 0);
  dma_reset(DMA1_CHANNEL1);

  dma_flexible_config(DMA1, FLEX_CHANNEL1, DMA_FLEXIBLE_ADC1);

  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = 3;
  dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  dma_init_struct.memory_base_addr = (uint32_t)adc_buf;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)&(ADC1->odt);
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_HIGH;
  dma_init_struct.loop_mode_enable = TRUE;
  dma_init(DMA1_CHANNEL1, &dma_init_struct);

  dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_DTERR_INT, TRUE);
}

static void adc_config(void)
{
  adc_base_config_type adc_base_struct;

  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
  adc_reset(ADC1);
  crm_adc_clock_div_set(CRM_ADC_DIV_4); /* 96MHz/4 = 24MHz, within Table42's 0.6-28MHz */

  adc_base_default_para_init(&adc_base_struct);
  adc_base_struct.sequence_mode = TRUE;
  adc_base_struct.repeat_mode = TRUE;
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
  adc_base_struct.ordinary_channel_length = 3;
  adc_base_config(ADC1, &adc_base_struct);

  /* ts=13.5 cycles: sufficient for Rth=2.5kOhm at 24MHz per the
   * porting-plan's Table44-anchored + interpolated analysis (Table44
   * @28MHz: 13.5cyc->3.4kOhm max; our 24MHz point interpolates to
   * ~4.07kOhm max, comfortable margin over 2.5kOhm). */
  adc_ordinary_channel_set(ADC1, PHASE_A_CHANNEL, 1, ADC_SAMPLETIME_13_5);
  adc_ordinary_channel_set(ADC1, PHASE_B_CHANNEL, 2, ADC_SAMPLETIME_13_5);
  adc_ordinary_channel_set(ADC1, PHASE_C_CHANNEL, 3, ADC_SAMPLETIME_13_5);

  adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_TMR1CH4, TRUE);
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
    int32_t diff_a = (int32_t)a - neutral;
    int32_t diff_b = (int32_t)b - neutral;
    int32_t diff_c = (int32_t)c - neutral;

    stage_d_adc_a = a;
    stage_d_adc_b = b;
    stage_d_adc_c = c;
    stage_d_neutral = neutral;
    stage_d_diff_a = diff_a;
    stage_d_diff_b = diff_b;
    stage_d_diff_c = diff_c;

    zc_filter_update(&zc_a, diff_a, &stage_d_zc_count_a);
    zc_filter_update(&zc_b, diff_b, &stage_d_zc_count_b);
    zc_filter_update(&zc_c, diff_c, &stage_d_zc_count_c);

    stage_d_scan_count++;
  }
  if (dma_interrupt_flag_get(DMA1_DTERR1_FLAG) != RESET) {
    dma_flag_clear(DMA1_DTERR1_FLAG);
    stage_d_dma_error++;
  }
}

int main(void)
{
  clock_config_96mhz();

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  tim1_pwm_gpio_config();
  adc_gpio_config();
  dma_config();
  adc_config();
  tim1_pwm_config(); /* start TIM1 last: CH4 begins firing ADC triggers immediately */

  dma_channel_enable(DMA1_CHANNEL1, TRUE);

  for (;;) {
    ++stage_d_heartbeat;
  }
}
