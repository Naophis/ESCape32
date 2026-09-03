/*
 * ESCape32 AT32F425 port -- Stage E3: PWM-synchronized floating-phase
 * ADC sampling for the same open-loop alignment+ramp as Stage E2 (that
 * ramp itself is UNCHANGED and stays as-is per explicit instruction --
 * only the ADC/ZC-detector design is redone here).
 *
 * WHY: Stage E2's real hardware results showed the ADC/ZC path was
 * broken, not the BEMF itself:
 *   - diff_min/diff_max sat at essentially exactly +-2730 across EVERY
 *     speed bucket. That number is not a coincidence: for a 12-bit ADC
 *     with one phase near 4095 and the others near 0, software-neutral
 *     ((a+b+c)/3) puts a phase's diff-from-neutral at almost exactly
 *     +-(4095*2/3) = +-2730. I.e. the detector was regularly sampling
 *     while one phase sat at a full PWM-switching rail extreme, not
 *     during any quiet/settled part of the waveform.
 *   - zc_correct/zc_wrong were both far larger than "at most 1 real
 *     zero-cross per 40-step bucket" could ever produce, and the two
 *     tracked each other almost 1:1 at every speed -- consistent with
 *     picking up switching-synchronous noise (present regardless of
 *     real rotor position) rather than the slow BEMF trapezoid.
 * Root cause: Stage E/E2 ran the ADC free-running (software-retriggered,
 * no relationship to the PWM edges at all), so a large fraction of its
 * samples landed on or near a switching transition.
 *
 * FIX, addressing the redesign requirements one by one:
 *
 * (1)(2)(3) PWM-synchronized sampling with edge blanking, in a quiet
 *     window: ADC is now hardware-triggered from TMR1_CH4 (internal
 *     compare event, no GPIO -- same mechanism already built and
 *     reasoned through in stage_d_active_main.c, confirmed against
 *     Artery's own tmr_trigger_automatic_preempted example), placed
 *     inside the current PWM cycle's quiet window (see the timing
 *     budget below for which window and why).
 *
 * (9)(10) Timing budget @ 24kHz PWM (period=41.67us), duty=8%:
 *     - HS-on (positive phase driven) window ~= 0.08*41.67 = 3.33us.
 *       The 3-channel ADC scan alone (ts=13.5cyc @24MHz) takes ~3.25us
 *       -- that LEAVES NO ROOM for any edge blanking on either side.
 *       Sampling during HS-on is therefore not viable at this duty.
 *     - LS-freewheel (off-time) window = period - HS-on ~= 38.34us --
 *       comfortably larger than scan+blanking needs.
 *     => This file samples during the OFF-time (LS-freewheel) window,
 *        not the ON-time window, specifically because of this budget.
 *     CCR4 (the ADC trigger point) = ccr + BLANK_AFTER_EDGE_TICKS,
 *     i.e. BLANK_AFTER_EDGE_US after the falling edge that starts
 *     off-time. Scan then finishes ~3.25us later, still ~33us before
 *     the next rising edge -- large margin on both sides of the scan.
 *     BLANK_AFTER_EDGE_US=2 is a first, conservative, UNVERIFIED guess
 *     (no scope) -- tune from real data if switching ringing is still
 *     visible in the captured diffs.
 *
 * (8) A/B/C all land in the same PWM state: because the whole 3-channel
 *     scan (~3.25us) is scheduled entirely inside the ~38us off-time
 *     window with margin on both sides, all three channels are
 *     guaranteed to be sampled while every leg is in the same
 *     switching state (positive leg's LS conducting, negative leg's LS
 *     conducting, floating leg open) -- no channel can straddle a
 *     transition that another one doesn't.
 *
 * (4) Post-commutation blanking: POST_COMMUTATION_BLANK_SCANS ADC
 *     scans (now 1 scan = 1 PWM period, so this is directly a PWM-
 *     cycle count) are ignored for zero-cross purposes right after
 *     apply_step() -- lets the freshly-floating phase's voltage settle
 *     after the commutation transient before trusting it.
 *
 * (5)(6)(7) Single accepted crossing per sector, expected-direction
 *     only: two independent filter instances are kept per sector --
 *     zc_expect, pre-armed to detect ONLY this step's expected
 *     direction (rising or falling), which LOCKS (stage_e3_zc_locked)
 *     the instant it fires once, so it structurally cannot accept a
 *     second crossing before the next apply_step() resets it; and
 *     zc_anti, pre-armed for the OPPOSITE (non-expected) direction,
 *     purely diagnostic (counted into stage_e3_zc_wrong_count, never
 *     locks, never "accepted"). Both are reset at every apply_step().
 *
 * The open-loop 6-step table, per-phase drive reasoning, GPIO safe-
 * state sequencing, alignment phase, and the 15ms->5ms bucket ramp are
 * copied UNCHANGED from stage_e2_main.c (which the user confirmed
 * works: motor tracked the whole ramp without stalling) -- per
 * instruction, that part is deliberately not touched.
 */

#include "clock_config.h"

#define PHASE_A_CHANNEL ADC_CHANNEL_0 /* PA0 */
#define PHASE_B_CHANNEL ADC_CHANNEL_4 /* PA4 */
#define PHASE_C_CHANNEL ADC_CHANNEL_5 /* PA5 */

#define PWM_ARR (96000000u / 24000u - 1u) /* 24kHz, unchanged from Stage B/E/E2 */
#define DEAD_TIME_COUNT 53u /* unverified placeholder, see porting-plan Section 18-C/H */

#define OPEN_LOOP_DUTY_PERCENT 8u /* held constant through the whole ramp, unchanged */
#define ALIGN_DURATION_MS 300u

static const uint32_t bucket_period_ms[] = {15, 13, 11, 9, 7, 6, 5}; /* unchanged */
#define NUM_BUCKETS (sizeof(bucket_period_ms) / sizeof(bucket_period_ms[0]))
#define STEPS_PER_BUCKET 40u

#define ZC_CONFIRM_COUNT 3
#define ZC_DEADBAND 8 /* PLACEHOLDER, unchanged -- re-tune once real (non-edge) BEMF data exists */

/* --- New in Stage E3: PWM-synchronized ADC trigger placement --- */
#define BLANK_AFTER_EDGE_US 2u          /* after the HS-off edge, before the ADC scan starts */
#define POST_COMMUTATION_BLANK_SCANS 2u /* PWM cycles to ignore right after a step change */
#define TIM1_TICKS_PER_US 96u           /* TIM1 clocked directly at 96MHz, no prescaler */

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
  uint32_t zc_correct;   /* accepted (locked) crossings in this bucket -- ideally <= step_count */
  uint32_t zc_wrong;     /* diagnostic: confirmed anti-expected-direction crossings */
  int32_t diff_min, diff_max;
  uint16_t adc_a, adc_b, adc_c;
  uint32_t dma_error;
} stage_e3_bucket_t;

stage_e3_bucket_t stage_e3_results[NUM_BUCKETS];
volatile uint32_t stage_e3_bucket_index;

volatile uint32_t stage_e3_heartbeat;
volatile int stage_e3_running;
volatile int stage_e3_aligning;
volatile uint32_t stage_e3_current_step_period_us;
volatile uint32_t stage_e3_step_index;
volatile uint32_t stage_e3_step_count;
volatile int stage_e3_floating_phase;
volatile int stage_e3_expected_dir;
volatile uint16_t stage_e3_adc_a, stage_e3_adc_b, stage_e3_adc_c;
volatile int32_t stage_e3_neutral;
volatile int32_t stage_e3_floating_diff;
volatile int32_t stage_e3_floating_diff_min, stage_e3_floating_diff_max; /* current bucket, live */
volatile uint32_t stage_e3_zc_correct_count; /* current bucket, live */
volatile uint32_t stage_e3_zc_wrong_count;   /* current bucket, live */
volatile uint32_t stage_e3_dma_error;
volatile int stage_e3_zc_locked; /* 1 once this sector's expected crossing has been accepted */
volatile uint32_t stage_e3_tim1_c4dt; /* ADC trigger point, for sanity-checking against ccr/arr */

void _init(void) {}
void _fini(void) {}

typedef struct {
  int confirmed_sign;
  int confirm_run;
} zc_filter_t;

/* zc_expect: pre-armed each step for ONLY the expected direction, and
 * never consulted again once stage_e3_zc_locked is set.
 * zc_anti: pre-armed for the opposite direction, diagnostic only,
 * never locks anything. */
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

/* CH4: internal-only ADC trigger anchor, same mechanism as
 * stage_d_active_main.c. duty is constant through the whole ramp here,
 * so CCR4 is computed once and never retargeted -- a variable-duty
 * (closed-loop) version will need to recompute this whenever ccr
 * changes, which this file deliberately does not attempt. */
static void tim1_adc_trigger_config(uint16_t duty_ccr)
{
  tmr_output_config_type oc;
  uint16_t ccr4 = (uint16_t)(duty_ccr + BLANK_AFTER_EDGE_US * TIM1_TICKS_PER_US);

  tmr_output_default_para_init(&oc);
  oc.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  oc.oc_output_state = FALSE; /* internal event only, not brought out on this package anyway */
  oc.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  oc.oc_idle_state = FALSE;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &oc);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, ccr4);

  stage_e3_tim1_c4dt = ccr4;
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

  stage_e3_step_index = (uint32_t)idx;
  stage_e3_floating_phase = (int)s->floating;
  stage_e3_expected_dir = s->expected_dir;

  zc_expect.confirmed_sign = (s->expected_dir == 1) ? -1 : 1; /* watches only the expected direction */
  zc_expect.confirm_run = 0;
  zc_anti.confirmed_sign = (s->expected_dir == 1) ? 1 : -1;   /* watches only the opposite, diagnostic */
  zc_anti.confirm_run = 0;
  stage_e3_zc_locked = 0;
  blank_scans_remaining = POST_COMMUTATION_BLANK_SCANS;
}

static void stop_and_force_off(void)
{
  tmr_output_enable(TMR1, FALSE);
  tmr_counter_enable(TMR1, FALSE);
  tmr_counter_enable(TMR3, FALSE);
  gate_pins_force_off();
  stage_e3_running = 0;
}

static void reset_bucket_accumulator(void)
{
  stage_e3_zc_correct_count = 0;
  stage_e3_zc_wrong_count = 0;
  stage_e3_floating_diff_min = 0x7fffffff;
  stage_e3_floating_diff_max = -0x7fffffff - 1;
}

static uint32_t bucket_step_count;
static uint16_t g_duty_ccr;

static void snapshot_bucket(uint32_t idx)
{
  stage_e3_bucket_t *r = &stage_e3_results[idx];
  r->period_us = stage_e3_current_step_period_us;
  r->step_count = bucket_step_count;
  r->zc_correct = stage_e3_zc_correct_count;
  r->zc_wrong = stage_e3_zc_wrong_count;
  r->diff_min = stage_e3_floating_diff_min;
  r->diff_max = stage_e3_floating_diff_max;
  r->adc_a = stage_e3_adc_a;
  r->adc_b = stage_e3_adc_b;
  r->adc_c = stage_e3_adc_c;
  r->dma_error = stage_e3_dma_error;
}

static void start_bucket(uint32_t idx)
{
  uint32_t period_ms = bucket_period_ms[idx];
  stage_e3_current_step_period_us = period_ms * 1000u;
  bucket_step_count = 0;
  reset_bucket_accumulator();
  TMR3->pr = period_ms - 1u;
}

void TMR3_GLOBAL_IRQHandler(void)
{
  if (tmr_flag_get(TMR3, TMR_OVF_FLAG) == RESET) return;
  tmr_flag_clear(TMR3, TMR_OVF_FLAG);

  if (!stage_e3_running) return;

  int next = (int)((stage_e3_step_index + 1u) % 6u);
  apply_step(next, g_duty_ccr);
  stage_e3_step_count++;
  bucket_step_count++;

  if (bucket_step_count >= STEPS_PER_BUCKET) {
    snapshot_bucket(stage_e3_bucket_index);
    stage_e3_bucket_index++;
    if (stage_e3_bucket_index >= NUM_BUCKETS) {
      stop_and_force_off();
      return;
    }
    start_bucket(stage_e3_bucket_index);
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

/* Now hardware-triggered from TMR1_CH4 instead of free-running/
 * software-triggered -- see the file header for why. */
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

    stage_e3_adc_a = a;
    stage_e3_adc_b = b;
    stage_e3_adc_c = c;
    stage_e3_neutral = neutral;

    if (stage_e3_running && !stage_e3_aligning) {
      int32_t v = (stage_e3_floating_phase == 0) ? a : (stage_e3_floating_phase == 1) ? b : c;
      int32_t diff = v - neutral;
      stage_e3_floating_diff = diff;

      if (blank_scans_remaining) {
        blank_scans_remaining--;
      } else {
        if (diff < stage_e3_floating_diff_min) stage_e3_floating_diff_min = diff;
        if (diff > stage_e3_floating_diff_max) stage_e3_floating_diff_max = diff;

        if (!stage_e3_zc_locked) {
          int r = zc_filter_update(&zc_expect, diff);
          if (r == stage_e3_expected_dir) {
            stage_e3_zc_locked = 1; /* at most one accepted crossing per sector */
            stage_e3_zc_correct_count++;
          }
        }
        int ra = zc_filter_update(&zc_anti, diff); /* diagnostic only, never locks */
        if (ra != 0) stage_e3_zc_wrong_count++;
      }
    }
  }
  if (dma_interrupt_flag_get(DMA1_DTERR1_FLAG) != RESET) {
    dma_flag_clear(DMA1_DTERR1_FLAG);
    stage_e3_dma_error++;
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

  /* 2) BEMF-phase ADC pins + DMA (trigger source configured below,
   * after TIM1/duty are known). */
  timestamp_timer_config();
  adc_gpio_config();
  dma_config();

  /* 3) TIM1 configured, pins still plain GPIO LOW -- nothing driven yet. */
  tim1_init();
  g_duty_ccr = (uint16_t)((PWM_ARR + 1u) * OPEN_LOOP_DUTY_PERCENT / 100u);
  tim1_adc_trigger_config(g_duty_ccr); /* CCR4 depends on g_duty_ccr */

  /* 4) Now that CH4/CCR4 is set, start the hardware-triggered ADC. */
  adc_config();
  dma_channel_enable(DMA1_CHANNEL1, TRUE);
  /* No software trigger call: ADC now free-runs off TMR1_CH4 events,
   * which only start occurring once TIM1's counter is enabled below. */

  /* 5) Switch to TIM1 AF and hold step 0 statically for rotor alignment. */
  tim1_pins_to_af();
  stage_e3_aligning = 1;
  stage_e3_running = 1;
  stage_e3_step_index = 0;
  apply_step(0, g_duty_ccr);
  tmr_output_enable(TMR1, TRUE);
  tmr_counter_enable(TMR1, TRUE);
  delay_us(ALIGN_DURATION_MS * 1000u);
  stage_e3_aligning = 0;

  /* 6) Start the open-loop acceleration ramp (unchanged from Stage E2). */
  stage_e3_bucket_index = 0;
  start_bucket(0);
  stage_e3_step_count = 1;
  step_timer_init(bucket_period_ms[0]);
  tmr_counter_enable(TMR3, TRUE);

  for (;;) {
    ++stage_e3_heartbeat;
  }
}
