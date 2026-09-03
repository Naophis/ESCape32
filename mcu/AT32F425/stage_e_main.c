/*
 * ESCape32 AT32F425 port -- Stage E, OPEN-LOOP active commutation
 * (first real MP6540HA-driven motor spin).
 *
 * Per the user's explicit instructions:
 *   - nSLEEP is hardwired HIGH on this board (confirmed, no MCU
 *     control -- see stage_d_main.c's corrected header). The driver
 *     is therefore ALWAYS awake; power-stage all-off is achieved
 *     ONLY by this firmware actively holding all six HSx/LSx inputs
 *     LOW, not by any driver-side sleep state.
 *   - Startup sequence: (1) force all six HSx/LSx GPIO outputs LOW
 *     first, before anything else; (2) only then switch
 *     PA7/PA8/PA9/PA10/PB0/PB1 over to TIM1 alternate function and
 *     start complementary PWM commutation.
 *   - Low duty, low step rate, bounded run length, then automatic
 *     return to the all-low safe state -- no closed-loop BEMF control
 *     yet. The BEMF/zero-cross work here is purely OBSERVATIONAL: for
 *     each commutation step, only the one floating phase implied by
 *     that step (not all 3 phases at once, unlike Stage D) is checked
 *     against that step's expected rising/falling crossing direction,
 *     matching the "final implementation must only look at the
 *     sector's floating phase + expected polarity" requirement
 *     recorded in stage_d_main.c/stage_d_active_main.c.
 *   - ISR is deliberately much lighter than Stage D's: one diff +
 *     one zc_filter_update() per ADC scan (the current floating
 *     phase only), no per-scan trace-buffer writes at all. Stage D's
 *     measured 3.25us-theoretical/4.42us-actual gap pointed at ISR
 *     load eating into the 307kHz DMA_FDT budget; this avoids
 *     repeating that with an even more time-critical live motor test.
 *
 * 6-step table: pos/neg driven phases + floating phase + that step's
 * expected zero-cross direction, in the standard alternating-rising/
 * falling pattern (each phase is floating exactly twice per electrical
 * revolution, once expected rising, once expected falling). The
 * letter-to-physical-winding mapping is NOT independently verified at
 * this open-loop stage (rotation direction may come out "backwards"
 * relative to some real-world expectation) -- this stage only checks
 * for a SELF-CONSISTENT alternating pattern, not a specific direction.
 *
 * Per-step phase drive (all reasoned directly from MP6540HA's HSx/LSx
 * truth table and this timer's documented output-compare semantics,
 * NOT reusing AT32F421's packed bit-table which relies on polarity
 * conventions not independently re-verified for this port):
 *   - positive (source) phase : oc=PWM_MODE_A, HSx=PWM(duty), LSx=
 *     complementary-of-HSx with dead-time (standard damped PWM, same
 *     as Stage B).
 *   - negative (sink) phase   : oc=FORCE_LOW (HSx driven constant 0),
 *     complementary channel enabled ACTIVE_HIGH so LSx = NOT(HSx) =
 *     constant 1, dead-time still applied by hardware to the FORCE_LOW
 *     -> complementary transition.
 *   - floating phase          : both channel outputs DISABLED
 *     (oc_output_state=occ_output_state=FALSE) with oc_idle_state=
 *     occ_idle_state=FALSE -- this is the exact same "disabled channel
 *     idle-state-LOW" mechanism Stage B already relied on for its safe
 *     idle convention, now used to keep both HSx/LSx at logic 0 (both
 *     FETs off) while that phase is not driven.
 * All three real TIM1 channels are reconfigured every step via
 * tmr_output_channel_config() (CCPC-buffered) and committed atomically
 * with tmr_event_sw_trigger(TMR_OVERFLOW_SWTRIG|TMR_HALL_SWTRIG), the
 * same UG|COM-equivalent pattern ESCape32's own nextstep()/laststep()
 * use on other targets, so there is no transient window where the old
 * and new step's phase assignments are simultaneously live.
 */

#include "clock_config.h"

#define PHASE_A_CHANNEL ADC_CHANNEL_0 /* PA0 */
#define PHASE_B_CHANNEL ADC_CHANNEL_4 /* PA4 */
#define PHASE_C_CHANNEL ADC_CHANNEL_5 /* PA5 */

#define PWM_ARR (96000000u / 24000u - 1u) /* 24kHz, same as Stage B */
#define DEAD_TIME_COUNT 53u /* unverified placeholder, see porting-plan Section 18-C/H */

/* --- Open-loop bring-up parameters: START CONSERVATIVE, TUNE UP LATER --- */
#define OPEN_LOOP_DUTY_PERCENT 8u     /* of PWM_ARR; low on purpose for the first spin */
#define OPEN_LOOP_STEP_INTERVAL_MS 15u /* fixed, no ramp yet -- very slow */
#define OPEN_LOOP_TOTAL_STEPS 60u      /* 10 electrical revolutions, then auto-stop */

#define ZC_CONFIRM_COUNT 3
#define ZC_DEADBAND 8 /* PLACEHOLDER -- carried over from Stage D; re-tune from real data */

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

static volatile uint16_t adc_buf[3]; /* DMA target: VA, VB, VC (sequence 1,2,3) */

volatile uint32_t stage_e_heartbeat;
volatile int stage_e_running;
volatile uint32_t stage_e_step_index;  /* 0..5 */
volatile uint32_t stage_e_step_count;  /* total steps completed since start */
volatile int stage_e_floating_phase;   /* 0=A,1=B,2=C */
volatile int stage_e_expected_dir;     /* 1=rising,2=falling */
volatile uint16_t stage_e_adc_a, stage_e_adc_b, stage_e_adc_c;
volatile int32_t stage_e_neutral;
volatile int32_t stage_e_floating_diff;
volatile uint32_t stage_e_zc_correct_count;
volatile uint32_t stage_e_zc_wrong_count;
volatile uint32_t stage_e_dma_error;

void _init(void) {}
void _fini(void) {}

typedef struct {
  int confirmed_sign;
  int confirm_run;
} zc_filter_t;

static volatile zc_filter_t zc; /* single instance: only the current floating phase uses it */

/* Same corrected Schmitt-trigger logic as stage_d_main.c (see that
 * file's comment for the bug this replaced): any non-qualifying
 * sample resets confirm_run, so confirm_run only ever counts a truly
 * consecutive run. Returns 0/1(rising)/2(falling). */
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

/* --- Safe-state GPIO control (same mechanism as stage_d_main.c's
 * gate_pins_force_off(), see that file for why nSLEEP can't do this
 * job on this board) --- */

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

  tmr_channel_buffer_enable(TMR1, TRUE); /* CCPC: channel changes need a commit event */
}

/* Reconfigure all 3 real TIM1 channels for the given step and commit
 * atomically. duty_ccr only matters for whichever channel ends up
 * "positive" this step. */
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
      oc.oc_mode = TMR_OUTPUT_CONTROL_FORCE_LOW; /* HSx=0 */
      oc.oc_output_state = TRUE;
      oc.oc_idle_state = FALSE;
      oc.occ_output_state = TRUE; /* LSx = NOT(HSx) = 1, dead-time applied */
      oc.occ_idle_state = FALSE;
      tmr_output_channel_config(TMR1, phase_channel[p], &oc);
    } else {
      oc.oc_mode = TMR_OUTPUT_CONTROL_FORCE_LOW;
      oc.oc_output_state = FALSE; /* HSx and LSx both disabled -> idle-state LOW */
      oc.oc_idle_state = FALSE;
      oc.occ_output_state = FALSE;
      oc.occ_idle_state = FALSE;
      tmr_output_channel_config(TMR1, phase_channel[p], &oc);
    }
  }

  tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG | TMR_HALL_SWTRIG); /* atomic commit */

  stage_e_step_index = (uint32_t)idx;
  stage_e_floating_phase = (int)s->floating;
  stage_e_expected_dir = s->expected_dir;

  /* Pre-arm the filter as if already on the "before crossing" side, so
   * it watches for exactly the transition this step expects. */
  zc.confirmed_sign = (s->expected_dir == 1) ? -1 : 1;
  zc.confirm_run = 0;
}

static void stop_and_force_off(void)
{
  tmr_output_enable(TMR1, FALSE);
  tmr_counter_enable(TMR1, FALSE);
  gate_pins_force_off(); /* belt-and-suspenders: don't rely solely on timer idle-state */
  stage_e_running = 0;
}

/* --- Step timer: TMR3, fixed OPEN_LOOP_STEP_INTERVAL_MS period --- */

static uint16_t g_duty_ccr;

void TMR3_GLOBAL_IRQHandler(void)
{
  if (tmr_flag_get(TMR3, TMR_OVF_FLAG) == RESET) return;
  tmr_flag_clear(TMR3, TMR_OVF_FLAG);

  if (!stage_e_running) return;

  if (stage_e_step_count >= OPEN_LOOP_TOTAL_STEPS) {
    tmr_counter_enable(TMR3, FALSE);
    stop_and_force_off();
    return;
  }

  int next = (int)((stage_e_step_index + 1u) % 6u);
  apply_step(next, g_duty_ccr);
  stage_e_step_count++;
}

static void step_timer_init(void)
{
  crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
  nvic_irq_enable(TMR3_GLOBAL_IRQn, 1, 0); /* lower priority than ADC DMA */

  tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
  /* 96MHz / 96000 = 1kHz tick (1ms); ARR=(interval_ms-1) */
  tmr_base_init(TMR3, OPEN_LOOP_STEP_INTERVAL_MS - 1u, 96000u - 1u);
  tmr_interrupt_enable(TMR3, TMR_OVF_INT, TRUE);
}

/* --- ADC: continuous free-running 3-channel scan, same as Stage D --- */

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
  nvic_irq_enable(DMA1_Channel1_IRQn, 0, 0); /* higher priority than the step timer */
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
  crm_adc_clock_div_set(CRM_ADC_DIV_4); /* 96MHz/4 = 24MHz */

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

/* Lightweight ISR: only the current floating phase gets a diff +
 * filter update, unlike Stage D's always-all-3-phases design (see
 * this file's header comment and stage_d_main.c's timing-discrepancy
 * writeup for why). */
void DMA1_Channel1_IRQHandler(void)
{
  if (dma_interrupt_flag_get(DMA1_FDT1_FLAG) != RESET) {
    dma_flag_clear(DMA1_FDT1_FLAG);

    uint16_t a = adc_buf[0], b = adc_buf[1], c = adc_buf[2];
    int32_t neutral = ((int32_t)a + (int32_t)b + (int32_t)c) / 3;

    stage_e_adc_a = a;
    stage_e_adc_b = b;
    stage_e_adc_c = c;
    stage_e_neutral = neutral;

    if (stage_e_running) {
      int32_t v = (stage_e_floating_phase == 0) ? a : (stage_e_floating_phase == 1) ? b : c;
      int32_t diff = v - neutral;
      stage_e_floating_diff = diff;

      int r = zc_filter_update(&zc, diff);
      if (r == stage_e_expected_dir) stage_e_zc_correct_count++;
      else if (r != 0) stage_e_zc_wrong_count++;
    }
  }
  if (dma_interrupt_flag_get(DMA1_DTERR1_FLAG) != RESET) {
    dma_flag_clear(DMA1_DTERR1_FLAG);
    stage_e_dma_error++;
  }
}

int main(void)
{
  clock_config_96mhz();

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  /* 1) Safe state FIRST, before anything else touches these pins. */
  gate_pins_force_off();

  /* 2) BEMF-phase ADC, running throughout (used for the diagnostic
   * floating-phase check once commutation starts). */
  adc_gpio_config();
  dma_config();
  adc_config();
  dma_channel_enable(DMA1_CHANNEL1, TRUE);
  adc_ordinary_software_trigger_enable(ADC1, TRUE);

  /* 3) TIM1 configured (dead-time etc.) but pins are still plain GPIO
   * LOW at this point -- nothing is driven yet. */
  tim1_init();

  g_duty_ccr = (uint16_t)((PWM_ARR + 1u) * OPEN_LOOP_DUTY_PERCENT / 100u);

  /* 4) NOW switch the six pins over to TIM1 AF and start step 0. This
   * is the moment MP6540HA starts actually seeing PWM. */
  tim1_pins_to_af();
  stage_e_step_index = 5; /* so apply_step(0,...) below is step "0 after 5" */
  stage_e_running = 1;
  apply_step(0, g_duty_ccr);
  stage_e_step_count = 1;
  tmr_output_enable(TMR1, TRUE);
  tmr_counter_enable(TMR1, TRUE);

  step_timer_init();
  tmr_counter_enable(TMR3, TRUE);

  for (;;) {
    ++stage_e_heartbeat;
  }
}
