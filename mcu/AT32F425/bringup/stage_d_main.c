/*
 * ESCape32 AT32F425 port -- Stage D, PASSIVE BEMF sub-stage.
 *
 * Per the user's explicit sequencing: the motor is now physically wired
 * to the 3-phase output (through MP6540HA), but MP6540HA does NOT
 * drive it yet -- the rotor is spun BY HAND to get passive BEMF.
 * Only once this passive capture is validated does real MP6540HA-
 * driven commutation happen (a later, separate firmware/stage).
 *
 * This file REPLACES an earlier stage_d_main.c draft that assumed
 * PWM-synchronized ADC triggering off an actively-running TIM1 -- that
 * doesn't apply here (there is no PWM at all in this sub-stage, so
 * there are no switching edges to dodge). That draft is kept as
 * stage_d_active_main.c for re-use once real commutation starts.
 *
 * BENCH SETUP for this passive test:
 *   - MP6540HA VIN: NORMAL supply voltage, powered as usual.
 *   - MP6540HA nSLEEP: HARDWIRED HIGH on this board (confirmed by the
 *     user -- there is no MCU control of nSLEEP, and there never will
 *     be for this design). The driver is therefore ALWAYS awake and
 *     its HSx/LSx outputs directly follow whatever logic level the
 *     MCU pins present -- nSLEEP provides NO automatic high-Z
 *     protection here. (An earlier version of this file assumed
 *     nSLEEP=LOW would do that job and left PA7/PA8/PA9/PA10/PB0/PB1
 *     untouched/floating -- that assumption was wrong for this board
 *     and has been corrected below: this firmware now actively drives
 *     all six HSx/LSx inputs LOW itself.)
 *   - PA7/PA8/PA9/PA10/PB0/PB1: driven to a DEFINED GPIO-output LOW
 *     state at startup, before anything else -- HSx=L, LSx=L on every
 *     leg gives MP6540HA's documented all-off/high-Z output state.
 *     TIM1/PWM is not used at all in this file.
 *   Rationale: if VIN were left at 0V while the hand-spun motor
 *   generates BEMF, that voltage can forward-bias the power MOSFETs'
 *   body diodes back into the (unpowered, low-impedance-looking) VIN
 *   rail, clamping/rectifying the phase waveform and corrupting
 *   exactly the passive BEMF shape this test exists to capture -- so
 *   VIN must be powered normally, not 0V, even though nothing is
 *   being commanded to drive the motor.
 *   IDEALLY: physically disconnect the motor phase wires from the
 *   power stage entirely and connect only the three 15k/3k ADC-divider
 *   taps directly to the motor leads. That removes MP6540HA (and its
 *   FETs' body diodes) from the signal path altogether -- the cleanest
 *   possible passive BEMF measurement. Use the VIN-powered/all-six-
 *   pins-LOW setup above when that physical disconnection isn't
 *   practical.
 *
 * ADC: continuous free-running 3-channel scan (no PWM to synchronize
 * to), same pin/channel mapping as Stage C/D-active:
 *   PA0 = VA = ADC1_IN0 (ADC_CHANNEL_0)
 *   PA4 = VB = ADC1_IN4 (ADC_CHANNEL_4)
 *   PA5 = VC = ADC1_IN5 (ADC_CHANNEL_5)
 * fADC=24MHz (CRM_ADC_DIV_4), ts=13.5 cycles -- same timing-budget
 * numbers already derived for the 15k/3k divider (Rth=2.5kOhm).
 *
 * Software neutral/reference: (VA+VB+VC)/3, difference-based zero-cross
 * judgement per the user's explicit instruction (the ~110-code common
 * offset found in Stage C cancels out in the difference).
 *
 * "Floating phase" / "expected crossing polarity": with no commanded
 * commutation, all three phases are simultaneously undriven, so there
 * is no single "the floating phase" or a commutation-step-derived
 * expected direction to check against -- this file logs crossing
 * EVENTS (which phase, rising or falling) for all three phases as they
 * occur; matching them up against "which step would have expected
 * this" is an offline/visual analysis of the dumped trace (e.g. against
 * a known rotation direction), not something this firmware assumes.
 *
 * NOT implemented here, deliberately, per explicit user instruction --
 * recorded as requirements for the real (non-bring-up) implementation:
 *   - The final MOUSEF425 firmware must NOT run all-3-phase zero-cross
 *     checking all the time like this file does. It must only look at
 *     the one floating phase implied by the current commutation
 *     sector, against that sector's expected crossing polarity.
 *   - The 100k-mechanical-RPM target cannot reuse this file's "3-ch
 *     scan + ZC_CONFIRM_COUNT=3" timing as-is. PWM synchronization,
 *     blanking, hysteresis, and end-to-end detection latency all need
 *     to be re-derived for that regime, not carried over unchanged
 *     from this passive bring-up test.
 * Both points apply to the active-commutation integration stage, not
 * to this file.
 *
 * RAM trace buffer: BRING-UP ONLY. This ~12.5KB buffer exists purely
 * to pull a waveform out over OpenOCD on hardware with no scope/logic
 * analyzer available. It must NOT be carried into the eventual
 * integrated MOUSEF425 firmware (shared src/main.c + Stage D/E's real
 * BEMF detector) -- that firmware runs on the same 20KB SRAM as every
 * other config/telemetry/protection state ESCape32 needs, and has no
 * room for a multi-KB debug log. Delete stage_d_trace[]/_trace_index
 * and everything that writes them when this bring-up file's logic
 * gets folded into the real target.
 *
 * A ring buffer of the most recent TRACE_LEN samples
 * (decimated -- logged every TRACE_DECIMATION-th ADC scan, so the
 * buffer covers several seconds of a hand-spin instead of a couple of
 * milliseconds; the confirm-count zero-cross filter itself still runs
 * on every undecimated scan, so no crossing is missed by the decimation,
 * only its logging is coalesced onto the nearest logged sample).
 * Dump procedure: halt the target over OpenOCD/GDB whenever you're
 * done spinning, then read `stage_d_trace[]` (TRACE_LEN entries,
 * 12 bytes each, see trace_entry_t) and `stage_d_trace_index` (total
 * writes so far -- entries are chronological starting at
 * `stage_d_trace_index % TRACE_LEN` and wrapping).
 */

#include "clock_config.h"

#define PHASE_A_CHANNEL ADC_CHANNEL_0 /* PA0 */
#define PHASE_B_CHANNEL ADC_CHANNEL_4 /* PA4 */
#define PHASE_C_CHANNEL ADC_CHANNEL_5 /* PA5 */

#define ZC_CONFIRM_COUNT 3

/* Deadband/hysteresis around neutral: |diff| <= ZC_DEADBAND counts is
 * treated as "still ambiguous" and does not update the candidate sign
 * or reset its confirm-run (this is what stops a few counts of noise
 * right at neutral from chattering the zero-cross state).
 * PLACEHOLDER VALUE -- 8 counts (out of 4095 full-scale) is a generic
 * starting guess, NOT derived from the user's captured stage_d_trace.bin.
 * Re-tune this from the actual noise amplitude seen in the neutral
 * region of that waveform before trusting it. */
#define ZC_DEADBAND 8

#define TRACE_LEN 800
#define TRACE_DECIMATION 128 /* log 1 in every N ADC scans: ~3.25us*128 = ~416us/sample, 800pts = ~0.33s */

typedef struct {
  uint32_t t_us;
  uint16_t a, b, c;
  uint8_t event; /* bit0=A rise,1=A fall,2=B rise,3=B fall,4=C rise,5=C fall */
  uint8_t reserved[3];
} trace_entry_t;

static volatile uint16_t adc_buf[3]; /* DMA target: VA, VB, VC (sequence 1,2,3) */

trace_entry_t stage_d_trace[TRACE_LEN];
volatile uint32_t stage_d_trace_index; /* total writes; slot = index % TRACE_LEN */

volatile uint32_t stage_d_heartbeat;
volatile uint32_t stage_d_scan_count;
volatile uint32_t stage_d_dma_error;
volatile uint16_t stage_d_adc_a, stage_d_adc_b, stage_d_adc_c;
volatile int32_t stage_d_neutral;
volatile uint32_t stage_d_zc_count_a, stage_d_zc_count_b, stage_d_zc_count_c;

/* Per-phase filter diagnostics, updated every UNDECIMATED raw scan
 * (i.e. every ~3.25us DMA transfer-complete, same rate zc_filter_update()
 * itself runs at -- not just the 128-decimated trace log rate). */
volatile int32_t stage_d_diag_diff_min_a, stage_d_diag_diff_min_b, stage_d_diag_diff_min_c;
volatile int32_t stage_d_diag_diff_max_a, stage_d_diag_diff_max_b, stage_d_diag_diff_max_c;
volatile uint32_t stage_d_diag_high_cross_a, stage_d_diag_high_cross_b, stage_d_diag_high_cross_c; /* raw diff>=+H sample count */
volatile uint32_t stage_d_diag_low_cross_a, stage_d_diag_low_cross_b, stage_d_diag_low_cross_c;     /* raw diff<=-H sample count */
volatile int stage_d_diag_candidate_sign_a, stage_d_diag_candidate_sign_b, stage_d_diag_candidate_sign_c;
volatile int stage_d_diag_confirmed_sign_a, stage_d_diag_confirmed_sign_b, stage_d_diag_confirmed_sign_c;
volatile int stage_d_diag_confirm_run_a, stage_d_diag_confirm_run_b, stage_d_diag_confirm_run_c;

void _init(void) {}
void _fini(void) {}

typedef struct {
  int confirmed_sign;  /* 0=unknown(bootstrap, treated as "low"), -1, +1 */
  int candidate_sign;  /* diagnostic only: the sign the current run is testing for */
  int confirm_run;     /* count of CONSECUTIVE qualifying raw samples so far */
  int32_t diff_min, diff_max;
  uint32_t high_cross, low_cross;
} zc_filter_t;

static zc_filter_t zc_a = {.diff_min = 0x7fffffff, .diff_max = -0x7fffffff - 1};
static zc_filter_t zc_b = {.diff_min = 0x7fffffff, .diff_max = -0x7fffffff - 1};
static zc_filter_t zc_c = {.diff_min = 0x7fffffff, .diff_max = -0x7fffffff - 1};

/*
 * Explicit Schmitt-trigger state machine (rewritten per user's exact
 * spec after Stage D trace2.bin showed massive spurious/asymmetric
 * crossing counts -- A=19568, B=77944, C=1 -- that the previous
 * "deadband skips silently, no reset" version could produce).
 *
 * THE BUG in the previous version: a sample landing inside the
 * deadband (or on the wrong side) just did `return 0` WITHOUT
 * resetting confirm_run. That made "confirm_run reaches N" mean "N
 * qualifying hits total since the last reset, with unlimited gaps
 * allowed in between" -- NOT "N consecutive samples" as intended. A
 * signal that only occasionally, sparsely pokes past the threshold
 * (e.g. noise riding on a slowly-changing BEMF trend near the
 * deadband boundary) could accumulate 3 such hits, spread across many
 * scans with plenty of deadband/wrong-side samples interleaved, and
 * still get "confirmed" -- which is not a real, coherent transition.
 * That asymmetric-noise-sensitive accumulation is the leading
 * candidate for A/B's inflated counts. It does NOT by itself explain
 * C reading exactly 1 despite clearing the deadband (that could be
 * the opposite failure mode: if C's raw diff flips sign almost every
 * single 3.25us sample near a genuine zero-crossing, it may rarely
 * produce even 3 truly consecutive same-side samples under a CORRECT
 * consecutive-count rule either -- the new stage_d_diag_* fields below
 * are there to tell the two apart from the next capture, rather than
 * guessing further from here).
 *
 * Fix: any non-qualifying sample (deadband OR wrong side) now resets
 * confirm_run to 0, so confirm_run truly only ever counts an unbroken
 * consecutive run. Direction to test for is derived purely from the
 * current confirmed_sign (confirmed=-1 or unknown -> watch for N
 * consecutive diff>=+H to flip to +1; confirmed=+1 -> watch for N
 * consecutive diff<=-H to flip to -1), matching the two cases the
 * user specified.
 *
 * Returns 0=no new confirmed crossing, 1=confirmed rising (->+1),
 * 2=confirmed falling (->-1).
 */
static int zc_filter_update(zc_filter_t *f, int32_t diff, volatile uint32_t *count)
{
  int qualifies, new_sign;

  if (diff < f->diff_min) f->diff_min = diff;
  if (diff > f->diff_max) f->diff_max = diff;
  if (diff >= ZC_DEADBAND) f->high_cross++;
  if (diff <= -ZC_DEADBAND) f->low_cross++;

  if (f->confirmed_sign <= 0) {
    new_sign = 1;
    qualifies = (diff >= ZC_DEADBAND);
  } else {
    new_sign = -1;
    qualifies = (diff <= -ZC_DEADBAND);
  }
  f->candidate_sign = new_sign;

  if (qualifies) {
    if (f->confirm_run < ZC_CONFIRM_COUNT) f->confirm_run++;
  } else {
    f->confirm_run = 0; /* ANY non-qualifying sample breaks the consecutive run */
  }

  if (f->confirm_run >= ZC_CONFIRM_COUNT) {
    f->confirm_run = 0;
    int prev = f->confirmed_sign;
    f->confirmed_sign = new_sign;
    if (prev != new_sign) {
      (*count)++;
      return new_sign > 0 ? 1 : 2;
    }
  }
  return 0;
}

/* Drive PA7/PA8/PA9/PA10/PB0/PB1 (-> MP6540HA HSA/LSA/HSB/LSB/HSC/LSC)
 * to a DEFINED GPIO-output LOW state. nSLEEP is hardwired HIGH on this
 * board (confirmed by the user), so the driver is always awake and
 * provides no automatic high-Z protection -- this firmware must
 * itself guarantee HSx=L/LSx=L on every leg (MP6540HA's documented
 * all-off state) before anything else happens. No TIM1 involvement in
 * this passive sub-stage. */
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

  /* Re-assert low after switching to output mode (mode change itself
   * doesn't glitch the ODR bit, but this makes the "all off" intent
   * explicit and independent of ODR's power-on-reset value). */
  gpio_bits_reset(GPIOA, GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10);
  gpio_bits_reset(GPIOB, GPIO_PINS_0 | GPIO_PINS_1);
}

/* TMR2: free-running 1us-resolution timestamp counter, independent of
 * ADC scan timing jitter. TMR2 is 32-bit (Table 4), so this wraps only
 * every ~71.6 minutes -- not a concern for a hand-spin test. */
static void timestamp_timer_config(void)
{
  crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
  tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
  tmr_base_init(TMR2, 0xFFFFFFFFu, (uint32_t)(system_core_clock / 1000000u) - 1u); /* /96 -> 1MHz */
  tmr_counter_enable(TMR2, TRUE);
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
  crm_adc_clock_div_set(CRM_ADC_DIV_4); /* 96MHz/4 = 24MHz */

  adc_base_default_para_init(&b);
  b.sequence_mode = TRUE;
  b.repeat_mode = TRUE; /* free-running: no PWM to synchronize to here */
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
    int32_t diff_a = (int32_t)a - neutral;
    int32_t diff_b = (int32_t)b - neutral;
    int32_t diff_c = (int32_t)c - neutral;

    stage_d_adc_a = a;
    stage_d_adc_b = b;
    stage_d_adc_c = c;
    stage_d_neutral = neutral;

    static uint8_t pending_event;
    int r;

    r = zc_filter_update(&zc_a, diff_a, &stage_d_zc_count_a);
    if (r) pending_event |= (r == 1) ? 0x01 : 0x02;
    stage_d_diag_diff_min_a = zc_a.diff_min;
    stage_d_diag_diff_max_a = zc_a.diff_max;
    stage_d_diag_high_cross_a = zc_a.high_cross;
    stage_d_diag_low_cross_a = zc_a.low_cross;
    stage_d_diag_candidate_sign_a = zc_a.candidate_sign;
    stage_d_diag_confirmed_sign_a = zc_a.confirmed_sign;
    stage_d_diag_confirm_run_a = zc_a.confirm_run;

    r = zc_filter_update(&zc_b, diff_b, &stage_d_zc_count_b);
    if (r) pending_event |= (r == 1) ? 0x04 : 0x08;
    stage_d_diag_diff_min_b = zc_b.diff_min;
    stage_d_diag_diff_max_b = zc_b.diff_max;
    stage_d_diag_high_cross_b = zc_b.high_cross;
    stage_d_diag_low_cross_b = zc_b.low_cross;
    stage_d_diag_candidate_sign_b = zc_b.candidate_sign;
    stage_d_diag_confirmed_sign_b = zc_b.confirmed_sign;
    stage_d_diag_confirm_run_b = zc_b.confirm_run;

    r = zc_filter_update(&zc_c, diff_c, &stage_d_zc_count_c);
    if (r) pending_event |= (r == 1) ? 0x10 : 0x20;
    stage_d_diag_diff_min_c = zc_c.diff_min;
    stage_d_diag_diff_max_c = zc_c.diff_max;
    stage_d_diag_high_cross_c = zc_c.high_cross;
    stage_d_diag_low_cross_c = zc_c.low_cross;
    stage_d_diag_candidate_sign_c = zc_c.candidate_sign;
    stage_d_diag_confirmed_sign_c = zc_c.confirmed_sign;
    stage_d_diag_confirm_run_c = zc_c.confirm_run;

    stage_d_scan_count++;

    static uint32_t decim;
    if (++decim >= TRACE_DECIMATION) {
      decim = 0;
      trace_entry_t *e = &stage_d_trace[stage_d_trace_index % TRACE_LEN];
      e->t_us = TMR2->cval;
      e->a = a;
      e->b = b;
      e->c = c;
      e->event = pending_event;
      pending_event = 0;
      stage_d_trace_index++;
    }
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
  gate_pins_force_off(); /* first thing: guarantee HSx=L/LSx=L before anything else */
  timestamp_timer_config();
  adc_gpio_config();
  dma_config();
  adc_config();

  dma_channel_enable(DMA1_CHANNEL1, TRUE);
  adc_ordinary_software_trigger_enable(ADC1, TRUE);

  for (;;) {
    ++stage_d_heartbeat;
  }
}
