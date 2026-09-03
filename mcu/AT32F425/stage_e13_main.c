/*
 * ESCape32 AT32F425 port -- Stage E13: static MP6540HA truth-table /
 * ADC-pin ground truth, with TMR1 PWM COMPLETELY OUT OF THE PICTURE.
 *
 * WHY: Stage E12 conclusively ruled out an ADC-channel-switch failure
 * (OSQ3 readback matched expected_channel exactly: 0/4/0 -> 0, 1/4/1
 * -> 4, 2/5/2 -> 5 across all three passes) as the cause of Stage E11's
 * near-identical PA0/PA4/PA5 waveforms. But Stage E12's own GPIO-level
 * diagnostic turned up something more fundamental: in the fixed sector
 * (pos=A/CH1, TMR1 driving PA8=HSA and PA7=LSA via CH1/CH1C
 * complementary output), pa8_high_count and pa7_high_count were BOTH
 * 128/128 during the HS-on window at several offsets -- i.e. the pad
 * level this code reads as "PA7=LSA" appears HIGH at the exact same
 * instants "PA8=HSA" is HIGH. Per the MP6540HA's documented truth
 * table (HS=0,LS=0 -> Hi-Z; HS=1,LS=0 -> VIN; HS=0,LS=1 -> GND;
 * HS=1,LS=1 -> Hi-Z, per its datasheet's stated "shoot-through
 * protection" behavior for a simultaneous-high input pair), HS=1/LS=1
 * would put the driver in Hi-Z, NOT actively driving VIN -- which
 * alone could explain why every "positive" phase ADC reading has been
 * inconsistent/flat/near-identical across Stages E7-E12: the phase
 * this code has been calling "positively driven" may never have
 * actually been driven to VIN at all.
 *
 * This file does not trust TMR1's CH1/CH1C complementary-output
 * generation (dead-time hardware, AF mux routing for PA7/CH1C which
 * was NEVER oscilloscope-verified per the porting-plan's own caveat)
 * to produce the HS/LS pair correctly. Instead it removes TMR1 and the
 * AF/MUX path from the test entirely: PA7/PA8/PA9/PA10/PB0/PB1 are
 * configured as PLAIN GPIO push-pull outputs, driven directly by this
 * code, one phase at a time, through all four HS/LS logic combinations
 * -- while the other two phases' six gate inputs stay forced LOW
 * (Hi-Z, the driver's documented safe state). For each of the 3
 * phases x 4 states, this file reads all three BEMF-divider ADC pins
 * (PA0/PA4/PA5) via single-channel software-triggered polling (128
 * samples each, no DMA/timer trigger needed since there is no PWM to
 * synchronize to) and records min/max/mean, plus a GPIOx->idt readback
 * of the commanded HS/LS pins themselves at settle time (proof the
 * commanded state was actually latched on the pad).
 *
 * Expected result if the ADC_CHANNEL_0/4/5 <-> PA0/PA4/PA5 <-> phase
 * A/B/C wiring assumption is correct: for phase-under-test = A,
 * state HS=1/LS=0 should show PA0 (phase A's own ADC pin) at
 * ~VIN/6-of-divider (roughly the same ~2700-2900 count range seen
 * consistently across E7-E12's "high" readings), state HS=0/LS=1
 * should show PA0 near 0, and states 00/11 should show PA0 floating
 * (whatever the 15k/3k divider settles to with nothing driving it --
 * likely near 0 given the pulldown-ish loading, but not assumed here,
 * only measured). PB/PC ADC pins during an A-phase test should reflect
 * whatever residual coupling exists through the (currently Hi-Z)
 * motor windings, which is itself useful data. Repeating for
 * phase-under-test = B and C establishes the full 3x3 gate<->ADC-pin
 * mapping without any PWM/dead-time/AF-routing assumptions at all.
 *
 * On the CH1/CH1C complementary-polarity question (separately raised):
 * source review of at32f425_tmr.c's tmr_output_channel_config() shows
 * CH1's complementary polarity bit (cctrl bit3, c1np) is written from
 * occ_polarity exactly like the main channel's c1p from oc_polarity
 * (both ACTIVE_HIGH=0 in Stage E10-E12's apply_fixed_step()), and the
 * hardware auto-generates OC1N as the dead-time-delayed complement of
 * OC1REF before that polarity bit is applied -- so with occ_polarity=
 * ACTIVE_HIGH (non-inverted), source review alone predicts LSA should
 * already be the logical inverse of HSA (minus two dead-time gaps per
 * cycle), which does NOT match the pa7/pa8-both-high readings actually
 * observed. That contradiction is exactly why this file bypasses CH1C/
 * TMR1 entirely rather than re-deriving the answer from code again --
 * per repeated experience in this project, only hardware measurement
 * settles it. Once this file's plain-GPIO ground truth is confirmed,
 * a follow-up stage can drive HS via TMR1 CH1 (PWM) while independently
 * driving LS via plain GPIO (bypassing CH1C) to isolate whether CH1C's
 * dead-time/AF path specifically is the fault, without reintroducing
 * any of today's ambiguity.
 *
 * No PWM, no TMR1CH4 ADC trigger, no DMA, no BEMF/ZC/commutation logic
 * in this file at all -- pure static GPIO + polled single-channel ADC.
 * Motor should be physically disconnected from the driver outputs if
 * possible while running this (per user instruction) since states
 * HS=1/LS=0 hold a phase at VIN indefinitely (no PWM off-time).
 */

#include "clock_config.h"

typedef struct {
  gpio_type *port;
  uint16_t pin;
} gate_pin_t;

/* A=CH1(PA8/PA7), B=CH2(PA9/PB0), C=CH3(PA10/PB1) -- same pin
 * assignments as every prior stage, just driven as plain GPIO here. */

#define HS_A_PIN GPIO_PINS_8
#define LS_A_PIN GPIO_PINS_7
#define HS_B_PIN GPIO_PINS_9
#define LS_B_PIN GPIO_PINS_0
#define HS_C_PIN GPIO_PINS_10
#define LS_C_PIN GPIO_PINS_1

static const gate_pin_t hs_pin[3] = {
  {GPIOA, HS_A_PIN}, {GPIOA, HS_B_PIN}, {GPIOA, HS_C_PIN}
};
static const gate_pin_t ls_pin[3] = {
  {GPIOA, LS_A_PIN}, {GPIOB, LS_B_PIN}, {GPIOB, LS_C_PIN}
};

/* pass 0/1/2 = PA0/ADC_CHANNEL_0, PA4/ADC_CHANNEL_4, PA5/ADC_CHANNEL_5 */
static const int16_t pass_adc_channel[3] = {
  (int16_t)ADC_CHANNEL_0, (int16_t)ADC_CHANNEL_4, (int16_t)ADC_CHANNEL_5,
};

#define NUM_PHASES 3
#define NUM_STATES 4 /* 0:HS0LS0  1:HS1LS0  2:HS0LS1  3:HS1LS1 */
#define SAMPLES_PER_READ 128u
#define SETTLE_US 3000u /* few ms settle, per instruction */

typedef struct {
  uint16_t v_min, v_max;
  uint32_t v_sum;
  uint16_t sample_count;
} stage_e13_adc_stat_t;

typedef struct {
  uint8_t hs_cmd, ls_cmd;           /* commanded levels for this state */
  uint8_t hs_readback, ls_readback; /* GPIOx->idt at settle time */
  stage_e13_adc_stat_t adc[3];      /* [0]=PA0 [1]=PA4 [2]=PA5 */
} stage_e13_state_result_t;

/* [phase_under_test][state]. phase 0/1/2 = A/B/C. state per NUM_STATES
 * ordering above. */
stage_e13_state_result_t stage_e13_results[NUM_PHASES][NUM_STATES];

volatile int stage_e13_done;
volatile uint32_t stage_e13_heartbeat;

void _init(void) {}
void _fini(void) {}

static void gates_gpio_init(void)
{
  gpio_init_type g;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

  gpio_bits_reset(GPIOA, HS_A_PIN | LS_A_PIN | HS_B_PIN | HS_C_PIN);
  gpio_bits_reset(GPIOB, LS_B_PIN | LS_C_PIN);

  gpio_default_para_init(&g);
  g.gpio_mode = GPIO_MODE_OUTPUT;
  g.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  g.gpio_pull = GPIO_PULL_NONE;
  g.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;

  g.gpio_pins = HS_A_PIN | LS_A_PIN | HS_B_PIN | HS_C_PIN;
  gpio_init(GPIOA, &g);
  g.gpio_pins = LS_B_PIN | LS_C_PIN;
  gpio_init(GPIOB, &g);

  gpio_bits_reset(GPIOA, HS_A_PIN | LS_A_PIN | HS_B_PIN | HS_C_PIN);
  gpio_bits_reset(GPIOB, LS_B_PIN | LS_C_PIN);
}

static void set_all_gates_low(void)
{
  gpio_bits_reset(GPIOA, HS_A_PIN | LS_A_PIN | HS_B_PIN | HS_C_PIN);
  gpio_bits_reset(GPIOB, LS_B_PIN | LS_C_PIN);
}

/* Drive exactly one phase's HS/LS pins to the commanded levels; every
 * other phase's HS/LS stays forced LOW (Hi-Z per MP6540HA truth
 * table). */
static void set_phase_state(uint32_t phase, uint8_t hs, uint8_t ls)
{
  set_all_gates_low();

  if (hs) gpio_bits_set(hs_pin[phase].port, hs_pin[phase].pin);
  else gpio_bits_reset(hs_pin[phase].port, hs_pin[phase].pin);

  if (ls) gpio_bits_set(ls_pin[phase].port, ls_pin[phase].pin);
  else gpio_bits_reset(ls_pin[phase].port, ls_pin[phase].pin);
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

static void adc_config_polled(void)
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

  adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);
  adc_dma_mode_enable(ADC1, FALSE);

  adc_enable(ADC1, TRUE);

  adc_calibration_init(ADC1);
  while (adc_calibration_init_status_get(ADC1));
  adc_calibration_start(ADC1);
  while (adc_calibration_status_get(ADC1));
}

static void timestamp_timer_config(void)
{
  crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
  tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
  tmr_base_init(TMR2, 0xFFFFFFFFu, (uint32_t)(system_core_clock / 1000000u) - 1u);
  tmr_counter_enable(TMR2, TRUE);
}

static void delay_us(uint32_t us)
{
  uint32_t start = TMR2->cval;
  while ((uint32_t)(TMR2->cval - start) < us);
}

/* Blocking, polled, software-triggered single-channel read: no DMA,
 * no timer trigger -- nothing to synchronize to since there is no
 * PWM in this file. */
static void read_channel_128(int16_t channel, stage_e13_adc_stat_t *out)
{
  uint16_t v_min = 0xffff, v_max = 0;
  uint32_t v_sum = 0;

  adc_ordinary_channel_set(ADC1, (adc_channel_select_type)channel, 1, ADC_SAMPLETIME_239_5);

  for (uint32_t i = 0; i < SAMPLES_PER_READ; i++) {
    adc_ordinary_software_trigger_enable(ADC1, TRUE);
    while (adc_flag_get(ADC1, ADC_CCE_FLAG) == RESET);
    adc_flag_clear(ADC1, ADC_CCE_FLAG);
    uint16_t v = adc_ordinary_conversion_data_get(ADC1);

    if (v < v_min) v_min = v;
    if (v > v_max) v_max = v;
    v_sum += v;
  }

  out->v_min = v_min;
  out->v_max = v_max;
  out->v_sum = v_sum;
  out->sample_count = (uint16_t)SAMPLES_PER_READ;
}

int main(void)
{
  clock_config_96mhz();

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  gates_gpio_init();
  adc_gpio_config();
  adc_config_polled();
  timestamp_timer_config();

  static const uint8_t state_hs[NUM_STATES] = {0, 1, 0, 1};
  static const uint8_t state_ls[NUM_STATES] = {0, 0, 1, 1};

  for (uint32_t phase = 0; phase < NUM_PHASES; phase++) {
    for (uint32_t state = 0; state < NUM_STATES; state++) {
      uint8_t hs = state_hs[state];
      uint8_t ls = state_ls[state];

      set_phase_state(phase, hs, ls);
      delay_us(SETTLE_US);

      stage_e13_state_result_t *r = &stage_e13_results[phase][state];
      r->hs_cmd = hs;
      r->ls_cmd = ls;
      r->hs_readback = (hs_pin[phase].port->idt & hs_pin[phase].pin) ? 1 : 0;
      r->ls_readback = (ls_pin[phase].port->idt & ls_pin[phase].pin) ? 1 : 0;

      for (uint32_t pass = 0; pass < 3; pass++) {
        read_channel_128(pass_adc_channel[pass], &r->adc[pass]);
      }
    }
  }

  set_all_gates_low();
  stage_e13_done = 1;

  for (;;) {
    ++stage_e13_heartbeat;
  }
}
