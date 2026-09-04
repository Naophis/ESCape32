/*
 * ESCape32 AT32F425 port -- Stage E15C: isolate why TMR1_CH1C(PA7) and
 * TMR1_CH2C(PB0) never drive their pins while TMR1_CH3C(PB1) does,
 * per Stage E15B's real-hardware finding (neg-role LS only worked on
 * steps where neg=C; LS_A/LS_B read 0 in every step that needed them,
 * LS_C read 1 in both steps that needed it). Stage E13's plain-GPIO
 * truth table showed all three physical LS gates work fine when
 * driven directly -- so this is specific to TMR1's hardware
 * complementary-output generation/routing for CH1C/CH2C, not wiring.
 *
 * PRIME SUSPECT: the GPIO AF/MUX index used for each complementary
 * pin. tim1_pins_to_af() (every Stage E-series file since Stage B)
 * uses:
 *   PA7  (CH1C) -> GPIO_MUX_3
 *   PB0  (CH2C) -> GPIO_MUX_1
 *   PB1  (CH3C) -> GPIO_MUX_2
 * while the three MAIN channel pins (PA8/PA9/PA10 = CH1/CH2/CH3) all
 * use GPIO_MUX_2. These three different MUX indices for the three
 * complementary pins were never independently confirmed against the
 * datasheet's actual per-pin AF table -- the porting-plan notes
 * explicitly flagged PA7/PB0/PB1 as "IOMUX assignments remain
 * oscilloscope-unverified" from the very first bring-up stage. CH3C's
 * MUX_2 happening to work, while CH1C's MUX_3 and CH2C's MUX_1 do NOT,
 * is consistent with MUX_2 simply being the correct AF index for ALL
 * SIX TMR1 pins on this package (a common pattern: one TMR1 AF slot
 * per pin group) and the 3/1/2 pattern this project used from the
 * start being wrong for two of the three complementary pins.
 *
 * This file does NOT assume that conclusion -- it tests it directly,
 * with the motor disconnected, no PWM ramp, no ADC/DMA/CH4 involved at
 * all (this is a pure digital gate-level test, like Stage E13, just
 * routed through TMR1's hardware complementary-output path instead of
 * plain GPIO):
 *
 * For CH1C and CH2C, TWO tests each are run back to back: first with
 * this project's CURRENT mux value (MUX_3 for CH1C, MUX_1 for CH2C),
 * then with the CANDIDATE fix (MUX_2, matching CH3C and all three main
 * channels). CH3C itself is re-tested once with its existing MUX_2 as
 * a control (expected to keep working, proving the test methodology
 * itself is sound). Each test:
 *   - forces the OTHER two phases fully off (oc/occ output disabled,
 *     matching apply_step()'s "floating" case),
 *   - configures the phase under test as FORCE_LOW main + complementary
 *     ENABLED/ACTIVE_HIGH (byte-for-byte the same oc_mode/oc_output_state/
 *     occ_output_state/occ_polarity combination apply_step() has always
 *     used for a sector's "neg" phase) with the test's mux value applied
 *     to ONLY the complementary pin (the main pin keeps its already-
 *     confirmed-working MUX_2),
 *   - settles ~3ms, then polls GPIOx->idt directly (no IRQ/DMA needed
 *     for a static level) 32 times and records the high-count for the
 *     complementary pin under test,
 *   - dumps the actual GPIOx->muxl/muxh raw register (so the nibble
 *     that latched can be read back directly, not just re-asserted),
 *     TMR1->cctrl raw, and that channel's specific enable/polarity/
 *     complementary-enable/complementary-polarity bits extracted from
 *     cctrl using the exact bit offsets tmr_output_channel_config()
 *     uses (chx_offset=(tmr_channel*2)+1 for polarity, tmr_channel*2
 *     for enable, +2/+3 for the complementary pair) -- this directly
 *     answers the "compare cctrl enable/polarity/dead-time bits
 *     between channels" part of the instruction: if those bits are
 *     identical in shape across CH1/CH2/CH3 (as the shared
 *     apply_step()-style config function should produce) while only
 *     the pin level differs, that is conclusive evidence the fault is
 *     in the GPIO MUX routing, not in TMR1's own enable/polarity/dead-
 *     time configuration. TMR1->brk (dead-time/MOE) is dumped once,
 *     since it is not per-channel.
 *
 * Note: this chip's TMR1 has a single CCTRL register covering CH1-4's
 * enable/polarity/complementary bits (confirmed from the vendor header
 * -- there is no separate CCTRL2 register on this part), so "cctrl2"
 * from the instruction does not apply here; CCTRL alone is dumped and
 * decoded per channel.
 */

#include "clock_config.h"

#define PWM_ARR (96000000u / 24000u - 1u) /* 24kHz, unchanged -- irrelevant to FORCE_LOW levels but keeps TMR1 in its normal running state */
#define DEAD_TIME_COUNT 53u /* unverified placeholder, see porting-plan Section 18-C/H */

#define SETTLE_US 3000u
#define POLL_SAMPLES 32u

typedef struct {
  const char *label;
  int tmr_channel_index;     /* 0=CH1, 1=CH2, 2=CH3 */
  tmr_channel_select_type channel;
  gpio_type *c_port;
  gpio_pins_source_type c_pin_source;
  uint16_t c_pin_mask;
  int c_pin_bit;              /* bit index in c_port->idt */
  gpio_mux_sel_type mux_to_test;
} e15c_test_case_t;

/* CH1 main=PA8/MUX_2 (untouched, already confirmed working).
 * CH2 main=PA9/MUX_2 (untouched). CH3 main=PA10/MUX_2 (untouched). */
static const e15c_test_case_t test_cases[5] = {
  {"CH1C_current_MUX3", 0, TMR_SELECT_CHANNEL_1, GPIOA, GPIO_PINS_SOURCE7, GPIO_PINS_7, 7, GPIO_MUX_3},
  {"CH1C_candidate_MUX2", 0, TMR_SELECT_CHANNEL_1, GPIOA, GPIO_PINS_SOURCE7, GPIO_PINS_7, 7, GPIO_MUX_2},
  {"CH2C_current_MUX1", 1, TMR_SELECT_CHANNEL_2, GPIOB, GPIO_PINS_SOURCE0, GPIO_PINS_0, 0, GPIO_MUX_1},
  {"CH2C_candidate_MUX2", 1, TMR_SELECT_CHANNEL_2, GPIOB, GPIO_PINS_SOURCE0, GPIO_PINS_0, 0, GPIO_MUX_2},
  {"CH3C_control_MUX2", 2, TMR_SELECT_CHANNEL_3, GPIOB, GPIO_PINS_SOURCE1, GPIO_PINS_1, 1, GPIO_MUX_2},
};
#define NUM_TESTS (sizeof(test_cases) / sizeof(test_cases[0]))

static const tmr_channel_select_type phase_channel[3] = {
  TMR_SELECT_CHANNEL_1, TMR_SELECT_CHANNEL_2, TMR_SELECT_CHANNEL_3
};

typedef struct {
  uint32_t c_pin_high_count; /* out of POLL_SAMPLES */
  uint32_t muxl_raw, muxh_raw;   /* raw register of whichever port owns the tested C pin */
  uint32_t muxl_b_raw, muxh_b_raw; /* the OTHER port's raw mux, for reference (GPIOA+GPIOB both dumped every test) */
  uint32_t cctrl_raw;
  uint32_t brk_raw;
  /* decoded from cctrl_raw using tmr_output_channel_config()'s own bit offsets */
  uint32_t c_en, c_p, c_nen, c_np; /* this channel's enable/polarity/comp-enable/comp-polarity */
} e15c_result_t;

e15c_result_t stage_e15c_results[NUM_TESTS];
volatile uint32_t stage_e15c_test_index;
volatile uint32_t stage_e15c_heartbeat;
volatile int stage_e15c_done;

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

/* Main-channel pins only (PA8/PA9/PA10) -- MUX_2, unchanged/untouched,
 * already confirmed working (Stage E9-E15 all rely on CH1/2/3 PWM
 * being correctly routed). Complementary pins are configured per-test
 * in apply_test(), not here. */
static void tim1_main_pins_to_af(void)
{
  gpio_init_type g;

  gpio_default_para_init(&g);
  g.gpio_mode = GPIO_MODE_MUX;
  g.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  g.gpio_pull = GPIO_PULL_NONE;
  g.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;

  g.gpio_pins = GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10;
  gpio_init(GPIOA, &g);

  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE8, GPIO_MUX_2);  /* CH1 */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_2);  /* CH2 */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_2); /* CH3 */
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

static void delay_us(uint32_t us)
{
  uint32_t start = TMR2->cval;
  while ((uint32_t)(uint16_t)(TMR2->cval - start) < us);
}

static void timestamp_timer_config(void)
{
  crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
  tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
  tmr_base_init(TMR2, 0xFFFFFFFFu, (uint32_t)(system_core_clock / 1000000u) - 1u);
  tmr_counter_enable(TMR2, TRUE);
}

/* Configure exactly one phase as FORCE_LOW main + complementary
 * ENABLED/ACTIVE_HIGH (identical shape to apply_step()'s "neg" case in
 * every Stage E file since E2), the other two phases fully OFF (the
 * "floating" case), and apply the test's mux value to ONLY the
 * complementary pin under test. */
static void apply_test(const e15c_test_case_t *t)
{
  tmr_output_config_type oc;

  for (int p = 0; p < 3; p++) {
    tmr_output_default_para_init(&oc);
    oc.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    oc.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;

    if (p == t->tmr_channel_index) {
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

  /* Complementary pin: MUX mode, test's mux value. */
  gpio_init_type g;
  gpio_default_para_init(&g);
  g.gpio_mode = GPIO_MODE_MUX;
  g.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  g.gpio_pull = GPIO_PULL_NONE;
  g.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  g.gpio_pins = t->c_pin_mask;
  gpio_init(t->c_port, &g);
  gpio_pin_mux_config(t->c_port, t->c_pin_source, t->mux_to_test);
}

static void run_test(uint32_t idx)
{
  const e15c_test_case_t *t = &test_cases[idx];
  e15c_result_t *r = &stage_e15c_results[idx];

  apply_test(t);
  delay_us(SETTLE_US);

  uint32_t high_count = 0;
  for (uint32_t i = 0; i < POLL_SAMPLES; i++) {
    uint32_t idt = t->c_port->idt;
    if (idt & (1u << t->c_pin_bit)) high_count++;
    delay_us(50u);
  }

  r->c_pin_high_count = high_count;
  r->muxl_raw = (t->c_port == GPIOA) ? GPIOA->muxl : GPIOB->muxl;
  r->muxh_raw = (t->c_port == GPIOA) ? GPIOA->muxh : GPIOB->muxh;
  r->muxl_b_raw = GPIOB->muxl;
  r->muxh_b_raw = GPIOB->muxh;
  r->cctrl_raw = TMR1->cctrl;
  r->brk_raw = TMR1->brk;

  /* Decode this channel's cctrl bits using tmr_output_channel_config()'s
   * own offsets (at32f425_tmr.c): chx_offset=(ch*2)+1 (polarity),
   * chcx_offset=(ch*2)+3 (comp polarity); enable=(ch*2), comp-enable=(ch*2)+2. */
  int ch = t->tmr_channel_index;
  uint32_t cctrl = r->cctrl_raw;
  r->c_en  = (cctrl >> (ch * 2)) & 0x1u;
  r->c_p   = (cctrl >> ((ch * 2) + 1)) & 0x1u;
  r->c_nen = (cctrl >> ((ch * 2) + 2)) & 0x1u;
  r->c_np  = (cctrl >> ((ch * 2) + 3)) & 0x1u;
}

int main(void)
{
  clock_config_96mhz();

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  gate_pins_force_off();
  timestamp_timer_config();

  tim1_init();
  tim1_main_pins_to_af();

  tmr_output_enable(TMR1, TRUE);
  tmr_counter_enable(TMR1, TRUE);

  for (stage_e15c_test_index = 0; stage_e15c_test_index < NUM_TESTS; stage_e15c_test_index++) {
    run_test(stage_e15c_test_index);
  }

  tmr_output_enable(TMR1, FALSE);
  tmr_counter_enable(TMR1, FALSE);
  gate_pins_force_off();
  stage_e15c_done = 1;

  for (;;) {
    ++stage_e15c_heartbeat;
  }
}
