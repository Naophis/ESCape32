/*
 * ESCape32 AT32F425 port -- Stage E26: software TMR_HALL_SWTRIG
 * preload-commit + single-firing verification. NOT a motor-control
 * test -- MOE stays off throughout (tmr_output_enable(TMR1, TRUE) is
 * never called in this file), no GPIO is routed to TIM1 AF, no real
 * PWM output exists.
 *
 * WHY: Stage E25 confirmed TMR3(TRGO, IS2) -> TMR1(COM) works 8/8 in
 * hardware, but per instruction the real AT32F425/MP6540HA backend
 * does NOT use that automatic hardware path -- 2us break-before-make
 * must land at a precise offset from the upstream-computed
 * commutation instant (gate-off 2us BEFORE target, commit exactly AT
 * target), which a single automatic TRGI->COM event cannot express by
 * itself. The backend design instead drives the commit via SOFTWARE
 * (tmr_event_sw_trigger(TMR1, TMR_HALL_SWTRIG), the exact mechanism
 * Stage E2-E23's apply_step()/nextstep()-equivalent commutation
 * already used successfully hundreds of times across real motor
 * spins) at the moment OUR OWN scheduling decides. Before finalizing
 * that design, this file isolates and re-confirms, at the register
 * level with the motor disconnected from the output stage entirely,
 * the two specific properties the design depends on:
 *
 *   (1) a config written via tmr_output_channel_config() while CCPC
 *       is enabled, followed by tmr_event_sw_trigger(TMR1,
 *       TMR_HALL_SWTRIG), results in TMR1's own registers reading
 *       back exactly the configuration that was written (the write
 *       path itself is correct and repeatable) -- NOTE: with MOE off
 *       and no scope/motor, this test can only confirm the register
 *       WRITE completed as intended; it cannot independently observe
 *       the internal shadow-vs-active silicon latch the way a real
 *       PWM pin or scope trace would. That distinction is called out
 *       explicitly rather than overclaimed.
 *   (2) each TMR_HALL_SWTRIG produces EXACTLY ONE COM ISR firing --
 *       not zero, not a double-fire/spurious repeat -- across many
 *       trials and across BOTH directions of a CH1 mode change
 *       (FORCE_LOW <-> FORCE_HIGH), so the commit mechanism this
 *       backend will call once per commutation is provably reliable
 *       before it is relied upon in the real integration.
 *
 * If confirmed, per instruction this ends the Stage E-series hardware
 * verification work and the real mcu/AT32F425/ target implementation
 * (config.c/config.h/config.cmake, COMMUTATION_BREAK() hook, the
 * two-stage gate-off/commit scheduler around IFTIM's CCR3 target
 * value) proceeds next.
 */

#include "clock_config.h"

#define NUM_TRIALS 16u
#define TRIAL_SPACING_US 500u

typedef struct {
  uint32_t requested_mode;  /* the oc_mode this trial configured CH1 to (TMR_OUTPUT_CONTROL_FORCE_LOW/HIGH) */
  uint32_t cm1_after;       /* TMR1->cm1 read back immediately after the SWTRIG */
  uint32_t cctrl_after;     /* TMR1->cctrl read back immediately after the SWTRIG */
  uint32_t isr_count_delta; /* how many times the COM ISR fired for this one SWTRIG -- must be exactly 1 */
  uint32_t readback_matches;/* 0/1 -- does cm1's c1octrl field match requested_mode */
} stage_e26_trial_t;

stage_e26_trial_t stage_e26_results[NUM_TRIALS];
volatile uint32_t stage_e26_com_event_count;
volatile uint32_t stage_e26_heartbeat;
volatile int stage_e26_done;

void _init(void) {}
void _fini(void) {}

static void timestamp_timer_config(void)
{
  crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
  tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
  tmr_base_init(TMR2, 0xFFFFFFFFu, (uint32_t)(system_core_clock / 1000000u) - 1u);
  tmr_counter_enable(TMR2, TRUE);
}

static void delay_us(uint32_t us)
{
  while (us) {
    uint16_t chunk = (us > 60000u) ? 60000u : (uint16_t)us;
    uint16_t start = (uint16_t)TMR2->cval;
    while ((uint16_t)((uint16_t)TMR2->cval - start) < chunk);
    us -= chunk;
  }
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

static void tmr1_test_init(void)
{
  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

  tmr_base_init(TMR1, 999u, 0);
  tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);

  tmr_channel_buffer_enable(TMR1, TRUE); /* CCPC-equivalent, same call proven throughout Stage E2-E23 */

  nvic_irq_enable(TMR1_BRK_OVF_TRG_HALL_IRQn, 1, 0);
  tmr_interrupt_enable(TMR1, TMR_HALL_INT, TRUE);

  tmr_output_enable(TMR1, FALSE); /* explicit: MOE stays off throughout this file, never set TRUE */
  tmr_counter_enable(TMR1, TRUE);
}

void TMR1_BRK_OVF_TRG_HALL_IRQHandler(void)
{
  if (tmr_flag_get(TMR1, TMR_HALL_FLAG) != RESET) {
    tmr_flag_clear(TMR1, TMR_HALL_FLAG);
    stage_e26_com_event_count++;
  }
  if (TMR1->ists & (1u << 6)) TMR1->ists &= ~(1u << 6); /* trgif, no vendor macro -- clear if set, harmless here */
  if (tmr_flag_get(TMR1, TMR_OVF_FLAG) != RESET) tmr_flag_clear(TMR1, TMR_OVF_FLAG);
  if (tmr_flag_get(TMR1, TMR_BRK_FLAG) != RESET) tmr_flag_clear(TMR1, TMR_BRK_FLAG);
}

/* Configure CH1 only (no GPIO/AF routing, no other channels needed for
 * this isolated test) to the given force-mode, matching the exact
 * shape apply_step()'s "neg"/"floating" cases have used since Stage
 * E15/E20-E23. */
static void configure_ch1(tmr_output_control_mode_type mode)
{
  tmr_output_config_type oc;
  tmr_output_default_para_init(&oc);
  oc.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  oc.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  oc.oc_mode = mode;
  oc.oc_output_state = TRUE;
  oc.oc_idle_state = FALSE;
  oc.occ_output_state = TRUE;
  oc.occ_idle_state = FALSE;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &oc);
}

int main(void)
{
  clock_config_96mhz();

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  gate_pins_force_off();
  timestamp_timer_config();
  tmr1_test_init();

  /* Establish a known baseline (FORCE_LOW) before the timed trials. */
  configure_ch1(TMR_OUTPUT_CONTROL_FORCE_LOW);
  tmr_event_sw_trigger(TMR1, TMR_HALL_SWTRIG);
  delay_us(50u);

  for (uint32_t i = 0; i < NUM_TRIALS; i++) {
    delay_us(TRIAL_SPACING_US);

    /* Alternate FORCE_LOW/FORCE_HIGH each trial -- exercises the
     * commit path in both directions, not just one fixed value. */
    tmr_output_control_mode_type mode = (i & 1u) ? TMR_OUTPUT_CONTROL_FORCE_HIGH : TMR_OUTPUT_CONTROL_FORCE_LOW;
    configure_ch1(mode);

    uint32_t before_count = stage_e26_com_event_count;
    tmr_event_sw_trigger(TMR1, TMR_HALL_SWTRIG);
    delay_us(50u); /* let the ISR run */

    stage_e26_trial_t *r = &stage_e26_results[i];
    r->requested_mode = (uint32_t)mode;
    r->cm1_after = TMR1->cm1;
    r->cctrl_after = TMR1->cctrl;
    r->isr_count_delta = stage_e26_com_event_count - before_count;
    r->readback_matches = (TMR1->cm1_output_bit.c1octrl == (uint32_t)mode) ? 1u : 0u;
  }

  stage_e26_done = 1;

  for (;;) {
    ++stage_e26_heartbeat;
  }
}
