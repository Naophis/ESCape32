/*
 * ESCape32 AT32F425 port -- Stage E28D: is TMR2's CVAL genuinely
 * stalling/slowing against REAL elapsed time around 40ms, or was that
 * an artifact of Stage E28B/E28C's own test harness (TMR7-based
 * delay_us(), CH1/CH3 configuration, or something else in those
 * files)? This uses a COMPLETELY INDEPENDENT wall-clock -- the
 * Cortex-M4's own DWT_CYCCNT free-running cycle counter -- instead of
 * any other timer, to busy-wait and timestamp, and touches NOTHING
 * else: no TIM1, no ADC/DMA, no TMR3, no TMR7, no output-compare, no
 * input-capture, no interrupts of any kind (global IRQs are disabled
 * for the entire measurement window). TMR2 itself is configured as
 * plainly as possible: Plus Mode (32-bit) enabled, PR=0xFFFFFFFF,
 * 500ns/tick, counting, nothing else touched on it at all.
 *
 * No motor involvement whatsoever.
 *
 * At each of 16 DWT-timed checkpoints (0/5/10/20/30/32/33/35/38/40/
 * 45/50/60/70/80/100 ms from a common start), records DWT_CYCCNT,
 * TMR2->cval, TMR2->ctrl1 (raw), the PMEN bit specifically, TMR2->pr,
 * and CRM's TMR2 clock-enable bit -- so if CVAL does stall, the
 * CTRL1/PMEN/clock-enable state at that exact moment is captured too,
 * not just the missing progress.
 *
 * Pass criteria: TMR2->cval tracks (DWT-measured elapsed_us * 2) to
 * within a few ticks at every one of the 16 checkpoints, all the way
 * out to 100ms.
 */

#include "clock_config.h"

#define DEMCR    (*(volatile uint32_t *)0xE000EDFCu)
#define DWT_CTRL (*(volatile uint32_t *)0xE0001000u)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004u)
#define DEMCR_TRCENA (1u << 24)
#define DWT_CTRL_CYCCNTENA (1u << 0)

#define CPU_HZ 96000000u

static const uint32_t checkpoint_ms[] = {0, 5, 10, 20, 30, 32, 33, 35, 38, 40, 45, 50, 60, 70, 80, 100};
#define NUM_CHECKPOINTS (sizeof(checkpoint_ms) / sizeof(checkpoint_ms[0]))

typedef struct {
  uint32_t target_ms;
  uint32_t dwt_cyccnt;     /* DWT_CYCCNT - start_cyccnt, i.e. elapsed CPU cycles */
  uint32_t tmr2_cval;
  uint32_t tmr2_ctrl1;     /* Raw CTRL1 register */
  uint32_t tmr2_pmen;      /* CTRL1.PMEN bit specifically, decoded */
  uint32_t tmr2_pr;
  uint32_t crm_tmr2_clken; /* CRM APB1EN.TMR2EN bit */
} stage_e28d_sample_t;

stage_e28d_sample_t stage_e28d_samples[NUM_CHECKPOINTS];
volatile uint32_t stage_e28d_heartbeat;
volatile int stage_e28d_done;

void _init(void) {}
void _fini(void) {}

static void dwt_init(void)
{
  DEMCR |= DEMCR_TRCENA;
  DWT_CYCCNT = 0;
  DWT_CTRL |= DWT_CTRL_CYCCNTENA;
}

static void tmr2_plain_freerun_init(void)
{
  crm_periph_reset(CRM_TMR2_PERIPH_RESET, TRUE);
  crm_periph_reset(CRM_TMR2_PERIPH_RESET, FALSE);
  crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);

  tmr_32_bit_function_enable(TMR2, TRUE); /* PMEN */
  tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
  tmr_base_init(TMR2, 0xFFFFFFFFu, 47u); /* 96MHz/48 = 2MHz, 500ns/tick */

  TMR2->iden = 0; /* All TMR2 interrupts explicitly disabled */

  tmr_event_sw_trigger(TMR2, TMR_OVERFLOW_SWTRIG); /* UG -- CNT starts at 0, commits PR/DIV */
  tmr_counter_enable(TMR2, TRUE);
}

static void snapshot(stage_e28d_sample_t *s, uint32_t target_ms, uint32_t start_cyccnt)
{
  s->target_ms = target_ms;
  s->dwt_cyccnt = DWT_CYCCNT - start_cyccnt;
  s->tmr2_cval = TMR2->cval;
  s->tmr2_ctrl1 = TMR2->ctrl1;
  s->tmr2_pmen = TMR2->ctrl1_bit.pmen;
  s->tmr2_pr = TMR2->pr;
  s->crm_tmr2_clken = CRM->apb1en_bit.tmr2en;
}

int main(void)
{
  clock_config_96mhz();

  dwt_init();
  tmr2_plain_freerun_init();

  __disable_irq(); /* No other code (including any NVIC-serviced interrupt) may interfere for the whole measurement window */

  uint32_t start_cyccnt = DWT_CYCCNT;

  for (uint32_t i = 0; i < NUM_CHECKPOINTS; i++) {
    uint32_t target_cycles = checkpoint_ms[i] * (CPU_HZ / 1000u);
    while ((uint32_t)(DWT_CYCCNT - start_cyccnt) < target_cycles);
    snapshot(&stage_e28d_samples[i], checkpoint_ms[i], start_cyccnt);
  }

  __enable_irq();

  stage_e28d_done = 1;

  for (;;) {
    ++stage_e28d_heartbeat;
  }
}
