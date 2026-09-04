/*
** ESCape32 AT32F425 (MOUSEF425) port -- backend implementation.
**
** libopencm3-facing (mirrors mcu/AT32F421/config.c's structure and, for
** pins/peripherals AT32F425 shares with AT32F421 -- TIM1 6PWM AF
** routing, IOTIM=TIM15/IO_PA2 -- reuses the exact same register
** values, since Stage E15D independently re-derived the identical
** MUX_2 encoding for TIM1's six PWM pins on this silicon). CRM/ADC/DMA
** are handled by artery_hal.c/h; see config.h's top comment for the
** compatibility rationale and the tim3_isr scheduler design.
*/

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/adc.h>
#include "common.h"
#include "artery_hal.h"

// Called explicitly from tim3_isr() below, NOT bound to any vector
// (see config.h's top comment) -- upstream's own prototype lives in
// src/common.h only for functions OWNED by config.c; iftim_isr() is
// owned by src/main.c (unmodified), so it needs its own declaration
// here.
void iftim_isr(void);

#ifdef BENCH_TEST
// AT32F425-only bench harness: bounded, logged first-spin validation of
// upstream ESCape32's own startup/ADC-ZC/break-before-make path, NOT a
// new stage_eXX-style FSM -- shared src/* motor-control code is
// untouched; throttle is injected via upstream's own ANALOG+THROT_SET
// mechanism (see add_target(MOUSEF425_BENCH ...), CMakeLists.txt),
// which skips initio()/IOTIM entirely (src/main.c: `#ifndef ANALOG
// initio(); #endif`) so no DSHOT/servo decode runs at all -- `throt`
// is simply left at its cfg.throt_set-initialized value forever, since
// this bench build's adctrig() never calls adcdata()'s analog-throttle
// path. All counters/log fields below are read via debugger/OpenOCD
// after the run, the same way Stage E24-E26 read back their results.
#define BENCH_TIME_LIMIT_TICKS 200 // 200 * 10ms = 2.0s
#define BENCH_COMMUTATION_LIMIT 60
#define BENCH_POST_ZC_COMMUTATION_LIMIT 12 // Stop after this many successful commutations following the first ADC-ZC confirm

enum { BENCH_STOP_NONE = 0, BENCH_STOP_COMMUTATION_LIMIT, BENCH_STOP_TIME_LIMIT, BENCH_STOP_POST_ZC_LIMIT, BENCH_STOP_FAULT };

// Internal bookkeeping only (not part of the read-back log set) --
// commutation count snapshotted the instant bench_zc_reached first
// became true, so BENCH_POST_ZC_COMMUTATION_LIMIT can be checked as a
// simple difference without adding another log field.
static uint32_t bench_commutation_count_at_zc;

volatile uint32_t bench_commutation_count;   // TIM1 COM events actually committed (tim3_isr's CC3 handler)
volatile uint32_t bench_confirm_count;       // ADC-ZC schmitt-trigger confirms (dma1_channel1_isr)
volatile uint32_t bench_zc_accepted_count;   // Confirms iftim_isr() itself accepted (CCR3 actually changed, not an early `t<ival>>1` reject)
// Min/max measured inter-commutation interval, in 100us units. NOTE:
// TIM7_CNT alone wraps every 10ms (ARR=99, see bench_test_init()) --
// too often to time-stamp commutation gaps that can span tens of ms --
// so the timestamp used here is a WIDE one combining the slow 10ms
// tick count with TIM7's own sub-tick count (bench_wide_timestamp()).
volatile uint32_t bench_ival_min_100us = 0xFFFFFFFFu;
volatile uint32_t bench_ival_max_100us;
volatile uint32_t bench_last_commit_100us;
volatile int bench_zc_reached;               // First ADC-ZC confirm seen -> genuinely left open-loop startup
volatile int bench_stopped;
volatile int bench_stop_reason; // BENCH_STOP_*
volatile uint32_t bench_elapsed_ticks; // TMR7 overflow count, 10ms/tick

static uint32_t bench_wide_timestamp(void) {
	// Two reads race against a TIM7 overflow landing in between (e.g.
	// elapsed_ticks read just before an overflow bumps it, then CNT
	// read just after it wraps to 0) -- reread elapsed_ticks after CNT
	// and use the later value if it changed, which is always safe
	// since both only ever increase.
	uint32_t ticks1 = bench_elapsed_ticks;
	uint32_t cnt = TIM7_CNT;
	uint32_t ticks2 = bench_elapsed_ticks;
	if (ticks2 != ticks1) cnt = TIM7_CNT; // An overflow landed between reads -- cnt is now post-wrap, reread it
	return ticks2 * 100u + cnt; // 100us units (10ms/tick = 100 * 100us)
}

static void bench_force_stop(int reason) {
	if (bench_stopped) return;
	bench_stopped = 1;
	bench_stop_reason = reason;
	TIM1_BDTR &= ~TIM_BDTR_MOE;
	throt = 0; // Also tell upstream's own control loop to want zero throttle from here on
}

void tim7_isr(void) {
	if (!(TIM7_SR & TIM_SR_UIF)) return;
	TIM7_SR = (uint16_t)~TIM_SR_UIF;
	if (bench_stopped) return;
	if (++bench_elapsed_ticks >= BENCH_TIME_LIMIT_TICKS) bench_force_stop(BENCH_STOP_TIME_LIMIT);
}

static void bench_test_init(void) {
	RCC_APB1ENR |= RCC_APB1ENR_TIM7EN;
	TIM7_PSC = CLK_MHZ * 100 - 1; // 100us/tick
	TIM7_ARR = 99;                // 100us * 100 = 10ms/overflow
	TIM7_CR1 = TIM_CR1_URS;
	TIM7_EGR = TIM_EGR_UG;
	TIM7_SR = (uint16_t)~TIM_SR_UIF;
	TIM7_DIER = TIM_DIER_UIE;
	nvic_set_priority(NVIC_TIM7_IRQ, 0x40);
	nvic_enable_irq(NVIC_TIM7_IRQ);
	TIM7_CR1 = TIM_CR1_CEN | TIM_CR1_URS;
}
#endif

#if COMP_MAP == 123
#define PHASE_IDX_1 0
#define PHASE_IDX_2 1
#define PHASE_IDX_3 2
#elif COMP_MAP == 231
#define PHASE_IDX_1 1
#define PHASE_IDX_2 2
#define PHASE_IDX_3 0
#elif COMP_MAP == 312
#define PHASE_IDX_1 2
#define PHASE_IDX_2 0
#define PHASE_IDX_3 1
#elif COMP_MAP == 132
#define PHASE_IDX_1 0
#define PHASE_IDX_2 2
#define PHASE_IDX_3 1
#elif COMP_MAP == 321
#define PHASE_IDX_1 2
#define PHASE_IDX_2 1
#define PHASE_IDX_3 0
#elif COMP_MAP == 213
#define PHASE_IDX_1 1
#define PHASE_IDX_2 0
#define PHASE_IDX_3 2
#endif
// at32_adc_buf[] index for each COMP_INn (1/2/3), i.e. which physical
// BEMF ADC channel (A=PA0=0, B=PA4=1, C=PA5=2) each comparator-input
// identity maps to on this board -- same COMP_MAP convention comparator
// backends use, just resolved to an ADC channel index instead of a
// comparator MUX selector. See src/defs.h/mcu/AT32F421/config.c for
// the identical table on the comparator side.
static const uint8_t comp_in_to_adc_idx[4] = {0xff, PHASE_IDX_1, PHASE_IDX_2, PHASE_IDX_3};

#define PWM_ARR (CLK_KHZ / 24 - 1) // 24kHz, matches upstream main()'s own TIM1_ARR at boot

#define ZC_CONFIRM_COUNT 3
#define ZC_DEADBAND 8
#define POST_COMMUTATION_BLANK_SCANS 2
#define ADC_TRIGGER_OFFSET_TICKS 115 // ~1.2us, Stage E14/E23-validated

// ZC state, armed by compctl() (called from upstream's nextstep(),
// ONE commutation ahead of when it's needed -- same one-ahead
// preparation nextstep() does for CCMR/CCER shadow content) and
// consumed by dma1_channel1_isr() (Stage E14's validated Schmitt-
// trigger dual-arm design: both rising and falling are armed
// simultaneously, whichever confirms first wins -- no assumption about
// which direction is "expected", since that table was invalidated and
// discarded during bring-up).
static volatile int zc_floating_idx = -1; // -1 = disarmed (compctl(0), e.g. at boot/stop)
static volatile int zc_blank_remaining;
typedef struct { int confirmed_sign; int confirm_run; } zc_filter_t;
static volatile zc_filter_t zc_rise, zc_fall;

void compctl(int x) {
	int sel = x & 3;
	if (!sel) {
		zc_floating_idx = -1;
		return;
	}
	zc_floating_idx = comp_in_to_adc_idx[sel];
	zc_rise.confirmed_sign = -1;
	zc_rise.confirm_run = 0;
	zc_fall.confirmed_sign = 1;
	zc_fall.confirm_run = 0;
	zc_blank_remaining = POST_COMMUTATION_BLANK_SCANS;
}

static int zc_filter_update(volatile zc_filter_t *f, int diff) {
	int qualifies, new_sign;
	if (f->confirmed_sign <= 0) {
		new_sign = 1;
		qualifies = diff >= ZC_DEADBAND;
	} else {
		new_sign = -1;
		qualifies = diff <= -ZC_DEADBAND;
	}
	if (qualifies) {
		if (f->confirm_run < ZC_CONFIRM_COUNT) f->confirm_run++;
	} else f->confirm_run = 0;
	if (f->confirm_run >= ZC_CONFIRM_COUNT) {
		f->confirm_run = 0;
		int prev = f->confirmed_sign;
		f->confirmed_sign = new_sign;
		if (prev != new_sign) return 1;
	}
	return 0;
}

void dma1_channel1_isr(void) {
	if (!at32_bemf_dma_transfer_complete()) return; // Artery-facing flag check -- see artery_hal.h
	if (zc_floating_idx < 0) return;
	int v = at32_adc_buf[zc_floating_idx];
	// Positive phase = the one energized this sector, always at
	// (zc_floating_idx+1)%3 or +2 -- but simplest/most robust is just
	// "whichever of the other two reads higher" (the negative phase
	// sits near 0 during ON-time in this drive scheme -- see Stage E14).
	int a = at32_adc_buf[0], b = at32_adc_buf[1], c = at32_adc_buf[2];
	int pos = a > b ? (a > c ? a : c) : (b > c ? b : c);
	int diff = v - pos / 2; // Stage E14-validated threshold
	if (zc_blank_remaining) {
		zc_blank_remaining--;
		return;
	}
	if (zc_filter_update(&zc_rise, diff) || zc_filter_update(&zc_fall, diff)) {
		zc_floating_idx = -1; // Disarm until compctl() re-arms for the next sector
		TIM3_EGR = TIM_EGR_CC1G; // Software "capture" -- Stage E24-validated: captures current
		                          // CNT into CCR1 and fires CC1 IRQ exactly like a real edge.
#ifdef BENCH_TEST
		if (!bench_stopped) {
			bench_confirm_count++;
			if (!bench_zc_reached) {
				bench_zc_reached = 1;
				bench_commutation_count_at_zc = bench_commutation_count;
			}
		}
#endif
	}
}

// F425-specific commutation scheduler. Wraps upstream's unmodified
// iftim_isr() (called explicitly, NOT bound to this vector -- see
// config.h). Upstream computes IFTIM_OCR (=TIM3_CCR3) as the delay to
// the next commutation and, on real STM32/AT32F421, relies on a
// hardware TRGO->TIM1-COM path to act on it (not used here -- see
// config.h). Instead we independently re-arm TIM3 CC2 (gate-off, 2us
// before target) and CC3 (commit, at target) every cycle, since
// upstream unconditionally rewrites TIM_DIER(IFTIM) on every call.
//
// KNOWN SIMPLIFICATION (flagged, not yet hardware-hardened): this
// processes UIF/CC1IF/CC2IF/CC3IF as independent sequential ifs against
// one SR snapshot rather than re-reading SR after each action. At the
// commutation rates this design targets that should be safe, but this
// is the least hardware-validated new code in this port (unlike TIM1
// PWM/GPIO/ADC/DMA, which reuse Stage A-E26-confirmed values) and is
// exactly the kind of thing Stage E24-E26 verified empirically before
// being trusted -- expect this scheduler specifically may need its own
// bring-up-style verification pass before being trusted at speed.
void tim3_isr(void) {
#ifdef BENCH_TEST
	if (bench_stopped) {
		TIM3_SR = 0;
		TIM1_BDTR &= ~TIM_BDTR_MOE; // Belt-and-suspenders: keep re-asserting on every subsequent tick
		return;
	}
#endif
	uint32_t sr = TIM3_SR;
	if (sr & (TIM_SR_UIF | TIM_SR_CC1IF)) {
#ifdef BENCH_TEST
		uint16_t ccr3_before = (uint16_t)TIM3_CCR3;
#endif
		iftim_isr();
#ifdef BENCH_TEST
		if (!(sr & TIM_SR_UIF) && (uint16_t)TIM3_CCR3 != ccr3_before) bench_zc_accepted_count++;
#endif
		if (sr & TIM_SR_UIF) {
			// Sync-loss timeout (see iftim_isr()'s UIF branch, src/main.c):
			// IFTIM_OCR/CCR3 was NOT updated by this call and still holds
			// a stale, meaningless target -- do NOT reschedule CC2/CC3 off
			// it. Disarm both and make sure we never leave gates stuck off
			// from a prior cycle's gate-off (CC2) that never got a
			// matching commit (CC3) before sync was declared lost.
			TIM3_DIER &= ~(TIM_DIER_CC2IE | TIM_DIER_CC3IE);
			TIM3_SR = (uint16_t)~(TIM_SR_CC2IF | TIM_SR_CC3IF);
			TIM1_BDTR |= TIM_BDTR_MOE;
#ifdef BENCH_TEST
			// A UIF timeout during the initial open-loop ramp is normal
			// (ARR/OCR are both set to the same large startup value in
			// main()'s "Start motor" sequence, so early commutations are
			// legitimately UIF-driven -- confirmed by the first bounded
			// run: 4 commutations at a rock-steady ~32.7ms, exactly
			// IFTIM's ARR period). Only treat it as a fault once we'd
			// already reached real ADC-ZC-driven operation and then lost
			// it -- that is an actual desync/stall, not startup ramping.
			if (!bench_stopped && bench_zc_reached) bench_force_stop(BENCH_STOP_FAULT);
#endif
		} else {
			uint16_t target = (uint16_t)TIM3_CCR3;
			// GATE_OFF_ADVANCE_TICKS can exceed target at very short
			// commutation intervals (IFTIM_OCR's own floor is 1, see
			// main.c) -- a plain subtraction would wrap a uint16_t to
			// near-ARR and schedule gate-off ~32ms in the future instead
			// of 2us before commit. When there isn't enough of the
			// interval left for a break, skip the gate-off phase for
			// this one cycle rather than mis-schedule it; the commit
			// (CC3, always upstream's own already-valid target) still
			// happens on time.
			if (target > GATE_OFF_ADVANCE_TICKS) {
				TIM3_CCR2 = (uint16_t)(target - GATE_OFF_ADVANCE_TICKS);
				TIM3_DIER |= TIM_DIER_CC2IE;
			} else {
				TIM3_DIER &= ~TIM_DIER_CC2IE;
			}
			TIM3_SR = (uint16_t)~(TIM_SR_CC2IF | TIM_SR_CC3IF); // Drop stale flags from the just-UG'd counter
			TIM3_DIER |= TIM_DIER_CC3IE;
		}
	}
	if (sr & TIM_SR_CC2IF) {
		TIM3_SR = (uint16_t)~TIM_SR_CC2IF;
		TIM1_BDTR &= ~TIM_BDTR_MOE; // All outputs Hi-Z -- break-before-make starts
	}
	if (sr & TIM_SR_CC3IF) {
		TIM3_SR = (uint16_t)~TIM_SR_CC3IF;
		TIM1_EGR = TIM_EGR_COMG; // Commits preloaded shadow->active (hardware-immediate) and
		                          // queues tim1_com_isr()->nextstep() to prepare the NEXT sector
		TIM1_BDTR |= TIM_BDTR_MOE; // Re-open gates -- the correct new sector is already active
#ifdef BENCH_TEST
		if (!bench_stopped) {
			uint32_t now = bench_wide_timestamp(); // 100us units, monotonic across the whole bounded run
			if (bench_commutation_count) { // Skip the very first commit -- no valid prior timestamp yet
				uint32_t interval = now - bench_last_commit_100us;
				if (interval < bench_ival_min_100us) bench_ival_min_100us = interval;
				if (interval > bench_ival_max_100us) bench_ival_max_100us = interval;
			}
			bench_last_commit_100us = now;
			++bench_commutation_count;
			if (bench_commutation_count >= BENCH_COMMUTATION_LIMIT) {
				bench_force_stop(BENCH_STOP_COMMUTATION_LIMIT);
			} else if (bench_zc_reached && bench_commutation_count - bench_commutation_count_at_zc >= BENCH_POST_ZC_COMMUTATION_LIMIT) {
				bench_force_stop(BENCH_STOP_POST_ZC_LIMIT);
			}
		}
#endif
	}
}

void init(void) {
	at32_clock_init(); // Artery-facing (CRM/PLL) -- see artery_hal.c
	at32_dma_flexible_routing_init(); // Artery-facing (DMA request routing) -- see artery_hal.c

	// Peripheral clock enables: plain libopencm3 RCC_* writes, same as
	// mcu/AT32F421/config.c -- CRM's peripheral-enable bit positions
	// match libopencm3's RCC assumptions on this AT32 family (confirmed
	// address/offset match for CRM_TMR1_PERIPH_CLOCK etc during this
	// session's header cross-referencing); it's CRM's clock-SOURCE/PLL
	// configuration registers that genuinely differ and are handled
	// exclusively via Artery calls in artery_hal.c, never here.
	RCC_AHBENR = RCC_AHBENR_DMAEN | RCC_AHBENR_SRAMEN | RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
	RCC_APB2ENR = RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM15EN;
	RCC_APB1ENR = RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_USART2EN;
	SCB_VTOR = (uint32_t)_rom; // Set vector table address

	// TIM1 6PWM: PA7(CH1N)/PA8(CH1)/PA9(CH2)/PB0(CH2N)/PA10(CH3)/PB1(CH3N),
	// all MUX_2 -- Stage E15D-confirmed encoding, numerically identical
	// to AT32F421/STM32F0's own AF2 assignment for these pins.
	GPIOA_AFRL = 0x20000000; // A7 (TIM1_CH1N)
	GPIOA_AFRH = 0x00000222; // A8 (TIM1_CH1), A9 (TIM1_CH2), A10 (TIM1_CH3)
	GPIOB_AFRL = 0x00000022; // B0 (TIM1_CH2N), B1 (TIM1_CH3N)
	{
		uint32_t mask = (3u << (2 * 7)) | (3u << (2 * 8)) | (3u << (2 * 9)) | (3u << (2 * 10));
		uint32_t val = (2u << (2 * 7)) | (2u << (2 * 8)) | (2u << (2 * 9)) | (2u << (2 * 10)); // AF mode = 0b10
		GPIOA_MODER = (GPIOA_MODER & ~mask) | val; // A7,A8,A9,A10 = AF
	}
	{
		uint32_t mask = (3u << (2 * 0)) | (3u << (2 * 1));
		uint32_t val = (2u << (2 * 0)) | (2u << (2 * 1)); // AF mode = 0b10
		GPIOB_MODER = (GPIOB_MODER & ~mask) | val; // B0,B1 = AF
	}

	// PA2: IOTIM(TIM15) CH1 input, plain digital input mode (not AF) --
	// same pattern mcu/AT32F421/config.c uses for its IO_PA2/TIM15
	// pairing (the timer's TI1 reads the GPIO input path directly on
	// this family; MODER stays INPUT(00), only pull-up is set).
	GPIOA_PUPDR = (GPIOA_PUPDR & ~(3 << (2 * 2))) | (1 << (2 * 2)); // A2 pull-up
	GPIOA_MODER &= ~(3 << (2 * 2)); // A2 input

	at32_bemf_adc_dma_init(); // PA0/PA4/PA5 analog -- configured entirely inside artery_hal.c

	nvic_set_priority(NVIC_TIM3_IRQ, 0x40);
	nvic_set_priority(NVIC_TIM1_BRK_UP_TRG_COM_IRQ, 0x40);
	nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 0x80);
	nvic_set_priority(NVIC_TIM15_IRQ, 0x40);
	nvic_set_priority(NVIC_USART2_IRQ, 0x40);
	nvic_set_priority(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ, 0x80);
	nvic_set_priority(NVIC_DMA1_CHANNEL4_7_DMA2_CHANNEL3_5_IRQ, 0x40);

	nvic_enable_irq(NVIC_TIM1_BRK_UP_TRG_COM_IRQ);
	nvic_enable_irq(NVIC_TIM3_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
	nvic_enable_irq(NVIC_TIM15_IRQ);
	nvic_enable_irq(NVIC_USART2_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL4_7_DMA2_CHANNEL3_5_IRQ);

	// TIM1 CH4: ADC hardware trigger, PWM_MODE_B (STM32 PWM2 -- output/
	// trigger transitions AT the CCR4 compare match, unlike PWM_MODE_A
	// which is fixed near CNT=0), fixed offset -- Stage E14/E23-validated.
	// Owned exclusively by this backend; nextstep()'s own CH4 use is
	// compiled out via COMP_BLANK_CH4_INIT/SET (config.h, ADC_ZC_BACKEND).
	TIM1_CCMR2 = TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M_PWM2;
	TIM1_CCER |= TIM_CCER_CC4E;
	TIM1_CCR4 = ADC_TRIGGER_OFFSET_TICKS;

#ifdef BENCH_TEST
	bench_test_init();
#endif
}

// initgpio()/inittelem()/sendtelem()/sendtelemdata() are NOT
// implemented here -- src/util.c's initgpio() and src/telem.c's
// telemetry functions are shared/portable (fully driven by config.h
// macros and optional per-target defines like BEC_MAP/ERPM_PIN, none
// of which this target sets), not per-backend hooks; defining them
// here would collide with those (real, non-weak) definitions. TIM1's
// BDTR dead-time/PSC(IFTIM) setup that would have lived in a
// initgpio() override is likewise unnecessary -- main() (src/main.c)
// already writes both itself (`TIM1_BDTR = TIM_DTG | ... | MOE;`,
// `TIM_PSC(IFTIM) = ...`) using the same portable macros.

void initled(void) {}
void ledctl(int x) { (void)x; }

void hsictl(int x) { (void)x; } // No internal-oscillator trim path validated yet

void io_serial(void) {
	nvic_disable_irq(NVIC_TIM15_IRQ);
	TIM15_CR1 = 0;
	GPIOA_AFRL = (GPIOA_AFRL & ~(0xfu << (4 * 2))) | (1u << (4 * 2)); // A2 = USART2_TX (AF1, unverified placeholder)
	GPIOA_MODER = (GPIOA_MODER & ~(3u << (2 * 2))) | (2u << (2 * 2)); // A2 = AF
}

void io_analog(void) {
	nvic_disable_irq(NVIC_TIM15_IRQ);
	TIM15_CR1 = 0;
	GPIOA_PUPDR &= ~(3u << (2 * 2));
	GPIOA_MODER |= (3u << (2 * 2)); // A2 = analog
}

void adctrig(void) {
	// Housekeeping ADC (temp/volt/curr/analog throttle) preempt-group
	// wiring (decision 4-2: PendSV only sets a pending flag, the
	// preempt conversion itself starts from a safe instant right after
	// BEMF ordinary-group DMA completion) is not yet implemented -- see
	// project notes. SENS_MAP is intentionally left undefined for this
	// first bring-up target (SENS_CNT=0), so adcdata()'s volt/curr path
	// is compiled out and this being a no-op does not block startup or
	// closed-loop entry, only telemetry/protection features.
}
