/*
** ESCape32 AT32F425 (MOUSEF425) port -- backend implementation.
**
** libopencm3-facing (mirrors mcu/AT32F421/config.c's structure and, for
** pins/peripherals AT32F425 shares with AT32F421 -- TIM1 6PWM AF
** routing, IOTIM=TIM15/IO_PA2 -- reuses the exact same register
** values, since Stage E15D independently re-derived the identical
** MUX_2 encoding for TIM1's six PWM pins on this silicon). CRM/ADC/DMA
** are handled by artery_hal.c/h; see config.h's top comment for the
** compatibility rationale.
**
** TIMER ROLES (see config.h's top comment for the Stage E27/E28/E28B/
** E28C/E28D history that led here, and the "invariant absorption"
** redesign below):
**   TIM1  : 6PWM + CH4 ADC hardware trigger
**   TIM2  : IFTIM -- upstream's BEMF timing core. 32-bit Plus Mode,
**           PMEN=1 and ARR=0xFFFFFFFF PERMANENTLY (established once in
**           init(), never changed again by anyone, upstream included).
**   TIM3  : 2us break-before-make scheduler ONLY (no longer IFTIM).
**   TIM7  : Independent 32.767ms IFTIM-timeout generator (replaces
**           IFTIM's own ARR-overflow-driven UIF -- see below).
**   TIM14 : BENCH_TEST elapsed-time watchdog (moved off TIM7).
**   TIM15 : IOTIM/DSHOT.
**
** WHY TIM2's ARR CANNOT ACTUALLY CHANGE, EVEN MOMENTARILY: upstream's
** own `t < ival >> 1` accept check reads TIM2_CNT live; if ARR is ever
** genuinely programmed to a small value (upstream's own code, ported
** from a 16-bit-timer assumption, routinely tries to set it to 0 or
** 0xFFFF) WHILE the real counter is running, a real hardware auto-
** reload/wrap can occur during that window and permanently corrupt the
** "ticks since last reset" value -- this is a real-time race that a
** repair *after* the fact (however immediate) cannot undo, since the
** wrap already happened in hardware before the repair runs. The fix
** implemented here is therefore ABSORPTION, not repair: upstream's own
** writes to IFTIM's CR1/ARR are routed through src/defs.h's portable
** IFTIM_CR1_WRITE()/IFTIM_ARR_WRITE()/IFTIM_RESET()/IFTIM_TIMEOUT_ARM()/
** IFTIM_TIMEOUT_DISARM() macros (default: expand to the original plain
** register writes, so every other target is byte-for-byte unaffected).
** This backend overrides them (config.h) so real TIM2 CR1/ARR are
** simply never exposed to upstream's intended values at all:
**   IFTIM_CR1_WRITE(v) -> at32_iftim_cr1_write(v): ORs upstream's
**     intended CEN/ARPE/URS/CKD bits with TIM_CR1_PMEN_BIT unconditionally
**     and writes that -- PMEN can never be observably cleared, because
**     this is the ONLY place in the F425 backend that ever writes TIM2_CR1
**     (init()'s own one-time setup routes through it too).
**   IFTIM_ARR_WRITE(v) -> discarded entirely (v is never applied to real
**     hardware) -- real TIM2_ARR is set to 0xFFFFFFFF exactly once, in
**     init(), and never touched again.
**   IFTIM_RESET() -> at32_iftim_reset(): still genuinely resets TIM2's
**     real CVAL to 0 (upstream's own reset-on-accept/sine-start/motor-
**     start/motor-stop semantics all need a REAL reset here -- ARR being
**     permanently 0xFFFFFFFF makes this safe, no wrap can race it) AND
**     re-arms TIM7's independent timeout from the same zero point.
**   IFTIM_TIMEOUT_ARM()/DISARM() -> left at src/defs.h's default (still
**     real TIM2 DIER writes) for now -- harmless, since real TIM2 UIF
**     essentially never fires with ARR pinned at 0xFFFFFFFF, and kept
**     unchanged until DIER/SR's independence from iftim_isr()'s own
**     internal reads is separately confirmed.
**
** THE 32.767ms TIMEOUT (upstream's stall/no-BEMF detector, originally a
** side effect of IFTIM's own 16-bit ARR overflow -- NOT just a width
** artifact, so it cannot simply be dropped): TIM7 is a completely
** independent peripheral (never referenced by any shared src/ code)
** free-running at the same 500ns/tick rate, ARR = upstream's own
** literal 65535 value, reset in lockstep with TIM2 by
** at32_iftim_reset() every time upstream issues IFTIM_RESET() (accepted
** ZC, sine-startup, motor start, motor stop -- ALL of upstream's own
** EGR=UG call sites, not just the accept path). When TIM7 itself times
** out (no reset arrived within 32.767ms), tim7_isr() calls
** iftim_timeout() -- a small helper extracted from iftim_isr()'s own
** UIF branch (src/main.c, behaviorally unchanged for every other
** target) -- DIRECTLY, rather than assuming a fabricated TIM2 UIF/SR
** state would make iftim_isr() take that branch itself; no TIM2 EGR=UG
** is used to synthesize a fake UIF anywhere.
**
** How a real ADC-ZC confirm flows through TIM2/TIM3/TIM7:
**   1. dma1_channel1_isr() confirms a zero-cross (Stage E14's
**      Schmitt-trigger dual-arm design) and calls iftim_isr()
**      (src/main.c, unmodified) DIRECTLY -- no capture register is
**      used at all (CH1 software capture is deliberately NOT used,
**      per instruction; TIM2 CH1 is never configured as an input-
**      capture channel). Since IFTIM_ICR is #define'd (config.h) to
**      TIM2_CNT -- a LIVE counter read, not a captured register --
**      upstream's `t = IFTIM_ICR` reads "ticks since IFTIM was last
**      reset" at the exact instant we call it.
**   2. If iftim_isr() accepts (detected by us: IFTIM_OCR, aka
**      TIM2_CCR3, changed value), upstream's own accept-branch
**      IFTIM_RESET() call has ALREADY reset TIM2's CVAL to 0 and
**      re-armed TIM7 (at32_iftim_reset()). We then resynchronize TIM3
**      (the scheduler) to the same zero point (iftim_reschedule()) and
**      reprogram its CC2 (gate-off)/CC3 (commit) targets directly from
**      the new IFTIM_OCR value -- both timers tick at the identical
**      500ns rate, so no unit conversion is needed, just a copy.
**   3. tim7_isr() mirrors this for the "no ZC arrived in time" case:
**      calls iftim_timeout() (resets sync but does NOT change
**      IFTIM_OCR), then at32_iftim_reset(), then UNCONDITIONALLY
**      reschedules TIM3 from whatever IFTIM_OCR currently holds. This
**      reproduces the "commutate every ~32.7ms" bootstrap behavior the
**      OLD (TIM3-as-IFTIM, pre-migration) design got for free from
**      ARR==OCR hardware coincidence.
**   4. tim3_isr() itself only does the 2us-before/commit scheduling
**      (CC2 -> MOE off, CC3 -> EGR_COMG + MOE on) -- no IFTIM/timeout
**      logic at all.
*/

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/f1/adc.h>
#include "common.h"
#include "artery_hal.h"

// Called explicitly from dma1_channel1_isr()/tim7_isr() below, NOT
// bound to any vector (see config.h's top comment) -- upstream's own
// prototypes live in src/common.h only for functions OWNED by
// config.c; iftim_isr()/iftim_timeout() are owned by src/main.c
// (unmodified/behaviorally-unmodified respectively), so they need
// their own declarations here.
void iftim_isr(void);
void iftim_timeout(void);

// CTRL1 bit10 (PMEN / "Plus Mode", i.e. 32-bit counting) -- reserved/
// unused at this bit position on real STM32 TIM_CR1, so libopencm3 has
// no macro for it. See artery_hal.c's at32f425_tmr.h cross-reference
// (tmr_32_bit_function_enable() sets exactly this bit) and this file's
// top comment for the "absorption, not repair" design this now backs.
#define TIM_CR1_PMEN_BIT (1u << 10)

// The ONLY place TIM2_CR1 is ever written in this backend (init() below
// routes its own one-time setup through this too) -- PMEN can therefore
// never be observably cleared by anyone, upstream included. Backs
// IFTIM_CR1_WRITE() (config.h).
void at32_iftim_cr1_write(uint32_t v) {
	TIM2_CR1 = v | TIM_CR1_PMEN_BIT;
}

#ifdef IFTIM_RESET_LOG
// Verification-only: logs DWT_CYCCNT (Cortex-M4's own free-running
// cycle counter, independent of TMR2/TMR3/TMR7 -- same technique Stage
// E28D used) at every IFTIM_RESET() call, so the spacing between resets
// (expected: TIM7's own 32.767ms timeout while idle, or whatever real
// ZC/commutation drives it once spinning) can be read back ONCE after
// letting the target run completely undisturbed for a while, instead of
// inferred from repeated debugger halt/resume polling -- halting over
// this board's SWD link has turned out to add highly variable (tens to
// hundreds of ms) latency of its own, which was confounding live
// TIM2/TIM7 register polling. Not gated behind BENCH_TEST -- enabled
// only via a dedicated build define, verification-only, never on by
// default.
#define DEMCR (*(volatile uint32_t *)0xE000EDFCu)
#define DWT_CTRL (*(volatile uint32_t *)0xE0001000u)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004u)
#define IFTIM_RESET_LOG_N 24
volatile uint32_t iftim_reset_log[IFTIM_RESET_LOG_N];
volatile uint32_t iftim_reset_log_count;

static void at32_iftim_reset_log_capture(void) {
	uint32_t n = iftim_reset_log_count++;
	if (n < IFTIM_RESET_LOG_N) iftim_reset_log[n] = DWT_CYCCNT;
}

static void at32_iftim_reset_log_init(void) {
	DEMCR |= (1u << 24); // TRCENA
	DWT_CYCCNT = 0;
	DWT_CTRL |= 1u; // CYCCNTENA
}
#endif

// Backs IFTIM_RESET() (config.h) -- upstream's own reset-on-accept/
// sine-start/motor-start/motor-stop semantics (src/main.c, every one of
// its own TIM_EGR(IFTIM)=UG call sites now routes through this). Real
// TIM2 CVAL genuinely resets to 0 here (safe: ARR is permanently
// 0xFFFFFFFF, so no auto-reload/wrap can ever race this). TIM7 (the
// independent 32.767ms timeout, see this file's top comment) is
// re-armed from the same zero point in lockstep.
void at32_iftim_reset(void) {
	TIM2_EGR = TIM_EGR_UG;
	TIM7_EGR = TIM_EGR_UG;
	TIM7_SR = (uint16_t)~TIM_SR_UIF; // Drop a stale pending timeout from before this reset
#ifdef IFTIM_RESET_LOG
	at32_iftim_reset_log_capture();
#endif
}

// --- HardFault diagnostics --------------------------------------------
// NOT BENCH_TEST-gated: the HardFault under investigation reproduces on
// plain MOUSEF425 too. Read via debugger after a fault (or by polling
// hardfault_diag.magic if still executing telemetry, though in practice
// hard_fault_handler() ends in a 1s delay + watchdog reset, so a
// debugger halt immediately after the fault is the intended path).
//
// SP/LR are captured INLINE by the HARDFAULT_CAPTURE() macro (config.h)
// at hard_fault_handler()'s very first statement, before any `bl` can
// clobber LR (which holds EXC_RETURN at that point) -- at32_hardfault_sp
// is therefore hard_fault_handler's OWN post-prologue SP, not the raw
// hardware-stacked frame. HARDFAULT_PROLOGUE_WORDS is the word offset
// from that SP up to the actual hardware frame, i.e. exactly what
// hard_fault_handler()'s own compiler-generated prologue pushed before
// reaching HARDFAULT_CAPTURE() -- verified by disassembling the built
// ELF (`arm-none-eabi-objdump -d` on hard_fault_handler), NOT guessed;
// re-verify this if hard_fault_handler()'s body, the compiler, or
// optimization level ever changes.
#define HARDFAULT_PROLOGUE_WORDS 2 // push {r3, lr} -- confirmed via objdump, see report

volatile uint32_t at32_hardfault_sp, at32_hardfault_lr;

typedef struct {
	uint32_t magic;
	uint32_t r0, r1, r2, r3, r12, lr, pc, xpsr; // Auto-stacked hardware frame
	uint32_t exc_lr;    // LR at hard_fault_handler entry == EXC_RETURN (bit2: 0=MSP/1=PSP; this target never uses PSP, so this should always read MSP-pattern)
	uint32_t cfsr, hfsr, icsr, shcsr, vtor;
	uint32_t vectactive;  // ICSR bits[8:0] -- which exception/IRQ was active when the fault entered
	uint32_t vectpending; // ICSR bits[20:12] -- highest-priority pending exception/IRQ at that instant
} hardfault_diag_t;
volatile hardfault_diag_t hardfault_diag;

#define HARDFAULT_DIAG_MAGIC 0xfa17fa17u

void at32_hardfault_capture(void) {
	uint32_t *frame = (uint32_t *)at32_hardfault_sp + HARDFAULT_PROLOGUE_WORDS;
	hardfault_diag.r0 = frame[0];
	hardfault_diag.r1 = frame[1];
	hardfault_diag.r2 = frame[2];
	hardfault_diag.r3 = frame[3];
	hardfault_diag.r12 = frame[4];
	hardfault_diag.lr = frame[5];
	hardfault_diag.pc = frame[6];
	hardfault_diag.xpsr = frame[7];
	hardfault_diag.exc_lr = at32_hardfault_lr;
	hardfault_diag.cfsr = SCB_CFSR;
	hardfault_diag.hfsr = SCB_HFSR;
	hardfault_diag.icsr = SCB_ICSR;
	hardfault_diag.shcsr = SCB_SHCSR;
	hardfault_diag.vtor = SCB_VTOR;
	hardfault_diag.vectactive = SCB_ICSR & 0x1ffu;
	hardfault_diag.vectpending = (SCB_ICSR >> 12) & 0x1ffu;
	hardfault_diag.magic = HARDFAULT_DIAG_MAGIC; // Written LAST -- a debugger checking this first is guaranteed to see fully-populated fields
}

#ifdef BENCH_TEST
// Read-only diagnostic visibility into upstream's iftim_isr() accept/
// reject decision (src/main.c: `if (t < ival >> 1) return;`), its
// sine-startup ramp state, and its own erpm-based duty release gate
// (`if (sync < 6) return;`, nextstep()) -- step/ival/sine/sync had
// `static` removed (main.c, zero behavior change) specifically for
// this. The algorithm itself is not touched anywhere. ertm/erpm are
// already non-static (src/common.h) and need no change.
extern int step, ival, sine;
extern char sync;
#endif

#ifdef BENCH_TEST
// AT32F425-only bench harness: bounded, logged validation of upstream
// ESCape32's own startup/ADC-ZC/break-before-make path, NOT a new
// stage_eXX-style FSM -- shared src/* motor-control code is untouched.
// Throttle is injected via upstream's own ANALOG mechanism (see
// add_target(MOUSEF425_BENCH ...), CMakeLists.txt -- skips
// initio()/IOTIM entirely, `throt` is fully ours to write).
//
// REDESIGNED (per instruction, correcting an earlier misframing of this
// harness's own goal): "reaching duty_max=100%" is NOT a meaningful
// target -- PWM duty and rotation speed are related but distinct
// (duty sets applied voltage; actual RPM is what the motor achieves in
// response, gated by how fast the closed loop can safely track
// acceleration). This harness no longer force-steps cfg.duty_max/
// duty_spup through fixed percentages at all. Instead, cfg.duty_max is
// set to its real final ceiling (100) ONCE, and cfg.duty_spup/
// duty_ramp/duty_rate -- upstream's OWN, unmodified erpm-gated
// acceleration governor, specifically designed to open up duty only as
// fast as measured rotation actually keeps up (avoiding desync/"step-
// out") -- are left to do the actual pacing, exactly as they would on
// a real product with a real throttle stick. throt is ramped up once
// (crossing cfg.sine_range's boundary so sine-startup engages, then on
// up toward max) and then upstream's own control loop is solely
// responsible for how much of that gets applied. What's measured here
// is the OUTCOME: achieved commutation interval (-> erpm), how far
// `sync` gets (0-6, upstream's own "how locked is closed-loop" signal),
// and whether/when synchronization is lost -- not any duty percentage.
#define BENCH_TIME_LIMIT_TICKS 500 // 500 * 10ms = 5.0s hard overall limit
#define BENCH_LOCK_SYNC_THRESHOLD 6 // upstream's own full-lock threshold (nextstep(): `if (sync < 6) return;`) -- once reached, a subsequent ZC timeout is treated as a genuine desync (stop), not startup noise

#define BENCH_CONFIRM_LOG_N 16
typedef struct {
	uint32_t cval_at_confirm; // TIM2_CNT (=upstream's `t`) read just before calling iftim_isr() -- no capture register involved
	uint32_t ival_snapshot;   // upstream's `ival`, read at the same moment (see extern above)
	uint32_t ival_half;       // ival_snapshot >> 1 -- the exact threshold `t` is compared against
	uint8_t accepted;         // 1 if iftim_isr() actually updated IFTIM_OCR (accepted), 0 if rejected
	uint8_t step_snapshot;    // upstream's `step` (1-6) at the same moment
	int8_t floating_phase;    // 0/1/2 = A/B/C (this backend's at32_adc_buf index), -1 = disarmed
	uint8_t polarity;         // compctl()'s x&4 bit for the sector that armed this confirm
	uint32_t since_commit_100us;       // elapsed time from the most recent commit to this confirm, 100us units
	uint32_t confirm_timestamp_100us;  // Absolute bench_wide_timestamp() at this confirm
	uint32_t since_prev_confirm_100us; // confirm_timestamp_100us minus the PREVIOUS confirm's (any confirm, accepted or not) -- 0 for the first-ever confirm
	uint32_t sine_snapshot;  // upstream's `sine` at this confirm -- nonzero means still in the sine-startup ramp, 0 means already handed over to 6-step
	uint32_t ertm_snapshot;  // upstream's `ertm` (electrical revolution time, us) at this confirm
	uint32_t throt_snapshot; // this file's own throt ramp value at this confirm (see bench_test_init()/tim14_isr())
} bench_confirm_log_t;
volatile bench_confirm_log_t bench_confirm_log[BENCH_CONFIRM_LOG_N];
volatile uint32_t bench_confirm_log_count; // Total confirms seen (keeps counting past BENCH_CONFIRM_LOG_N)

enum { BENCH_STOP_NONE = 0, BENCH_STOP_TIME_LIMIT_LOCKED, BENCH_STOP_TIME_LIMIT_UNLOCKED, BENCH_STOP_FAULT, BENCH_STOP_UNSAFE_RESET, BENCH_STOP_ZC_TIMEOUT };

// Reset cause (AT32_RESET_CAUSE_* bitmask, artery_hal.h), read/cleared
// in init() BEFORE at32_clock_init() (crm_reset() may clear the CRM
// flags). A non-power-on cause holds the motor off entirely for the
// rest of this boot -- see bench_test_init().
volatile unsigned bench_reset_cause;

// Single continuous run -- no more per-duty-percentage stages (see this
// file's top comment). Tracks the OUTCOME of letting upstream's own
// throttle/erpm-gated acceleration run: how fast rotation actually got
// (interval_100us_min = the achieved TOP speed) and how far upstream's
// own lock indicator (`sync`) got, not any duty percentage.
typedef struct {
	uint32_t confirm_count;     // ADC-ZC schmitt-trigger confirms (candidates only -- see zc_accepted_count for the accept-gated count)
	uint32_t zc_accepted_count; // Of those, how many iftim_isr() itself accepted (IFTIM_OCR actually changed, not an early `t<ival>>1` reject)
	uint32_t commutation_count; // TIM1 COM events committed
	uint32_t uif_count;         // IFTIM timeouts (expected during initial bootstrap, before lock is ever established -- see BENCH_LOCK_SYNC_THRESHOLD)
	uint32_t fault_count;       // 0 or 1 -- see tim3_isr()'s fault trigger
	uint32_t sync_max;          // Highest upstream `sync` value (0-6) ever observed -- 6 means full lock was genuinely reached at least once
	uint32_t erpm_max;          // Highest upstream `erpm` value ever observed -- upstream's own electrical-RPM estimate, only meaningful once sync>=6 (ertm is a real measurement, not the 1e8 UIF-timeout placeholder, from that point on)
	// ival ESTIMATE reconstructed from IFTIM_OCR via upstream's own
	// formula inverted: OCR = max((ival-(ival*cfg.timing>>5))>>1, 1);
	// with cfg.timing left at its default (16, not overridden by this
	// target), that reduces to OCR ~= ival/4, so ival ~= OCR*4. This is
	// an approximation (integer truncation in the original formula is
	// not perfectly invertible) -- NOT a direct read of upstream's
	// `ival` (that IS separately available now via the extern above,
	// but this field predates that and is kept as a cross-check). The
	// independently, directly MEASURED wall-clock gap between commits
	// (interval_100us_*) is the authoritative figure -- interval_100us_min
	// in particular IS the achieved top speed (shortest commutation
	// period), the actual metric of interest for this harness now.
	uint32_t ival_est_min_ticks, ival_est_max_ticks, ival_est_sum_ticks, ival_est_n;
	// Directly measured (TIM14-derived wide timestamp) inter-commit gap, 100us units.
	uint32_t interval_100us_min, interval_100us_max, interval_100us_sum, interval_100us_n;
} bench_run_log_t;

volatile bench_run_log_t bench_run;
volatile int bench_lock_established; // 1 once sync has ever reached BENCH_LOCK_SYNC_THRESHOLD -- gates whether a subsequent ZC timeout counts as a real desync (stop) vs still-expected startup bootstrapping
volatile int bench_stopped;
volatile int bench_stop_reason; // BENCH_STOP_*
volatile uint32_t bench_elapsed_ticks; // TIM14 overflow count, 10ms/tick -- moved off TIM7, which is now the F425-wide IFTIM-timeout generator (see this file's top comment)

// Internal bookkeeping only (not part of the read-back log set).
static uint32_t bench_last_commit_100us;             // Previous commit's bench_wide_timestamp(), for interval_100us_* computation
static uint32_t bench_last_target_ticks;             // Most recent accepted IFTIM_OCR, for the ival_est_* reconstruction
static uint32_t bench_last_confirm_100us;            // Any confirm, accepted or not -- confirm-to-confirm spacing
static volatile uint8_t bench_last_reschedule_accepted; // 1 if the most recent iftim_reschedule() call was accept-driven (dma1_channel1_isr), 0 if timeout-driven (tim7_isr)

static uint32_t bench_wide_timestamp(void) {
	// Two reads race against a TIM14 overflow landing in between (e.g.
	// elapsed_ticks read just before an overflow bumps it, then CNT
	// read just after it wraps to 0) -- reread elapsed_ticks after CNT
	// and use the later value if it changed, which is always safe
	// since both only ever increase.
	uint32_t ticks1 = bench_elapsed_ticks;
	uint32_t cnt = TIM14_CNT;
	uint32_t ticks2 = bench_elapsed_ticks;
	if (ticks2 != ticks1) cnt = TIM14_CNT; // An overflow landed between reads -- cnt is now post-wrap, reread it
	return ticks2 * 100u + cnt; // 100us units (10ms/tick = 100 * 100us)
}

static void bench_force_stop(int reason) {
	if (bench_stopped) return;
	bench_stopped = 1;
	bench_stop_reason = reason;
	TIM1_BDTR &= ~TIM_BDTR_MOE;
	throt = 0; // Also tell upstream's own control loop to want zero throttle from here on
}

// Throttle is ramped once, from within cfg.sine_range's window up to
// max, so upstream's own sine-startup engages (gated on the THROTTLE
// INPUT being within that window, not on the resulting duty) and then
// hands over to 6-step -- no separate startup FSM of our own. From
// there, upstream's OWN duty_spup/duty_ramp/duty_rate mechanism (see
// this file's top comment) is solely responsible for how much of that
// commanded throttle actually reaches the motor.
#define BENCH_THROT_RAMP_STEP 10  // Per 10ms tick (this function's own cadence)
#define BENCH_THROT_RAMP_MAX 2000 // Full throttle range -- upstream's own duty_spup/duty_ramp governor (not this) is what actually paces real output

void tim14_isr(void) {
	if (!(TIM14_SR & TIM_SR_UIF)) return;
	TIM14_SR = (uint16_t)~TIM_SR_UIF;
	if (bench_stopped) {
		// Independent of TIM2/TIM3/TIM7/the motor-control path entirely
		// (TIM14 is our own peripheral, always running once this is set
		// up) -- this is the belt-and-suspenders enforcement for the
		// unsafe-reset case: if the motor never starts (throt stays 0),
		// neither TIM2's nor TIM3's interrupts may ever fire at all,
		// meaning their own MOE-off reassertions (see tim3_isr()'s
		// stopped-branch) would never run either. TIM14 fires every 10ms
		// unconditionally, so this is what actually guarantees MOE stays
		// off for the unsafe-reset case specifically.
		TIM1_BDTR &= ~TIM_BDTR_MOE;
		return;
	}
	if (++bench_elapsed_ticks >= BENCH_TIME_LIMIT_TICKS) bench_force_stop(bench_lock_established ? BENCH_STOP_TIME_LIMIT_LOCKED : BENCH_STOP_TIME_LIMIT_UNLOCKED);
	if (throt < BENCH_THROT_RAMP_MAX) throt += BENCH_THROT_RAMP_STEP;
}

static void bench_test_init(void) {
	// TIM14, not TIM7 -- TIM7 is now the F425-wide IFTIM-timeout
	// generator (see this file's top comment), needed even in
	// BENCH_TEST builds, so BENCH_TEST's own elapsed-time watchdog moved
	// to a separate, still-unused-elsewhere peripheral.
	RCC_APB1ENR |= RCC_APB1ENR_TIM14EN;
	TIM14_PSC = CLK_MHZ * 100 - 1; // 100us/tick
	TIM14_ARR = 99;                // 100us * 100 = 10ms/overflow
	TIM14_CR1 = TIM_CR1_URS;
	TIM14_EGR = TIM_EGR_UG;
	TIM14_SR = (uint16_t)~TIM_SR_UIF;
	TIM14_DIER = TIM_DIER_UIE;
	nvic_set_priority(NVIC_TIM14_IRQ, 0x40);
	nvic_enable_irq(NVIC_TIM14_IRQ);
	TIM14_CR1 = TIM_CR1_CEN | TIM_CR1_URS;

	// Reset-safety gate: refuse to auto-start the motor after any
	// non-power-on reset (external/SW/WDT/WWDT/lowpower) -- an
	// unexpected reset mid-run must not be followed by an automatic
	// restart. bench_reset_cause was captured in init(), before
	// at32_clock_init(). MOE=0 is asserted once immediately here and
	// then continuously by this function's own tim14_isr() every 10ms
	// (above) for as long as bench_stopped stays set -- which, since
	// bench_force_stop() is never called to UN-set it, is until the
	// next power-on reset or an explicit debugger write of
	// bench_stopped=0 (never done automatically by this firmware).
	if (bench_reset_cause & ~(unsigned)AT32_RESET_CAUSE_POR) {
		bench_stopped = 1;
		bench_stop_reason = BENCH_STOP_UNSAFE_RESET;
		TIM1_BDTR &= ~TIM_BDTR_MOE;
		return; // throt stays 0 -- main()'s control loop never sets running=1
	}

	// Enable upstream's own sine-startup ramp (off by default,
	// SINE_RANGE=0 -- see src/defs.h) and start throt WITHIN its window
	// (range = cfg.sine_range*20 = 300, minus the ramp's own hysteresis
	// delta) so main()'s control loop actually enters the sine branch
	// instead of jumping straight to 6-step. tim14_isr() (above) ramps
	// throt upward every 10ms until it naturally crosses the sine_range
	// boundary, triggering upstream's own (unmodified) handover to
	// 6-step -- no separate startup FSM.
	cfg.sine_range = 15;
	throt = 50;

	// Real final duty ceiling (100, set ONCE -- not stepped) plus
	// upstream's own erpm-gated acceleration governor: duty_spup=30 is
	// the floor applied while erpm==0 (matches the empirical finding
	// that 15% gave this motor visibly juddering, stalling rotation --
	// see this file's top comment -- while 30% did not); duty_ramp=10
	// lets the ceiling open smoothly from that floor up to 100% as
	// upstream's own measured erpm climbs from 0 toward 10000, instead
	// of DUTY_RAMP=0's default instant-100%-the-moment-erpm-is-nonzero
	// jump (scale()'s degenerate zero-width-range behavior). duty_rate
	// (curduty slew-rate limit) is left at upstream's own default.
	cfg.duty_max = 100;
	cfg.duty_spup = 30;
	cfg.duty_ramp = 10;
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
// Phase 1 (upstream closed-loop lock at 15%): the confirm log showed
// EVERY logged "ZC candidate" at 15% duty arriving 129-923us after
// commutation (cluster ~130-165us) and being rejected by upstream's own
// t<(ival>>1) check (t is far below ival_half=2500us at ival's
// post-"start motor" default 10000<<XRES) -- consistent with post-
// commutation switching ringing, not real BEMF, at a PWM period of
// ~41.7us (24kHz) 2 blank scans (~83us) clearly isn't outlasting. Widened
// to comfortably outlast the observed cluster (and the one 923us
// outlier) while staying far shorter than the ~32.767ms open-loop
// bootstrap step -- a data-driven experiment, not a guess; revisit if
// real accepts still don't appear.
#define POST_COMMUTATION_BLANK_SCANS 24
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
#ifdef BENCH_TEST
static volatile uint8_t zc_armed_polarity; // Most recent compctl() x&4 bit, for the confirm log
#endif

void compctl(int x) {
	int sel = x & 3;
	if (!sel) {
		zc_floating_idx = -1;
		return;
	}
	zc_floating_idx = comp_in_to_adc_idx[sel];
#ifdef BENCH_TEST
	zc_armed_polarity = (x & 4) ? 1 : 0;
#endif
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

// Resynchronizes TIM3 (the 2us break-before-make scheduler) to a fresh
// IFTIM_OCR target -- called every time IFTIM (TIM2) was just reset
// (either because iftim_isr() accepted a real ZC, or because tim2_isr()
// is repeating the bootstrap/timeout cycle -- see this file's top
// comment). TIM3 runs at the identical 500ns/tick rate as TIM2, so
// `target` is used directly with no unit conversion. This function
// touches ONLY TIM3's own registers -- it never reads or writes
// anything on TIM2 itself.
static void iftim_reschedule(uint16_t target) {
	TIM3_EGR = TIM_EGR_UG; // Reset TIM3 to 0, synchronized with IFTIM's own reset
	// GATE_OFF_ADVANCE_TICKS can exceed target at very short
	// commutation intervals (IFTIM_OCR's own floor is 1, see main.c) --
	// when there isn't enough of the interval left for a break, skip
	// the gate-off phase for this one cycle rather than mis-schedule
	// it; the commit (CC3, always upstream's own already-valid target)
	// still happens on time.
	if (target > GATE_OFF_ADVANCE_TICKS) {
		TIM3_CCR2 = (uint16_t)(target - GATE_OFF_ADVANCE_TICKS);
		TIM3_DIER |= TIM_DIER_CC2IE;
	} else {
		TIM3_DIER &= ~TIM_DIER_CC2IE;
	}
	TIM3_CCR3 = target;
	TIM3_SR = (uint16_t)~(TIM_SR_CC2IF | TIM_SR_CC3IF); // Drop stale flags from the just-UG'd counter
	TIM3_DIER |= TIM_DIER_CC3IE;
}

// Backs IFTIM_OCR_WRITE() (config.h). On upstream's own single-IFTIM
// backends, writing IFTIM_OCR is itself what arms the next commutation
// (IFTIM's own compare event fires it); here the commutation event comes
// from TIM3, so the value has to be pushed into TIM3's schedule on every
// write. This is what makes upstream's sine-startup microstep pacing
// work at all -- nextstep()'s sine branch writes IFTIM_OCR once per
// microstep and expects the commutation to follow at exactly that
// interval, which previously never reached TIM3 (the only callers of
// iftim_reschedule() were the ADC-ZC accept path -- disarmed during sine
// startup by upstream's own compctl(0) -- and TIM7's 32.767ms timeout).
void at32_iftim_ocr_write(uint32_t v) {
	TIM2_CCR3 = v;
	iftim_reschedule((uint16_t)v);
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
#ifdef BENCH_TEST
		int8_t confirm_phase = (int8_t)zc_floating_idx;
		uint32_t confirm_now = 0, confirm_since_commit = 0, confirm_since_prev = 0;
		uint32_t diag_cval = 0, diag_ival = 0;
		uint8_t diag_step = 0;
		uint32_t diag_sine = 0, diag_ertm = 0, diag_throt = 0;
		if (!bench_stopped) {
			confirm_now = bench_wide_timestamp();
			confirm_since_commit = bench_last_commit_100us ? confirm_now - bench_last_commit_100us : 0;
			confirm_since_prev = bench_last_confirm_100us ? confirm_now - bench_last_confirm_100us : 0;
			bench_last_confirm_100us = confirm_now;
			bench_run.confirm_count++; // Candidate only -- accept/reject determined below
			if ((uint32_t)sync > bench_run.sync_max) bench_run.sync_max = (uint32_t)sync;
			if (sync >= BENCH_LOCK_SYNC_THRESHOLD) {
				bench_lock_established = 1;
				if ((uint32_t)erpm > bench_run.erpm_max) bench_run.erpm_max = (uint32_t)erpm;
			}
			diag_cval = TIM2_CNT;
			diag_ival = (uint32_t)ival;
			diag_step = (uint8_t)step;
			diag_sine = (uint32_t)sine;
			diag_ertm = (uint32_t)ertm;
			diag_throt = (uint32_t)throt;
		}
#endif
		uint32_t ccr3_before = TIM2_CCR3; // Always needed -- accept detection is not BENCH_TEST-only
		zc_floating_idx = -1; // Disarm until compctl() re-arms for the next sector

		iftim_isr(); // Direct call -- IFTIM_ICR (=TIM2_CNT) is read live inside, no capture register involved. PMEN/ARR are permanent invariants now (see this file's top comment) -- no reassertion needed.

		uint32_t ccr3_after = TIM2_CCR3;
		int accepted = ccr3_after != ccr3_before;
		if (accepted) iftim_reschedule((uint16_t)ccr3_after);

#ifdef BENCH_TEST
		if (!bench_stopped) {
			if (accepted) {
				bench_run.zc_accepted_count++;
				bench_last_target_ticks = ccr3_after;
				bench_last_reschedule_accepted = 1;
			}
			uint32_t n = bench_confirm_log_count++;
			if (n < BENCH_CONFIRM_LOG_N) {
				volatile bench_confirm_log_t *e = &bench_confirm_log[n];
				e->cval_at_confirm = diag_cval;
				e->ival_snapshot = diag_ival;
				e->ival_half = diag_ival >> 1;
				e->accepted = (uint8_t)accepted;
				e->step_snapshot = diag_step;
				e->floating_phase = confirm_phase;
				e->polarity = zc_armed_polarity;
				e->since_commit_100us = confirm_since_commit;
				e->confirm_timestamp_100us = confirm_now;
				e->since_prev_confirm_100us = confirm_since_prev;
				e->sine_snapshot = diag_sine;
				e->ertm_snapshot = diag_ertm;
				e->throt_snapshot = diag_throt;
			}
		}
#endif
	}
}

// F425-specific 2us break-before-make scheduler. Purely reactive to
// TIM3's own CC2 (gate-off)/CC3 (commit) compare events -- see
// iftim_reschedule() for how those get programmed, and this file's top
// comment for the full accept/timeout flow. No IFTIM/UIF logic lives
// here anymore (that moved to tim2_isr()).
//
// KNOWN SIMPLIFICATION (flagged, not yet hardware-hardened): CC2IF/
// CC3IF are processed as independent ifs against one SR snapshot
// rather than re-reading SR after each action. At the commutation
// rates this design targets that should be safe, but this remains the
// least hardware-validated new code in this port and may need its own
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
			if (!(TIM1_BDTR & TIM_BDTR_MOE)) {
				// Driver all-off suspicion: we just wrote MOE=1 above and
				// it reads back low -- something external (a hardware
				// BRK/fault input, or the driver itself) is holding the
				// outputs off despite our request. Stop immediately.
				bench_run.fault_count = 1;
				bench_force_stop(BENCH_STOP_FAULT);
			} else {
				volatile bench_run_log_t *s = &bench_run;
				uint32_t now = bench_wide_timestamp(); // 100us units, monotonic across the whole bounded run
				if (bench_last_commit_100us) { // Skip the very first-ever commit -- no valid prior timestamp yet
					uint32_t interval = now - bench_last_commit_100us;
					if (!s->interval_100us_n || interval < s->interval_100us_min) s->interval_100us_min = interval;
					if (interval > s->interval_100us_max) s->interval_100us_max = interval;
					s->interval_100us_sum += interval;
					s->interval_100us_n++;
				}
				bench_last_commit_100us = now;
				if (bench_last_target_ticks) { // Set once iftim_isr() has accepted at least one real ZC
					uint32_t ival_est = bench_last_target_ticks * 4u;
					if (!s->ival_est_n || ival_est < s->ival_est_min_ticks) s->ival_est_min_ticks = ival_est;
					if (ival_est > s->ival_est_max_ticks) s->ival_est_max_ticks = ival_est;
					s->ival_est_sum_ticks += ival_est;
					s->ival_est_n++;
				}
				s->commutation_count++;
			}
		}
#endif
	}
}

// TIM2's real hardware UIF should essentially never fire now (ARR is
// permanently 0xFFFFFFFF -- see this file's top comment), and nothing
// in this backend enables NVIC_TIM2_IRQ or relies on a real TIM2
// interrupt for anything (real ZC confirms call iftim_isr() directly
// from dma1_channel1_isr(); the 32.767ms timeout is TIM7's, handled by
// tim7_isr() below). This is a defensive stub only, kept in case a
// spurious flag (e.g. CH1's own compare-match against its untouched,
// zero-valued CCR1 -- CH1 is never configured as a capture channel,
// and upstream's own nextstep() keeps IFTIM_ICIE/CC1IE enabled in DIER
// regardless) is ever pending; it does nothing but clear SR.
void tim2_isr(void) {
	TIM2_SR = 0;
}

// TIM7's own timeout -- upstream's "no ZC in 32.767ms" stall detector,
// now sourced independently of IFTIM's own (permanently disabled) ARR
// overflow (see this file's top comment). Calls iftim_timeout()
// (src/main.c) DIRECTLY -- does NOT call iftim_isr() and does NOT
// fabricate a TIM2 UIF/SR state for it to detect; iftim_timeout() is
// the exact processing iftim_isr()'s own UIF branch would have run,
// extracted so both paths can share it verbatim. Mirrors
// dma1_channel1_isr()'s accept path: reset the shared timebase
// (at32_iftim_reset(), TIM2+TIM7 together), then unconditionally
// reschedule TIM3 from whatever IFTIM_OCR currently holds (unchanged
// by iftim_timeout()) -- reproduces the "commutate every ~32.7ms until
// a real ZC takes over" bootstrap behavior the old single-timer design
// got for free from hardware ARR==OCR coincidence.
void tim7_isr(void) {
	if (!(TIM7_SR & TIM_SR_UIF)) {
		TIM7_SR = 0;
		return;
	}
	TIM7_SR = (uint16_t)~TIM_SR_UIF;
#ifdef BENCH_TEST
	bench_last_reschedule_accepted = 0;
	// ZC timeout stop condition: a genuine 32.767ms timeout AFTER
	// upstream's own full-lock signal (`sync`) has ever reached
	// BENCH_LOCK_SYNC_THRESHOLD means real BEMF sync was genuinely lost
	// -- stop immediately. Before that point (still bootstrapping),
	// timeouts are an expected, tolerated part of first lock-in.
	if (!bench_stopped) {
		bench_run.uif_count++;
		if (bench_lock_established) bench_force_stop(BENCH_STOP_ZC_TIMEOUT);
	}
#endif
	iftim_timeout();
	at32_iftim_reset();
	iftim_reschedule((uint16_t)TIM2_CCR3);
}

void init(void) {
#ifdef IFTIM_RESET_LOG
	at32_iftim_reset_log_init();
#endif
#ifdef BENCH_TEST
	// Must happen BEFORE at32_clock_init() (clock_config_96mhz() starts
	// with crm_reset(), which may clear CRM's reset-cause flags) --
	// see bench_test_init() for what this gates. NOTE: whether POR and
	// NRST_PIN co-assert together on a genuine power-up on THIS
	// silicon has not been independently confirmed yet (only debugger-
	// initiated core resets have been observed so far) -- a real
	// power-cycle test is needed to check that a legitimate power-on
	// isn't ever misclassified as "unsafe" by bench_test_init()'s
	// `& ~POR` check.
	bench_reset_cause = at32_reset_cause_get();
	at32_reset_cause_clear();
#endif
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
	RCC_APB1ENR = RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN | RCC_APB1ENR_USART2EN;
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

	// IFTIM = TIM2, 32-bit Plus Mode, 500ns/tick, free-running. CH1 is
	// deliberately left unconfigured (reset default) -- no capture is
	// used (see this file's top comment). Reset the peripheral first
	// for a clean, known state before enabling Plus Mode. PMEN=1 and
	// ARR=0xFFFFFFFF are set here ONCE and never touched again by
	// anyone (see this file's top comment) -- CR1 is written through
	// at32_iftim_cr1_write() even here, so it really is the single,
	// sole place TIM2_CR1 is ever assigned in this backend.
	//
	// ORDERING MATTERS: PMEN must be set BEFORE the 32-bit ARR write,
	// not after -- Stage E28's original finding (PR truncated to
	// 0xFFFF) turned out to be a missing PMEN call entirely, but even
	// with PMEN present, writing ARR before PMEN is set truncates the
	// WRITE itself to 16 bits (confirmed by re-testing motor-off after
	// this file's IFTIM_ARR_WRITE()/IFTIM_CR1_WRITE() refactor moved
	// the CR1/PMEN write to the end of this sequence -- ARR read back
	// as 0x0000FFFF instead of 0xFFFFFFFF). Stage E28B/D's own working
	// bring-up code always called tmr_32_bit_function_enable() (PMEN)
	// before tmr_base_init() (ARR/PSC) -- restored here.
	RCC_APB1RSTR = RCC_APB1RSTR_TIM2RST;
	RCC_APB1RSTR = 0;
	TIM2_CR1 = TIM_CR1_PMEN_BIT; // PMEN first, alone -- CEN/ARPE follow once ARR is actually 32-bit-written below
	TIM_PSC(IFTIM) = (CLK_MHZ >> (IFTIM_XRES + 1)) - 1; // Same derivation as upstream's own main() would have used for a 16-bit IFTIM -- unchanged rate
	TIM2_ARR = 0xFFFFFFFFu; // Full 32-bit range, permanent -- see this file's top comment
	TIM2_DIER = 0; // No interrupts yet -- upstream's own nextstep()/main() enable UIE/ICIE as needed
	TIM2_EGR = TIM_EGR_UG;
	at32_iftim_cr1_write(TIM_CR1_CEN | TIM_CR1_ARPE);

	// TIM3 = 2us break-before-make scheduler ONLY, matching IFTIM's own
	// 500ns/tick rate (no unit conversion needed between the two, see
	// iftim_reschedule()). Plain 16-bit is sufficient -- this timer is
	// reset (UG) at every accepted ZC / bootstrap retry, so it only
	// ever needs to represent a single upcoming commutation's worth of
	// ticks, and upstream's own IFTIM_OCR ceiling (65535) already fits.
	// No CH1 capture-mode setup needed here anymore (that requirement
	// moved away entirely with the software-capture design -- TIM3 no
	// longer does any capture at all).
	TIM_PSC(TIM3) = (CLK_MHZ >> (IFTIM_XRES + 1)) - 1;
	TIM3_ARR = 0xFFFF;
	TIM3_DIER = 0;
	TIM3_EGR = TIM_EGR_UG;
	TIM3_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;

	// TIM7 = independent 32.767ms IFTIM-timeout generator (see this
	// file's top comment) -- same 500ns/tick rate as TIM2/TIM3, ARR
	// matches upstream's own literal 65535 value exactly. Reset in
	// lockstep with TIM2 by at32_iftim_reset() (IFTIM_RESET()).
	TIM_PSC(TIM7) = (CLK_MHZ >> (IFTIM_XRES + 1)) - 1;
	TIM7_ARR = (1 << (IFTIM_XRES + 16)) - 1;
	TIM7_CR1 = TIM_CR1_URS;
	TIM7_EGR = TIM_EGR_UG;
	TIM7_SR = (uint16_t)~TIM_SR_UIF;
	TIM7_DIER = TIM_DIER_UIE;
	TIM7_CR1 = TIM_CR1_CEN | TIM_CR1_URS;

	nvic_set_priority(NVIC_TIM2_IRQ, 0x40);
	nvic_set_priority(NVIC_TIM3_IRQ, 0x40);
	nvic_set_priority(NVIC_TIM7_IRQ, 0x40);
	nvic_set_priority(NVIC_TIM1_BRK_UP_TRG_COM_IRQ, 0x40);
	nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 0x80);
	nvic_set_priority(NVIC_TIM15_IRQ, 0x40);
	nvic_set_priority(NVIC_USART2_IRQ, 0x40);
	nvic_set_priority(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ, 0x80);
	nvic_set_priority(NVIC_DMA1_CHANNEL4_7_DMA2_CHANNEL3_5_IRQ, 0x40);

	nvic_enable_irq(NVIC_TIM1_BRK_UP_TRG_COM_IRQ);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_enable_irq(NVIC_TIM3_IRQ);
	nvic_enable_irq(NVIC_TIM7_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
	// NVIC_TIM15_IRQ is NOT enabled here -- deferred to initio()'s own
	// last statement via IOTIM_NVIC_ENABLE() (see this file's top-of-
	// config.h note) so it can never fire before ioirq (src/io.c) is
	// non-NULL.
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
