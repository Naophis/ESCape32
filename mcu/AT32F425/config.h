/*
** ESCape32 AT32F425 (MOUSEF425) port -- real product target.
**
** Address/vector compatibility with libopencm3's STM32F0 map (confirmed
** by direct header comparison against Artery's at32f425.h and vendored
** startup_at32f425.s during bring-up, not assumed):
**   GPIOA/B/C/D_BASE, TIM1_BASE, TIM2_BASE, TIM3_BASE match exactly.
**   NVIC vector positions match exactly: DMA1_Channel1=9, ADC1=12,
**   TIM1_BRK_UP_TRG_COM=13, TIM1_CC=14, TIM2=15, TIM3=16.
**   EGR/SWTRIG bit positions match exactly (STM32 COMG=bit5 ==
**   Artery hallswtr=bit5), so TIM1_EGR=TIM_EGR_COMG and Artery's
**   tmr_event_sw_trigger(TMR1, TMR_HALL_SWTRIG) hit the SAME bit.
** This is why TIM1/GPIO/IFTIM below are driven with plain libopencm3
** macros exactly like mcu/AT32F421/config.h, while CRM/ADC/DMA/clock
** (genuinely incompatible -- confirmed throughout this port's Stage
** A-E26 bring-up) are handled exclusively by artery_hal.c/h using the
** vendored Artery driver.
**
** Found incompatibility: STM32's CCUS bit (TIM_CR2, used by upstream's
** main() to enable hardware TRGI-triggered CCPC shadow commits) is,
** at the same bit position on this silicon, Artery's "ccfs" / Hall
** sensor interface select bit (tmr_hall_select()). Upstream's own
** `TIM1_CR2 = TIM_CR2_CCPC | TIM_CR2_CCUS | TIM_CR2_MMS_COMPARE_PULSE`
** therefore has the convenient side effect of also enabling exactly
** the Hall/COM software-trigger mode this backend depends on (Stage
** E25/E26 validated tmr_hall_select(TMR1,TRUE) + TMR_HALL_SWTRIG on
** real hardware) -- no shared-code change was needed for this.
**
** TIMER ROLE ASSIGNMENT (revised -- see config.c's top comment for the
** full history of why):
**   TIM1  : 6PWM + CH4 ADC hardware trigger
**   TIM2  : IFTIM -- upstream's BEMF timing core (iftim_isr()/ival/
**           timing advance), 32-bit "Plus Mode" (PMEN), 0.5us/tick.
**           TIM2_CNT is read LIVE (not via a capture register) as
**           IFTIM_ICR -- see config.c's ADC-ZC confirm handler. PMEN=1
**           and ARR=0xFFFFFFFF are PERMANENT invariants -- never
**           changed again after init(), by anyone (see below).
**   TIM3  : 2us break-before-make scheduler ONLY. No longer IFTIM.
**   TIM7  : Independent 32.767ms IFTIM-timeout generator (replaces
**           IFTIM's own ARR-overflow-driven UIF).
**   TIM14 : BENCH_TEST elapsed-time watchdog (moved off TIM7).
**   TIM15 : IOTIM/DSHOT
**
** IFTIM was originally TIM3 (16-bit). Stage E27 found upstream's own
** commutation-timing math relies on IFTIM's counter being reset (UG)
** exactly at each accepted ZC and staying valid across arbitrarily
** long intervals; on a 16-bit counter with upstream's own ARR=65535
** ceiling, that assumption silently breaks before the first accept
** (the counter free-runs on its own 32.767ms auto-reload instead of
** being ZC-synchronized) -- Stage E27's software-overflow-extension
** attempt to patch around this was abandoned (own boundary races) in
** favor of moving IFTIM to a genuinely 32-bit-capable timer instead.
** Stage E28 first found TIM2 behaving as 16-bit too -- turned out to
** be a missing tmr_32_bit_function_enable(TMR2, TRUE) (PMEN) call, not
** a hardware limitation (AT32F425's datasheet: TIM2 is 32-bit-capable,
** 16-bit unless Plus Mode is explicitly enabled). Stage E28B confirmed
** PMEN+PR=0xFFFFFFFF works (CVAL correctly free-runs/does not wrap
** across 35/40/70ms); Stage E28D independently re-confirmed, against
** the Cortex-M4's own DWT_CYCCNT wall clock with all other timers/
** interrupts/motor hardware untouched, that CVAL tracks perfectly
** linearly out to 100ms with zero drift at every checkpoint -- Stage
** E28B/C's earlier ~40ms anomaly was therefore that test harness's own
** artifact (TMR7 delay reference / CH1-CH3 setup interaction), not a
** TIM2 hardware limitation.
**
** upstream's own nextstep() does `TIM_CR1(IFTIM) = TIM_CR1_CEN |
** TIM_CR1_ARPE | TIM_CR1_URS [| ...];` every commutation -- a PLAIN,
** WHOLE-REGISTER assignment that would silently clear PMEN (bit10,
** unknown to libopencm3's TIM_CR1_* macros) on every single
** commutation, and upstream also routinely tries to set IFTIM's ARR to
** 0/0xFFFF (16-bit-timer assumptions baked into its own startup/stop
** logic) -- which, if it ever actually reached real hardware even
** momentarily WHILE the counter is running, can race a real auto-
** reload/wrap and permanently corrupt the "ticks since last reset"
** value (a repair after the fact, however immediate, cannot undo a
** wrap that already happened in hardware). Rather than repairing these
** after the fact, this port ABSORBS them: src/defs.h defines portable
** IFTIM_CR1_WRITE(v)/IFTIM_ARR_WRITE(v)/IFTIM_RESET()/
** IFTIM_TIMEOUT_ARM()/IFTIM_TIMEOUT_DISARM() macros (default: expand to
** upstream's original plain register writes, so every other target is
** byte-for-byte unaffected), which main.c now uses at every site that
** used to write IFTIM's CR1/ARR/EGR/DIER directly. This backend
** overrides them below so upstream's intended CR1/ARR values are never
** actually applied to real TIM2 hardware at all -- see config.c's top
** comment for the full design (at32_iftim_cr1_write()/
** at32_iftim_reset(), and TIM7 as the independent 32.767ms timeout that
** replaces IFTIM's own now-permanently-disabled ARR overflow).
*/

#pragma once

#define CLK 96000000

#define IFTIM_XRES 0
// IFTIM (TIM2) ticks at 2^(IFTIM_XRES+1) MHz -- 2MHz/500ns per tick at
// IFTIM_XRES=0 (see main.c's TIM_PSC(IFTIM) derivation, and config.c's
// TIM2 base_init using the same 47 prescaler value). TIM3 (the 2us
// scheduler) is configured at the identical tick rate so its CCR2/
// CCR3 targets can be copied directly from IFTIM_OCR with no unit
// conversion. GATE_OFF_ADVANCE_TICKS is 2us worth of ticks at that rate.
#define GATE_OFF_ADVANCE_TICKS (2 << (IFTIM_XRES + 1))

#define IFTIM TIM2
#define IFTIM_ICFL 128
#define IFTIM_ICMR TIM2_CCMR1
#define IFTIM_ICM1 (TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_DTF_DIV_16_N_8)
#define IFTIM_ICM2 (TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_DTF_DIV_8_N_8)
#define IFTIM_ICM3 (TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_DTF_DIV_4_N_8)
#define IFTIM_ICIE TIM_DIER_CC1IE
// IFTIM_ICR is a LIVE counter read, not a capture register: TIM2 CH1 is
// never configured as an input-capture channel at all (no CH1 software
// capture is used -- per instruction, not required as a mechanism).
// Every real ADC-ZC confirm calls iftim_isr() directly from
// dma1_channel1_isr() the instant it's confirmed, so `t = IFTIM_ICR`
// (src/main.c) reading TIM2_CNT AT THAT EXACT MOMENT correctly means
// "ticks since IFTIM was last reset" -- exactly upstream's own
// intended semantics, achieved with no capture hardware involved.
#define IFTIM_ICR TIM2_CNT
#define IFTIM_OCR TIM2_CCR3
// NOTE: iftim_isr is intentionally NOT #define'd to any ISR name. It
// keeps its real name and is called explicitly from TWO places in
// config.c: dma1_channel1_isr() (every real ADC-ZC confirm) and
// tim7_isr() (the independent 32.767ms timeout, via iftim_timeout() --
// see below and config.c's top comment).

// Portable IFTIM register-access abstraction overrides (src/defs.h has
// the defaults; this file's top comment has the full rationale).
// CR1/ARR are absorbed -- upstream's intended values never reach real
// TIM2 hardware. EGR/OCR still reach it unchanged (genuine reset/
// commutation-delay semantics are required). TIMEOUT_ARM/DISARM are
// intentionally left at src/defs.h's default (still real TIM2 DIER
// writes) for now.
void at32_iftim_cr1_write(uint32_t v);
void at32_iftim_reset(void);
void at32_iftim_ocr_write(uint32_t v);
#define IFTIM_CR1_WRITE(v) at32_iftim_cr1_write(v)
#define IFTIM_ARR_WRITE(v) ((void)(v))
#define IFTIM_RESET() at32_iftim_reset()
// Writing IFTIM_OCR is what ARMS the next commutation on a single-IFTIM
// backend (the timer's own compare event fires it). Here the commutation
// event comes from TIM3 instead, so the new target has to be pushed into
// TIM3 explicitly on every write -- without this, upstream's own
// sine-startup microstep pacing (nextstep()'s sine branch writes
// IFTIM_OCR once per microstep and expects the commutation to follow at
// exactly that interval) never reaches the scheduler at all, and
// microsteps only advance at TIM7's 32.767ms timeout rate, which on real
// hardware is a standing vibration instead of rotation.
#define IFTIM_OCR_WRITE(v) at32_iftim_ocr_write(v)

#define tim1_com_isr tim1_brk_up_trg_com_isr

// This is an ADC-based (not comparator-based) ZC backend: TIM1 CH4 is
// owned exclusively by our own PWM_MODE_B ADC-trigger config (Stage
// E14-E23, hardware-validated).
//
// BUG FOUND ON REAL HARDWARE (first BENCH_TEST run): nextstep()
// (src/main.c) writes TIM1_CCMR2 and TIM1_CCER as PLAIN, WHOLE-REGISTER
// assignments (`TIM1_CCMR2 = m2; TIM1_CCER = er;`), not read-modify-
// write -- so a no-op COMP_BLANK_CH4_INIT (leaving CH4's bits out of
// m2/er entirely) does NOT "leave CH4 alone" as intended, it makes
// nextstep() ZERO CH4's OC4PE/OC4M/CC4E bits on every single call. This
// silently killed the ADC hardware trigger moments after boot (first
// commutation), which is exactly why at32_adc_buf stayed all-zero and
// ADC-ZC confirm count was 0 through a full 60-commutation bench run.
// The fix is the opposite of "no-op": re-inject our own CH4 config
// into m2/er EVERY time, so it survives nextstep()'s overwrite.
// COMP_BLANK_CH4_SET(x) (a separate, standalone `TIM1_CCR4 = x;`
// assignment, not part of a whole-register overwrite) does NOT have
// this problem -- discarding it there really does leave CCR4 (our
// ADC_TRIGGER_OFFSET_TICKS) alone, so that one stays a genuine no-op.
#define ADC_ZC_BACKEND
#define COMP_BLANK_CH4_INIT(m2, er) do { (m2) |= TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M_PWM2; (er) |= TIM_CCER_CC4E; } while (0)
#define COMP_BLANK_CH4_SET(x) ((void)(x))

// DSHOT/servo signal decode (src/io.c, unmodified, requires SOME IOTIM
// definition unconditionally). Pin/AF choice below (PA2 -> TMR15_CH1)
// mirrors AT32F421's IO_PA2/TIM15 convention and is NOT yet hardware-
// verified on this silicon (Stage A-E26 never exercised it) -- but is
// low-risk if wrong: worst case the signal is simply never decoded and
// `throt` stays at its cfg.throt_set-initialized value. IO_PA2 (not the
// no-pin default) is required here for safety, NOT just convention: if
// no valid edges are ever seen on IOTIM's input, src/io.c's entryirq()
// falls back, after ~1s, to a CLI-over-serial state -- WITH IO_PA2 that
// fallback only touches USART2 (io_serial()); WITHOUT it, it directly
// reconfigures TIM3 (hardcoded in src/io.c, not via the IFTIM macro) --
// which is now our 2us break-before-make scheduler, so this protection
// still matters even though TIM3 is no longer IFTIM.
// GENUINE HardFault found (motor off, no signal wired): config.c's
// init() used to unconditionally nvic_enable_irq(NVIC_TIM15_IRQ)
// before main() ever calls initio() (src/io.c) -- exactly the same
// order AT32F421's own config.c uses too, so this ordering itself is
// not inherently wrong. On this board a HardFault was nonetheless
// captured (src/main.c's HARDFAULT_CAPTURE()) squarely inside TIM15's
// handler (stacked xPSR IPSR=36 -> NVIC IRQ 20 = TIM15), branching to
// PC=0 -- io.c's `iotim_isr(void) { ioirq(); }` calls a `static void
// (*ioirq)(void)` that stays NULL until initio() runs
// (`ioirq = entryirq;`), so any TIM15 interrupt that manages to fire
// in the window between config.c's init() and main()'s initio() call
// is a call through a NULL function pointer. IOTIM_NVIC_ENABLE()
// (src/defs.h default: no-op) defers the real enable to initio()'s own
// last statement instead, once ioirq is guaranteed non-NULL -- see
// config.c (NVIC_TIM15_IRQ is no longer enabled in init()).
#define IOTIM_NVIC_ENABLE() nvic_enable_irq(NVIC_TIM15_IRQ)

#define IO_PA2
#define IOTIM TIM15
#define IOTIM_IDR (GPIOA_IDR & 0x4) // A2
#define IOTIM_DMA 5
#define iotim_isr tim15_isr
#define iotim_dma_isr dma1_channel4_7_dma2_channel3_5_isr

// HardFault diagnostics (src/defs.h's HARDFAULT_CAPTURE() default is a
// no-op). LR at hard_fault_handler()'s entry holds EXC_RETURN -- the
// instant ANY function is called (even a plain `bl`), LR is clobbered
// with a return address instead, so SP/LR must be read INLINE, here,
// before at32_hardfault_capture() (config.c) is called to do the rest
// (SP itself is read post-hard_fault_handler's-own-prologue -- see
// config.c's HARDFAULT_PROLOGUE_WORDS for how the original hardware-
// stacked frame is located from that).
extern volatile uint32_t at32_hardfault_sp, at32_hardfault_lr;
void at32_hardfault_capture(void);
#define HARDFAULT_CAPTURE() do { \
	__asm__ volatile ("mov %0, sp" : "=r" (at32_hardfault_sp)); \
	__asm__ volatile ("mov %0, lr" : "=r" (at32_hardfault_lr)); \
	at32_hardfault_capture(); \
} while (0)

#define USART1_RX_DMA 3
#define USART1_TX_DMA 2
#define usart1_tx_dma_isr dma1_channel2_3_dma2_channel1_2_isr
#define USART2_RX_DMA 5
#define USART2_TX_DMA 4
