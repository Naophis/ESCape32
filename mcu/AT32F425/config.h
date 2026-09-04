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
**           IFTIM_ICR -- see config.c's ADC-ZC confirm handler.
**   TIM3  : 2us break-before-make scheduler ONLY. No longer IFTIM.
**   TIM7  : BENCH_TEST elapsed-time watchdog
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
** TIM_CR1_ARPE | TIM_CR1_URS [| ...];` -- a PLAIN, WHOLE-REGISTER
** assignment, every commutation. Since PMEN lives in that same CTRL1/
** CR1 register (bit10) and libopencm3's TIM_CR1_* macros don't know
** about it, this write silently clears PMEN on every single
** commutation. config.c re-asserts PMEN defensively at every point it
** (or upstream, via a call config.c makes) is about to rely on TIM2's
** 32-bit behavior, bounding how long it can stay cleared -- see
** config.c's PMEN_REASSERT use sites.
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
// tim2_isr() (IFTIM's own UIF timeout). See config.c's top comment.

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
#define IO_PA2
#define IOTIM TIM15
#define IOTIM_IDR (GPIOA_IDR & 0x4) // A2
#define IOTIM_DMA 5
#define iotim_isr tim15_isr
#define iotim_dma_isr dma1_channel4_7_dma2_channel3_5_isr

#define USART1_RX_DMA 3
#define USART1_TX_DMA 2
#define usart1_tx_dma_isr dma1_channel2_3_dma2_channel1_2_isr
#define USART2_RX_DMA 5
#define USART2_TX_DMA 4
