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
** This is why TIM1/GPIO/IFTIM(=TIM3) below are driven with plain
** libopencm3 macros exactly like mcu/AT32F421/config.h, while
** CRM/ADC/DMA/clock (genuinely incompatible -- confirmed throughout
** this port's Stage A-E26 bring-up) are handled exclusively by
** artery_hal.c/h using the vendored Artery driver.
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
** Commutation timing: unlike STM32/AT32F421 (hardware TRGO from IFTIM's
** OC3REF drives TIM1's COM event automatically via SMCR/ITRx), this
** silicon's equivalent auto path (Stage E25, TMR3 TRGO -> TMR1 COM via
** internal trigger IS2) was validated to work but explicitly NOT
** adopted for production, because it cannot express the required
** "gate-off 2us before commutation, commit exactly at commutation"
** split. Instead, our own tim3_isr() (artery_hal.c) wraps upstream's
** unmodified iftim_isr(), reading back the IFTIM_OCR (=TIM3_CCR3)
** value iftim_isr() just computed and independently re-arming TIM3's
** CC2 (gate-off, OCR-2us) and CC3 (commit) compare interrupts every
** cycle -- since upstream unconditionally zeroes/rewrites TIM_DIER
** (IFTIM) on every call, our own enable bits must be re-asserted after
** every iftim_isr() invocation, which is exactly what this wrapper
** does. See artery_hal.c's top comment for the full scheduler design.
*/

#pragma once

#define CLK 96000000

#define IFTIM_XRES 0
// IFTIM (TIM3) ticks at CLK/((CLK_MHZ>>(IFTIM_XRES+1))*1e6) = 2^(IFTIM_XRES+1) MHz
// (see main.c's TIM_PSC(IFTIM) derivation) -- 2MHz/500ns per tick at
// IFTIM_XRES=0. GATE_OFF_ADVANCE_TICKS is therefore 2us worth of ticks.
#define GATE_OFF_ADVANCE_TICKS (2 << (IFTIM_XRES + 1))

#define IFTIM TIM3
#define IFTIM_ICFL 128
#define IFTIM_ICMR TIM3_CCMR1
#define IFTIM_ICM1 (TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_DTF_DIV_16_N_8)
#define IFTIM_ICM2 (TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_DTF_DIV_8_N_8)
#define IFTIM_ICM3 (TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_DTF_DIV_4_N_8)
#define IFTIM_ICIE TIM_DIER_CC1IE
#define IFTIM_ICR TIM3_CCR1
#define IFTIM_OCR TIM3_CCR3
// NOTE: iftim_isr is intentionally NOT #define'd to tim3_isr here (unlike
// AT32F421). It keeps its real name and is called explicitly, once, from
// our own tim3_isr() in artery_hal.c, which also does the F425-specific
// gate-off/commit scheduling that upstream's hardware-TRGO design doesn't
// need. See artery_hal.c.

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
// reconfigures TIM3, which is our IFTIM and would silently break BEMF/
// commutation scheduling after about a second with no signal wired.
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
