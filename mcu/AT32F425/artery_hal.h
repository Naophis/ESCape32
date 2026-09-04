/*
** ESCape32 AT32F425 (MOUSEF425) port -- compatibility layer.
**
** Plain-C prototypes only, no Artery vendor types leak across this
** header -- config.c (libopencm3-facing, mirrors mcu/AT32F421/config.c)
** calls into these for the genuinely incompatible pieces (CRM/clock,
** ADC, DMA request routing), confirmed incompatible throughout Stage
** A-E26 bring-up (register layout/semantics differ from what
** libopencm3's STM32F0 headers assume, unlike TIM1/TIM3/GPIO/NVIC,
** which ARE address/vector-compatible and are driven directly with
** libopencm3 macros in config.c -- see config.h's top comment).
*/

#pragma once

#include <stdint.h>

// CRM/clock. Wraps the already hardware-validated clock_config_96mhz().
void at32_clock_init(void);

// Reset-cause flags (CRM CTRLSTS register), read via Artery's own
// crm_flag_get()/CRM_*_RESET_FLAG -- must be called BEFORE
// at32_clock_init() (which calls crm_reset(), and clock_config_96mhz()
// starts with crm_reset() too) in case that clears them. Returns a
// bitmask of AT32_RESET_CAUSE_* bits (multiple can be set on some
// resets; report all of them rather than picking one).
enum {
	AT32_RESET_CAUSE_POR       = 1 << 0, // Power-on
	AT32_RESET_CAUSE_NRST_PIN  = 1 << 1, // External reset pin
	AT32_RESET_CAUSE_SW        = 1 << 2, // Software reset
	AT32_RESET_CAUSE_WDT       = 1 << 3, // Independent watchdog
	AT32_RESET_CAUSE_WWDT      = 1 << 4, // Window watchdog
	AT32_RESET_CAUSE_LOWPOWER  = 1 << 5, // Low-power/brownout-adjacent (closest flag this silicon exposes)
};
unsigned at32_reset_cause_get(void);
void at32_reset_cause_clear(void); // Call AFTER reading it once at boot -- otherwise flags accumulate across resets

// ADC1 ordinary group (3-phase BEMF, PA0/PA4/PA5) + DMA1 channel1,
// externally triggered by TIM1 CH4 (PWM_MODE_B, configured by config.c
// -- CH4 itself is a libopencm3-compatible TIM1 register and is NOT
// touched here). Mirrors Stage E14-E23's hardware-validated setup.
// at32_adc_buf[0..2] = A(PA0)/B(PA4)/C(PA5), updated by DMA on every
// completed 3-channel scan; config.c's DMA1 ISR (libopencm3-facing)
// reads it directly after each transfer-complete interrupt.
extern volatile uint16_t at32_adc_buf[3];
void at32_bemf_adc_dma_init(void);

// DMA is one of the registers genuinely confirmed incompatible with
// libopencm3's STM32F0 assumptions (unlike TIM/GPIO/NVIC) -- Stage
// E14-E26 always read/cleared DMA1 channel1's completion flag via
// Artery's own dma_interrupt_flag_get()/dma_flag_clear(DMA1_FDT1_FLAG),
// never libopencm3's DMA_ISR/DMA_IFCR macros, and that bit-position
// equivalence was never independently checked (unlike GPIO/TIM/NVIC).
// config.c's dma1_channel1_isr() must go through this function rather
// than touching DMA1_ISR/DMA1_IFCR itself. Returns nonzero and clears
// the flag if channel1's transfer just completed; also clears any
// pending DMA transfer-error flag (reported via Stage E14/E23's own
// diagnostic counter convention, not surfaced further yet).
int at32_bemf_dma_transfer_complete(void);

// DMA1 request routing is software-flexible on this silicon (not a
// fixed per-channel wiring like real STM32F0) and must be configured
// once at boot for every DMA channel config.c/src/telem.c/src/io.c
// use. Channel numbers match config.h's USART1_RX_DMA=3/TX_DMA=2,
// USART2_RX_DMA=5/TX_DMA=4 (IOTIM_DMA=5 shares USART2_RX_DMA's
// channel, exactly as mcu/AT32F421/config.h already does -- the two
// uses are time-exclusive, never simultaneous).
void at32_dma_flexible_routing_init(void);
