/*
** Copyright (C) Arseny Vakhrushev <arseny.vakhrushev@me.com>
**
** This firmware is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** This firmware is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this firmware. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#if DEAD_TIME < 128
#define TIM_DTG DEAD_TIME
#elif DEAD_TIME < 256
#define TIM_DTG (((DEAD_TIME - 128) >> 1) | 0x80)
#elif DEAD_TIME < 512
#define TIM_DTG (((DEAD_TIME - 256) >> 3) | 0xc0)
#elif DEAD_TIME < 1024
#define TIM_DTG (((DEAD_TIME - 512) >> 4) | 0xe0)
#endif

#if COMP_MAP == 123
#define COMP_IN1 1
#define COMP_IN2 2
#define COMP_IN3 3
#elif COMP_MAP == 231
#define COMP_IN1 2
#define COMP_IN2 3
#define COMP_IN3 1
#elif COMP_MAP == 312
#define COMP_IN1 3
#define COMP_IN2 1
#define COMP_IN3 2
#elif COMP_MAP == 132
#define COMP_IN1 1
#define COMP_IN2 3
#define COMP_IN3 2
#elif COMP_MAP == 321
#define COMP_IN1 3
#define COMP_IN2 2
#define COMP_IN3 1
#elif COMP_MAP == 213
#define COMP_IN1 2
#define COMP_IN2 1
#define COMP_IN3 3
#endif

#ifndef SENS_MAP
#define SENS_MAP 0
#define SENS_CNT 0
#define SENS_CHAN 0
#elif SENS_MAP <= 0xff
#define SENS_CNT 1
#elif SENS_MAP <= 0xffff
#define SENS_CNT 2
#elif SENS_MAP <= 0xffffff
#define SENS_CNT 3
#endif

#ifndef LED_MAP
#ifdef LED_WS2812
#define LED_CNT 3
#else
#define LED_CNT 0
#endif
#elif LED_MAP <= 0xff
#define LED_CNT 1
#elif LED_MAP <= 0xffff
#define LED_CNT 2
#elif LED_MAP <= 0xffffff
#define LED_CNT 3
#elif LED_MAP <= 0xffffffff
#define LED_CNT 4
#endif

// Comparator-based backends use TIM1 CH4 as a hardware blanking window
// around each commutation edge (see nextstep() in main.c). ADC-based ZC
// backends (AT32F425) do their blanking in software (ADC-ZC state
// machine: blank scans -> armed -> confirm) and need CH4 left entirely
// alone for their own ADC-trigger use -- see mcu/AT32F425/config.h,
// which overrides both macros to no-ops when ADC_ZC_BACKEND is defined.
#ifndef COMP_BLANK_CH4_INIT
#define COMP_BLANK_CH4_INIT(m2, er) do { (m2) |= TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M_PWM1; (er) |= TIM_CCER_CC4E; } while (0)
#endif
#ifndef COMP_BLANK_CH4_SET
#define COMP_BLANK_CH4_SET(x) (TIM1_CCR4 = (x))
#endif

// Portable break-before-make hook, called once per nextstep() call
// immediately before the new sector's CCMR/CCER shadow registers are
// written (which only become ACTIVE at the FOLLOWING COM event -- see
// mcu/AT32F425/config.h/.c for why the real 2us gate-off/commit timing
// lives in an independent scheduler rather than here). Default: no-op.
#ifndef COMMUTATION_BREAK
#define COMMUTATION_BREAK() ((void)0)
#endif

// Portable IFTIM register-access abstraction. Every place main.c
// touches IFTIM's CR1/ARR or issues a timebase reset/timeout arm/disarm
// goes through these instead of the raw TIM_xxx(IFTIM) macros, so a
// backend whose IFTIM implementation has invariants a plain-16-bit-
// auto-reload-timer assumption would violate (see mcu/AT32F425/config.h
// -- IFTIM there is a 32-bit "Plus Mode" timer that must never actually
// have its ARR changed or its Plus Mode bit cleared, even momentarily)
// can absorb those writes before they ever reach real hardware, rather
// than repairing them after the fact. Default: expands to exactly the
// original plain register writes, so every other target is unaffected.
#ifndef IFTIM_CR1_WRITE
#define IFTIM_CR1_WRITE(v) (TIM_CR1(IFTIM) = (v))
#endif
#ifndef IFTIM_ARR_WRITE
#define IFTIM_ARR_WRITE(v) (TIM_ARR(IFTIM) = (v))
#endif
#ifndef IFTIM_RESET
#define IFTIM_RESET() (TIM_EGR(IFTIM) = TIM_EGR_UG)
#endif
#ifndef IFTIM_TIMEOUT_ARM
#define IFTIM_TIMEOUT_ARM() (TIM_DIER(IFTIM) = TIM_DIER_UIE | IFTIM_ICIE)
#endif
#ifndef IFTIM_TIMEOUT_DISARM
#define IFTIM_TIMEOUT_DISARM() (TIM_DIER(IFTIM) = 0)
#endif

// Portable hook, called as the very LAST statement of initio()
// (src/io.c), after ioirq (io.c's static ISR-dispatch function
// pointer) and all IOTIM/IOTIM2 registers are fully configured.
// Default: no-op, since every existing target enables its IOTIM NVIC
// line unconditionally in its own init() (before initio() runs) and
// that is fine for them -- a backend overrides this only if it needs
// to defer that enable until ioirq is actually non-NULL instead.
#ifndef IOTIM_NVIC_ENABLE
#define IOTIM_NVIC_ENABLE() ((void)0)
#endif

// Portable HardFault-diagnostic hook, called as the very FIRST
// statement in hard_fault_handler() (src/main.c) -- before that point
// only the compiler's own function-entry prologue has touched the
// stack, so a backend override gets the earliest possible look at the
// CPU state (SP/LR, and via SP the auto-stacked exception frame) that
// caused a genuine CPU HardFault, as opposed to hard_fault_handler()'s
// other two DELIBERATE call sites (invalid Hall code / unstable
// signal), which aren't real faults and have no meaningful frame to
// capture. Default: no-op.
#ifndef HARDFAULT_CAPTURE
#define HARDFAULT_CAPTURE() ((void)0)
#endif

#ifndef TEMP_SENS
#define TEMP_SENS NTC10K3455UP2K
#endif
#ifndef SERIAL_BR
#define SERIAL_BR 460800
#endif
#ifndef ERPM_PORT
#define ERPM_PORT B
#endif
#ifndef PARK_PORT
#define PARK_PORT B
#endif
#ifndef BEC_MIN
#define BEC_MIN 0
#endif
#ifndef BEC_MAX
#define BEC_MAX (BEC_MIN + 3)
#endif

// Default settings

#ifndef ARM
#define ARM 1
#endif
#ifndef DAMP
#define DAMP 1
#endif
#ifndef REVDIR
#define REVDIR 0
#endif
#ifndef BRUSHED
#define BRUSHED 0
#endif
#ifndef TIMING
#define TIMING 16
#endif
#ifndef SINE_RANGE
#define SINE_RANGE 0
#endif
#ifndef SINE_POWER
#define SINE_POWER 8
#endif
#ifndef FREQ_MIN
#define FREQ_MIN 24
#endif
#ifndef FREQ_MAX
#define FREQ_MAX 48
#endif
#ifndef DUTY_MIN
#define DUTY_MIN 1
#endif
#ifndef DUTY_MAX
#define DUTY_MAX 100
#endif
#ifndef DUTY_SPUP
#define DUTY_SPUP 15
#endif
#ifndef DUTY_RAMP
#define DUTY_RAMP 0
#endif
#ifndef DUTY_RATE
#define DUTY_RATE 30
#endif
#ifndef DUTY_DRAG
#define DUTY_DRAG 0
#endif
#ifndef DUTY_LOCK
#define DUTY_LOCK 0
#endif
#ifndef THROT_MODE
#define THROT_MODE 0
#endif
#ifndef THROT_ZTC
#define THROT_ZTC 0
#endif
#ifndef THROT_REV
#define THROT_REV 0
#endif
#ifndef THROT_BRK
#define THROT_BRK 100
#endif
#ifndef THROT_SET
#define THROT_SET 0
#endif
#ifndef THROT_CAL
#ifdef USE_HSE
#define THROT_CAL 0
#else
#define THROT_CAL 1
#endif
#endif
#ifndef THROT_MIN
#define THROT_MIN 1000
#endif
#ifndef THROT_MID
#define THROT_MID 1500
#endif
#ifndef THROT_MAX
#define THROT_MAX 2000
#endif
#ifndef ANALOG_MIN
#define ANALOG_MIN 100
#endif
#ifndef ANALOG_MAX
#define ANALOG_MAX 3200
#endif
#ifndef INPUT_MODE
#define INPUT_MODE 0
#endif
#ifndef INPUT_CH1
#define INPUT_CH1 0
#endif
#ifndef INPUT_CH2
#define INPUT_CH2 0
#endif
#ifndef TELEM_MODE
#define TELEM_MODE 0
#endif
#ifndef TELEM_PHID
#define TELEM_PHID 0
#endif
#ifndef TELEM_POLES
#define TELEM_POLES 14
#endif
#ifndef TELEM_VOLT
#define TELEM_VOLT 0
#endif
#ifndef TELEM_CURR
#define TELEM_CURR 0
#endif
#ifndef PROT_STALL
#define PROT_STALL 0
#endif
#ifndef PROT_TEMP
#define PROT_TEMP 0
#endif
#ifndef PROT_SENS
#define PROT_SENS 0
#endif
#ifndef PROT_VOLT
#define PROT_VOLT 0
#endif
#ifndef PROT_CELLS
#define PROT_CELLS 0
#endif
#ifndef PROT_CURR
#define PROT_CURR 0
#endif
#ifndef PROT_PARK
#define PROT_PARK 0
#endif
#ifndef MUSIC
#define MUSIC "dfa#"
#endif
#ifndef VOLUME
#define VOLUME 25
#endif
#ifndef BEACON
#define BEACON 50
#endif
#ifndef BEC
#define BEC 0
#endif
#ifndef LED
#define LED 0
#endif
