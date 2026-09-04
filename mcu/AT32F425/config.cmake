# AT32F425 build config -- serves TWO consumers from this one file
# (both `include(mcu/AT32F425/config.cmake)`):
#
# 1. The temporary Stage A-E26 standalone bring-up harnesses
#    (add_at32f425_bringup() in the top-level CMakeLists.txt), which
#    read `at32f425_opts`/`at32f425_defs` and compile directly against
#    the vendored Artery driver tree (mcu/AT32F425/vendor/*), no
#    libopencm3, no src/*.c. Kept unchanged from the original bring-up
#    config so none of Stage A-E26 are disturbed.
#
# 2. The real MOUSEF425 product target (add_target(MOUSEF425 AT32F425
#    ...) in the top-level CMakeLists.txt), which reads the generic
#    `opts`/`libs`/`defs` add_target() expects and compiles src/*.c +
#    mcu/AT32F425/*.c together against libopencm3.
#
#    TIM1/GPIO/TIM2/TIM3(IFTIM)/DMA1/ADC-vector positions are
#    confirmed (direct header comparison against libopencm3's STM32F0
#    memory map and NVIC table, not assumed) to match exactly -- see
#    config.h's top comment for the specific base-address/IRQ-number
#    pairs checked. This mirrors AT32F421's own established approach
#    (also opencm3_stm32f0 + AT32F4) for the same reason: TIM1/GPIO
#    are STM32F0-compatible on these AT32F4-series parts, CRM/ADC/DMA
#    are NOT (confirmed extensively throughout this port's bring-up,
#    Stage A onward) and are handled exclusively by
#    mcu/AT32F425/artery_hal.c using the vendored Artery driver, never
#    by libopencm3 macros -- see artery_hal.c's own header comment.

# --- (1) Stage A-E26 bring-up harnesses ---
set(at32f425_opts -mcpu=cortex-m4 -mthumb -mfloat-abi=soft)
# AT32F425xx / AT32F425Kx / AT32F425x8 are auto-derived from this by at32f425.h
set(at32f425_defs AT32F425K8U7_4)

# --- (2) Real MOUSEF425 product target ---
set(opts -mcpu=cortex-m4 -mthumb -mfloat-abi=soft)
set(libs opencm3_stm32f0)
set(defs STM32F0 AT32F4)
