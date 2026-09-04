cmake_minimum_required(VERSION 3.15)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_C_FLAGS "-Os -g")
set(CMAKE_C_FLAGS_DEBUG "-Og")
project(ESCape32 C)
if(UNIX AND NOT CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT) # Use 'CMAKE_INSTALL_PREFIX' as alternative system root
	set(CMAKE_PREFIX_PATH ${CMAKE_INSTALL_PREFIX})
	include(GNUInstallDirs)
	include_directories(${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR})
	link_directories(${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})
endif()
if(LIBOPENCM3_DIR)
	include_directories(${LIBOPENCM3_DIR}/include)
	link_directories(${LIBOPENCM3_DIR}/lib)
endif()
add_compile_options(-ffreestanding -ffunction-sections -fdata-sections -fsingle-precision-constant
	-Wall -Wextra -Wpedantic -Wundef -Wshadow -Wredundant-decls -Wstrict-prototypes -Wmissing-prototypes
	-Wno-variadic-macros -Wno-unused-result -Wno-unused-parameter)
add_link_options(-nostartfiles -Wl,--gc-sections)

function(add_target name mcu)
	include(mcu/${mcu}/config.cmake)
	file(READ src/main.c main_c)
	string(REGEX MATCH "REVISION [0-9]+" rev1 ${main_c})
	string(REGEX MATCH "REVPATCH [0-9]+" rev2 ${main_c})
	string(REGEX REPLACE "[^0-9]+" "-rev" rev1 ${rev1})
	if (rev2)
		string(REGEX REPLACE "[^0-9]+" "." rev2 ${rev2})
	endif()
	set(elf ${name}${rev1}${rev2}.elf)
	set(bin ${name}${rev1}${rev2}.bin)
	set(hex ${name}${rev1}${rev2}.hex)
	file(GLOB srcs src/*.c mcu/${mcu}/*.c)
	add_executable(${elf} ${srcs})
	if("SMALL_DEVICE" IN_LIST ARGN)
		target_compile_options(${elf} PRIVATE -Wno-unused-variable -Wno-unused-function)
		set(size 32K)
	else()
		set(size 64K)
	endif()
	target_include_directories(${elf} PRIVATE src mcu/${mcu})
	target_compile_options(${elf} PRIVATE ${opts})
	target_compile_definitions(${elf} PRIVATE ${mcu} ${name} ${defs} TARGET_NAME="${name}" ${ARGN})
	target_link_options(${elf} PRIVATE -Wl,--defsym,FLASH_SIZE=${size} -T${CMAKE_CURRENT_SOURCE_DIR}/mcu/${mcu}/config.ld -T${CMAKE_CURRENT_SOURCE_DIR}/mcu/common.ld ${opts})
	target_link_libraries(${elf} c_nano gcc nosys ${libs})
	add_custom_command(
		OUTPUT ${bin} ${hex}
		COMMAND arm-none-eabi-objcopy -O binary ${elf} ${bin}
		COMMAND arm-none-eabi-objcopy -O ihex ${elf} ${hex}
		DEPENDS ${elf}
	)
	add_custom_target(${name} ALL DEPENDS ${hex})
	add_custom_target(flash-${name} COMMAND st-flash --reset --connect-under-reset --format ihex write ${hex} DEPENDS ${hex})
endfunction()

add_subdirectory(boot)

add_target(AART1 AT32F421 DEAD_TIME=0 COMP_MAP=123 ARM=0 VOLUME=0 INPUT_MODE=1 ANALOG_CHAN=6 ANALOG_MIN=270 ANALOG_MAX=1440 DUTY_MIN=100 DUTY_SPUP=25 FULL_DUTY IO_RXTX)
add_target(AIRBOT1 AT32F421 DEAD_TIME=66 COMP_MAP=321 SENS_MAP=0xA3A6 VOLT_MUL=74 CURR_MUL=30 LED_MAP=0xAFB3B4)
add_target(AIRBOT2 STM32F051 DEAD_TIME=26 COMP_MAP=321 SENS_MAP=0xA3 VOLT_MUL=74 IO_PA2 SMALL_DEVICE DISABLE_HOTT)
add_target(EMAX1 STM32F051 DEAD_TIME=26 COMP_MAP=123 IO_PA2 SMALL_DEVICE)
add_target(ESCAPE1 STM32G071 DEAD_TIME=35 COMP_MAP=123 HALL_MAP=0xAFB35 SENS_MAP=0xA6A5A4 VOLT_MUL=110 CURR_MUL=40 BEC_MAP=0xCEF LED_WS2812 LED_STAT IO_PA2)
add_target(ESCAPE2 STM32G071 DEAD_TIME=35 COMP_MAP=123 SENS_MAP=0xA6 VOLT_MUL=110 BEC_MAP=0xADE IO_PA2)
add_target(ESCAPE3 AT32F421 DEAD_TIME=66 COMP_MAP=123 SENS_MAP=0xA6 VOLT_MUL=110 BEC_MAP=0xADE)
add_target(ESCAPE4 STM32G071 DEAD_TIME=35 COMP_MAP=123 HALL_MAP=0xB358 SENS_MAP=0xA6A5A4 VOLT_MUL=110 CURR_MUL=40 BEC_MAP=0xCEF LED_MAP=0xF2AF LED1_INV LED_STAT IO_PA2)
add_target(ESCAPE5 STM32G071 DEAD_TIME=35 COMP_MAP=123 HALL_MAP=0xAFB35 SENS_MAP=0xA6 VOLT_MUL=110 BEC_MAP=0xA45 LED_WS2812 LED_STAT IO_PA2)
add_target(FLYCOLOR1 STM32F051 DEAD_TIME=26 COMP_MAP=123 SENS_MAP=0xA6 VOLT_MUL=110 LED_MAP=0xB5B4B3 IO_PA2 SMALL_DEVICE DISABLE_HOTT)
add_target(FLYCOLOR2 STM32G071 DEAD_TIME=35 COMP_MAP=123 SENS_MAP=0xA6 VOLT_MUL=110 LED_MAP=0xB8 LED_STAT)
add_target(FLYCOLOR3 STM32G071 DEAD_TIME=50 COMP_MAP=123 SENS_MAP=0xA6A4 VOLT_MUL=210 CURR_MUL=30)
add_target(FLYINGRC1 AT32F421 DEAD_TIME=66 COMP_MAP=123)
add_target(FREELYRC1 AT32F421 DEAD_TIME=66 COMP_MAP=123 SENS_MAP=0xA3 VOLT_MUL=110 LED_WS2812 LED_STAT)
add_target(GEPRC1 AT32F421 DEAD_TIME=66 COMP_MAP=123 SENS_MAP=0xA3 VOLT_MUL=110)
add_target(HAKRC1 STM32F051 DEAD_TIME=26 COMP_MAP=213 SENS_MAP=0xA3 VOLT_MUL=110 LED_MAP=0xAFB5B3 LED_INV LED_OD SMALL_DEVICE)
add_target(HAKRC2 AT32F421 DEAD_TIME=66 COMP_MAP=213 SENS_MAP=0xA3 VOLT_MUL=110 LED_MAP=0xAFB5B3 LED_INV LED_OD)
add_target(HAKRC3 STM32G071 DEAD_TIME=35 COMP_MAP=132 SENS_MAP=0xA1A5 VOLT_MUL=110 CURR_MUL=20 LED_WS2812 LED_STAT)
add_target(HGLRC1 STM32F051 DEAD_TIME=26 COMP_MAP=123 SENS_MAP=0xA6 VOLT_MUL=210 IO_PA2 SMALL_DEVICE DISABLE_HOTT)
add_target(HHOBBIES1 GD32E230 DEAD_TIME=40 COMP_MAP=321 SENS_MAP=0xA6 VOLT_MUL=110 LED_MAP=0xAF LED_INV LED_STAT)
add_target(HOBBYWING1 STM32F051 DEAD_TIME=40 COMP_MAP=123 IO_PA2 SMALL_DEVICE)
add_target(HOBBYWING2 AT32F421 DEAD_TIME=96 COMP_MAP=123)
add_target(HOLYBRO1 GD32F350 DEAD_TIME=57 COMP_MAP=321 SENS_MAP=0xA6A0 VOLT_MUL=129 CURR_MUL=50 USE_COMP2)
add_target(IFLIGHT1 STM32F051 DEAD_TIME=26 COMP_MAP=321 SENS_MAP=0xA3A6 VOLT_MUL=110 CURR_MUL=60 LED_MAP=0xB8B5B3 LED_INV SMALL_DEVICE)
add_target(IFLIGHT2 STM32G071 DEAD_TIME=35 COMP_MAP=213 SENS_MAP=0xA5A4 VOLT_MUL=110 CURR_MUL=20 LED_WS2812 IO_PA6)
add_target(IFLIGHT3 STM32G071 DEAD_TIME=35 COMP_MAP=132 SENS_MAP=0xA0 VOLT_MUL=110 LED_WS2812)
add_target(IFLIGHT4 STM32G071 DEAD_TIME=35 COMP_MAP=213 SENS_MAP=0xA5 VOLT_MUL=110 LED_WS2812 IO_PA6)
add_target(LUMENIER1 GD32F350 DEAD_TIME=57 COMP_MAP=321 SENS_MAP=0xA3 VOLT_MUL=110)
add_target(LYI1 AT32F421 DEAD_TIME=66 COMP_MAP=321 SENS_MAP=0xA3 VOLT_MUL=110)
add_target(NEUTRON1 AT32F421 DEAD_TIME=66 COMP_MAP=321)
add_target(NEUTRON2 AT32F421 DEAD_TIME=66 COMP_MAP=321 INVERTED_HIGH)
add_target(NEUTRON3 STM32L431 DEAD_TIME=44 COMP_MAP=123 SENS_MAP=0xA6A3 VOLT_MUL=210 CURR_MUL=125 USE_COMP2 USE_OPAMP)
add_target(ODDITYRC1 AT32F421 DEAD_TIME=66 COMP_MAP=123 SENS_MAP=0xA3BF VOLT_MUL=110 CURR_MUL=40)
add_target(PHOTONDRIVE1 STM32G431 DEAD_TIME=155 COMP_MAP=132 SENS_MAP=0xBFA6 VOLT_MUL=224 CURR_MUL=63 TEMP_ADC2 TEMP_CHAN=14 TEMP_FUNC=NTC10K3455UP10K LED_WS2812 LED_STAT USE_PB1 USE_HSE=8 DUTY_SPUP=5 DUTY_RATE=20 PROT_TEMP=100)
add_target(RHINO1 STM32F051 DEAD_TIME=26 COMP_MAP=321 SENS_MAP=0xA6 VOLT_MUL=110 SMALL_DEVICE)
add_target(SEQURE1 STM32G071 DEAD_TIME=35 COMP_MAP=123 SENS_MAP=0xA6A4 VOLT_MUL=210 CURR_MUL=30 BEC_MAP=0xCEF BEC_MIN=1 LED_WS2812 LED_STAT)
add_target(SEQURE2 STM32G071 DEAD_TIME=50 COMP_MAP=123 SENS_MAP=0xA6A4 VOLT_MUL=210 CURR_MUL=87 BEC_MAP=0xCEF BEC_MIN=1 LED_WS2812 LED_STAT ERPM_PIN=5)
add_target(SEQURE3 STM32G071 DEAD_TIME=60 COMP_MAP=123 SENS_MAP=0xA6A4 VOLT_MUL=480 CURR_MUL=205 BEC_MAP=0xCEF BEC_MIN=1 LED_WS2812 LED_STAT ERPM_PIN=5)
add_target(SKYSTARS1 GD32E230 DEAD_TIME=40 COMP_MAP=321 SENS_MAP=0xA3 VOLT_MUL=110 LED_MAP=0xB5B3AF)
add_target(TMOTOR1 STM32F051 DEAD_TIME=26 COMP_MAP=132 IO_PA2 SMALL_DEVICE)
add_target(TMOTOR2 STM32F051 DEAD_TIME=26 COMP_MAP=321 SMALL_DEVICE)
add_target(TMOTOR3 STM32G071 DEAD_TIME=35 COMP_MAP=231 IO_PA6)
add_target(TMOTOR4 STM32G071 DEAD_TIME=35 COMP_MAP=123)

# MOUSEF425: custom board, AT32F425K8U7-4 + MP6540HGU-A-Z. DEAD_TIME=53
# and COMP_MAP=123 are the only two values carried over from Stage
# A-E26 hardware bring-up (53 ticks -- Stage E15D onward; COMP_MAP is
# an identity-mapping PLACEHOLDER, not yet independently verified
# against this board's actual phase wiring, unlike DEAD_TIME/the TIM1
# AF pinout/BEMF ADC channels -- see mcu/AT32F425/config.c's
# comp_in_to_adc_idx[] table if it needs correcting later). No throttle
# input has been hardware-validated (see config.h) -- the first
# low-duty spin test needs a real servo/Oneshot PWM signal wired to
# PA2, ARM/THROT_SET are intentionally left at their safe upstream
# defaults (250ms zero-throttle arm wait, no preset throttle).
add_target(MOUSEF425 AT32F425 DEAD_TIME=53 COMP_MAP=123)
# artery_hal.c pulls in the vendored Artery driver (see its own header
# comment) -- that vendor code needs the same device-select define and
# CMSIS core header path the standalone bring-up harnesses pass via
# at32f425_defs/cmsis_core (mcu/AT32F425/config.cmake), which
# add_target()'s own generic ${defs}/include-dirs don't cover. Applied
# directly to the already-defined target rather than touching
# add_target() itself (shared upstream build infrastructure).
target_compile_definitions(MOUSEF425-rev17.1.elf PRIVATE AT32F425K8U7_4)
target_include_directories(MOUSEF425-rev17.1.elf PRIVATE mcu/AT32F425/cmsis_core)

# Bounded, logged first-spin bench validation of the real MOUSEF425
# backend (upstream startup/ADC-ZC/break-before-make path) -- NOT a
# stage_eXX-style standalone harness: compiles the same shared src/*
# control core + mcu/AT32F425/config.c unmodified. ANALOG+THROT_SET is
# upstream's own supported mechanism for a fixed preset throttle with
# no signal wiring (skips initio()/IOTIM entirely -- see config.c's
# BENCH_TEST comment); DUTY_MAX/DUTY_SPUP cap actual output at 15%
# regardless. BENCH_TEST adds only logging/a bounded forced stop,
# entirely inside mcu/AT32F425/config.c behind #ifdef BENCH_TEST.
# DUTY_MAX/DUTY_SPUP compile-time defaults just need to safely cover
# the highest bench stage (40%) before bench_apply_stage() takes over
# at runtime (see mcu/AT32F425/config.c) -- the actual per-stage duty
# ceiling injection happens there, not via these compile constants.
add_target(MOUSEF425_BENCH AT32F425 DEAD_TIME=53 COMP_MAP=123 ANALOG DUTY_MAX=40 DUTY_SPUP=40 BENCH_TEST)
target_compile_definitions(MOUSEF425_BENCH-rev17.1.elf PRIVATE AT32F425K8U7_4)
target_include_directories(MOUSEF425_BENCH-rev17.1.elf PRIVATE mcu/AT32F425/cmsis_core)

# --- AT32F425 (MOUSEF425) staged bring-up harnesses ------------------------
# Temporary, standalone targets: do NOT use add_target() / mcu/common.ld,
# and do NOT compile src/*.c (the shared ESCape32 control loop touches
# TIM1/MP6540HA and calls compctl(), which has no real implementation
# until the Stage D BEMF design lands -- see the porting-plan Section 18
# report and mcu/AT32F425/bringup/stage_*_main.c). Each stage_*_main.c is its own
# standalone main(); this function wires one up as its own firmware image.
# No other target above this line is affected by this addition.
include(mcu/AT32F425/config.cmake)
enable_language(ASM)
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -x assembler-with-cpp")

function(add_at32f425_bringup name mainfile)
	set(elf ${name}-rev17.1.elf)
	set(bin ${name}-rev17.1.bin)
	set(hex ${name}-rev17.1.hex)
	add_executable(${elf}
		${mainfile}
		${ARGN}
		mcu/AT32F425/bringup/clock_config.c
		mcu/AT32F425/vendor/system_at32f425.c
		mcu/AT32F425/vendor/at32f425_crm.c
		mcu/AT32F425/vendor/at32f425_gpio.c
		mcu/AT32F425/vendor/at32f425_flash.c
		mcu/AT32F425/vendor/at32f425_tmr.c
		mcu/AT32F425/vendor/at32f425_misc.c
		mcu/AT32F425/vendor/at32f425_adc.c
		mcu/AT32F425/vendor/at32f425_dma.c
		mcu/AT32F425/vendor/startup_at32f425.s
	)
	target_include_directories(${elf} PRIVATE
		mcu/AT32F425
		mcu/AT32F425/vendor
		mcu/AT32F425/cmsis_core
	)
	target_compile_options(${elf} PRIVATE ${at32f425_opts})
	target_compile_definitions(${elf} PRIVATE ${at32f425_defs})
	target_link_options(${elf} PRIVATE
		-T${CMAKE_CURRENT_SOURCE_DIR}/mcu/AT32F425/bringup.ld
		-nostartfiles -Wl,--gc-sections
		${at32f425_opts}
	)
	target_link_libraries(${elf} c_nano gcc nosys)
	add_custom_command(
		OUTPUT ${bin} ${hex}
		COMMAND arm-none-eabi-objcopy -O binary ${elf} ${bin}
		COMMAND arm-none-eabi-objcopy -O ihex ${elf} ${hex}
		DEPENDS ${elf}
	)
	add_custom_target(${name} ALL DEPENDS ${hex})
	add_custom_target(flash-${name} COMMAND st-flash --reset --connect-under-reset --format ihex write ${hex} DEPENDS ${hex})
endfunction()

add_at32f425_bringup(MOUSEF425_STAGE_A mcu/AT32F425/bringup/stage_a_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_B mcu/AT32F425/bringup/stage_b_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_C mcu/AT32F425/bringup/stage_c_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_D mcu/AT32F425/bringup/stage_d_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_D_ACTIVE mcu/AT32F425/bringup/stage_d_active_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E mcu/AT32F425/bringup/stage_e_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E2 mcu/AT32F425/bringup/stage_e2_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E3 mcu/AT32F425/bringup/stage_e3_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E4 mcu/AT32F425/bringup/stage_e4_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E5 mcu/AT32F425/bringup/stage_e5_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E6 mcu/AT32F425/bringup/stage_e6_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E7 mcu/AT32F425/bringup/stage_e7_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E8 mcu/AT32F425/bringup/stage_e8_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E9 mcu/AT32F425/bringup/stage_e9_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E10 mcu/AT32F425/bringup/stage_e10_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E11 mcu/AT32F425/bringup/stage_e11_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E12 mcu/AT32F425/bringup/stage_e12_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E13 mcu/AT32F425/bringup/stage_e13_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E14 mcu/AT32F425/bringup/stage_e14_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E15 mcu/AT32F425/bringup/stage_e15_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E15B mcu/AT32F425/bringup/stage_e15b_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E15C mcu/AT32F425/bringup/stage_e15c_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E15D mcu/AT32F425/bringup/stage_e15d_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E16 mcu/AT32F425/bringup/stage_e16_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E16B mcu/AT32F425/bringup/stage_e16b_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E16C mcu/AT32F425/bringup/stage_e16c_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E16D mcu/AT32F425/bringup/stage_e16d_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E16E mcu/AT32F425/bringup/stage_e16e_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E16F mcu/AT32F425/bringup/stage_e16f_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E17 mcu/AT32F425/bringup/stage_e17_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E18 mcu/AT32F425/bringup/stage_e18_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E19 mcu/AT32F425/bringup/stage_e19_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E20 mcu/AT32F425/bringup/stage_e20_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E20B mcu/AT32F425/bringup/stage_e20b_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E21 mcu/AT32F425/bringup/stage_e21_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E22 mcu/AT32F425/bringup/stage_e22_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E23 mcu/AT32F425/bringup/stage_e23_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E24 mcu/AT32F425/bringup/stage_e24_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E25 mcu/AT32F425/bringup/stage_e25_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E26 mcu/AT32F425/bringup/stage_e26_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E27 mcu/AT32F425/bringup/stage_e27_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E28 mcu/AT32F425/bringup/stage_e28_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E28B mcu/AT32F425/bringup/stage_e28b_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E28C mcu/AT32F425/bringup/stage_e28c_main.c)
add_at32f425_bringup(MOUSEF425_STAGE_E28D mcu/AT32F425/bringup/stage_e28d_main.c)
