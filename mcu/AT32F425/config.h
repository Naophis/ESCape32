#pragma once

/*
 * Placeholder for the eventual shared-main.c AT32F425 backend (Stage B+).
 * Stage A (stage_a_main.c) does not include this file -- it talks to
 * vendor/at32f425.h directly and does not use src/common.h or src/defs.h,
 * since it deliberately does not join the shared ESCape32 control loop
 * yet (see stage_a_main.c header comment and porting-plan Section 18).
 */

#define CLK 96000000
