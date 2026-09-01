# AT32F425 Stage A bring-up build config.
# NOT wired into the shared add_target()/common.ld path yet -- see
# CMakeLists.txt (MOUSEF425_STAGE_A block) and stage_a_main.c for why.
# Kept here only so the device/family defines are colocated with the
# other mcu/<chip>/config.cmake files, for consistency across targets.
set(at32f425_opts -mcpu=cortex-m4 -mthumb -mfloat-abi=soft)
# AT32F425xx / AT32F425Kx / AT32F425x8 are auto-derived from this by at32f425.h
set(at32f425_defs AT32F425K8U7_4)
