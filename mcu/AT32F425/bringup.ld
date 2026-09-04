/*
 * Linker script shared by all Stage A-E26 standalone bring-up harnesses
 * for AT32F425K8 (64K flash / 20K SRAM). Standalone: does NOT use
 * mcu/common.ld (that script assumes the libopencm3 vector table +
 * ESCape32 boot/cfg layout, neither of which the bring-up harnesses
 * use -- see mcu/AT32F425/stage_a_main.c for rationale).
 * Symbol names (_sidata/_sdata/_edata/_sbss/_ebss/_estack) match what
 * vendor/startup_at32f425.s (Artery's own gcc startup file) expects.
 *
 * Renamed from config.ld (was Stage A-only originally) so the real
 * MOUSEF425 product target (add_target(MOUSEF425 AT32F425 ...) in the
 * top-level CMakeLists.txt) can use mcu/AT32F425/config.ld for its own,
 * completely different purpose: add_target()'s generic build path
 * expects config.ld to be a minimal MEMORY-only script (boot/cfg/rom/
 * ram regions, mirroring mcu/AT32F421/config.ld), combined with
 * mcu/common.ld's shared SECTIONS/vector-table layout -- NOT this
 * file's standalone, full custom script. The two link models are
 * incompatible in one file, hence the split.
 */

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 64K
  RAM (rwx)   : ORIGIN = 0x20000000, LENGTH = 20K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

ENTRY(Reset_Handler)

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text.Reset_Handler)
    *(.text .text.*)
    *(.rodata .rodata.*)
    *(.glue_7) *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data .data.*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT>FLASH

  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss .bss.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } >RAM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    . = . + 0x400; /* minimum heap */
    . = . + 0x400; /* minimum stack margin, actual stack grows down from _estack */
    . = ALIGN(8);
  } >RAM
}
