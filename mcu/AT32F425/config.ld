/*
 * Real MOUSEF425 product target.
 *
 * UNLIKE mcu/AT32F421/config.ld, "boot" here is zero-length: AT32F421
 * real products are always flashed behind a separate bootloader image
 * (boot/CMakeLists.txt's add_target(BOOT3_PA2 ...)) that itself owns
 * the vector table at the hardware reset address (0x08000000) and
 * jumps into "rom" after boot/cfg checks. No AT32F425 bootloader has
 * been built yet -- flashing goes directly via SWD/OpenOCD, so this
 * firmware IS what the CPU fetches its initial SP/reset vector from at
 * 0x08000000, and "rom" (where cortex-m-generic.ld/libopencm3 place
 * the vector table) MUST start exactly there. (Discovered on real
 * hardware: with the AT32F421-style boot/cfg offset, the chip was
 * silently executing a stale, unrelated Stage E26 bring-up image left
 * over in the first 4KB of flash from an earlier flash session --
 * SCB_VTOR only redirects interrupt vector fetches for code already
 * running, it cannot fix the initial hardware boot fetch.) "cfg"
 * (upstream's runtime-savable Cfg struct mirror, src/util.c's
 * savecfg()) is moved to the LAST 1K of flash instead, since its
 * position relative to "rom" doesn't matter for that mechanism.
 * ram=20K reflects AT32F425's actual SRAM size (vs AT32F421's 8K).
 */

MEMORY {
	boot (rx) : ORIGIN = 0x8000000, LENGTH = 0
	rom (rx) : ORIGIN = ORIGIN(boot) + LENGTH(boot), LENGTH = FLASH_SIZE - 1K
	cfg (rx) : ORIGIN = ORIGIN(rom) + LENGTH(rom), LENGTH = 1K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}
