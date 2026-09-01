#!/usr/bin/env bash
# Start a persistent OpenOCD server (ST-Link + AT32F4x) for inspecting
# the running Stage A/B/C firmware -- GDB (port 3333) or telnet (4444).
#
# Usage: mcu/AT32F425/openocd.sh
#
# Then, in another shell:
#   telnet localhost 4444
#   > mdw 0x20000000 8      ; dump 8 words from RAM start (see the
#                            ; stage_*_main.c symbol table for offsets)
#   > reg
#   > resume
#
# or attach GDB:
#   arm-none-eabi-gdb build/MOUSEF425_STAGE_C-rev17.1.elf \
#       -ex "target extended-remote localhost:3333"

set -euo pipefail

: "${OPENOCD:=/home/naoto/tools/openocd-install/bin/openocd}"

if [ ! -x "$OPENOCD" ]; then
	echo "OpenOCD not found/executable at: $OPENOCD" >&2
	echo "Set OPENOCD=/path/to/openocd (must support target/artery/at32f4x.cfg)." >&2
	exit 1
fi

exec "$OPENOCD" \
	-f interface/stlink.cfg \
	-f target/artery/at32f4x.cfg
