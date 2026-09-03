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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

: "${OPENOCD:=/home/naoto/tools/openocd-install/bin/openocd}"
OPENOCD_SCRIPTS="${OPENOCD_SCRIPTS:-/home/naoto/tools/openocd-install/share/openocd/scripts}"
BOARD_CFG="$SCRIPT_DIR/openocd_at32f425.cfg"

if [ ! -x "$OPENOCD" ]; then
	echo "OpenOCD not found/executable at: $OPENOCD" >&2
	echo "Set OPENOCD=/path/to/openocd (must support AT32F4x via artery flash driver)." >&2
	exit 1
fi

exec "$OPENOCD" \
	-s "$OPENOCD_SCRIPTS" \
	-f "$BOARD_CFG"
