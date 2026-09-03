#!/usr/bin/env bash
# Build + flash one of the AT32F425 bring-up stages via ST-Link/OpenOCD.
#
# Usage: mcu/AT32F425/flash.sh [A|B|C|D|E|E2|E3] [--build-only] [--no-reset]
#   A|B|C|D|E|E2|E3  which stage to build/flash (default: E3, the most recent one)
#   --build-only  build only, don't touch the debugger/hardware
#   --no-reset    program+verify but don't reset-and-run afterward
#                 (leaves the target halted, useful before an OpenOCD/GDB
#                 session so you don't race the firmware's early init)
#
# Requires an OpenOCD build with Artery AT32F4x support. Override with
# OPENOCD=/path/to/openocd if needed; this script defaults to the
# AT32F4x-capable build already present on this machine.
#
# Uses the repo-local mcu/AT32F425/openocd_at32f425.cfg (verified working
# on real hardware) instead of the generic target/artery/at32f4x.cfg.
# Do not switch this back to a /tmp-based config -- those don't survive
# reboots.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="$REPO_ROOT/build"

STAGE="E9"
BUILD_ONLY=0
RESET_AFTER=1

for arg in "$@"; do
	case "$arg" in
		A|a) STAGE="A" ;;
		B|b) STAGE="B" ;;
		C|c) STAGE="C" ;;
		D|d) STAGE="D" ;;
		E|e) STAGE="E" ;;
		E2|e2) STAGE="E2" ;;
		E3|e3) STAGE="E3" ;;
		E4|e4) STAGE="E4" ;;
		E5|e5) STAGE="E5" ;;
		E6|e6) STAGE="E6" ;;
		E7|e7) STAGE="E7" ;;
		E8|e8) STAGE="E8" ;;
		E9|e9) STAGE="E9" ;;
		--build-only) BUILD_ONLY=1 ;;
		--no-reset) RESET_AFTER=0 ;;
		-h|--help)
			sed -n '2,15p' "$0"
			exit 0
			;;
		*)
			echo "Unknown argument: $arg" >&2
			exit 1
			;;
	esac
done

TARGET="MOUSEF425_STAGE_${STAGE}"
ELF="$BUILD_DIR/${TARGET}-rev17.1.elf"

: "${OPENOCD:=/home/naoto/tools/openocd-install/bin/openocd}"
OPENOCD_SCRIPTS="${OPENOCD_SCRIPTS:-/home/naoto/tools/openocd-install/share/openocd/scripts}"
BOARD_CFG="$SCRIPT_DIR/openocd_at32f425.cfg"

echo "==> Building ${TARGET}"
if [ ! -d "$BUILD_DIR" ]; then
	mkdir -p "$BUILD_DIR"
	(cd "$BUILD_DIR" && cmake .. -DCMAKE_BUILD_TYPE=Release)
fi
cmake --build "$BUILD_DIR" --target "$TARGET" -- -j"$(nproc)"

if [ "$BUILD_ONLY" -eq 1 ]; then
	echo "==> --build-only: not touching hardware. ELF: $ELF"
	exit 0
fi

if [ ! -x "$OPENOCD" ]; then
	echo "OpenOCD not found/executable at: $OPENOCD" >&2
	echo "Set OPENOCD=/path/to/openocd (must support AT32F4x via artery flash driver)." >&2
	exit 1
fi

if [ ! -f "$BOARD_CFG" ]; then
	echo "Board config not found: $BOARD_CFG" >&2
	exit 1
fi

RESET_CMD="reset"
[ "$RESET_AFTER" -eq 0 ] && RESET_CMD=""

echo "==> Flashing ${ELF} via OpenOCD (ST-Link + AT32F425, $BOARD_CFG)"
"$OPENOCD" \
	-s "$OPENOCD_SCRIPTS" \
	-f "$BOARD_CFG" \
	-c "program \"$ELF\" verify $RESET_CMD exit"

echo "==> Done."
if [ "$RESET_AFTER" -eq 0 ]; then
	echo "    Target left halted (--no-reset). Start mcu/AT32F425/openocd.sh"
	echo "    and attach GDB, or re-run without --no-reset to just run it."
fi
