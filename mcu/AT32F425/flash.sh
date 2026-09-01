#!/usr/bin/env bash
# Build + flash one of the AT32F425 bring-up stages via ST-Link/OpenOCD.
#
# Usage: mcu/AT32F425/flash.sh [A|B|C|D] [--build-only] [--no-reset]
#   A|B|C|D   which stage to build/flash (default: D, the most recent one)
#   --build-only  build only, don't touch the debugger/hardware
#   --no-reset    program+verify but don't reset-and-run afterward
#                 (leaves the target halted, useful before an OpenOCD/GDB
#                 session so you don't race the firmware's early init)
#
# Requires an OpenOCD build with Artery AT32F4x support (target/artery/
# at32f4x.cfg) -- the stock distro package usually does NOT have this.
# Override with OPENOCD=/path/to/openocd if needed; this script defaults
# to the AT32F4x-capable build already present on this machine.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="$REPO_ROOT/build"

STAGE="D"
BUILD_ONLY=0
RESET_AFTER=1

for arg in "$@"; do
	case "$arg" in
		A|a) STAGE="A" ;;
		B|b) STAGE="B" ;;
		C|c) STAGE="C" ;;
		D|d) STAGE="D" ;;
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
INTERFACE_CFG="interface/stlink.cfg"
TARGET_CFG="target/artery/at32f4x.cfg"

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
	echo "Set OPENOCD=/path/to/openocd (must support target/artery/at32f4x.cfg)." >&2
	exit 1
fi

RESET_CMD="reset"
[ "$RESET_AFTER" -eq 0 ] && RESET_CMD=""

echo "==> Flashing ${ELF} via OpenOCD (ST-Link + AT32F4x)"
"$OPENOCD" \
	-f "$INTERFACE_CFG" \
	-f "$TARGET_CFG" \
	-c "program \"$ELF\" verify $RESET_CMD exit"

echo "==> Done."
if [ "$RESET_AFTER" -eq 0 ]; then
	echo "    Target left halted (--no-reset). Start mcu/AT32F425/openocd.sh"
	echo "    and attach GDB, or re-run without --no-reset to just run it."
fi
