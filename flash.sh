#!/usr/bin/env bash
#
# Build and flash the MOUSEG431 target (STM32G431KBU6 + MP6540HA).
#
# Usage:
#   ./flash.sh            build, then flash
#   ./flash.sh -n         flash only (skip the build)
#   ./flash.sh -c         connection check only (no build, no flash)
#   ./flash.sh -a         app only (skip the bootloader)
#   ./flash.sh -t NAME    use a different target (default: MOUSEG431)
#
# Environment overrides:
#   OPENOCD          path to the openocd binary
#   OPENOCD_SCRIPTS  path to openocd's script directory
#
set -euo pipefail

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SRC_DIR/build"
CFG="$SRC_DIR/openocd_stm32g431.cfg"

TARGET=MOUSEG431
# ESCape32 reserves the first 4K of flash for its own bootloader
# (mcu/STM32G431/config.ld: boot=0x08000000+4K, cfg=+2K, app from
# 0x08001800). Flashing only the app leaves 0x08000000 erased, so the
# reset vector is 0xFFFFFFFF and the CPU locks up immediately -- with
# the app itself verifying just fine, which makes it a confusing
# failure. Both images are therefore flashed together by default.
BOOT_TARGET=BOOT4_PA2
DO_BUILD=1
DO_FLASH=1
DO_BOOT=1

: "${OPENOCD:=/home/naoto/tools/openocd-install/bin/openocd}"
: "${OPENOCD_SCRIPTS:=/home/naoto/tools/openocd-install/share/openocd/scripts}"

while [ $# -gt 0 ]; do
	case "$1" in
		-n|--no-build) DO_BUILD=0 ;;
		-a|--app-only) DO_BOOT=0 ;;
		-c|--check) DO_BUILD=0; DO_FLASH=0 ;;
		-t|--target) TARGET="$2"; shift ;;
		-h|--help) sed -n '3,14p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
		*) echo "unknown option: $1" >&2; exit 1 ;;
	esac
	shift
done

die() { echo "ERROR: $*" >&2; exit 1; }

[ -x "$OPENOCD" ] || die "openocd not found/executable at: $OPENOCD (set OPENOCD=...)"
[ -f "$CFG" ] || die "openocd config not found: $CFG"

# --- ST-Link presence -------------------------------------------------
# Checked before anything else: a missing probe is by far the most common
# failure, and OpenOCD's own message for it ("open failed") is unhelpful.
if ! lsusb 2>/dev/null | grep -qiE '0483:(374[0-9b-f]|3752)'; then
	die "no ST-Link found on USB. Connect the probe and retry."
fi

# --- Build ------------------------------------------------------------
if [ "$DO_BUILD" = 1 ]; then
	if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
		echo "==> configuring build directory"
		# LIBOPENCM3_DIR must be absolute: CMakeLists passes it straight to
		# include_directories(), which resolves relative paths against the
		# SOURCE dir, not the build dir.
		cmake -S "$SRC_DIR" -B "$BUILD_DIR" -DLIBOPENCM3_DIR="$SRC_DIR/libopencm3"
	fi
	echo "==> building $TARGET"
	cmake --build "$BUILD_DIR" --target "$TARGET" -j"$(nproc)"
	if [ "$DO_BOOT" = 1 ]; then
		echo "==> building $BOOT_TARGET"
		cmake --build "$BUILD_DIR" --target "$BOOT_TARGET" -j"$(nproc)"
	fi
fi

# --- Locate the hex ---------------------------------------------------
# The filename carries the firmware revision (e.g. MOUSEG431-rev17.1.hex),
# so glob for it rather than hardcoding the current revision.
shopt -s nullglob
hexes=("$BUILD_DIR/$TARGET"-rev*.hex)
shopt -u nullglob
[ ${#hexes[@]} -gt 0 ] || die "no hex found for $TARGET in $BUILD_DIR (build first, or drop -n)"
[ ${#hexes[@]} -eq 1 ] || die "multiple hex files for $TARGET: ${hexes[*]}"
HEX="${hexes[0]}"

BOOT_HEX=""
if [ "$DO_BOOT" = 1 ]; then
	shopt -s nullglob
	boot_hexes=("$BUILD_DIR/boot/$BOOT_TARGET"-rev*.hex)
	shopt -u nullglob
	[ ${#boot_hexes[@]} -eq 1 ] ||
		die "expected exactly one hex for $BOOT_TARGET in $BUILD_DIR/boot (found ${#boot_hexes[@]})"
	BOOT_HEX="${boot_hexes[0]}"
fi

# --- Connection check -------------------------------------------------
# The log is captured to a file rather than piped into grep: `grep -q`
# exits on its first match, which SIGPIPEs openocd/tee upstream, and
# `set -o pipefail` would then report a failure for a perfectly good
# connection. openocd's own exit status is not used as the verdict
# either -- the "processor detected" line is the reliable signal.
echo "==> checking SWD connection"
probe_log="$(mktemp)"
trap 'rm -f "$probe_log"' EXIT
"$OPENOCD" -s "$OPENOCD_SCRIPTS" -f "$CFG" \
	-c "init" -c "halt" -c "shutdown" >"$probe_log" 2>&1 || true
cat "$probe_log"
grep -q "processor detected" "$probe_log" ||
	die "target not responding on SWD (check SWDIO/SWCLK/GND wiring, NRST, and 3V3)"

if [ "$DO_FLASH" != 1 ]; then
	echo "==> connection OK (check only, nothing flashed)"
	exit 0
fi

# --- Flash ------------------------------------------------------------
if [ "$DO_BOOT" = 1 ]; then
	echo "==> flashing $(basename "$BOOT_HEX") + $(basename "$HEX")"
	"$OPENOCD" -s "$OPENOCD_SCRIPTS" -f "$CFG" \
		-c "program $BOOT_HEX verify" \
		-c "program $HEX verify reset exit"
else
	echo "==> flashing $(basename "$HEX") (app only)"
	"$OPENOCD" -s "$OPENOCD_SCRIPTS" -f "$CFG" \
		-c "program $HEX verify reset exit"
fi

echo "==> done"
