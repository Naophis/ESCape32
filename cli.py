#!/usr/bin/env python3
"""ESCape32 serial CLI client -- half-duplex, 38400 8N1 on the signal wire.

The MOUSEG431 target is built with IO_PA2, so when nothing drives PA2 for
about a second after power-up the firmware gives up on pulse input and falls
back to the CLI (src/io.c entryirq() -> cliirq()).  That is the state the
board sits in with the throttle wire unconnected, and this talks to it.

Wiring -- a 3V3 USB-UART with TX and RX both on the signal line:

    USB-UART TX --[1k]--+
    USB-UART RX --------+---- PA2 (signal)
    USB-UART GND ------------ board GND

The series resistor keeps the adapter from fighting the MCU while the ESC
drives the line.  Because the wire is shared, everything sent here comes
straight back into our own RX; cmd() drops that echo.

Send `throt 0` before anything else.  With cfg.arm = 1 (the default) main()
parks in a loop that only exits after 250ms of zero throttle, and it primes
`throt` to 1 on the way in (src/main.c:631).  A receiver supplies that zero
on its own; in CLI mode nothing does, so the ESC waits there indefinitely.
arm() below does it on every connect -- it is also the safe setpoint, so it
costs nothing.

Two more things worth knowing:

  * In CLI mode ESCape32 never starts the IWDG -- it is armed only on the
    pulse-input path -- so a throttle set with `throt` stays applied until
    something changes it.  There is no failsafe if this script dies or the
    cable falls out.  Every exit path below sends `throt 0` first.

  * A framing error or a full receive buffer makes the firmware trip the
    window watchdog on purpose (src/io.c cliirq()), i.e. noise on the line
    reboots the ESC.  Keep the wire short; if the board keeps restarting,
    suspect the wiring before the firmware.
"""

from __future__ import annotations

import argparse
import re
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is missing:  pip install pyserial")

TERMINATORS = ("OK", "ERROR")


class Escape32:
    def __init__(self, port: str, baud: int = 38400, timeout: float = 1.0):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.timeout = timeout

    def close(self) -> None:
        self.ser.close()

    def cmd(self, line: str, timeout: float | None = None) -> tuple[bool, list[str]]:
        """Send one command; return (ok, output lines).

        execcmd() always finishes with a bare "OK" or "ERROR" line, so that
        is what we read until -- there is no prompt to synchronise on.
        """
        line = line.strip()
        if not line:
            return True, []
        self.ser.reset_input_buffer()
        self.ser.write((line + "\n").encode())
        self.ser.flush()

        deadline = time.time() + (timeout or self.timeout)
        buf = ""
        while time.time() < deadline:
            chunk = self.ser.read(256)
            if not chunk:
                continue
            buf += chunk.decode("ascii", "replace")
            if any(l.strip() in TERMINATORS for l in buf.split("\n")[:-1]):
                break

        ok = None
        out: list[str] = []
        for raw in buf.split("\n"):
            text = raw.strip()
            if text == line:  # our own bytes, looped back off the shared wire
                continue
            if text in TERMINATORS:
                ok = text == "OK"
                break
            if text:
                out.append(text)
        if ok is None:
            raise TimeoutError(
                f"no OK/ERROR within {timeout or self.timeout}s -- received {buf!r}"
            )
        return ok, out

    def expect(self, line: str, timeout: float | None = None) -> list[str]:
        ok, out = self.cmd(line, timeout)
        if not ok:
            raise RuntimeError(f"ESC rejected: {line}")
        return out

    def throt(self, value: int) -> None:
        self.expect(f"throt {int(value)}")

    def arm(self) -> None:
        """Release the power-on arming loop by supplying the zero throttle
        it is waiting for.  Harmless to repeat; the ESC is already past the
        loop on every connect after the first."""
        self.throt(0)

    def erpm(self) -> int | None:
        for line in self.expect("info"):
            m = re.match(r"ERPM:\s*(-?\d+)", line)
            if m:
                return int(m.group(1))
        return None


def do_repl(esc: Escape32) -> None:
    print("ESCape32 CLI -- 'help' for commands, Ctrl-D to quit (sends throt 0)")
    while True:
        try:
            line = input("> ")
        except (EOFError, KeyboardInterrupt):
            print()
            return
        try:
            ok, out = esc.cmd(line)
        except TimeoutError as e:
            print(f"timeout: {e}", file=sys.stderr)
            continue
        for l in out:
            print(l)
        print("OK" if ok else "ERROR")


def do_ramp(esc: Escape32, stop: int, step: int, dwell: float) -> None:
    """Step the throttle up, reading ERPM at each level.

    This is deliberately dumb: it only moves the setpoint and reports what
    came back.  ESCape32's own duty_spup / duty_ramp / duty_rate governor
    does the actual pacing, and cfg.duty_max caps how much of the supply
    ever reaches the motor -- set those with `set` before ramping rather
    than trying to shape the curve from here.
    """
    print(f"ramping 0 -> {stop} in steps of {step}, {dwell}s each")
    print("Ctrl-C stops and cuts the throttle.")
    level = 0
    try:
        while level < stop:
            level = min(level + step, stop)
            esc.throt(level)
            time.sleep(dwell)
            rpm = esc.erpm()
            pct = level * 100 / 2000
            print(f"  throt {level:>5}  ({pct:5.1f}% of range)   ERPM {rpm}")
    finally:
        esc.throt(0)
        print("throttle cut")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("-p", "--port", default="/dev/ttyUSB0")
    p.add_argument("-b", "--baud", type=int, default=38400)
    p.add_argument("-c", "--cmd", action="append", metavar="CMD",
                   help="run a command and exit (repeatable)")
    p.add_argument("--ramp", action="store_true",
                   help="step the throttle up, printing ERPM at each level")
    p.add_argument("--max", type=int, default=400, metavar="N",
                   help="--ramp: highest throttle, 0..2000 (default 400 = 20%%)")
    p.add_argument("--step", type=int, default=100, metavar="N",
                   help="--ramp: throttle increment (default 100)")
    p.add_argument("--dwell", type=float, default=1.0, metavar="SEC",
                   help="--ramp: seconds to hold each level (default 1.0)")
    args = p.parse_args()

    try:
        esc = Escape32(args.port, args.baud)
    except serial.SerialException as e:
        return f"cannot open {args.port}: {e}"

    try:
        esc.arm()  # required before the ESC will accept anything else
        if args.cmd:
            for c in args.cmd:
                ok, out = esc.cmd(c)
                for l in out:
                    print(l)
                print("OK" if ok else "ERROR")
                if not ok:
                    return 1
        elif args.ramp:
            if not 0 < args.max <= 2000:
                return "--max must be in 1..2000"
            do_ramp(esc, args.max, args.step, args.dwell)
        else:
            do_repl(esc)
    except (TimeoutError, RuntimeError) as e:
        return str(e)
    finally:
        # Nothing else cuts the motor if we leave unexpectedly.
        try:
            esc.cmd("throt 0", timeout=0.5)
        except Exception:
            pass
        esc.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
