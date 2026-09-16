#!/usr/bin/env python3
"""ESCape32 throttle console and commutation tracer over SWD.

The only link between this machine and the board is an ST-Link/V2, which has
no virtual COM port, so the firmware's own serial CLI on PA2 is out of reach.
This gets to the same place by another route: OpenOCD reads and writes target
RAM while the CPU keeps running, and ESCape32 keeps its whole control state in
globals (src/main.c:80-85).  Writing `throt` here is precisely what the
firmware's `throt <value>` CLI command does (src/prog.c:255).

Symbol addresses are resolved from the ELF on every start, so a rebuild can
never leave this pointing at stale globals.

Arming: with cfg.arm = 1 (the default) main() parks in a loop that primes
`throt` to 1 and waits for 250ms of zero throttle (src/main.c:631).  A
receiver supplies that zero on its own; here nothing does, so the ESC sits
in the loop until we write 0 -- which connect does.  `throt` reading back as
1 on a fresh connect is that loop, not a fault.

CAUTION -- nothing on the ESC times the throttle out in this mode.  Whatever
is written stays applied until something writes again, so if this process is
killed outright the motor keeps spinning.  Keep a hand on the bench supply;
that is the real emergency stop, not this script.
"""

from __future__ import annotations

import argparse
import os
import shutil
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OPENOCD = os.environ.get(
    "OPENOCD", "/home/naoto/tools/openocd-install/bin/openocd")
DEFAULT_SCRIPTS = os.environ.get(
    "OPENOCD_SCRIPTS", "/home/naoto/tools/openocd-install/share/openocd/scripts")
DEFAULT_CFG = os.path.join(HERE, "openocd_stm32g431.cfg")

# name -> (size in bytes, signed).  These all live within a few dozen bytes
# of each other in .bss, which is what makes the single-shot snapshot below
# possible; see Escape32.snapshot().
WANTED = {
    "reverse": (1, False),   # commutation direction, 0 = forward
    "lock":    (1, False),
    "fast":    (1, False),
    "sync":    (1, False),   # zero-crossing sync counter, 0..6
    "prep":    (1, False),
    "analog":  (1, False),
    "cutback": (4, True),
    "ival":    (4, True),    # commutation interval
    "sine":    (4, True),    # non-zero only during sine startup
    "step":    (4, True),    # commutation step, 1..6
    "oldstep": (4, True),
    "tick":    (4, False),   # 16kHz counter
    "erpm":    (4, True),
    "ertm":    (4, True),    # electrical revolution time, us
    "throt":   (4, True),
}


def load_symbols(elf: str) -> dict[str, tuple[int, int, bool]]:
    """Map name -> (address, size, signed) using nm on the freshly built ELF."""
    nm = shutil.which("arm-none-eabi-nm") or shutil.which("nm")
    if not nm:
        raise RuntimeError("neither arm-none-eabi-nm nor nm is on PATH")
    out = subprocess.run([nm, elf], capture_output=True, text=True,
                         check=True).stdout
    syms = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] in WANTED:
            size, signed = WANTED[parts[2]]
            syms[parts[2]] = (int(parts[0], 16), size, signed)
    missing = set(WANTED) - set(syms)
    if missing:
        raise RuntimeError(f"{elf}: symbols not found: {sorted(missing)}")
    return syms


class OpenOCD:
    """OpenOCD spawned as a child, driven over its Tcl RPC port.

    A fresh openocd process per command would cost about a second each, which
    is far too slow to hold a throttle ramp together, let alone trace
    commutation, so it stays resident.  Note that plain `init` does not reset
    the target -- the firmware keeps running across a connect, which is what
    lets us look at a board that has been up since power-on.
    """

    def __init__(self, cfg: str, openocd: str, scripts: str, port: int = 6666):
        self.port = port
        self.proc = subprocess.Popen(
            [openocd, "-s", scripts, "-f", cfg],
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        self.sock = None
        deadline = time.time() + 10
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError("openocd exited:\n" + self.proc.stderr.read())
            try:
                self.sock = socket.create_connection(("127.0.0.1", port),
                                                     timeout=2)
                break
            except OSError:
                time.sleep(0.1)
        if self.sock is None:
            self.close()
            raise RuntimeError(f"openocd did not open its Tcl port {port}")

    def cmd(self, line: str) -> str:
        self.sock.sendall(line.encode() + b"\x1a")
        buf = b""
        while not buf.endswith(b"\x1a"):
            chunk = self.sock.recv(4096)
            if not chunk:
                raise RuntimeError("openocd closed the connection")
            buf += chunk
        return buf[:-1].decode().strip()

    def read_bytes(self, addr: int, count: int) -> bytes:
        words = self.cmd(f"read_memory {addr:#x} 8 {count}").split()
        return bytes(int(w, 0) & 0xff for w in words)

    def read(self, addr: int, width: int = 32) -> int:
        return int(self.cmd(f"read_memory {addr:#x} {width} 1").split()[0], 0)

    def write(self, addr: int, value: int, width: int = 32) -> None:
        masked = value & ((1 << width) - 1)
        self.cmd(f"write_memory {addr:#x} {width} {{{masked}}}")

    def close(self) -> None:
        if self.sock:
            try:
                self.cmd("shutdown")
            except Exception:
                pass
            self.sock.close()
        try:
            self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.proc.kill()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class Escape32:
    def __init__(self, ocd: OpenOCD, syms: dict[str, tuple[int, int, bool]]):
        self.ocd = ocd
        self.syms = syms
        self.base = min(a for a, _, _ in syms.values())
        self.span = max(a + s for a, s, _ in syms.values()) - self.base

    def get(self, name: str) -> int:
        addr, size, signed = self.syms[name]
        v = self.ocd.read(addr, size * 8)
        return v - (1 << (size * 8)) if signed and v >> (size * 8 - 1) else v

    def put(self, name: str, value: int) -> None:
        addr, size, _ = self.syms[name]
        self.ocd.write(addr, value, size * 8)

    def snapshot(self) -> dict[str, int]:
        """Every field in one RPC round trip, so a row is one instant.

        Reading the variables one at a time would take a round trip each and
        smear a single sample across several commutations -- useless for
        deciding whether `step` and `sync` move together.
        """
        blob = self.ocd.read_bytes(self.base, self.span)
        out = {}
        for name, (addr, size, signed) in self.syms.items():
            off = addr - self.base
            out[name] = int.from_bytes(blob[off:off + size], "little",
                                       signed=signed)
        return out

    def arm(self) -> None:
        """Supply the zero throttle the power-on arming loop waits for."""
        self.set_throt(0)

    def set_throt(self, value) -> None:
        value = int(value)
        if not -2000 <= value <= 2000:
            raise ValueError("throt must be within -2000..2000")
        # The firmware's own `throt` command clears `analog` too, so that a
        # board configured for analog input cannot immediately overwrite what
        # we just set (src/prog.c:255).
        self.put("analog", 0)
        self.put("throt", value)

    def safe_stop(self) -> None:
        try:
            self.set_throt(0)
        except Exception:
            # Last resort: a reset clears MOE, so the bridge goes open and
            # the motor coasts rather than staying driven.
            try:
                self.ocd.cmd("reset halt")
            except Exception:
                pass


def fmt_status(st: dict[str, int]) -> str:
    """Only the commutation state is worth showing on this build.

    volt and curr are always 0 because the target is compiled without
    SENS_MAP (PA6/PF1 are unconnected), and temp1 comes off an ADC whose
    readings on this board swing far too wide to mean anything.  None of
    that touches commutation: BEMF on G431 is detected with the comparators,
    not the ADC.
    """
    return (f"throt {st['throt']:>5}  step {st['step']}  sync {st['sync']}"
            f"  rev {st['reverse']}  ival {st['ival']:>8}"
            f"  ERPM {st['erpm']:>7}  ertm {st['ertm']:>8}us")


def do_trace(esc: Escape32, throt: int, secs: float) -> None:
    """Hold a throttle and record how the commutation state evolves.

    What to read out of it:

      sync  counts accepted zero crossings, capped at 6 (src/main.c).  If it
            climbs to 6 and stays there the ESC is running closed-loop on
            BEMF.  If it keeps collapsing to 0, commutation is being driven
            by the interval timeout instead -- open loop -- and the rotor is
            free to slip, stall, or turn whichever way it likes.  That is the
            state to suspect first when the direction is not repeatable.

      step  always advances 1->2->...->6->1 while `reverse` is 0, regardless
            of which way the shaft actually turns, so it does not report
            physical direction.  It does show whether commutation is running
            at all, and how fast.

      ival  the commutation interval: falling means accelerating.
    """
    print(f"trace: throt {throt} for {secs}s  (Ctrl-C cuts throttle)")
    samples: list[tuple[float, dict[str, int]]] = []
    t0 = time.time()
    try:
        esc.set_throt(throt)
        while time.time() - t0 < secs:
            samples.append((time.time() - t0, esc.snapshot()))
    except KeyboardInterrupt:
        pass
    finally:
        esc.safe_stop()

    if not samples:
        print("no samples")
        return

    print(f"  {len(samples)} samples in {samples[-1][0]:.2f}s "
          f"({len(samples) / max(samples[-1][0], 1e-6):.0f}/s)")
    print("    t(ms)  step  sync  prep  rev      ival     ERPM    ertm(us)")
    prev = None
    shown = 0
    for t, s in samples:
        key = (s["step"], s["sync"], s["prep"], s["reverse"])
        if key == prev:
            continue
        prev = key
        if shown < 60:  # a long run changes state thousands of times
            print(f"  {t * 1000:7.1f}  {s['step']:>4}  {s['sync']:>4}"
                  f"  {s['prep']:>4}  {s['reverse']:>3}  {s['ival']:>8}"
                  f"  {s['erpm']:>7}  {s['ertm']:>9}")
        shown += 1
    if shown > 60:
        print(f"  ... {shown - 60} more transitions not shown")

    syncs = [s["sync"] for _, s in samples]
    revs = {s["reverse"] for _, s in samples}
    erpms = [s["erpm"] for _, s in samples if s["erpm"]]
    ivals = [s["ival"] for _, s in samples if s["ival"]]
    drops = sum(1 for a, b in zip(syncs, syncs[1:]) if b < a)
    print()
    print(f"  sync: max {max(syncs)}, ended at {syncs[-1]}, dropped {drops}x")
    print(f"  reverse: {sorted(revs)}"
          + ("" if len(revs) == 1 else "   <-- direction flag changed!"))
    if erpms:
        print(f"  ERPM: {min(erpms)}..{max(erpms)}")
    if ivals:
        print(f"  ival: {min(ivals)}..{max(ivals)}")
    print()
    if max(syncs) < 6:
        print("  sync never reached 6: the ESC never locked onto BEMF, so "
              "commutation ran\n  open loop off the interval timeout.  An "
              "unrepeatable shaft direction is expected\n  in that state -- "
              "look at the comparator inputs and phase wiring, not at the\n"
              "  direction settings.")
    elif drops:
        print("  sync reached 6 but kept collapsing: BEMF is being detected "
              "but not reliably.")
    else:
        print("  sync held at 6: the ESC ran closed-loop on BEMF for this run.")


def do_ramp(esc: Escape32, stop: int, step: int, dwell: float) -> None:
    """Walk the throttle setpoint up, reporting ERPM at each level.

    Deliberately dumb: it only moves the setpoint.  ESCape32's own
    duty_spup / duty_ramp / duty_rate governor paces the actual duty, and
    cfg.duty_max caps it -- shape the curve by rebuilding with different
    values in CMakeLists.txt, not by stepping faster here.
    """
    print(f"ramping 0 -> {stop} in steps of {step}, {dwell}s each"
          "  (Ctrl-C cuts throttle)")
    level = 0
    try:
        while level < stop:
            level = min(level + step, stop)
            esc.set_throt(level)
            time.sleep(dwell)
            print(f"  {level * 100 / 2000:5.1f}% of range   "
                  f"{fmt_status(esc.snapshot())}")
    finally:
        esc.safe_stop()
        print("throttle cut")


def do_repl(esc: Escape32) -> None:
    print("commands:  throt <-2000..2000> | stop | status | watch [sec]")
    print("           trace <throt> [sec] | ramp <max> [step] [dwell]")
    print("           reset | quit")
    while True:
        try:
            line = input("swd> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not line:
            continue
        parts = line.split()
        verb, args = parts[0], parts[1:]
        try:
            if verb in ("quit", "exit", "q"):
                return
            elif verb == "throt":
                esc.set_throt(args[0])
                print(fmt_status(esc.snapshot()))
            elif verb == "stop":
                esc.set_throt(0)
                print("throttle cut")
            elif verb == "status":
                print(fmt_status(esc.snapshot()))
            elif verb == "watch":
                secs = float(args[0]) if args else 5.0
                end = time.time() + secs
                try:
                    while time.time() < end:
                        print(fmt_status(esc.snapshot()))
                        time.sleep(0.2)
                except KeyboardInterrupt:
                    esc.safe_stop()
                    print("\nthrottle cut")
            elif verb == "trace":
                do_trace(esc, int(args[0]),
                         float(args[1]) if len(args) > 1 else 2.0)
            elif verb == "ramp":
                do_ramp(esc, int(args[0]),
                        int(args[1]) if len(args) > 1 else 100,
                        float(args[2]) if len(args) > 2 else 1.0)
            elif verb == "reset":
                esc.ocd.cmd("reset run")
                time.sleep(1.5)  # let the arming loop come back up
                esc.arm()
                print("reset, re-armed")
            else:
                print(f"unknown command: {verb}")
        except KeyboardInterrupt:
            esc.safe_stop()
            print("\nthrottle cut")
        except (IndexError, ValueError) as e:
            print(f"bad arguments: {e}")
        except RuntimeError as e:
            print(f"error: {e}")


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--elf",
                   help="firmware ELF (default: newest build/MOUSEG431-rev*.elf)")
    p.add_argument("--cfg", default=DEFAULT_CFG)
    p.add_argument("--openocd", default=DEFAULT_OPENOCD)
    p.add_argument("--scripts", default=DEFAULT_SCRIPTS)
    p.add_argument("--trace", type=int, metavar="THROT",
                   help="hold this throttle, record commutation state, exit")
    p.add_argument("--secs", type=float, default=2.0, help="--trace duration")
    p.add_argument("--ramp", type=int, metavar="MAX",
                   help="ramp to this throttle and exit (1..2000)")
    p.add_argument("--step", type=int, default=100)
    p.add_argument("--dwell", type=float, default=1.0)
    args = p.parse_args()

    elf = args.elf
    if not elf:
        import glob
        found = sorted(glob.glob(os.path.join(HERE, "build",
                                              "MOUSEG431-rev*.elf")))
        if not found:
            return "no build/MOUSEG431-rev*.elf -- run ./flash.sh first, or pass --elf"
        elf = found[-1]

    try:
        syms = load_symbols(elf)
    except (RuntimeError, subprocess.CalledProcessError) as e:
        return str(e)

    try:
        ocd = OpenOCD(args.cfg, args.openocd, args.scripts)
    except (RuntimeError, OSError) as e:
        return str(e)

    with ocd:
        esc = Escape32(ocd, syms)
        print(f"state block {esc.base:#x}..{esc.base + esc.span:#x} "
              f"({esc.span} bytes) from {os.path.relpath(elf, HERE)}")
        if esc.get("throt") == 1:
            print("ESC is in the power-on arming loop; sending zero throttle")
        esc.arm()
        try:
            if args.trace is not None:
                do_trace(esc, args.trace, args.secs)
            elif args.ramp is not None:
                if not 0 < args.ramp <= 2000:
                    return "--ramp must be within 1..2000"
                do_ramp(esc, args.ramp, args.step, args.dwell)
            else:
                do_repl(esc)
        finally:
            esc.safe_stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
