#!/usr/bin/env python3
"""ESCape32 throttle console over SWD -- no USB-UART required.

The only link between this machine and the board is an ST-Link/V2, which has
no virtual COM port, so the firmware's own serial CLI on PA2 is out of reach.
This gets to the same place by another route: OpenOCD reads and writes target
RAM while the CPU keeps running, and ESCape32 keeps `throt` and `erpm` in
plain globals (src/main.c:80).  Writing `throt` here is precisely what the
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

# Globals we touch.  int on this target is 32-bit; `analog` is a char.
WANTED = {"throt": 32, "erpm": 32, "ertm": 32, "temp1": 32, "volt": 32,
          "curr": 32, "brake": 32, "analog": 8, "rearm": 8}


def load_symbols(elf: str) -> dict[str, tuple[int, int]]:
    """Map name -> (address, width) using nm on the freshly built ELF."""
    nm = shutil.which("arm-none-eabi-nm") or shutil.which("nm")
    if not nm:
        raise RuntimeError("neither arm-none-eabi-nm nor nm is on PATH")
    out = subprocess.run([nm, elf], capture_output=True, text=True,
                         check=True).stdout
    syms = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] in WANTED:
            syms[parts[2]] = (int(parts[0], 16), WANTED[parts[2]])
    missing = set(WANTED) - set(syms)
    if missing:
        raise RuntimeError(f"{elf}: symbols not found: {sorted(missing)}")
    return syms


class OpenOCD:
    """OpenOCD spawned as a child, driven over its Tcl RPC port.

    A fresh openocd process per command would cost about a second each, which
    is far too slow to hold a throttle ramp together, so it stays resident.
    Note that plain `init` does not reset the target -- the firmware keeps
    running across a connect, which is what lets us look at a board that has
    been up since power-on.
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


def signed(v: int) -> int:
    return v - (1 << 32) if v & 0x80000000 else v


class Escape32:
    def __init__(self, ocd: OpenOCD, syms: dict[str, tuple[int, int]]):
        self.ocd = ocd
        self.syms = syms

    def get(self, name: str) -> int:
        addr, width = self.syms[name]
        v = self.ocd.read(addr, width)
        return signed(v) if width == 32 else v

    def put(self, name: str, value: int) -> None:
        addr, width = self.syms[name]
        self.ocd.write(addr, value, width)

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

    def status(self) -> dict[str, int]:
        return {k: self.get(k) for k in
                ("throt", "erpm", "ertm", "temp1", "volt", "curr", "rearm")}

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
    """Only throt/erpm/ertm are worth showing on this build.

    volt and curr are always 0 because the target is compiled without
    SENS_MAP (PA6/PF1 are unconnected), and temp1 comes off an ADC whose
    readings on this board swing far too wide to mean anything -- the
    VREFINT channel alone wanders over 1095..1499 where it should sit near
    1518.  None of that touches commutation: BEMF on G431 is detected with
    the comparators, not the ADC.
    """
    return (f"throt {st['throt']:>5}   ERPM {st['erpm']:>7}   "
            f"ertm {st['ertm']:>7}us"
            + ("   [rearm]" if st["rearm"] else ""))


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
            st = esc.status()
            print(f"  {level * 100 / 2000:5.1f}% of range   {fmt_status(st)}")
    finally:
        esc.safe_stop()
        print("throttle cut")


def do_repl(esc: Escape32) -> None:
    print("commands:  throt <-2000..2000> | stop | status | watch [sec] | "
          "ramp <max> [step] [dwell] | reset | quit")
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
                print(fmt_status(esc.status()))
            elif verb == "stop":
                esc.set_throt(0)
                print("throttle cut")
            elif verb == "status":
                print(fmt_status(esc.status()))
            elif verb == "watch":
                secs = float(args[0]) if args else 5.0
                end = time.time() + secs
                try:
                    while time.time() < end:
                        print(fmt_status(esc.status()))
                        time.sleep(0.2)
                except KeyboardInterrupt:
                    esc.safe_stop()
                    print("\nthrottle cut")
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
    print(f"symbols from {os.path.relpath(elf, HERE)}: "
          + "  ".join(f"{k}@{v[0]:#x}" for k, v in sorted(syms.items())))

    try:
        ocd = OpenOCD(args.cfg, args.openocd, args.scripts)
    except (RuntimeError, OSError) as e:
        return str(e)

    with ocd:
        esc = Escape32(ocd, syms)
        if esc.get("throt") == 1:
            print("ESC is in the power-on arming loop; sending zero throttle")
        esc.arm()
        try:
            if args.ramp is not None:
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
