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

Saving: `save` burns the live cfg into the flash config page the same way the
firmware's own savecfg() does.  Two things about this board make that
delicate.  OpenOCD's flash loader runs in the work area at 0x20000000, i.e. on
top of the live cfg (stm32g4x.cfg, work-area-backup 0), so RAM is junk until
the firmware reboots and reloads the page.  And the bootloader (BOOT4_PA2)
only jumps to the application when the page starts with the magic 0x32ea
(boot/src/main.c:100); a page without it parks the ESC in the bootloader for
good, which from here looks like garbage in every field and no `tick`.
`cfg` and `save` therefore check that the application is actually running
first, `save` verifies the page and waits for the reboot, and `restore`
rewrites the build's default page from the ELF to get out of a bootloader
lock-up.
"""

from __future__ import annotations

import argparse
import os
import shutil
import socket
import subprocess
import sys
import select
import tempfile
import termios
import time
import tty

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

# Where each field can legitimately be.  A bulk read that came back shifted
# by a word (the failure mode of these SWD wires next to the bridge, worse at
# 16.8 V) lands outside these at once -- step 129940, rev 6, throt 0 while
# pwm climbs -- so a snapshot is re-read until it fits, and reported as a
# glitch rather than as data when it never does.
SANE = {
    "step": (0, 6), "oldstep": (0, 6), "sync": (0, 6), "reverse": (0, 1),
    "prep": (0, 1), "fast": (0, 1), "lock": (0, 2), "analog": (0, 1),
    "throt": (-2000, 2000), "ival": (0, 1 << 20), "sine": (0, 1 << 20),
    "ertm": (0, 100000000), "erpm": (0, 2000000), "cutback": (0, 1000),
}


# Comparator registers, and the three configurations compctl() installs --
# copied verbatim from mcu/STM32G431/config.c, where they are commented
# "A1>A0", "A1>A4" (both on COMP1) and "A3>A5" (COMP2).
#
# Note what that implies about the board: the virtual neutral has to reach
# BOTH PA1 and PA3.  A star network on only one of them leaves a third of the
# commutation steps comparing a phase against a floating pin, which is enough
# to keep the ESC from ever holding sync.
COMP1_CSR = 0x40010200
COMP2_CSR = 0x40010204
TIM2_TISEL = 0x4000005C   # 0x1 = TI1 from COMP1_OUT, 0x2 = from COMP2_OUT
# Clearing MOE is the only way to actually float the phases: at rest the
# firmware leaves every channel forced low with its complementary output
# enabled, so all three windings sit shorted to ground.  src/main.c:577 sets
# BDTR once at init and never touches it again, so borrowing it is safe as
# long as it is put back.
TIM1_BDTR = 0x40012C44
TIM_BDTR_MOE = 1 << 15
TIM1_ARR = 0x40012C2C
TIM1_CCR1 = 0x40012C34

# Runtime copy of cfg (src/common.h Cfg, base 0x20000000): name -> (addr, min, max).
# The CLI's `set` runs checkcfg() to clamp values; a raw RAM write does not, so
# the ranges are enforced here.  RAM only: reverts on reset unless `save`d.
CFG_FIELDS = {
    "timing":     (0x20000018, 1, 31),    # commutation advance: 16 = 15 deg, 31 ~ 29 deg
    "sine_range": (0x20000019, 0, 25),    # 0 = off, else 5..25 (% of throttle)
    "sine_power": (0x2000001a, 1, 15),
    "freq_min":   (0x2000001b, 16, 48),   # PWM kHz
    "freq_max":   (0x2000001c, 16, 96),
    "duty_min":   (0x2000001d, 1, 100),
    "duty_max":   (0x2000001e, 1, 100),
    "duty_spup":  (0x2000001f, 1, 100),   # spin-up duty cap (%)
    "duty_ramp":  (0x20000020, 0, 100),   # kERPM; 0 = governor off
    "duty_rate":  (0x20000021, 1, 100),   # %/ms
    # Offsets 327/328 per the ELF's DWARF (after music[256]).  beep() plays
    # at max(volume, 25), so below 25 nothing changes; 100 is the loudest.
    "volume":     (0x20000147, 0, 100),   # beep / startup music volume (%)
    "beacon":     (0x20000148, 0, 100),   # DSHOT beacon volume (%)
}
# Cfg starts with `id` = 0x32ea (src/main.c cfgdata).  The bootloader checks
# that halfword at the start of the flash page before it will jump to the
# application (boot/src/main.c:100), so it doubles as the test for "is the
# application running and is 0x20000000 really the cfg" -- anything else
# there is bootloader stack or OpenOCD flash-loader residue.
CFG_BASE = 0x20000000
CFG_ID = 0x32ea

RCC_CSR = 0x40021094
RESET_FLAGS = [(31, "LPWR"), (30, "WWDG"), (29, "IWDG"), (28, "SFT"),
               (27, "BOR"), (26, "PIN"), (25, "OBL")]
# Cortex-M4 fault / debug status, for a core found halted with no reset flag:
# then it did not reset, it crashed or was halted by the debugger.
SCB_CFSR = 0xE000ED28
SCB_HFSR = 0xE000ED2C
SCB_BFAR = 0xE000ED38
DCB_DHCSR = 0xE000EDF0
HFSR_BITS = [(1, "VECTTBL"), (30, "FORCED"), (31, "DEBUGEVT")]
CFSR_BITS = [(0, "IACCVIOL"), (1, "DACCVIOL"), (3, "MUNSTKERR"), (4, "MSTKERR"),
             (7, "MMARVALID"), (8, "IBUSERR"), (9, "PRECISERR"), (10, "IMPRECISERR"),
             (11, "UNSTKERR"), (12, "STKERR"), (15, "BFARVALID"), (16, "UNDEFINSTR"),
             (17, "INVSTATE"), (18, "INVPC"), (19, "NOCP"), (24, "UNALIGNED"),
             (25, "DIVBYZERO")]
DHCSR_S_LOCKUP = 1 << 19


def bits(value: int, table: list[tuple[int, str]]) -> str:
    names = [n for b, n in table if value >> b & 1]
    return " (" + " ".join(names) + ")" if names else ""

# Used by do_hsls() to drive one gate input at a time and read the phase back.
TIM1_EGR = 0x40012C14
TIM1_CCMR1 = 0x40012C18
TIM1_CCMR2 = 0x40012C1C
TIM1_CCER = 0x40012C20
TIM_EGR_COMG = 1 << 5          # CCMR/CCER are preloaded (CR2.CCPC); COM applies them
ADC1 = 0x50000000
ADC2 = 0x50000100
ADC_ISR, ADC_CR, ADC_CFGR, ADC_SQR1, ADC_DR = 0x00, 0x08, 0x0C, 0x30, 0x40
ADC1_CFGR, ADC1_SQR1 = ADC1 + ADC_CFGR, ADC1 + ADC_SQR1
ADC2_CFGR, ADC2_SQR1 = ADC2 + ADC_CFGR, ADC2 + ADC_SQR1
# Every BEMF-side pin, with the ADC that can see it (STM32G431 datasheet
# pin table): PA0/PA1/PA3 are ADC1 channels, PA4/PA5 are ADC2-only.
BEMF_ADC = [
    ("PA0", ADC1, 1),    # BEMF_A
    ("PA4", ADC2, 17),   # BEMF_B
    ("PA5", ADC2, 13),   # BEMF_C
    ("PA1", ADC1, 2),    # BEMF_N (COMP1 reference)
    ("PA3", ADC1, 4),    # BEMF_N (COMP2 reference)
]
ADC_CR_ADSTART = 1 << 2
ADC_CR_ADSTP = 1 << 4
ADC_CR_ADVREGEN = 1 << 28
ADC_ISR_EOC = 1 << 2
ADC_CFGR_CONT_OVRMOD = (1 << 13) | (1 << 12)   # free-run, DR always newest
VREFINT_CAL_ADDR = 0x1FFF75AA                   # factory VREFINT at 3.0 V
# DMA1 channel 4 feeds ADC1 in the firmware; adctrig() returns without
# touching ADC1 while its EN bit is set (mcu/STM32G431/config.c adctrig()).
DMA1_CCR4 = 0x40020044
DMA1_CNDTR4 = 0x40020048
DMA_CCR_EN = 1
COMP_VALUE_BIT = 1 << 30  # COMPx_CSR VALUE, the comparator output
COMP_CFG = [
    ("phase A: PA0 vs neutral PA1", COMP1_CSR, 0x80071),
    ("phase B: PA4 vs neutral PA1", COMP1_CSR, 0x80061),
    ("phase C: PA5 vs neutral PA3", COMP2_CSR, 0x80161),
]


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


# Linker exports (src/common.h): _cfg is the flash page the saved config lives
# in, _cfg_start.._cfg_end the SRAM copy that is the live `cfg`.  savecfg()
# writes the latter over the former; `save` here does exactly that over SWD.
LINKER = ("_cfg", "_cfg_start", "_cfg_end", "_boot")


def load_linker_symbols(elf: str) -> dict[str, int]:
    nm = shutil.which("arm-none-eabi-nm") or shutil.which("nm")
    out = subprocess.run([nm, elf], capture_output=True, text=True,
                         check=True).stdout
    found = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] in LINKER:
            found[parts[2]] = int(parts[0], 16)
    missing = set(LINKER) - set(found)
    if missing:
        raise RuntimeError(f"{elf}: linker symbols not found: {sorted(missing)}")
    return found


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
        # OpenOCD logs everything to stderr.  Handing it a pipe nobody drains
        # deadlocks it as soon as the pipe buffer fills -- it blocks on the
        # write and stops answering the Tcl port -- so the log goes to a file
        # we can still quote if startup fails.
        self.log = tempfile.NamedTemporaryFile(
            prefix="openocd-", suffix=".log", mode="w+", delete=False)
        self.proc = subprocess.Popen(
            [openocd, "-s", scripts, "-f", cfg],
            stdout=subprocess.DEVNULL, stderr=self.log, text=True)
        self.sock = None
        deadline = time.time() + 10
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError("openocd exited:\n" + self._log_tail())
            try:
                self.sock = socket.create_connection(("127.0.0.1", port),
                                                     timeout=5)
                break
            except OSError:
                time.sleep(0.1)
        if self.sock is None:
            self.close()
            raise RuntimeError(f"openocd did not open its Tcl port {port}:\n"
                               + self._log_tail())
        # Halt the core at its reset vector on any self-reset, so RCC_CSR can
        # be read before main() clears it (src/main.c:619).  That is the only
        # way to tell a 3V3 brownout (BOR) from NRST noise (PIN) from the
        # CLI's deliberate watchdog reset on UART noise at PA2 (WWDG,
        # src/io.c cliirq()) -- and the board has already reset itself once
        # the instant throttle was applied.
        self.cmd("cortex_m vector_catch reset")

    def state(self) -> str:
        """'running', 'halted', 'reset' or 'unknown', from `targets`."""
        return self.cmd("targets").splitlines()[-1].split()[-1]

    def _log_tail(self, lines: int = 20) -> str:
        try:
            self.log.flush()
            with open(self.log.name) as f:
                return "".join(f.readlines()[-lines:])
        except OSError:
            return "(log unavailable)"

    def log_grep(self, needles: tuple[str, ...], lines: int = 80) -> list[str]:
        """Recent OpenOCD log lines mentioning any of `needles`.  OpenOCD says
        why a core halted ("target halted due to ...") and when it saw a reset
        or lost the port, and none of that comes back over the Tcl socket."""
        return [l.strip() for l in self._log_tail(lines).splitlines()
                if any(n in l for n in needles)]

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
        """Word-sized reads whenever the block is aligned.

        One AP transaction per byte dominates the snapshot rate -- 84 of them
        is the difference between roughly 80 and several hundred samples a
        second, which decides whether a trace can resolve individual
        commutations at all.
        """
        if addr % 4 == 0:
            err = None
            for _ in range(self.RETRIES):
                try:
                    words = self._nums(
                        self.cmd(f"read_memory {addr:#x} 32 {(count + 3) // 4}"),
                        f"read {addr:#x}")
                    return b"".join(w.to_bytes(4, "little") for w in words)
                except RuntimeError as e:
                    err = e
                    time.sleep(0.02)
            raise RuntimeError(f"{err} (after {self.RETRIES} tries)\n"
                               + self._log_tail(3).rstrip())
        byts = self._nums(self.cmd(f"read_memory {addr:#x} 8 {count}"),
                          f"read {addr:#x}")
        return bytes(b & 0xff for b in byts)

    @staticmethod
    def _nums(reply: str, what: str) -> list[int]:
        """OpenOCD answers a failed access with prose rather than numbers --
        "Failed to read memory at 0x..." when the target has reset or dropped
        off SWD -- and int() turning that into a bare ValueError hid exactly
        the event we most needed to see.  Surface the message instead."""
        try:
            return [int(w, 0) for w in reply.split()]
        except ValueError:
            raise RuntimeError(f"{what}: openocd said {reply!r}") from None

    # A failed SWD transaction is retried before it counts.  Motor switching
    # noise on the SWD wires, or the target browning out for a moment at the
    # start-up kick, makes OpenOCD answer 'failed to read/write memory' on an
    # otherwise healthy link; a retry 20 ms later usually goes through.  When
    # it still fails, the last openocd log lines are attached: "Fail reading
    # CTRL/STAT ... Force reconnect" there means the debug port itself dropped
    # -- a reset or power event, not a glitch -- which is worth knowing at once.
    RETRIES = 3

    def read(self, addr: int, width: int = 32) -> int:
        err = None
        for _ in range(self.RETRIES):
            try:
                return self._nums(self.cmd(f"read_memory {addr:#x} {width} 1"),
                                  f"read {addr:#x}")[0]
            except RuntimeError as e:
                err = e
                time.sleep(0.02)
        raise RuntimeError(f"{err} (after {self.RETRIES} tries)\n"
                           + self._log_tail(3).rstrip())

    def write(self, addr: int, value: int, width: int = 32) -> None:
        masked = value & ((1 << width) - 1)
        reply = ""
        for _ in range(self.RETRIES):
            # A successful write_memory answers with nothing at all.  Anything
            # else is an error, and a silently failed `throt 0` is the one
            # failure this tool must never swallow.
            reply = self.cmd(f"write_memory {addr:#x} {width} {{{masked}}}")
            if not reply:
                return
            time.sleep(0.02)
        raise RuntimeError(f"write {addr:#x}: openocd said {reply!r} "
                           f"(after {self.RETRIES} tries)\n"
                           + self._log_tail(3).rstrip())

    def close(self) -> None:
        if self.sock:
            try:
                self.cmd("cortex_m vector_catch none")
                self.cmd("shutdown")
            except Exception:
                pass
            self.sock.close()
        try:
            self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        # Keep the log whenever OpenOCD complained -- it is the only record
        # of what the probe saw when the target went away.
        try:
            self.log.close()
            with open(self.log.name) as f:
                text = f.read()
            if "Error" in text or "Warn" in text:
                tail = "".join(text.splitlines(True)[-8:])
                print(f"openocd log kept at {self.log.name}:\n{tail}", end="")
            else:
                os.unlink(self.log.name)
        except OSError:
            pass

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
        # `tick` counts up at 16 kHz from boot and never otherwise goes
        # backwards, so a snapshot in which it has shrunk means the firmware
        # rebooted underneath us.  A power-class (BOR) reset does exactly
        # that without tripping the vector catch, and it leaves the ESC in
        # the arming loop waiting for a zero throttle that a plain `throt N`
        # would never supply.
        self.last_tick: int | None = None
        self.rebooted = False
        # Flash address of the cfg page (_cfg from the ELF); main() fills it in.
        self.cfg_page: int | None = None
        # Snapshots that never read back sane (see SANE); shown by trace.
        self.glitches = 0
        # Set when safe_stop() had to fall back to `reset halt`, so the next
        # check_reset() does not report our own reset as the target's.
        self.forced_reset = False

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
        for attempt in range(6):
            blob = self.ocd.read_bytes(self.base, self.span)
            out = {}
            for name, (addr, size, signed) in self.syms.items():
                off = addr - self.base
                out[name] = int.from_bytes(blob[off:off + size], "little",
                                           signed=signed)
            if (all(lo <= out[n] <= hi for n, (lo, hi) in SANE.items() if n in out)
                    and self._consistent(out)):
                break
            time.sleep(0.005)
        else:
            self.glitches += 1
            raise RuntimeError("snapshot corrupted 6x in a row (shifted bulk "
                               "read, SWD noise)")
        # A tick below the last one is only a *suspected* reboot: a single
        # corrupted read (SWD noise while the motor runs) looks identical.
        # The suspicion is not allowed to move last_tick, and the next sane
        # read clears it; check_reset() confirms before acting on it.
        t = out["tick"]
        if self.last_tick is None or t >= self.last_tick:
            self.last_tick = t
            self.rebooted = False
        else:
            self.rebooted = True
        return out

    @staticmethod
    def _consistent(out: dict[str, int]) -> bool:
        """erpm is recomputed from ertm every main-loop pass (src/main.c:744,
        60000000/ertm; 0 while ertm is 0 or the 100000000 'no sync' value),
        so a snapshot in which the two disagree was not read from one
        instant -- a shifted read whose values all happened to be in range
        (`ertm 0` next to `ERPM 787`)."""
        ertm, erpm = out["ertm"], out["erpm"]
        if out["sync"] == 6 and out["step"] and not 0 < ertm < 100000000:
            return False   # locked and commutating, yet no revolution time
        if ertm <= 0 or ertm >= 100000000:
            return erpm == 0
        return abs(erpm - 60000000 // ertm) <= max(erpm // 8, 100)

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

    def firmware_alive(self) -> bool:
        """True when the application is running: cfg.id in RAM carries the
        bootloader's magic and `tick` is advancing.  False while the core is
        halted, while the bootloader still has the chip, and while RAM holds
        OpenOCD's flash-loader residue after a `save`."""
        try:
            if self.ocd.read(CFG_BASE, 16) != CFG_ID:
                return False
            t0 = self.get("tick")
            time.sleep(0.02)
            return self.get("tick") != t0
        except RuntimeError:
            return False

    def wait_alive(self, timeout: float) -> bool:
        """Poll firmware_alive() for up to `timeout` seconds."""
        end = time.time() + timeout
        while True:
            if self.firmware_alive():
                return True
            if time.time() >= end:
                return False
            time.sleep(0.1)

    def read_image(self, addr: int, size: int, tries: int = 5) -> bytes:
        """A bulk read that is trusted only once it repeats.

        A bulk read_memory over these noisy SWD wires can come back *shifted*
        -- a word lost mid-block with the length intact -- and look perfectly
        sane, which is how a saved cfg page once ended up with every field
        from freq_max on moved down one word (duty_min 1 -> 30, duty_max
        100 -> 30, ...).  So read until two consecutive reads agree.
        """
        prev = None
        for _ in range(tries):
            cur = self.ocd.read_bytes(addr, size)[:size]
            if len(cur) == size and cur == prev:
                return cur
            prev = cur
            time.sleep(0.02)
        raise RuntimeError(f"bulk read of {size} bytes at {addr:#x} did not "
                           f"repeat in {tries} tries -- SWD noise; try again")

    def not_alive_hint(self) -> str:
        """Where the chip is when the application is not running."""
        parts = [f"core {self.ocd.state()}"]
        try:
            parts.append(f"RAM {CFG_BASE:#x} reads {self.ocd.read(CFG_BASE, 16):#06x} "
                         f"(the app puts {CFG_ID:#06x} there)")
            if self.cfg_page is not None:
                magic = self.ocd.read(self.cfg_page, 16)
                parts.append(f"flash page {self.cfg_page:#x} starts with {magic:#06x}")
                if magic != CFG_ID:
                    parts.append("<-- the bootloader will not jump to the app; "
                                 "`restore` rewrites the page")
        except RuntimeError as e:
            parts.append(f"(read failed: {str(e).splitlines()[0]})")
        return ", ".join(parts)

    def reboot(self, timeout: float = 3.0) -> bool:
        """Reset the ESC and wait for the application to come back up.

        The vector catch is lifted for the duration: with it armed, `reset run`
        parks the core at its reset vector, nothing runs until something
        resumes it, and the next command starts with a spurious "TARGET RESET
        ITSELF".  After the reset the bootloader sees SFTRSTF, answers the
        (absent) updater and sits out one 500 ms receive timeout before it
        jumps to the application (boot/src/main.c), so coming back takes
        about a second.
        """
        self.ocd.cmd("cortex_m vector_catch none")
        try:
            self.ocd.cmd("reset run")
            ok = self.wait_alive(timeout)
        finally:
            self.ocd.cmd("cortex_m vector_catch reset")
        self.last_tick = None
        self.rebooted = False
        if ok:
            self.arm()
        return ok

    def reset_flags(self) -> list[str]:
        csr = self.ocd.read(RCC_CSR)
        return [n for b, n in RESET_FLAGS if csr >> b & 1]

    def reset_cause(self) -> str:
        """Why the core is sitting halted.

        RCC_CSR names a real reset (the vector catch parks the core before
        main() can clear the flags).  No flag means no reset: the core
        crashed or the debugger halted it, and then the PC, the fault status
        registers and OpenOCD's own "target halted due to ..." line say which.
        """
        csr = self.ocd.read(RCC_CSR)
        names = [n for b, n in RESET_FLAGS if csr >> b & 1]
        out = [f"RCC_CSR={csr:#010x} ({', '.join(names) or 'no flags set'})"]
        try:
            out.append("pc " + self.ocd.cmd("reg pc").split()[-1])
            hfsr, cfsr = self.ocd.read(SCB_HFSR), self.ocd.read(SCB_CFSR)
            if hfsr or cfsr:
                out.append(f"HFSR {hfsr:#010x}{bits(hfsr, HFSR_BITS)} "
                           f"CFSR {cfsr:#010x}{bits(cfsr, CFSR_BITS)}")
                if cfsr & (1 << 15):
                    out.append(f"BFAR {self.ocd.read(SCB_BFAR):#010x}")
            if self.ocd.read(DCB_DHCSR) & DHCSR_S_LOCKUP:
                out.append("S_LOCKUP (double fault)")
        except (RuntimeError, IndexError):
            pass
        said = self.ocd.log_grep(("halted due to", "lockup", "reset detected",
                                  "Force reconnect", "Reconnecting", "Polling"))
        if said:
            out.append("openocd: " + " | ".join(said[-2:]))
        return "; ".join(out)

    def check_reset(self, since: float | None = None) -> bool:
        """Notice either kind of self-reset and get the firmware going again.

        A system reset (NRST, watchdog, SYSRESETREQ) trips the vector catch
        and leaves the core halted at the vector with RCC_CSR intact.  A
        power-class reset (BOR/POR) resets the debug logic too, so the
        firmware simply reboots; the only trace is `tick` starting over.
        Either way the ESC ends up in its arming loop, and only a zero
        throttle gets it out.  Returns True if a reset was found.
        """
        when = "" if since is None else f" {time.time() - since:.3f}s after throttle-on"
        if self.ocd.state() == "halted":
            if self.forced_reset:
                self.forced_reset = False
                print("  core halted by swdcli's own `reset halt` fallback; rebooting the ESC")
                ok = self.reboot()
                if not ok:
                    print("  firmware did not come back: " + self.not_alive_hint())
                    self.rebooted = False
                    self.last_tick = None
                return True
            cause = self.reset_cause()
            if self.reset_flags():
                print(f"  TARGET RESET ITSELF{when}: {cause}")
                self.ocd.cmd("resume")   # boots from reset; .bss init zeroes throt
                # The bootloader runs first and, after a software reset, sits
                # out a 500 ms receive timeout before jumping to the app; RAM
                # read before then is its stack, not the cfg.
                ok = self.wait_alive(3.0)
            else:
                # No reset happened: the core crashed (fault handler) or the
                # debugger halted it.  Resuming a crashed core just parks it
                # in the handler again, so reboot instead.
                print(f"  CORE HALTED WITHOUT A RESET{when}: {cause}")
                print("  rebooting the ESC")
                ok = self.reboot()
            if not ok:
                print("  firmware did not come back: " + self.not_alive_hint())
                self.rebooted = False
                self.last_tick = None
                return True
        else:
            try:
                self.snapshot()      # refreshes self.rebooted
            except RuntimeError:
                return False         # unreadable, not evidence of a reset
            if not self.rebooted:
                return False
            # Confirm before acting.  Acting on a false reboot re-arms with
            # throttle 0 -- the motor stops for no visible reason.  A real
            # reboot is unmistakable on a second look 30 ms later: tick is
            # still below the old value, and the firmware is back in its
            # arming loop (throt primed to 1) or at the very least not
            # commutating.  A glitch reads sane again, or shows step != 0.
            time.sleep(0.03)
            s2 = self.snapshot()
            if not self.rebooted or not (s2["throt"] == 1 or s2["step"] == 0):
                self.rebooted = False
                return False
            print(f"\n  FIRMWARE REBOOTED{when}: power-class reset (BOR/POR) -- "
                  "the debug port was\n  lost with it, so the cause flags "
                  "were already cleared by boot.")
        self.rebooted = False
        self.last_tick = None
        self.arm()
        print("  re-armed; throttle is now 0 -- send it again")
        return True

    def safe_stop(self) -> None:
        try:
            self.set_throt(0)
            return
        except Exception:
            pass
        # The write failed: the debug port dropped for a moment, either from
        # noise on the SWD wires or because the target itself browned out.
        # Look before resetting anything.  `tick` tells the two apart: it
        # restarts from 0 on a reboot and only ever climbs otherwise.  RCC_CSR
        # is shown too, but with PA2 floating on the bench the bootloader
        # exits at once on UART noise and the app clears the flags within
        # milliseconds, so "no flags" alone proves nothing.
        for _ in range(3):
            time.sleep(0.1)
            try:
                flags = self.reset_flags()
                tick = self.get("tick")
                rebooted = self.last_tick is not None and tick < self.last_tick
                self.set_throt(0)
                self.last_tick = tick
                self.rebooted = False
                print("  throttle 0 applied once the link came back: "
                      + ("TICK RESTARTED -- the ESC rebooted (power-class reset: "
                         "3V3 dipped or VIN collapsed)" if rebooted else
                         "tick kept counting -- the ESC did not reset, only the SWD link dropped")
                      + f"; RCC_CSR flags: {', '.join(flags) or 'none'}")
                return
            except Exception:
                continue
        # Last resort: a reset clears MOE, so the bridge goes open and the
        # motor coasts rather than staying driven.
        try:
            self.ocd.cmd("reset halt")
            self.forced_reset = True
            print("  target unreachable for 0.3 s -- forced `reset halt` so the "
                  "motor coasts (SFT/PIN flags below are from this)")
        except Exception:
            pass


def pwm_str(esc: Escape32) -> str:
    """Actual PWM duty on the bridge, read from TIM1 itself.

    `throt 2000` is 100 % of the throttle range, and with duty_max=100 that
    should land here as ~100 % duty.  This is the number that says whether it
    really does, or whether duty_max / duty_ramp / the spin-up governor is
    holding it back.  Only meaningful while the motor is running (step != 0);
    at idle CCR1 is stale.
    """
    arr = esc.ocd.read(TIM1_ARR)
    ccr = esc.ocd.read(TIM1_CCR1)
    return f"  pwm {ccr * 100 / (arr + 1):5.1f}%" if arr else ""


def do_cfg(esc: Escape32, args: list[str],
           lsyms: dict[str, int] | None = None) -> None:
    """Read or write the runtime cfg fields listed in CFG_FIELDS.

    Refuses to show or edit RAM while the application is not running: in
    that state 0x20000000 holds bootloader stack or flash-loader residue, and
    the numbers merely look like settings (that was the garbage after a
    failed `save`).  With no arguments it falls back to the values in the
    flash page, which is what the ESC will load once it does boot.
    """
    if not esc.firmware_alive():
        print("  firmware is not running -- RAM holds no cfg ("
              + esc.not_alive_hint() + ")")
        if lsyms is not None and not args:
            page = lsyms["_cfg"]
            print(f"  values saved in the flash page {page:#x}:")
            for n, (a, lo, hi) in CFG_FIELDS.items():
                v = esc.ocd.read(page + (a - CFG_BASE), 8)
                print(f"  {n:11s} {v:3d}   [{lo}..{hi}]")
        print("  `reset` to start it; if it does not come back, `restore`")
        return
    if not args:
        for n, (a, lo, hi) in CFG_FIELDS.items():
            print(f"  {n:11s} {esc.ocd.read(a, 8):3d}   [{lo}..{hi}]")
        return
    name = args[0]
    if name not in CFG_FIELDS:
        print("  unknown field; known: " + ", ".join(CFG_FIELDS))
        return
    a, lo, hi = CFG_FIELDS[name]
    if len(args) == 1:
        print(f"  {name} = {esc.ocd.read(a, 8)}")
        return
    v = int(args[1])
    if not lo <= v <= hi or (name == "sine_range" and 0 < v < 5):
        print(f"  {name}: allowed {lo}..{hi}" + (" (or 0)" if name == "sine_range" else ""))
        return
    esc.ocd.write(a, v, 8)
    print(f"  {name} = {esc.ocd.read(a, 8)}   (RAM only -- reverts on reset)")


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
    glitches = 0
    try:
        esc.set_throt(throt)
        n = misses = 0
        while time.time() - t0 < secs:
            try:
                samples.append((time.time() - t0, esc.snapshot()))
                misses = 0
            except RuntimeError:
                # A corrupted read is dropped, not recorded; only a run of
                # them means the link (or the target) is really gone.
                glitches += 1
                misses += 1
                if misses >= 10:
                    raise
                continue
            n += 1
            # After a self-reset the core sits halted (vector catch) and reads
            # keep succeeding against frozen RAM, so the halt must be polled
            # for; every 50th sample keeps that to a few RPCs a second.
            if n % 50 == 0 and esc.check_reset(t0):
                break
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        # Reads fail for a few ms while the reset is in progress; by the time
        # we get here the core is usually already halted at the vector.
        print(f"  ABORTED at {time.time() - t0:.3f}s after "
              f"{len(samples)} samples: {e}")
        time.sleep(0.1)
        try:
            esc.check_reset(t0)
        except RuntimeError as e2:
            print(f"  (and still unreachable: {e2})")
    finally:
        esc.safe_stop()

    if not samples:
        print("no samples")
        return

    print(f"  {len(samples)} samples in {samples[-1][0]:.2f}s "
          f"({len(samples) / max(samples[-1][0], 1e-6):.0f}/s)"
          + (f", {glitches} corrupted reads dropped" if glitches else ""))
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
    if not any(s["step"] for _, s in samples):
        # step stays 0 until main() actually starts the motor, so without
        # this the "never synced" verdict below would fire on every idle run.
        print("  step stayed 0: the motor never started, so there is nothing "
              "to conclude about\n  sync here.  Re-run with a non-zero "
              "throttle.")
    elif max(syncs) < 6:
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


def do_bemf(esc: Escape32, secs: float = 5.0) -> None:
    """Check the BEMF front end with the motor unpowered, by hand.

    While the motor is stopped the firmware leaves both comparators cleared
    (compctl(0)), so we can borrow them: configure one exactly the way
    compctl() would for a given step, then watch COMPx_CSR VALUE while the
    shaft is turned by hand.  Spinning a motor by hand generates real BEMF,
    so a phase that is wired and divided correctly makes the output toggle
    several times per revolution.  A phase that never toggles is not reaching
    the comparator -- or its neutral reference is not.

    The bridge has to be opened first.  While the motor is stopped ESCape32
    leaves all three phases clamped to ground (OCxM force-low with CCxNE set
    and MOE still on), which both shorts the windings -- so turning the shaft
    produces almost no BEMF -- and parks every comparator input right on its
    threshold, where it chatters.  Measured on this board that was 403 false
    transitions per second with the shaft held still.  So MOE is cleared for
    the duration and restored afterwards.

    Each phase is sampled twice: once with the shaft still, to establish a
    baseline, and once while it is turned.  Only the comparison between the
    two means anything.

    The metric is the longest run of identical samples, not the number of
    transitions.  With the bridge open and the shaft still, a floating phase
    sits on the comparator threshold and free-runs on noise -- measured on
    this board at roughly 370 transitions a second on every phase, wired or
    not -- so edge counting cannot tell a connected phase from a dead one.
    Real BEMF is slow by comparison: at a few revolutions a second the output
    holds one level for hundreds of consecutive samples.  That is what
    separates them.

    This drives nothing, so it is safe to run with the motor supply on.
    """
    st = esc.snapshot()
    if st["step"] or st["throt"]:
        print("motor is still running -- 'stop' first")
        return

    def profile(reg: int, secs: float) -> tuple[int, float, int]:
        """Return (samples, fraction high, longest run of one level)."""
        end = time.time() + secs
        runs, prev, run, high = [], None, 0, 0
        while time.time() < end:
            v = bool(esc.ocd.read(reg) & COMP_VALUE_BIT)
            high += v
            if v == prev:
                run += 1
            else:
                if prev is not None:
                    runs.append(run)
                run, prev = 1, v
        runs.append(run)
        n = sum(runs)
        return n, high / max(n, 1), max(runs)

    bdtr = esc.ocd.read(TIM1_BDTR)
    results = []
    try:
        esc.ocd.write(TIM1_BDTR, bdtr & ~TIM_BDTR_MOE)  # float all phases
        print(f"bridge opened (BDTR {bdtr:#x} -> {bdtr & ~TIM_BDTR_MOE:#x})")
        for name, reg, cfg in COMP_CFG:
            esc.ocd.write(COMP1_CSR, cfg if reg == COMP1_CSR else 0)
            esc.ocd.write(COMP2_CSR, cfg if reg == COMP2_CSR else 0)
            print(f"\n  {name}")
            print("    hold the shaft still ...", end="", flush=True)
            time.sleep(0.5)
            n1, hi1, idle = profile(reg, 1.0)
            print(f" baseline: longest run {idle} of {n1} samples, "
                  f"{hi1 * 100:.0f}% high")
            try:
                input(f"    now turn the shaft by hand for {secs}s, "
                      "then press Enter to start ...")
            except EOFError:
                print("    (no terminal -- skipping the spin half)")
                results.append((name, idle, None))
                continue
            n2, hi2, spun = profile(reg, secs)
            print(f"    turning:  longest run {spun} of {n2} samples, "
                  f"{hi2 * 100:.0f}% high")
            results.append((name, idle, spun))
    except KeyboardInterrupt:
        print()
    finally:
        esc.ocd.write(COMP1_CSR, 0)  # as compctl(0) leaves them
        esc.ocd.write(COMP2_CSR, 0)
        esc.ocd.write(TIM1_BDTR, bdtr)
        print(f"\nbridge restored (BDTR {bdtr:#x})")

    scored = [(n, i, s) for n, i, s in results if s is not None]
    if not scored:
        return
    print()
    # A wired phase holds a level for hundreds of samples once BEMF appears;
    # noise chatter never manages more than a handful.
    dead = [n for n, i, s in scored if s < max(3 * i, 20)]
    if not dead:
        print("  every phase holds a level far longer while turning than at "
              "rest: the BEMF\n  dividers and both neutral pins are wired.")
    elif len(dead) == len(scored):
        # Hand speed on a high-Kv motor gives tens of millivolts of BEMF
        # before the divider, which is below the comparator's own offset --
        # so an all-dead result is an inconclusive test, not a broken board.
        # Note that all three phases sharing one signature (same %high, same
        # run lengths) is itself informative: a genuinely floating neutral
        # pin would rail its comparator rather than chatter like the others.
        print("  no phase responded to turning.  With a small high-Kv motor "
              "that is the\n  expected outcome -- hand speed produces less "
              "BEMF than the comparator can\n  resolve through the divider -- "
              "so this run proves nothing either way.\n  Use `trace` with "
              "the motor actually driven; BEMF there is 10-100x larger.")
    else:
        print("  no response on: " + ", ".join(dead))
        if any("PA3" in n for n in dead) and not any("PA1" in n for n in dead):
            print("  Only the PA3-referenced phase is dead, which points "
                  "straight at the virtual\n  neutral: PA3 needs the same "
                  "star network as PA1, not just PA1 alone.")
        else:
            print("  Check those phases' dividers and the star network "
                  "feeding PA1/PA3.")


def do_hsls(esc: Escape32) -> None:
    """Find out on real hardware which driver input pin is the high side.

    The schematic wires TIM1_CH1 (net HSA) to driver pin 6 and TIM1_CH1N
    (net LSA) to pin 3, drawn with an MP6540H symbol whose pin names are
    PWMA/ENA.  The part fitted is the MP6540HA, on which pins 3-8 are the
    separate HS/LS inputs instead -- so whether pin 6 is the high side or
    the low side decides whether every commutation step drives the motor
    with the intended polarity or the opposite one.  Reversed polarity still
    spins the motor, but against the zero-crossing pattern the ESC expects,
    so sync never holds and the shaft direction wanders.

    Rather than trust any pin table, drive one gate input at a time and read
    the phase voltage back through the BEMF divider on PA0 with ADC1_IN1.
    With a single MOSFET on there is no current path -- the phase either
    rises to VIN (high side) or sits at ground (low side) -- so this is safe
    with the motor supply on.  The CPU is halted for the duration so the
    firmware cannot touch TIM1 or the ADC mid-test, and everything is put
    back before it resumes.
    """
    st = esc.snapshot()
    if st["step"] or st["throt"]:
        print("motor is still running -- 'stop' first")
        return
    ocd = esc.ocd
    ocd.cmd("halt")
    saved = {r: ocd.read(r) for r in
             (TIM1_CCMR1, TIM1_CCMR2, TIM1_CCER,
              ADC1_CFGR, ADC1_SQR1, ADC2_CFGR, ADC2_SQR1)}
    started = {b: ocd.read(b + ADC_CR) & ADC_CR_ADSTART for b in (ADC1, ADC2)}

    def outputs(ccmr1: int, ccer: int) -> None:
        ocd.write(TIM1_CCMR1, ccmr1)
        ocd.write(TIM1_CCMR2, 0x40)          # OC3M force-low
        ocd.write(TIM1_CCER, ccer)
        ocd.write(TIM1_EGR, TIM_EGR_COMG)
        time.sleep(0.05)

    def adc(base: int, chan: int) -> int:
        """One software-triggered conversion of `chan` on the ADC at `base`."""
        # ADSTART stays set while the ADC waits on TIM1_TRGO, and SQR/CFGR
        # may only be changed once it is clear.
        if ocd.read(base + ADC_CR) & ADC_CR_ADSTART:
            ocd.write(base + ADC_CR, ADC_CR_ADVREGEN | ADC_CR_ADSTP)
            for _ in range(100):
                if not ocd.read(base + ADC_CR) & ADC_CR_ADSTART:
                    break
        ocd.write(base + ADC_CFGR, 0)        # software trigger, no DMA
        ocd.write(base + ADC_SQR1, chan << 6)
        ocd.write(base + ADC_ISR, 0x1E)      # clear EOSMP/EOC/EOS/OVR
        ocd.write(base + ADC_CR, ADC_CR_ADVREGEN | ADC_CR_ADSTART)
        for _ in range(100):
            if ocd.read(base + ADC_ISR) & ADC_ISR_EOC:
                break
        return ocd.read(base + ADC_DR) & 0xFFF

    def bemf_pins() -> dict[str, int]:
        return {name: adc(base, chan) for name, base, chan in BEMF_ADC}

    try:
        print("CPU halted; driving one gate input at a time, reading the BEMF pins")
        # OC1M force-high = 0x50, force-low = 0x40; OC2M force-low = 0x4000.
        # CCER 0x444 = CC1NE|CC2NE|CC3NE (the firmware's idle set),
        #      0x441 = CC1E |CC2NE|CC3NE.
        outputs(0x4040, 0x444)
        base = bemf_pins()                   # everything low
        outputs(0x4050, 0x441)
        pin6 = bemf_pins()                   # CH1 high; CH1N driven low
        outputs(0x4040, 0x444)
        outputs(0x4050, 0x444)
        pin3 = bemf_pins()                   # CH1N = OC1REF = high; CH1 low
        outputs(0x4040, 0x444)
    finally:
        for r, v in saved.items():
            ocd.write(r, v)
        ocd.write(TIM1_EGR, TIM_EGR_COMG)
        for b, was in started.items():       # back onto TIM1_TRGO
            if was:
                ocd.write(b + ADC_CR, ADC_CR_ADVREGEN | ADC_CR_ADSTART)
        ocd.cmd("resume")
        print("registers restored, CPU resumed")

    def mv(c: int) -> int:
        return c * 3300 // 4095

    names = [n for n, _, _ in BEMF_ADC]
    print("                       " + "".join(f"{n:>12}" for n in names))
    for label, row in (("all inputs low", base), ("pin 6 (CH1)  high", pin6),
                       ("pin 3 (CH1N) high", pin3)):
        print(f"  {label:20s} " + "".join(f"{mv(row[n]):8d} mV" for n in names))
    # With one high side on and nothing else conducting, the other two
    # phases float up to the same rail through the windings, so all three
    # dividers and both neutral pins should read alike.  One that does not
    # is a divider or a trace, not the firmware.
    hi = pin6
    ref = max(hi.values())
    odd = [n for n in names if ref > 1000 and hi[n] < ref // 2]
    if odd:
        print(f"  with pin 6 high, these pins did not follow the rail: "
              f"{', '.join(odd)}  <-- check that divider / neutral wiring")
    pin6, pin3, base = pin6["PA0"], pin3["PA0"], base["PA0"]
    # 12V VIN through the 56k/10k divider lands near 1.8V, about 2250 counts.
    high = 1000
    print()
    if pin6 > high and pin3 < high:
        print("  pin 6 is the HIGH side and pin 3 the LOW side: the wiring "
              "matches the build\n  (CH1 -> HS, CH1N -> LS).  Polarity is not "
              "the problem; look at zero-crossing.")
    elif pin3 > high and pin6 < high:
        print("  pin 3 is the HIGH side and pin 6 the LOW side: HS and LS are "
              "SWAPPED relative\n  to the build.  Every step drives the "
              "windings with reversed polarity.")
    elif pin6 < high and pin3 < high:
        print("  neither input raised the phase.  Is the motor supply on?  "
              "(If TIM1 was frozen\n  by the debugger, the forced outputs "
              "may not have applied -- retry once.)")
    else:
        print("  both inputs raised the phase -- unexpected; only one input "
              "was driven at a time.")


def do_power(esc: Escape32, throt: int, secs: float) -> None:
    """Spin as `trace` does, but watch the supply rails while doing it.

    Two runs have already ended with the debug port itself dropping off SWD
    and the reset vector catch not holding -- the signature of a power-class
    (BOR) reset, which is the one kind the catch cannot survive.  This
    measures instead of infers: VDDA through VREFINT on ADC1, and VIN
    through the phase-B divider on ADC2, both free-running in continuous
    mode and read from their DR next to every state snapshot.

    Borrowing ADC1 is safe here.  adctrig() gives up whenever DMA channel 4
    is still enabled, so leaving that armed keeps the firmware's hands off
    the ADC for the duration, and nothing control-critical consumes its
    samples on this build (no SENS_MAP, PROT_TEMP=0).  ADC2 is idle on this
    build (len2 = 0).  Everything is put back with a clean reboot at the end
    rather than trusting a partial restore.
    """
    st = esc.snapshot()
    if st["step"] or st["throt"]:
        print("motor is still running -- 'stop' first")
        return
    ocd = esc.ocd
    vcal = ocd.read(VREFINT_CAL_ADDR, 16)
    # Take the ADCs while the firmware is stopped, so adctrig() cannot
    # restart ADC1 underneath the reconfiguration.
    ocd.cmd("halt")
    try:
        ocd.write(DMA1_CNDTR4, 1)
        ocd.write(DMA1_CCR4, DMA_CCR_EN)     # parks adctrig() for the duration
        for base, chan in ((ADC1, 18), (ADC2, 17)):
            if ocd.read(base + ADC_CR) & ADC_CR_ADSTART:
                ocd.write(base + ADC_CR, ADC_CR_ADVREGEN | ADC_CR_ADSTP)
                for _ in range(100):
                    if not ocd.read(base + ADC_CR) & ADC_CR_ADSTART:
                        break
            ocd.write(base + ADC_CFGR, ADC_CFGR_CONT_OVRMOD)
            ocd.write(base + ADC_SQR1, chan << 6)
            ocd.write(base + ADC_ISR, 0x1E)
            ocd.write(base + ADC_CR, ADC_CR_ADVREGEN | ADC_CR_ADSTART)
    finally:
        ocd.cmd("resume")
    time.sleep(0.05)

    def rails() -> tuple[float, float]:
        vref = ocd.read(ADC1 + ADC_DR) & 0xFFF
        pb = ocd.read(ADC2 + ADC_DR) & 0xFFF
        vdda = 3.0 * vcal / vref if vref else 0.0
        return vdda, pb * vdda / 4095 * 6.6   # 56k/10k divider on the phase

    idle = [rails() for _ in range(20)]
    print(f"idle : VDDA {min(v for v, _ in idle):.3f}..{max(v for v, _ in idle):.3f} V"
          f"   VIN(phase B, floating) {max(x for _, x in idle):.2f} V")
    print(f"power: throt {throt} for {secs}s  (Ctrl-C cuts throttle)")
    rows: list[tuple[float, float, float, int, int]] = []
    t0 = time.time()
    note = None
    try:
        esc.set_throt(throt)
        n = 0
        while time.time() - t0 < secs:
            s = esc.snapshot()
            v, x = rails()
            rows.append((time.time() - t0, v, x, s["step"], s["sync"]))
            n += 1
            if n % 50 == 0 and ocd.state() == "halted":
                note = (f"core halted at reset vector {time.time() - t0:.3f}s "
                        f"after throttle-on: {esc.reset_cause()}")
                break
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        note = f"target dropped off SWD {time.time() - t0:.3f}s after throttle-on: {e}"
    finally:
        try:
            esc.set_throt(0)
        except Exception:
            pass
        # A reboot puts ADC1/ADC2/DMA back exactly as init() wants them; the
        # vector catch then holds the core at the vector, hence the resume.
        try:
            if esc.reboot():
                print("firmware rebooted and re-armed")
            else:
                print("firmware rebooted but did not come back: "
                      + esc.not_alive_hint())
        except RuntimeError as e:
            print(f"could not reboot/re-arm: {e}")

    if note:
        print("  " + note)
    if not rows:
        print("no samples")
        return
    print(f"  {len(rows)} samples in {rows[-1][0]:.2f}s")
    print("   t(ms)   VDDA min / mean     VIN min / max    steps  sync")
    bins: dict[int, list] = {}
    for r in rows:
        bins.setdefault(int(r[0] * 10), []).append(r)
    for b in sorted(bins):
        rs = bins[b]
        steps = sum(1 for a, c in zip(rs, rs[1:]) if c[3] != a[3])
        print(f"  {b * 100:5d}    {min(r[1] for r in rs):.3f} / {sum(r[1] for r in rs) / len(rs):.3f} V"
              f"    {min(r[2] for r in rs):5.2f} / {max(r[2] for r in rs):5.2f} V"
              f"    {steps:3d}   {max(r[4] for r in rs)}")
    print("  last samples before the end:")
    for t, v, x, stp, sy in rows[-8:]:
        print(f"    {t * 1000:7.1f} ms  VDDA {v:.3f} V  VIN {x:5.2f} V  step {stp}  sync {sy}")
    lo = min(r[1] for r in rows)
    print(f"  VDDA floor during run: {lo:.3f} V"
          + ("   <-- sagging" if lo < 3.0 else ""))


def do_zc(esc: Escape32, throt: int, secs: float) -> None:
    """Spin, and per commutation step check whether the ACTIVE comparator's
    output actually crosses.

    The trace shows sync climbing then collapsing at one fixed step, every
    revolution -- the signature of a single BEMF phase whose zero-crossing is
    never seen, so that step always waits out the full timeout.  This finds
    which one, without guessing: read the live COMPx_CSR while spinning.  A
    healthy step's active comparator toggles its VALUE bit (bit 30) within
    the step (a real crossing); the failing step's stays stuck at one level.

    compctl() drives only one comparator at a time (the other CSR is 0), so
    "active" = whichever CSR has its EN bit set.  Its config value also tells
    us the polarity bit (0x8000), which is the thing that differs between the
    working and failing use of the same comparator.
    """
    st = esc.snapshot()
    if st["step"] or st["throt"]:
        print("motor is still running -- 'stop' first")
        return
    ocd = esc.ocd
    step_addr = esc.syms["step"][0]
    # per step, keyed by the input-select signature of the comparator the
    # timer is actually capturing (TIM2_TISEL picks COMP1 vs COMP2; the CSR
    # config's low bits pick which pin within COMP1).  compctl() leaves the
    # other CSR enabled, so TISEL -- not CSR.EN -- is the authority.
    SIG = {0x071: "COMP1 PA0>PA1 (phaseA)",
           0x061: "COMP1 PA4>PA1 (phaseB)",
           0x161: "COMP2 PA5>PA3 (phaseC)"}
    acc: dict[int, dict] = {s: {} for s in range(1, 7)}
    t0 = time.time()
    rebooted = False
    try:
        esc.set_throt(throt)
        n = 0
        while time.time() - t0 < secs:
            step = ocd.read(step_addr)
            tisel = ocd.read(TIM2_TISEL) & 0xF
            csr = ocd.read(COMP1_CSR if tisel == 1 else COMP2_CSR)
            if step in acc and tisel in (1, 2):
                sig = csr & 0x1FF
                d = acc[step].setdefault(
                    sig, {"pol": csr & 0x8000, "lo": 0, "hi": 0, "n": 0})
                d["n"] += 1
                if csr & (1 << 30):
                    d["hi"] += 1
                else:
                    d["lo"] += 1
            n += 1
            if n % 60 == 0 and ocd.state() == "halted":
                rebooted = True
                break
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        rebooted = True
    finally:
        esc.safe_stop()
    if rebooted:
        esc.check_reset(t0)

    print(f"\n  per-step comparator activity at throt {throt}:")
    print("  step  comparator (TISEL-selected)   pol    output          verdict")
    for s in range(1, 7):
        if not acc[s]:
            print(f"   {s}    (never observed)")
            continue
        sig = max(acc[s], key=lambda k: acc[s][k]["n"])
        d = acc[s][sig]
        cname = SIG.get(sig, f"?sig={sig:#05x}")
        pol = "flip" if d["pol"] else "norm"
        toggled = d["lo"] and d["hi"]
        bal = f"{d['lo']}lo/{d['hi']}hi"
        verdict = "crosses (ok)" if toggled else (
            "STUCK HIGH -- no ZC" if d["hi"] else "STUCK LOW -- no ZC")
        print(f"   {s}    {cname:26s} {pol}   {bal:14s}  {verdict}")
    print("\n  A step whose active comparator is STUCK is the one that stalls "
          "commutation.\n  Compare it with the same comparator's other step: "
          "if only one polarity is\n  stuck, the crossing is real but lands "
          "outside the detectable window (offset\n  or blanking); if the "
          "comparator is stuck in both its steps, that phase's\n  divider or "
          "neutral reference is the suspect.")


CFG_SINE_RANGE = 0x20000019   # cfg.sine_range, 1 byte (cfg base 0x20000000)
CFG_SINE_POWER = 0x2000001A   # cfg.sine_power, 1 byte


def do_sine(esc: Escape32, throt: int, power: int, secs: float) -> None:
    """Run ESCape32's open-loop sine startup and hold it, without a rebuild.

    sine_range / sine_power are runtime cfg fields, so they can be set in RAM
    and take effect on the next throttle command.  Measured on this board,
    sine startup spins the motor up smoothly with no BEMF lock at all (it is
    open loop) -- useful both as a working "it just turns" mode and as the
    ramp that hands over to 6-step once the throttle exceeds sine_range*20.

    This keeps the throttle *below* that handover point (sine_range is set
    high enough that `throt` stays inside it), so it exercises sine alone.
    cfg is restored on exit; a reset would restore it anyway since these are
    only RAM copies of the saved config.
    """
    st = esc.snapshot()
    if st["step"] or st["throt"]:
        print("motor is still running -- 'stop' first")
        return
    ocd = esc.ocd
    # Keep throt strictly inside the sine window: range = sine_range*20 must
    # exceed throt by more than delta(=10), so sine never hands over.
    sine_range = min((throt + 40) // 20, 25)
    save = (ocd.read(CFG_SINE_RANGE, 8), ocd.read(CFG_SINE_POWER, 8))
    print(f"sine: throt {throt}, sine_power {power}, sine_range {sine_range} "
          f"(range {sine_range * 20} > throt, so pure sine)  Ctrl-C cuts throttle")
    rows = []
    t0 = time.time()
    reb = False
    try:
        ocd.write(CFG_SINE_RANGE, sine_range, 8)
        ocd.write(CFG_SINE_POWER, max(1, min(power, 15)), 8)
        esc.set_throt(throt)
        n = 0
        while time.time() - t0 < secs:
            s = esc.snapshot()
            rows.append((s["sine"], s["step"], s["ertm"]))
            n += 1
            if n % 50 == 0 and ocd.state() == "halted":
                reb = True
                break
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        reb = True
    finally:
        esc.safe_stop()
        ocd.write(CFG_SINE_RANGE, save[0], 8)
        ocd.write(CFG_SINE_POWER, save[1], 8)
        print(f"throttle cut, cfg restored (sine_range={save[0]}, "
              f"sine_power={save[1]})")
    if reb:
        esc.check_reset(t0)
    if rows:
        insine = sum(1 for r in rows if r[0]) * 100 // len(rows)
        erpms = [60000000 // r[2] for r in rows if 0 < r[2] < 100000000]
        print(f"  {len(rows)} samples | in sine mode {insine}% of the time"
              + (f" | ERPM {min(erpms)}..{max(erpms)}" if erpms else ""))


def do_knob(esc: Escape32, step: int, start: int = 250) -> None:
    """Drive the throttle from the keyboard, live.

      up / down      +step / -step        (also + / -)
      PgUp / PgDn    +4*step / -4*step
      right / left   double / halve the step
      space or 0     throttle 0
      q or Esc       throttle 0 and back to the prompt

    From rest the first nudge up jumps straight to `start` (default 250 =
    12.5 %) rather than to one step: ESCape32's sensorless start needs a
    decisive first kick, and creeping up from 2-3 % duty only makes the rotor
    cog in place until the duty is finally large enough -- by which time the
    rotor and the interval estimator are in a mess.  Coming down is left
    alone.

    The status line refreshes about five times a second between keys, so
    sync / ERPM / pwm can be watched while nudging.  Leaving always cuts the
    throttle: a prompt sitting in front of a spinning motor is how accidents
    happen.  A failed SWD write is reported on its own line and the knob
    keeps working; a reboot re-arms and drops the level back to 0.
    """
    if not sys.stdin.isatty():
        print("knob needs a terminal")
        return
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    level = esc.get("throt")
    if level < 0 or level == 1:   # 1 is the arming-loop primer, not a setpoint
        level = 0
    print(f"knob: step {step}, first press from 0 -> {start}   up/down  PgUp/PgDn x4  "
          "left/right = step /2 x2   space = 0   q = quit (cuts throttle)")
    tty.setcbreak(fd)   # no echo, no line buffering; Ctrl-C still works
    try:
        while True:
            try:
                st = esc.snapshot()
                line = fmt_status(st) + pwm_str(esc)
            except RuntimeError as e:
                line = f"(read failed: {str(e).splitlines()[0]})"
            sys.stdout.write(f"\r\x1b[K[step {step:4d}] {line}")
            sys.stdout.flush()
            if esc.rebooted and esc.check_reset():   # confirmed, not a glitch
                level = 0
                continue
            ready, _, _ = select.select([fd], [], [], 0.2)
            if not ready:
                continue
            key = os.read(fd, 1)
            delta = None
            if key == b"\x1b":
                ready, _, _ = select.select([fd], [], [], 0.05)
                seq = os.read(fd, 8) if ready else b""
                if seq == b"":            # lone Esc
                    break
                elif seq == b"[A":
                    delta = step
                elif seq == b"[B":
                    delta = -step
                elif seq == b"[5~":
                    delta = 4 * step
                elif seq == b"[6~":
                    delta = -4 * step
                elif seq == b"[C":
                    step = min(step * 2, 500)
                elif seq == b"[D":
                    step = max(step // 2, 1)
            elif key in (b"+", b"="):
                delta = step
            elif key in (b"-", b"_"):
                delta = -step
            elif key in (b" ", b"0"):
                level, delta = 0, 0
            elif key in (b"q", b"Q"):
                break
            if delta is None:
                continue
            if level == 0 and delta > 0:
                level = min(2000, max(start, delta))   # decisive first kick from rest
            else:
                level = max(0, min(2000, level + delta))
            try:
                esc.set_throt(level)
            except RuntimeError as e:
                sys.stdout.write(f"\n  write failed ({str(e).splitlines()[0]}) -- "
                                 "level not applied, press again\n")
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        esc.safe_stop()
        print("\nthrottle cut")


def write_cfg_page(esc: Escape32, page: int, data: bytes) -> bool:
    """Burn `data` into the cfg flash page and bring the application back.

    OpenOCD flashes with the core halted, and its flash loader runs in the
    work area at 0x20000000 -- on top of the live cfg and the firmware's
    state (stm32g4x.cfg, work-area-backup 0).  So: halt at reset, write, read
    the page back, then a clean `reset run` so the firmware rebuilds its RAM
    from the page it just got.  `program` is deliberately not used: it leaves
    the core halted at the vector, and a `resume` from there followed by an
    early read is what used to report garbage as a "MISMATCH".  Returns True
    once the application is running on the new page.
    """
    size = len(data)
    if int.from_bytes(data[:2], "little") != CFG_ID:
        print(f"  refusing to write: image does not start with {CFG_ID:#06x}, "
              "the bootloader would never jump to the app again")
        return False
    with tempfile.NamedTemporaryFile(prefix="escape32-cfg-", suffix=".bin",
                                     delete=False) as f:
        f.write(data)
        path = f.name
    ocd = esc.ocd
    ocd.cmd("cortex_m vector_catch none")
    try:
        ocd.cmd("reset halt")
        if ocd.state() != "halted":
            print(f"  could not halt the core for flashing (state {ocd.state()})")
            ocd.cmd("reset run")
            return False
        print(f"writing {size} bytes of cfg to {page:#x} ...")
        reply = ocd.cmd(f"flash write_image erase {path} {page:#x}")
        try:
            back = esc.read_image(page, size)
        except RuntimeError as e:
            print(f"  could not read the page back: {e}")
            back = None
        ok_page = back == data
        if ok_page:
            print(f"  flash page verified ({size} bytes match, read twice)")
        elif back is not None:
            diff = next(i for i in range(size) if back[i] != data[i])
            print(f"  FLASH MISMATCH at offset {diff} -- page not written as intended"
                  + (f"\n  openocd: {reply}" if reply else ""))
        ocd.cmd("reset run")
        alive = esc.wait_alive(3.0)
    finally:
        ocd.cmd("cortex_m vector_catch reset")
        os.unlink(path)
    esc.last_tick = None
    esc.rebooted = False
    if not alive:
        print("  firmware did not come back after the reset: " + esc.not_alive_hint())
        return False
    esc.arm()
    return ok_page


def cfg_defaults_from_elf(elf: str) -> bytes:
    """The ELF's .cfg section: the page flash.sh programs, i.e. the build-time
    defaults (CMakeLists.txt add_target ... TIMING=.. and friends)."""
    objcopy = shutil.which("arm-none-eabi-objcopy")
    if not objcopy:
        raise RuntimeError("arm-none-eabi-objcopy is not on PATH")
    with tempfile.NamedTemporaryFile(prefix="escape32-cfgdef-", suffix=".bin",
                                     delete=False) as f:
        path = f.name
    try:
        subprocess.run([objcopy, "-O", "binary", "--only-section=.cfg", elf, path],
                       check=True, capture_output=True, text=True)
        with open(path, "rb") as f:
            return f.read()
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"objcopy failed: {e.stderr.strip()}") from None
    finally:
        os.unlink(path)


def do_save(esc: Escape32, lsyms: dict[str, int]) -> None:
    """Persist the live cfg to the flash config page -- ESCape32's `save`.

    The firmware's own savecfg() copies _cfg_start.._cfg_end (the SRAM cfg,
    i.e. what `cfg name value` has been editing) over the _cfg flash page and
    reads it back at every boot (src/main.c:559).  This does the same over
    SWD, byte for byte, then reboots the ESC and checks that it reloaded the
    same values.  Refused while the motor runs, like the firmware refuses
    (`ertm`), and refused while the application is not running -- RAM then
    holds nothing worth saving (see write_cfg_page).
    """
    if not esc.firmware_alive():
        print("  firmware is not running -- RAM holds no cfg to save ("
              + esc.not_alive_hint() + ")")
        print("  `reset` first; if the ESC does not come back, `restore` "
              "rewrites the default page")
        return
    st = esc.snapshot()
    if st["step"] or st["throt"] or st["ertm"]:
        print("motor is running -- `stop` first (the ESC reboots during save)")
        return
    ram, ram_end, page = lsyms["_cfg_start"], lsyms["_cfg_end"], lsyms["_cfg"]
    size = ram_end - ram
    # The image is taken with a repeated bulk read and then checked field by
    # field against single-byte reads: a shifted bulk read is the one way a
    # page full of plausible-looking wrong values gets written (read_image).
    data = esc.read_image(ram, size)
    before = {n: esc.ocd.read(a, 8) for n, (a, _, _) in CFG_FIELDS.items()}
    bad = [f"{n} {data[a - CFG_BASE]}!={before[n]}"
           for n, (a, _, _) in CFG_FIELDS.items() if data[a - CFG_BASE] != before[n]]
    if bad:
        print("  RAM image disagrees with field reads (SWD noise), nothing written: "
              + ", ".join(bad) + "\n  run `save` again")
        return
    if not write_cfg_page(esc, page, data):
        return
    after = {n: esc.ocd.read(a, 8) for n, (a, _, _) in CFG_FIELDS.items()}
    bad = [n for n in before if before[n] != after[n]]
    if bad:
        print("  MISMATCH after reboot: "
              + ", ".join(f"{n} {before[n]}->{after[n]}" for n in bad))
    else:
        print("  saved; ESC rebooted and reloaded the same values:")
        for n in CFG_FIELDS:
            print(f"    {n:11s} {after[n]}")


def do_restore(esc: Escape32, lsyms: dict[str, int], elf: str) -> None:
    """Rewrite the cfg page with the build-time defaults from the ELF.

    The way out of a bootloader lock-up (a page without the 0x32ea magic),
    and a plain factory reset otherwise.  Everything `save`d so far is lost,
    including the spin direction set over DShot -- run `send_file.py dshotdir`
    again afterwards.
    """
    if esc.firmware_alive():
        st = esc.snapshot()
        if st["step"] or st["throt"] or st["ertm"]:
            print("motor is running -- `stop` first (the ESC reboots during restore)")
            return
    size = lsyms["_cfg_end"] - lsyms["_cfg_start"]
    data = cfg_defaults_from_elf(elf)
    if len(data) != size or int.from_bytes(data[:2], "little") != CFG_ID:
        print(f"  .cfg section of {os.path.relpath(elf, HERE)} looks wrong "
              f"({len(data)} bytes, expected {size}) -- wrong ELF?")
        return
    print(f"restoring the build's default cfg page from {os.path.relpath(elf, HERE)}")
    if write_cfg_page(esc, lsyms["_cfg"], data):
        print("  restored; the ESC is running on the default page:")
        do_cfg(esc, [], lsyms)


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
            try:
                line = fmt_status(esc.snapshot()) + pwm_str(esc)
            except RuntimeError as e:
                # One unreadable level is not worth aborting the ramp; the
                # throttle stays where it is and the next level reads again.
                line = f"(read glitch: {str(e).splitlines()[0]})"
            print(f"  {level * 100 / 2000:5.1f}% of range   {line}")
            try:
                if esc.check_reset():
                    break
            except RuntimeError:
                pass
    finally:
        esc.safe_stop()
        print("throttle cut")


def do_repl(esc: Escape32, lsyms: dict[str, int] | None = None,
            elf: str | None = None) -> None:
    print("commands:  throt <-2000..2000> | stop | status | watch [sec]")
    print("           trace <throt> [sec]        commutation/sync trace")
    print("           zc <throt> [sec]           per-step comparator ZC check")
    print("           sine <throt> [pow] [sec]   open-loop sine startup")
    print("           ramp <max> [step] [dwell]  step throttle up")
    print("           bemf [sec] | hsls | power <throt> [sec]")
    print("           knob [step] [start]   keyboard throttle: arrows / PgUp PgDn / space=0 / q")
    print("           cfg [field [value]]   runtime cfg (timing, duty_max, ...; RAM until saved)")
    print("           save                  write the live cfg to the flash config page (ESC reboots)")
    print("           restore               rewrite the page with the build's defaults (bootloader lock-up fix)")
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
            # A reset between commands would otherwise go unnoticed until a
            # write silently landed on a halted core.
            esc.check_reset()
            if verb in ("quit", "exit", "q"):
                return
            elif verb == "throt":
                esc.set_throt(args[0])
                # A reboot in the first moments after throttle-on is the
                # failure this board actually has; give it a chance to show.
                time.sleep(0.3)
                if not esc.check_reset():
                    print(fmt_status(esc.snapshot()) + pwm_str(esc))
            elif verb == "stop":
                esc.set_throt(0)
                print("throttle cut")
            elif verb == "status":
                print(fmt_status(esc.snapshot()) + pwm_str(esc))
            elif verb == "watch":
                secs = float(args[0]) if args else 5.0
                end = time.time() + secs
                try:
                    while time.time() < end:
                        try:
                            if esc.check_reset():
                                break
                            print(fmt_status(esc.snapshot()) + pwm_str(esc))
                        except RuntimeError as e:
                            print(f"(read glitch: {str(e).splitlines()[0]})")
                        time.sleep(0.2)
                except KeyboardInterrupt:
                    esc.safe_stop()
                    print("\nthrottle cut")
            elif verb == "trace":
                do_trace(esc, int(args[0]),
                         float(args[1]) if len(args) > 1 else 2.0)
            elif verb == "bemf":
                do_bemf(esc, float(args[0]) if args else 5.0)
            elif verb == "hsls":
                do_hsls(esc)
            elif verb == "zc":
                do_zc(esc, int(args[0]) if args else 200,
                      float(args[1]) if len(args) > 1 else 3.0)
            elif verb == "sine":
                do_sine(esc, int(args[0]) if args else 200,
                        int(args[1]) if len(args) > 1 else 10,
                        float(args[2]) if len(args) > 2 else 3.0)
            elif verb == "power":
                do_power(esc, int(args[0]),
                         float(args[1]) if len(args) > 1 else 1.5)
            elif verb in ("knob", "k"):
                do_knob(esc, int(args[0]) if args else 50,
                        int(args[1]) if len(args) > 1 else 250)
            elif verb == "ramp":
                do_ramp(esc, int(args[0]),
                        int(args[1]) if len(args) > 1 else 100,
                        float(args[2]) if len(args) > 2 else 1.0)
            elif verb == "cfg":
                do_cfg(esc, args, lsyms)
            elif verb == "save":
                if lsyms is None:
                    print("save unavailable (no ELF symbols)")
                else:
                    do_save(esc, lsyms)
            elif verb == "restore":
                if lsyms is None or elf is None:
                    print("restore unavailable (no ELF)")
                else:
                    do_restore(esc, lsyms, elf)
            elif verb == "reset":
                if esc.reboot():
                    print("reset; firmware is back up and re-armed")
                else:
                    print("reset, but the firmware did not come back: "
                          + esc.not_alive_hint())
            else:
                print(f"unknown command: {verb}")
        except KeyboardInterrupt:
            esc.safe_stop()
            print("\nthrottle cut")
        except (IndexError, ValueError) as e:
            print(f"bad arguments: {e}")
        except RuntimeError as e:
            print(f"error: {e}")
            # Tell the two cases apart for the user: a link glitch leaves the
            # firmware running (tick still advancing, command simply not
            # applied) while a brownout/reset shows up as a reboot.
            try:
                if not esc.check_reset():
                    print("  link glitch, target did not reboot -- the command was NOT "
                          "applied, send it again")
            except RuntimeError:
                print("  target unreachable -- power or reset event? check the bench "
                      "supply and the SWD/GND wiring")


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
    p.add_argument("--bemf", action="store_true",
                   help="motor-off BEMF check; turn the shaft by hand")
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
        lsyms = load_linker_symbols(elf)
    except (RuntimeError, subprocess.CalledProcessError) as e:
        return str(e)

    try:
        ocd = OpenOCD(args.cfg, args.openocd, args.scripts)
    except (RuntimeError, OSError) as e:
        return str(e)

    with ocd:
        esc = Escape32(ocd, syms)
        esc.cfg_page = lsyms["_cfg"]
        print(f"state block {esc.base:#x}..{esc.base + esc.span:#x} "
              f"({esc.span} bytes) from {os.path.relpath(elf, HERE)}")
        if ocd.state() == "halted":
            # Left over from a previous session's reset catch, or a
            # `reset halt` fallback: the firmware is not running at all.
            print(f"target was halted ({esc.reset_cause()}); resuming")
            ocd.cmd("resume")
            alive = esc.wait_alive(3.0)
        else:
            alive = esc.firmware_alive()
        if not alive:
            print("firmware is not running: " + esc.not_alive_hint())
        elif esc.get("throt") == 1:
            print("ESC is in the power-on arming loop; sending zero throttle")
        esc.arm()
        # bemf/power clear MOE to float the phases and restore it in a
        # finally -- but a killed process skips that finally and strands the
        # bridge disabled, so the firmware commutates with no output and the
        # motor stays silent.  Catch that leftover on the next connect.
        if alive and not ocd.read(TIM1_BDTR) & TIM_BDTR_MOE:
            print("TIM1 MOE is cleared (left over from a motor-off test); "
                  "rebooting to restore it")
            if not esc.reboot():
                print("firmware did not come back: " + esc.not_alive_hint())
        try:
            if args.bemf:
                do_bemf(esc, args.secs if args.secs != 2.0 else 5.0)
            elif args.trace is not None:
                do_trace(esc, args.trace, args.secs)
            elif args.ramp is not None:
                if not 0 < args.ramp <= 2000:
                    return "--ramp must be within 1..2000"
                do_ramp(esc, args.ramp, args.step, args.dwell)
            else:
                do_repl(esc, lsyms, elf)
        finally:
            esc.safe_stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
