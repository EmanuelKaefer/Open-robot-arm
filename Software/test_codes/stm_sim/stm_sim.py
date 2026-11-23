#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
# See LICENSE-SOFTWARE in the project root for details.
"""
stm_sim.py - simulated STM32 driver board for testing without hardware (Linux only).

Creates a virtual serial port (pty) and speaks the binary protocol from
doc/Software/UART-Protokoll.md. Rules follow the firmware
(Core/Src/uartconfig.c, motion.c, homing.c). Frame encoding, CRC and constants
come from ../Home/robot_proto.py.

Usage:
    python3 stm_sim.py -v                      # port: /tmp/ttySTM
    python3 stm_sim.py --no-drv                # VM "off": DRV_OK never set
    python3 stm_sim.py --homing-time 5         # homing finishes after 5 s
    python3 stm_sim.py --homing-never          # like the current firmware (placeholder)

Clients open /tmp/ttySTM like a normal serial port (the baudrate is ignored):
    Link("/tmp/ttySTM")                         # robot_proto.py
    <param name="port">/tmp/ttySTM</param>      # arm_hardware/ArmSystem

Simplifications compared to the real STM:
- Position follows the interpolated setpoint directly (limited to vmax),
  no P-controller, no acceleration limit while RUNNING.
- STOPPING brakes every axis with its own accel (real STM: synchronised).
- Homing is a mock: after --homing-time all positions jump to 0 and all axes are "homed".
- No stall, no driver faults, STATE is never skipped.
"""
import argparse
import math
import os
import select
import signal
import struct
import sys
import time
import tty

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Home"))
from robot_proto import (  # noqa: E402
    crc16, frame, flag_names, home_names, STATE_NAMES, N_AX, FIFO_N,
    T_SETPOINT, T_CONFIG, T_ENABLE, T_CLEARFAULT, T_SETZERO, T_HOMING, T_STATE, T_DIAG,
    DISABLED, IDLE, RUNNING, STOPPING, FAULT,
    FL_UNDERRUN, FL_OVERRUN, FL_CFG_OK, FL_JUMP, FL_DRV_OK, FL_STARVED,
    HS_HOMED_MASK, HS_ACTIVE, HS_ERROR,
)

# ---------------- firmware constants (motion.c, uartconfig.c) ----------------
TICK = 0.001            # 1 kHz motion tick
PREFILL = 3             # setpoints before motion starts
FILL_TARGET = 4         # desired FIFO level
FILL_BAND = 2           # tempo is adapted only from a deviation >= 2
EXTRAP_MAX = 50         # ticks (ms) of extrapolation, then STARVED
JUMP_FACT = 2.5         # max jump = 2.5 * vmax * T + 2
MAX_PAYLOAD = 64
SOF = 0xA5
RX_TIMEOUT = 0.050      # > 50 ms between bytes -> partial frame dropped
STATE_PERIOD = 10       # ms
DIAG_PERIOD = 100       # ms

BAD_FRAME = 1           # dispatch() return value: counted as "bad"


class SimSTM:
    def __init__(self, args):
        self.args = args
        self.verbose = args.verbose

        # motion controller
        self.state = DISABLED
        self.after_stop = DISABLED
        self.cfg_ok = False
        self.drv_ok = False
        self.flags = 0                          # UNDERRUN / OVERRUN / JUMP / STARVED
        self.n_ticks = 10
        self.n_cur = 10
        self.vmax = [1.0] * N_AX
        self.accel = [1.0] * N_AX
        self.max_jump = [0.0] * N_AX
        self.pos = [0.0] * N_AX                 # output position [steps], float
        self.vel = [0.0] * N_AX                 # steps per tick
        self.fifo = []
        self.last_pushed = [0] * N_AX
        self.seg_s = [0] * N_AX
        self.seg_e = [0] * N_AX
        self.last_span = [0] * N_AX
        self.still = True
        self.tick = 0
        self.owed = 0
        self.ex_ticks = 0

        # homing
        self.home = 0
        self.home_t0 = 0.0

        # diagnostics
        self.diag = dict(ok=0, crc=0, len=0, tout=0, full=0, uerr=0, bad=0, rej=0)

        # receiver state machine (like Proto_Poll)
        self.ps = "SOF"
        self.fb = bytearray()
        self.need = 0
        self.t_last_rx = 0.0

        # transmitter
        self.tx_seq = 0
        self.ms = 0
        self.t_start = time.perf_counter()
        self.n_setpoints = 0

        self._last_logged = None

    # ------------------------------------------------------------------ logging
    def log(self, msg, level=1):
        if self.verbose >= level:
            t = time.perf_counter() - self.t_start
            print(f"[{t:8.3f}] {msg}", flush=True)

    def flags_out(self):
        return self.flags | (FL_CFG_OK if self.cfg_ok else 0) | (FL_DRV_OK if self.drv_ok else 0)

    def pos_out(self):
        return [int(round(p)) for p in self.pos]

    def log_changes(self):
        cur = (self.state, self.flags_out(), self.home)
        if cur != self._last_logged:
            self.log(f"state={STATE_NAMES[self.state]} flags={flag_names(cur[1])} "
                     f"home={home_names(self.home)} pos={self.pos_out()} free={FIFO_N - len(self.fifo)}")
            self._last_logged = cur

    # ------------------------------------------------------------------ commands (motion.c)
    def mc_config(self, period, v, a):
        if self.state != DISABLED:
            return -1
        if period < 2 or period > 100:
            return -2
        if any(not (x > 0.0) for x in v + a):  # also catches NaN
            return -3
        self.n_ticks = period
        self.vmax = list(v)
        self.accel = list(a)
        self.max_jump = [x * (period / 1000.0) * JUMP_FACT + 2.0 for x in v]
        self.cfg_ok = True
        return 0

    def mc_enable(self):
        if not self.cfg_ok or not self.drv_ok or self.state != DISABLED:
            return -1
        p = self.pos_out()
        self.fifo.clear()
        self.seg_s, self.seg_e, self.last_pushed = list(p), list(p), list(p)
        self.last_span = [0] * N_AX
        self.vel = [0.0] * N_AX
        self.still = True
        self.flags = 0
        self.tick = 0
        self.n_cur = self.n_ticks
        self.owed = 0
        self.ex_ticks = 0
        self.state = IDLE
        return 0

    def mc_disable(self):
        if self.state in (IDLE, RUNNING):
            self.after_stop = DISABLED
            self.state = STOPPING

    def fault_stop(self):
        if self.state in (IDLE, RUNNING):
            self.after_stop = FAULT
            self.state = STOPPING

    def mc_clear_fault(self):
        if self.state == FAULT:
            self.state = DISABLED
        self.flags = 0

    def mc_set_zero(self):
        if self.state != DISABLED:
            return -1
        self.pos = [0.0] * N_AX
        return 0

    def mc_push(self, p):
        if self.state not in (IDLE, RUNNING):
            return -1
        if len(self.fifo) >= FIFO_N:
            self.flags |= FL_OVERRUN
            return -2
        for i in range(N_AX):
            if abs(p[i] - self.last_pushed[i]) > self.max_jump[i]:
                self.flags |= FL_JUMP
                self.log(f"JUMP on axis {i}: {self.last_pushed[i]} -> {p[i]} "
                         f"(max {self.max_jump[i]:.0f})")
                self.fault_stop()
                return -3
        self.fifo.append(list(p))
        self.last_pushed = list(p)
        return 0

    # ------------------------------------------------------------------ homing (homing.c)
    def home_start(self, offsets):
        if self.home & HS_ACTIVE:
            return 0                            # already running: repeat is ignored
        if self.state != DISABLED:
            return -1
        if not self.drv_ok:
            return -2
        self.home = HS_ACTIVE
        self.home_t0 = time.perf_counter()
        self.log(f"homing started, offsets={offsets}")
        return 0

    def home_abort(self):
        if not self.home & HS_ACTIVE:
            return
        self.home = (self.home & ~HS_ACTIVE) | HS_ERROR
        self.log("homing aborted")
        self.mc_disable()

    def home_service(self):
        if not self.home & HS_ACTIVE or self.args.homing_never:
            return
        if time.perf_counter() - self.home_t0 < self.args.homing_time:
            return
        if self.args.homing_fail:
            self.home = HS_ERROR
            self.log("homing finished with ERROR (--homing-fail)")
        else:
            self.pos = [0.0] * N_AX
            self.home = HS_HOMED_MASK
            self.log("homing finished, all axes homed, pos = 0")

    # ------------------------------------------------------------------ dispatch (uartconfig.c)
    def dispatch(self, typ, pl):
        n = len(pl)
        if self.home & HS_ACTIVE and typ != T_HOMING and not (typ == T_ENABLE and n == 1 and pl[0] == 0):
            return -1
        if typ == T_SETPOINT:
            if n != 4 * N_AX:
                return BAD_FRAME
            self.n_setpoints += 1
            p = list(struct.unpack("<6i", pl))
            self.log(f"SETPOINT {p}", 2)
            return self.mc_push(p)
        if typ == T_CONFIG:
            if n != 2 + 8 * N_AX:
                return BAD_FRAME
            vals = struct.unpack("<H6f6f", pl)
            r = self.mc_config(vals[0], list(vals[1:7]), list(vals[7:13]))
            self.log(f"CONFIG period={vals[0]} vmax={[round(x) for x in vals[1:7]]} "
                     f"accel={[round(x) for x in vals[7:13]]} -> {'ok' if r == 0 else 'rejected'}")
            return r
        if typ == T_ENABLE:
            if n != 1:
                return BAD_FRAME
            if pl[0]:
                r = self.mc_enable()
                self.log(f"ENABLE 1 -> {'ok' if r == 0 else 'rejected'}")
                return r
            self.log("ENABLE 0")
            self.home_abort()
            self.mc_disable()
            return 0
        if typ == T_CLEARFAULT:
            if n != 0:
                return BAD_FRAME
            self.log("CLEARFAULT")
            self.mc_clear_fault()
            return 0
        if typ == T_SETZERO:
            if n != 0:
                return BAD_FRAME
            r = self.mc_set_zero()
            self.log(f"SETZERO -> {'ok' if r == 0 else 'rejected'}")
            return r
        if typ == T_HOMING:
            if n != 4 * N_AX:
                return BAD_FRAME
            r = self.home_start(list(struct.unpack("<6i", pl)))
            if r < 0:
                self.log("HOMING -> rejected")
            return r
        return BAD_FRAME

    def on_frame(self, typ, pl):
        self.diag["ok"] += 1
        r = self.dispatch(typ, pl)
        if r == BAD_FRAME:
            self.diag["bad"] += 1
            self.log(f"bad frame: type=0x{typ:02X} len={len(pl)}")
        elif r < 0:
            self.diag["rej"] += 1

    # ------------------------------------------------------------------ receiver (Proto_Poll)
    def rx_bytes(self, data, now):
        for b in data:
            self.t_last_rx = now
            if self.ps == "SOF":
                if b == SOF:
                    self.ps = "LEN"
            elif self.ps == "LEN":
                if b > MAX_PAYLOAD:
                    self.diag["len"] += 1
                    self.ps = "LEN" if b == SOF else "SOF"
                    continue
                self.fb = bytearray([b])
                self.need = b + 5               # len + type + seq + payload + 2 CRC
                self.ps = "BODY"
            else:
                self.fb.append(b)
                if len(self.fb) == self.need:
                    n = self.fb[0]
                    rx_crc = self.fb[n + 3] | (self.fb[n + 4] << 8)
                    if crc16(bytes(self.fb[:n + 3])) == rx_crc:
                        self.on_frame(self.fb[1], bytes(self.fb[3:3 + n]))
                    else:
                        self.diag["crc"] += 1
                    self.ps = "SOF"

    def rx_timeout(self, now):
        if self.ps != "SOF" and now - self.t_last_rx > RX_TIMEOUT:
            self.ps = "SOF"
            self.diag["tout"] += 1

    # ------------------------------------------------------------------ motion (MC_Tick)
    def next_segment(self):
        while self.owed > 0 and self.fifo:      # setpoints of extrapolated slots are stale
            self.fifo.pop(0)
            self.owed -= 1
        self.seg_s = list(self.seg_e)

        cnt = len(self.fifo) - (1 if self.fifo else 0)
        self.n_cur = self.n_ticks
        if self.n_ticks >= 4:
            if cnt >= FILL_TARGET + FILL_BAND:
                self.n_cur = self.n_ticks - 1   # too full -> faster
            elif cnt + FILL_BAND <= FILL_TARGET:
                self.n_cur = self.n_ticks + 1   # too empty -> slower

        if self.fifo:
            self.seg_e = self.fifo.pop(0)
            self.last_span = [e - s for e, s in zip(self.seg_e, self.seg_s)]
            self.still = not any(self.last_span)
            self.ex_ticks = 0
        elif not self.still:
            # buffer empty while moving: continue with the last speed
            self.seg_e = [s + d for s, d in zip(self.seg_s, self.last_span)]
            self.owed += 1
            self.flags |= FL_UNDERRUN
        # else: standing still -> hold (seg_e == seg_s)
        self.tick = 0

    def redirect(self):
        # late setpoint for the slot that is being extrapolated: replaces the estimate
        while self.owed > 1:
            self.fifo.pop(0)
            self.owed -= 1
        self.seg_e = self.fifo.pop(0)
        self.last_span = [e - s for e, s in zip(self.seg_e, self.seg_s)]
        self.still = not any(self.last_span)
        self.owed = 0
        self.ex_ticks = 0

    def motion_tick(self):
        if self.state in (DISABLED, FAULT):
            self.vel = [0.0] * N_AX
            return

        if self.state == IDLE:
            if len(self.fifo) >= PREFILL:
                self.seg_e = self.pos_out()     # start = current position
                self.next_segment()
                self.state = RUNNING
        elif self.state == RUNNING:
            if self.tick >= self.n_cur:
                self.next_segment()
            elif self.owed > 0 and len(self.fifo) >= self.owed:
                self.redirect()
            if self.owed > 0:
                self.ex_ticks += 1
                if self.ex_ticks > EXTRAP_MAX:
                    self.flags |= FL_STARVED
                    self.log("STARVED: no setpoints for > 50 ms while moving")
                    self.after_stop = FAULT
                    self.state = STOPPING

        if self.state == RUNNING:
            frac = min(1.0, (self.tick + 1) / self.n_cur)
            for a in range(N_AX):
                target = self.seg_s[a] + (self.seg_e[a] - self.seg_s[a]) * frac
                vlim = self.vmax[a] * TICK
                step = max(-vlim, min(vlim, target - self.pos[a]))
                self.pos[a] += step
                self.vel[a] = step
            self.tick += 1
        elif self.state == STOPPING:
            moving = False
            for a in range(N_AX):
                dv = self.accel[a] * TICK * TICK
                v = self.vel[a]
                v = math.copysign(max(0.0, abs(v) - dv), v)
                self.vel[a] = v
                self.pos[a] += v
                moving = moving or v != 0.0
            if not moving:
                self.pos = [float(round(p)) for p in self.pos]
                self.state = self.after_stop
        # IDLE: hold

    # ------------------------------------------------------------------ transmitter
    def tx_frames(self):
        out = b""
        if self.ms % STATE_PERIOD == 0:
            pl = bytes([self.state, FIFO_N - len(self.fifo), self.flags_out(), self.home])
            pl += struct.pack("<6i", *self.pos_out())
            out += frame(T_STATE, pl, self.tx_seq)
            self.tx_seq = (self.tx_seq + 1) & 0xFF
        if self.ms % DIAG_PERIOD == 5:          # in a gap between two STATE frames
            d = self.diag
            pl = struct.pack("<8I", d["ok"], d["crc"], d["len"], d["tout"],
                             d["full"], d["uerr"], d["bad"], d["rej"])
            out += frame(T_DIAG, pl, self.tx_seq)
            self.tx_seq = (self.tx_seq + 1) & 0xFF
        return out

    # ------------------------------------------------------------------ 1 ms step
    def step(self, now):
        self.rx_timeout(now)
        if not self.drv_ok and not self.args.no_drv and now - self.t_start >= self.args.drv_delay:
            self.drv_ok = True
        self.home_service()
        self.motion_tick()
        self.log_changes()
        out = self.tx_frames()
        self.ms += 1
        return out


def open_pty(link):
    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    name = os.ttyname(slave)
    if os.path.islink(link) or os.path.exists(link):
        os.unlink(link)
    os.symlink(name, link)
    return master, slave, name


def main():
    ap = argparse.ArgumentParser(description="Simulated STM32 driver board on a virtual serial port")
    ap.add_argument("--link", default="/tmp/ttySTM", help="symlink to the virtual port (default /tmp/ttySTM)")
    ap.add_argument("--no-drv", action="store_true", help="never set DRV_OK (motor voltage off)")
    ap.add_argument("--drv-delay", type=float, default=0.5, help="seconds until DRV_OK is set (default 0.5)")
    ap.add_argument("--homing-time", type=float, default=3.0, help="seconds until homing finishes (default 3)")
    ap.add_argument("--homing-fail", action="store_true", help="homing ends with HS_ERROR")
    ap.add_argument("--homing-never", action="store_true",
                    help="homing never finishes (like the current firmware placeholder)")
    ap.add_argument("-v", "--verbose", action="count", default=0,
                    help="-v: commands and state changes, -vv: also every SETPOINT")
    args = ap.parse_args()

    # SIGTERM (kill, timeout) like Ctrl+C, so the symlink is always removed on exit
    def on_sigterm(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_sigterm)

    master, slave, name = open_pty(args.link)
    sim = SimSTM(args)
    print(f"Simulated STM on {args.link} -> {name}   (Ctrl+C to quit)", flush=True)

    next_t = time.perf_counter()
    last_summary = next_t
    try:
        while True:
            now = time.perf_counter()
            r, _, _ = select.select([master], [], [], max(0.0, next_t - now))
            now = time.perf_counter()
            if r:
                try:
                    data = os.read(master, 4096)
                except (BlockingIOError, OSError):
                    data = b""
                if data:
                    sim.rx_bytes(data, now)
            if now - next_t > 0.1:              # fell far behind (e.g. suspended): resync
                next_t = now
            while now >= next_t:
                out = sim.step(next_t)
                if out:
                    try:
                        os.write(master, out)
                    except (BlockingIOError, OSError):
                        pass                    # nobody reading / buffer full: drop like a busy UART
                next_t += TICK
            if sim.verbose and now - last_summary >= 5.0:
                d = sim.diag
                sim.log(f"diag ok={d['ok']} crc={d['crc']} bad={d['bad']} rej={d['rej']} "
                        f"tout={d['tout']} setpoints={sim.n_setpoints}")
                last_summary = now
    except KeyboardInterrupt:
        print()
    finally:
        try:
            if os.path.islink(args.link):
                os.unlink(args.link)
        except OSError:
            pass
        os.close(slave)
        os.close(master)


if __name__ == "__main__":
    main()
