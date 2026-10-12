"""Binärprotokoll PC <-> Driver-Board (gemeinsam für alle Testskripte).

Frame: A5 | len | type | seq | payload[len] | crc_lo crc_hi
CRC-16/CCITT-FALSE über len, type, seq, payload. Alles little-endian.
"""
import serial, struct, threading, time

# PC -> MCU
T_SETPOINT, T_CONFIG, T_ENABLE, T_CLEARFAULT, T_SETZERO = 0x01, 0x02, 0x03, 0x04, 0x05
# MCU -> PC
T_STATE, T_DIAG = 0x81, 0x82

DISABLED, IDLE, RUNNING, STOPPING, FAULT = 0, 1, 2, 3, 4
STATE_NAMES = {DISABLED: "DISABLED", IDLE: "IDLE", RUNNING: "RUNNING", STOPPING: "STOPPING", FAULT: "FAULT"}
FL_UNDERRUN, FL_OVERRUN, FL_CFG_OK, FL_JUMP, FL_DRV_OK, FL_STARVED = 1, 2, 4, 8, 16, 32
DIAG_KEYS = ("ok", "crc", "len", "tout", "full", "uerr", "bad", "rej")
FIFO_N = 8
N_AX = 6


def crc16(data: bytes) -> int:
    c = 0xFFFF
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def frame(typ: int, payload: bytes = b"", seq: int = 0) -> bytes:
    body = bytes([len(payload), typ, seq & 0xFF]) + payload
    return bytes([0xA5]) + body + struct.pack("<H", crc16(body))


# Selbsttest der CRC und des Frame-Aufbaus
assert crc16(b"123456789") == 0x29B1
assert frame(3, b"\x01").hex() == "a50103000105bb"


def flag_names(fl: int) -> str:
    names = [n for b, n in ((FL_UNDERRUN, "UNDERRUN"), (FL_OVERRUN, "OVERRUN"),
                            (FL_CFG_OK, "CFG_OK"), (FL_JUMP, "JUMP"), (FL_DRV_OK, "DRV_OK"), (FL_STARVED, "STARVED")) if fl & b]
    return "|".join(names) or "-"


class State:
    __slots__ = ("t", "state", "free", "flags", "pos")

    def __init__(self, t, state, free, flags, pos):
        self.t, self.state, self.free, self.flags, self.pos = t, state, free, flags, pos

    def __repr__(self):
        return (f"{STATE_NAMES.get(self.state, self.state)} free={self.free} "
                f"flags={flag_names(self.flags)} pos={list(self.pos)}")


class Link:
    """Serielle Verbindung mit Empfänger-Thread. Alle Zustandsframes landen in self.states."""

    def __init__(self, port, baud=115200):
        self.s = serial.Serial(port, baud, timeout=0.01)
        self.s.reset_input_buffer()
        self.lock = threading.RLock()
        self.tx_seq = 0
        self.synced = False          # Müll vor dem ersten gültigen Frame nicht zählen
        self.frames = 0
        self.crc_err = 0
        self.seq_gaps = 0
        self.last_seq = None
        self.states = []             # alle empfangenen State-Objekte
        self.diag = None             # (t, dict)
        self._run = True
        self._th = threading.Thread(target=self._reader, daemon=True)
        self._th.start()

    # ---------- Empfang ----------
    def _reader(self):
        buf = bytearray()
        while self._run:
            buf += self.s.read(256)
            self._feed(buf)

    def _feed(self, buf: bytearray):
        while True:
            i = buf.find(0xA5)
            if i < 0: buf.clear(); return
            del buf[:i]
            if len(buf) < 2: return
            n = buf[1]
            if n > 64: del buf[:1]; continue
            if len(buf) < n + 6: return
            body = bytes(buf[1:n + 4])
            if crc16(body) != (buf[n + 4] | (buf[n + 5] << 8)):
                if self.synced: self.crc_err += 1
                del buf[:1]          # nur das Startbyte verwerfen, neu synchronisieren
                continue
            del buf[:n + 6]
            self._on_frame(body[1], body[2], body[3:])

    def _on_frame(self, typ, seq, pl):
        t = time.perf_counter()
        with self.lock:
            self.synced = True
            if self.last_seq is not None and seq != ((self.last_seq + 1) & 0xFF):
                self.seq_gaps += 1
            self.last_seq = seq
            self.frames += 1
            if typ == T_STATE and len(pl) == 28:
                self.states.append(State(t, pl[0], pl[1], pl[2], struct.unpack("<6i", pl[4:])))
            elif typ == T_DIAG and len(pl) == 32:
                self.diag = (t, dict(zip(DIAG_KEYS, struct.unpack("<8I", pl))))

    # ---------- Abfragen ----------
    @property
    def state(self) -> State:
        with self.lock:
            return self.states[-1] if self.states else None

    def wait(self, cond, timeout=1.0) -> bool:
        """Wartet, bis cond(state) für einen neuen Zustandsframe wahr ist."""
        t_end = time.perf_counter() + timeout
        while time.perf_counter() < t_end:
            st = self.state
            if st is not None and cond(st): return True
            time.sleep(0.002)
        return False

    # Marge: bereits gesendete Bytes müssen erst über die Leitung und verarbeitet sein
    MARGIN = 0.05

    def fresh_state(self, timeout=0.5) -> State:
        """Erster Zustandsframe, der sicher nach allem bisher Gesendeten erzeugt wurde."""
        t0 = time.perf_counter() + self.MARGIN
        if not self.wait(lambda st: st.t > t0, timeout): raise TimeoutError("kein Zustandsframe")
        return self.state

    def fresh_diag(self, timeout=0.5) -> dict:
        t0 = time.perf_counter() + self.MARGIN
        t_end = t0 + timeout
        while time.perf_counter() < t_end:
            with self.lock:
                d = self.diag
            if d is not None and d[0] > t0: return dict(d[1])
            time.sleep(0.005)
        raise TimeoutError("kein Diagnoseframe")

    # ---------- Senden ----------
    def send(self, typ, payload=b""):
        self.s.write(frame(typ, payload, self.tx_seq))
        self.tx_seq = (self.tx_seq + 1) & 0xFF

    def setpoint(self, p):                 self.send(T_SETPOINT, struct.pack("<6i", *p))
    def config(self, period_ms, vmax, acc): self.send(T_CONFIG, struct.pack("<H6f6f", period_ms, *vmax, *acc))
    def enable(self, on=True):             self.send(T_ENABLE, bytes([1 if on else 0]))
    def clear_fault(self):                 self.send(T_CLEARFAULT)
    def set_zero(self):                    self.send(T_SETZERO)

    def close(self):
        self._run = False
        self._th.join()
        self.s.close()


def sleep_until(t):
    """Genauer als time.sleep allein: grob schlafen, die letzten ~2 ms aktiv warten."""
    while True:
        d = t - time.perf_counter()
        if d <= 0: return
        time.sleep(d - 0.002 if d > 0.003 else 0)
