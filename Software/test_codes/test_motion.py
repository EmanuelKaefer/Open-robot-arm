"""Test für motion.c (virtuelle Achsen) über das Binärprotokoll.

Ablauf: Ablehnungen prüfen -> konfigurieren -> Fahrt mit Flusssteuerung ->
Endposition + Gleichlauf prüfen -> Disable während der Fahrt -> Overrun -> Sprung/Fault.
"""
import math, statistics, time
from robot_proto import *

PORT = "COM10"      # bei dir ggf. anders (Windows: COMx)

PERIOD_MS = 10
T = PERIOD_MS / 1000
VMAX = [20000.0] * N_AX          # Schritte/s
ACC = [50000.0] * N_AX           # Schritte/s²
AMPS = [6000, -4000, 3000, -2000, 1000, 500]   # Zielpositionen Fahrt 1
MOVE_S, HOLD_S = 3.0, 0.5

results = []

def check(name, ok, info=""):
    ok = bool(ok)
    results.append((name, ok))
    print(f"{'OK  ' if ok else 'FAIL'} {name}" + (f"  ({info})" if info else ""))

def diff(a, b):
    return {k: b[k] - a[k] for k in a if b[k] != a[k]}

def half_cos(A, t, D):
    """Weiche Fahrt 0 -> A in D Sekunden (Geschwindigkeit startet und endet bei 0)."""
    t = min(max(t, 0.0), D)
    return A * (1 - math.cos(math.pi * t / D)) / 2

def stream(link, points):
    """Sollwerte im Takt T senden, Füllstand des MCU-Puffers über 'free' regeln.

    Der MCU verbraucht genau einen Punkt pro Periode nach SEINER Uhr. Weicht die
    PC-Uhr ab (HSI ~1 %), läuft der Puffer ohne Regelung leer oder über.
    """
    push_t = []
    skipped = extra = 0
    min_free, max_free = FIFO_N, 0
    t_next = time.perf_counter()
    i = 0
    while i < len(points):
        sleep_until(t_next)
        t_next += T
        st = link.state
        min_free, max_free = min(min_free, st.free), max(max_free, st.free)
        fill = FIFO_N - st.free
        if fill >= 6:                                   # Puffer zu voll: Slot auslassen
            skipped += 1
            continue
        link.setpoint(points[i]); push_t.append(time.perf_counter()); i += 1
        if fill <= 1 and st.state == RUNNING and i < len(points):   # zu leer: einen extra
            link.setpoint(points[i]); push_t.append(time.perf_counter()); i += 1
            extra += 1
    return push_t, skipped, extra, min_free, max_free

def wait_still(link, n=20, timeout=3.0):
    """Wartet, bis sich die Position über n Zustandsframes nicht mehr ändert."""
    t_end = time.perf_counter() + timeout
    while time.perf_counter() < t_end:
        with link.lock:
            last = link.states[-n:]
        if len(last) == n and all(s.pos == last[0].pos for s in last):
            return last[-1]
        time.sleep(0.02)
    return None


link = Link(PORT)
if not link.wait(lambda st: True, 3.0):
    raise SystemExit("Kein Zustandsframe empfangen - Firmware geflasht? Port richtig?")
print("Start:", link.state)

# ---------- 0) Definierter Ausgangszustand (falls ein früherer Lauf abgebrochen wurde) ----------
link.enable(False)
link.wait(lambda st: st.state in (DISABLED, FAULT), 2.0)
link.clear_fault()
check("Ausgangszustand DISABLED", link.wait(lambda st: st.state == DISABLED, 1.0), link.state)
dg_start = link.fresh_diag()                        # MCU-Zähler laufen seit dem Boot

# ---------- 1) Ungültige / unzulässige Befehle ----------
d0 = link.fresh_diag()
link.setpoint([0] * N_AX)                           # im DISABLED nicht erlaubt   -> rej
link.config(1, VMAX, ACC)                           # Periode < 2 ms             -> rej
link.config(PERIOD_MS, [0.0] * N_AX, ACC)           # vmax 0                     -> rej
link.send(T_SETPOINT, b"\x00" * 10)                 # falsche Länge              -> bad
link.send(0x7F)                                     # unbekannter Typ            -> bad
d1 = link.fresh_diag()
check("Ablehnungen gezählt", diff(d0, d1) == {"ok": 5, "rej": 3, "bad": 2}, diff(d0, d1))

# ---------- 2) Nullen + Konfigurieren ----------
link.set_zero()
link.config(PERIOD_MS, VMAX, ACC)
st = link.fresh_state()
check("Konfiguration angenommen", st.flags & FL_CFG_OK and st.pos == (0,) * N_AX, st)

# ---------- 3) Enable ----------
link.enable(True)
check("Enable -> IDLE", link.wait(lambda st: st.state == IDLE, 0.5), link.state)
d0 = link.fresh_diag()
link.config(PERIOD_MS, VMAX, ACC)                   # Config nur im DISABLED erlaubt
check("Config während Enable abgelehnt", diff(d0, link.fresh_diag()) == {"ok": 1, "rej": 1})

# ---------- 4) Fahrt 1: weich auf AMPS, dann halten ----------
n_pts = int(round((MOVE_S + HOLD_S) / T))
points = [[round(half_cos(A, (k + 1) * T, MOVE_S)) for A in AMPS] for k in range(n_pts)]
with link.lock: i_start = len(link.states)
push_t, skipped, extra, min_free, max_free = stream(link, points)
st_end = link.state                                 # Flags direkt nach dem letzten Sollwert
with link.lock: log = link.states[i_start:]

print(f"     Fahrt 1: {n_pts} Sollwerte, {skipped} Slots ausgelassen, {extra} extra, "
      f"free min/max = {min_free}/{max_free}")
check("Fahrt startet (RUNNING)", any(s.state == RUNNING for s in log))
check("kein Under-/Overrun während der Fahrt",
      not st_end.flags & (FL_UNDERRUN | FL_OVERRUN), flag_names(st_end.flags))

st = wait_still(link)
dev = [p - q for p, q in zip(st.pos, points[-1])] if st else None
check("Endposition erreicht (±1 Schritt)", st and all(abs(d) <= 1 for d in dev), f"Abweichung {dev}")

# Gleichlauf: Alle Achsen fahren dasselbe Profil, skaliert mit ihrer Amplitude.
# Aus Achse 0 lässt sich also für jede andere Achse die Sollposition berechnen.
run = [s for s in log if s.state == RUNNING]
sync_err = max((abs(s.pos[a] - s.pos[0] * AMPS[a] / AMPS[0]) for s in run for a in range(1, N_AX)),
               default=float("inf"))
check("Gleichlauf aller Achsen (max. 2 Schritte)", sync_err <= 2.0, f"max {sync_err:.2f}")

# Verzögerung Sollwert -> Istposition: Achse 0 zurückrechnen auf den Sollwert-Index
lags = []
for s in run:
    r = s.pos[0] / AMPS[0]
    if 0.05 < r < 0.95:
        k = MOVE_S / math.pi * math.acos(1 - 2 * r) / T - 1     # gebrochener Sollwert-Index
        i0 = int(k)
        if 0 <= i0 < len(push_t) - 1:
            tp = push_t[i0] + (push_t[i0 + 1] - push_t[i0]) * (k - i0)
            lags.append(s.t - tp)
if lags:
    print(f"     Verzögerung Sollwert -> Ist: Median {statistics.median(lags)*1000:.0f} ms, "
          f"max {max(lags)*1000:.0f} ms (Puffer + Prefill + USB)")

# ---------- 5) Fahrt 2 zurück, mittendrin Disable ----------
start = list(st.pos)
points = [[round(p0 - half_cos(p0, (k + 1) * T, MOVE_S)) for p0 in start] for k in range(n_pts)]
with link.lock: i_start = len(link.states)
stream(link, points[: n_pts // 3])
link.enable(False)
ok = link.wait(lambda st: st.state == DISABLED, 1.0)
with link.lock: log = link.states[i_start:]
seen = [STATE_NAMES[s.state] for s in log]
check("Disable während der Fahrt -> STOPPING -> DISABLED",
      ok and "STOPPING" in seen, f"{seen.count('STOPPING')} Frames STOPPING")
st = wait_still(link)
check("steht nach Disable", st is not None and st.state == DISABLED, st)

# ---------- 6) Overrun: 16 Sollwerte auf einmal ----------
link.enable(True)
link.wait(lambda st: st.state == IDLE, 0.5)
d0 = link.fresh_diag()
hold = list(link.state.pos)
for _ in range(16):
    link.setpoint(hold)                             # aktuelle Position -> keine Bewegung
st = link.fresh_state()
d = diff(d0, link.fresh_diag())
check("Overrun erkannt", st.flags & FL_OVERRUN and d.get("rej", 0) >= 1,
      f"{flag_names(st.flags)}, {d.get('rej', 0)} abgelehnt")
link.enable(False)
link.wait(lambda st: st.state == DISABLED, 1.0)
link.clear_fault()

# ---------- 7) Sprung -> FAULT ----------
link.enable(True)
link.wait(lambda st: st.state == IDLE, 0.5)
hold = list(link.state.pos)
for _ in range(3):
    link.setpoint(hold)
jump = list(hold); jump[0] += 5000                  # max. erlaubt: vmax*T*2.5+2 = 502
link.setpoint(jump)
ok = link.wait(lambda st: st.state == FAULT, 1.0)
check("Sprung -> FAULT mit Flag JUMP", ok and link.state.flags & FL_JUMP, link.state)
link.clear_fault()
st = link.fresh_state()
check("ClearFault -> DISABLED, Flags gelöscht", st.state == DISABLED and st.flags == FL_CFG_OK, st)

# ---------- Zusammenfassung Verbindung ----------
time.sleep(0.2)
dg = diff(dg_start, link.fresh_diag())
with link.lock:
    ts = [s.t for s in link.states]
    print(f"\nFrames empfangen: {link.frames}  CRC-Fehler: {link.crc_err}  Seq-Lücken: {link.seq_gaps}")
rate = (len(ts) - 1) / (ts[-1] - ts[0])
print(f"Zustandsframes: {len(ts)}  Rate {rate:.1f} Hz")
print("MCU-Zähler (Zuwachs im Test):", dg)
check("Verbindung sauber (PC-CRC, Seq, MCU-crc/len/tout/full/uerr = 0)",
      link.crc_err == 0 and link.seq_gaps == 0 and not any(dg.get(k) for k in ("crc", "len", "tout", "full", "uerr")))
link.close()

n_ok = sum(ok for _, ok in results)
print(f"\n{n_ok}/{len(results)} Prüfungen bestanden")
