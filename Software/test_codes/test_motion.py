"""Test für motion.c (virtuelle Achsen) über das Binärprotokoll.

Ablauf: Ablehnungen -> konfigurieren -> Fahrt im starren Takt (Firmware regelt das Tempo selbst) ->
Fahrt mit 70-ms-Aussetzer (Extrapolation) -> Disable während der Fahrt -> Stream-Abriss (STARVED) ->
Overrun -> Sprung/Fault.
"""
import math, statistics, time
from robot_proto import *

PORT = "COM10"      # bei dir ggf. anders (Windows: COMx)

PERIOD_MS = 10
T = PERIOD_MS / 1000
VMAX = [20000.0] * N_AX          # Schritte/s
ACC = [50000.0] * N_AX           # Schritte/s²
AMPS = [6000, -4000, 3000, -2000, 1000, 500]   # Zielpositionen
MOVE_S, HOLD_S = 3.0, 0.5
N_PTS = int(round((MOVE_S + HOLD_S) / T))

results = []

def check(name, ok, info=""):
    ok = bool(ok)
    results.append((name, ok))
    print(f"{'OK  ' if ok else 'FAIL'} {name}" + (f"  ({info})" if info else ""))

def diff(a, b):
    return {k: b[k] - a[k] for k in a if b[k] != a[k]}

def half_cos(t, D):
    """Weiches Profil 0 -> 1 in D Sekunden (Geschwindigkeit startet und endet bei 0)."""
    t = min(max(t, 0.0), D)
    return (1 - math.cos(math.pi * t / D)) / 2

def move(start, target):
    """Sollwerte für eine weiche Fahrt start -> target, danach HOLD_S halten."""
    return [[round(s + (e - s) * half_cos((k + 1) * T, MOVE_S)) for s, e in zip(start, target)]
            for k in range(N_PTS)]

def stream(link, points, gap_at=None, gap_s=0.0):
    """Sollwerte stur im Takt T senden - ohne Flusssteuerung, die Firmware passt ihr Tempo an.

    gap_at/gap_s: an dieser Stelle nichts senden, bis der MCU-Puffer leergelaufen ist und
    danach noch gap_s lang (diese Zeit muss die Firmware extrapolieren). Die Sendezeitpunkte
    bleiben absolut, danach kommen die fehlenden Punkte also als Schwall (wie ein ruckelnder PC).
    """
    push_t = []
    min_free, max_free = FIFO_N, 0
    t_next = time.perf_counter()
    for i, p in enumerate(points):
        if i == gap_at:
            time.sleep((FIFO_N - link.state.free) * T + gap_s)
        sleep_until(t_next)
        t_next += T
        st = link.state
        min_free, max_free = min(min_free, st.free), max(max_free, st.free)
        link.setpoint(p)
        push_t.append(time.perf_counter())
    return push_t, min_free, max_free

def sync_error(log, start, target):
    """Gleichlauf: Alle Achsen fahren dasselbe Profil, nur skaliert. Aus dem Fortschritt von
    Achse 0 lässt sich für jede andere Achse die Position berechnen - Abweichung davon."""
    d = [e - s for s, e in zip(start, target)]
    return max((abs((st.pos[a] - start[a]) - (st.pos[0] - start[0]) * d[a] / d[0])
                for st in log for a in range(1, N_AX)), default=float("inf"))

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

def log_since(i):
    with link.lock:
        return link.states[i:]

def log_mark():
    with link.lock:
        return len(link.states)

# Gleichlauf-Toleranz in Schritten. Grundrauschen ~1.5: Sollwerte sind ganzzahlig, die langsamste
# Achse bekommt nur 2-3 Schritte pro Segment. Wird extrapoliert oder gebremst, wirkt deren
# Geschwindigkeits-Rundung (bis ±20 %) über mehrere Segmente -> bis ~4 Schritte (< 1/4 Vollschritt).
SYNC_TOL = 2.0
SYNC_TOL_EXTRAP = 4.0

def drive(name, start, target, gap_at=None, gap_s=0.0, tol=SYNC_TOL):
    """Ganze Fahrt mit Halten am Ende, prüft Endposition und Gleichlauf."""
    points = move(start, target)
    i0 = log_mark()
    push_t, min_free, max_free = stream(link, points, gap_at, gap_s)
    st_end = link.state                             # Flags direkt nach dem letzten Sollwert
    st = wait_still(link)
    log = log_since(i0)
    print(f"     {name}: {len(points)} Sollwerte, free min/max = {min_free}/{max_free}, "
          f"Flags {flag_names(st_end.flags)}")
    dev = [p - q for p, q in zip(st.pos, points[-1])] if st else None
    check(f"{name}: Endposition erreicht (+-1 Schritt)",
          st and st.state == RUNNING and all(abs(x) <= 1 for x in dev), f"Abweichung {dev}, {st}")
    err = sync_error([s for s in log if s.state == RUNNING], start, target)
    check(f"{name}: Gleichlauf aller Achsen (max. {tol:g} Schritte)", err <= tol, f"max {err:.2f}")
    return st, st_end, push_t, log


link = Link(PORT)
if not link.wait(lambda st: True, 3.0):
    raise SystemExit("Kein Zustandsframe empfangen - Firmware geflasht? Port richtig?")
print("Start:", link.state)
if not link.wait(lambda st: st.flags & FL_DRV_OK, 3.0):
    raise SystemExit("Treiber nicht initialisiert (DRV_OK fehlt) - Enable ist gesperrt. "
                     "Treiber gesteckt? Motorspannung an?")

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

# ---------- 4) Fahrt 1: starrer Takt, keine Flusssteuerung am PC ----------
zero = [0] * N_AX
st, st_end, push_t, log = drive("Fahrt 1", zero, AMPS)
# UNDERRUN ist nur ein Hinweis (PC hat kurz gestockt, Firmware hat extrapoliert)
check("Fahrt 1: kein Overrun, kein Abbruch (Firmware regelt Füllstand)",
      not st_end.flags & FL_OVERRUN and not any(s.state in (STOPPING, FAULT) for s in log),
      flag_names(st_end.flags))

# Verzögerung Sollwert -> Istposition: Achse 0 zurückrechnen auf den Sollwert-Index
lags = []
for s in log:
    r = s.pos[0] / AMPS[0]
    if s.state == RUNNING and 0.05 < r < 0.95:
        k = MOVE_S / math.pi * math.acos(1 - 2 * r) / T - 1     # gebrochener Sollwert-Index
        i0 = int(k)
        if 0 <= i0 < len(push_t) - 1:
            lags.append(s.t - (push_t[i0] + (push_t[i0 + 1] - push_t[i0]) * (k - i0)))
if lags:
    print(f"     Verzögerung Sollwert -> Ist: Median {statistics.median(lags)*1000:.0f} ms, "
          f"max {max(lags)*1000:.0f} ms (Puffer + Prefill + USB)")

# ---------- 5) Fahrt 2 zurück mit Aussetzer bei voller Geschwindigkeit ----------
# Puffer läuft leer, dann 30 ms ohne Sollwerte -> Extrapolation überbrückt, der Schwall danach
# ersetzt die Schätzung bzw. verwirft veraltete Punkte.
start = list(st.pos)
st, st_end, _, log = drive("Fahrt 2 (Aussetzer)", start, zero, gap_at=int(MOVE_S / T / 2), gap_s=0.03,
                          tol=SYNC_TOL_EXTRAP)
check("Fahrt 2: Aussetzer überbrückt (UNDERRUN gemeldet, kein FAULT)",
      st_end.flags & FL_UNDERRUN and not any(s.state in (STOPPING, FAULT) for s in log),
      flag_names(st_end.flags))

# ---------- 6) Fahrt 3, mittendrin Disable -> gleichlaufend bremsen ----------
start = list(st.pos)
i0 = log_mark()
stream(link, move(start, AMPS)[: N_PTS // 3])
link.enable(False)
ok = link.wait(lambda st: st.state == DISABLED, 1.0)
log = log_since(i0)
seen = [STATE_NAMES[s.state] for s in log]
check("Disable während der Fahrt -> STOPPING -> DISABLED",
      ok and "STOPPING" in seen, f"{seen.count('STOPPING')} Frames STOPPING")
err = sync_error(log, start, AMPS)
check(f"Bremsen im Gleichlauf (max. {SYNC_TOL_EXTRAP:g} Schritte)", err <= SYNC_TOL_EXTRAP, f"max {err:.2f}")
st = wait_still(link)
check("steht nach Disable", st is not None and st.state == DISABLED, st)

# ---------- 7) Fahrt 4, Stream reisst ab -> >50 ms extrapolieren, bremsen, FAULT ----------
link.enable(True)
link.wait(lambda st: st.state == IDLE, 0.5)
start = list(link.state.pos)
i0 = log_mark()
stream(link, move(start, zero)[: N_PTS // 3])       # danach kommt einfach nichts mehr
ok = link.wait(lambda st: st.state == FAULT, 1.0)
log = log_since(i0)
check("Stream-Abriss -> FAULT mit Flag STARVED", ok and link.state.flags & FL_STARVED, link.state)
err = sync_error(log, start, zero)
check(f"Abriss: Extrapolation + Bremsen im Gleichlauf (max. {SYNC_TOL_EXTRAP:g} Schritte)",
      err <= SYNC_TOL_EXTRAP, f"max {err:.2f}")
link.clear_fault()
link.wait(lambda st: st.state == DISABLED, 0.5)

# ---------- 8) Overrun: 16 Sollwerte auf einmal ----------
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

# ---------- 9) Sprung -> FAULT ----------
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
check("ClearFault -> DISABLED, Flags gelöscht", st.state == DISABLED and st.flags == FL_CFG_OK | FL_DRV_OK, st)

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
