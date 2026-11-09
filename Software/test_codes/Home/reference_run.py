#!/usr/bin/env python3
"""
reference_run.py - Referenzfahrt für das Homing

Misst pro Achse den Offset = Schritte von der Nullpose bis zum mechanischen Anschlag.
Der STM stoppt beim Stall sofort (MC_faststop) und schreibt den Wert in home_offset[ax].
Das Skript liest denselben Wert aus dem Zustandsframe und speichert ihn in offsets.json.

Ablauf:
  1. Arm in die Nullpose stellen
  2. zero          Config senden + SetZero (nullt ALLE Achsen, nur einmal am Anfang)
  3. enable        Motion Controller freigeben
  4. run <achse>   Achse mit SPEED/ACCEL in den Anschlag fahren, Offset speichern
  5. nächste Achse: enable, run <achse>

Nach einem Mechanikwechsel: Arm in Nullpose, zero, enable, run <betroffene Achse>.
In offsets.json wird nur diese Achse überschrieben.

Hinweise:
  - Die Achse muss im STM in "axis" (main.h) enthalten sein, sonst wird kein Stall erkannt.
  - Die TMC sind bestromt, sobald VM an ist. Arm von Hand bewegen nur mit VM aus.
  - Das UART-Protokoll wird nicht verändert (robot_proto.py aus uart_tests).

Aufruf: python reference_run.py   (braucht pyserial)
"""
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "uart_tests"))
from robot_proto import (Link, sleep_until, N_AX, DISABLED, IDLE, RUNNING, STOPPING, FAULT,  # noqa: E402
                         FL_CFG_OK, FL_DRV_OK, FL_JUMP, FL_STARVED, flag_names)

# ---------------- Einstellungen ----------------
PORT = "COM10"
SPEED = 3000.0                          # Schritte/s in den Anschlag (>= MIN_FREQ_STALL)
ACCEL = 6000.0                          # Schritte/s^2
HOME_DIR = [+1, +1, +1, +1, +1, +1]     # Richtung zum Anschlag pro Achse (+1 oder -1)
MAX_STEPS = 50000                       # weiter wird nicht gefahren -> Abbruch "kein Anschlag"
MIN_FREQ_STALL = 2000.0                 # wie min_freq_stall im STM (TMC_monitor.c)
T_MS = 10                               # Sollwertperiode in ms
MARGIN = 0.8                            # Fahrt nutzt 80 % der Grenzen im STM (Reserve für den Folgeregler)

T = T_MS / 1000.0
OFFSET_FILE = os.path.join(HERE, "offsets.json")

HELP = """Befehle:
  zero         Arm steht in der Nullpose -> Config senden + SetZero (alle Achsen)
  enable       Motion Controller freigeben (löscht vorher einen Fault)
  run <achse>  Achse in den Anschlag fahren, Offset speichern
  show         Zustand, gespeicherte Offsets und C-Zeile anzeigen
  quit         beenden"""


# ---------------- Offsets (Datei am PC) ----------------
def load_offsets():
    if not os.path.exists(OFFSET_FILE):
        return {}
    with open(OFFSET_FILE) as f:
        return {int(k): int(v) for k, v in json.load(f).items()}


def save_offsets(offsets):
    with open(OFFSET_FILE, "w") as f:
        json.dump({str(k): v for k, v in sorted(offsets.items())}, f, indent=2)


def c_line(offsets):
    vals = ", ".join(str(offsets.get(a, 0)) for a in range(N_AX))
    return f"int32_t home_offset[N_AX] = {{{vals}}};"


# ---------------- Hilfen ----------------
def to_disabled(link, timeout=5.0):
    """STM in DISABLED bringen: Fahrt bremsen bzw. Fault löschen. Rückgabe: Zustand oder None."""
    t_end = time.perf_counter() + timeout
    while time.perf_counter() < t_end:
        st = link.fresh_state()
        if st.state == DISABLED:
            return st
        if st.state == FAULT:
            link.clear_fault()
        elif st.state in (IDLE, RUNNING):
            link.enable(False)
        # STOPPING: Bremsen abwarten
    return None


def stop(link):
    """Kontrolliert bremsen (MC_Disable) und Endzustand anzeigen."""
    link.enable(False)
    link.wait(lambda s: s.state in (DISABLED, FAULT), 5.0)
    print("Zustand:", link.state)


# ---------------- Befehle ----------------
def cmd_zero(link):
    if to_disabled(link) is None:
        print("STM erreicht DISABLED nicht:", link.state)
        return
    link.clear_fault()                                   # alte Flags löschen
    link.config(T_MS, [SPEED / MARGIN] * N_AX, [ACCEL / MARGIN] * N_AX)
    link.set_zero()
    st = link.fresh_state()
    if not st.flags & FL_CFG_OK:
        print("Config nicht angenommen:", st)
        return
    if any(st.pos):
        print("SetZero hat nicht geklappt:", st)
        return
    print("Nullpunkt gesetzt:", st)


def cmd_enable(link):
    st = link.fresh_state()
    if st.state == IDLE:
        print("Schon freigegeben:", st)
        return
    st = to_disabled(link)
    if st is None:
        print("STM erreicht DISABLED nicht:", link.state)
        return
    if not st.flags & FL_DRV_OK:
        print("DRV_OK fehlt: TMC-Treiber nicht initialisiert (VM an? Achse in 'axis'?)")
        return
    if not st.flags & FL_CFG_OK:
        print("Keine Config im STM: erst 'zero'")
        return
    link.clear_fault()                                   # alte Flags (z. B. JUMP) löschen
    link.enable(True)
    if not link.wait(lambda s: s.state == IDLE, 2.0):
        print("Enable abgelehnt:", link.state)
        return
    print("Freigegeben:", link.state)


def cmd_run(link, ax, offsets):
    st = link.fresh_state()
    if st.state != IDLE:
        print("Erst 'enable'. Zustand:", st)
        return

    p = list(st.pos)
    start = p[ax]
    d = HOME_DIR[ax]
    x, v = float(start), 0.0
    with link.lock:
        j0 = len(link.states)                            # ab hier alle Zustandsframes auswerten

    print(f"Achse {ax} fährt Richtung {d:+d} mit {SPEED:.0f} Schritte/s ... (Strg+C = Abbruch)")
    t0 = time.perf_counter()
    k = 0
    try:
        while link.state.state not in (FAULT, DISABLED):
            if abs(x - start) >= MAX_STEPS:
                print(f"Kein Anschlag nach {MAX_STEPS} Schritten gefunden, bremse ...")
                stop(link)
                return
            v = min(SPEED, v + ACCEL * T)                # anfahren mit ACCEL, dann konstant SPEED
            x += d * v * T
            p[ax] = int(round(x))
            k += 1
            sleep_until(t0 + k * T)
            link.setpoint(p)
    except KeyboardInterrupt:
        print("\nAbbruch, bremse kontrolliert ...")
        stop(link)
        return

    st = link.fresh_state()                              # Position steht nach dem Stall still
    with link.lock:
        frames = link.states[j0:]
    via_stopping = any(f.state == STOPPING for f in frames)

    # Stall = MC_faststop: direkt RUNNING -> FAULT. Alle anderen Fehler bremsen erst über STOPPING.
    if st.state != FAULT or via_stopping or st.flags & (FL_JUMP | FL_STARVED):
        print("Kein Stall, sondern ein anderer Fehler. Nichts gespeichert.")
        print(f"  Zustand: {st}  (STOPPING gesehen: {via_stopping}, Flags: {flag_names(st.flags)})")
        return

    off = st.pos[ax]
    offsets[ax] = off
    save_offsets(offsets)
    print(f"Stall erkannt. Offset Achse {ax}: {off} Schritte  (gespeichert in {OFFSET_FILE})")
    print("  " + c_line(offsets))
    print("Nächste Achse: enable, run <achse>")


def cmd_show(link, offsets):
    print("Zustand:", link.fresh_state())
    if offsets:
        for a in sorted(offsets):
            print(f"  Achse {a}: {offsets[a]}")
    else:
        print("  noch keine Offsets gespeichert")
    print("  " + c_line(offsets))


# ---------------- Hauptprogramm ----------------
def main():
    if SPEED < MIN_FREQ_STALL:
        sys.exit(f"SPEED ({SPEED:.0f}) muss >= MIN_FREQ_STALL ({MIN_FREQ_STALL:.0f}) sein, "
                 f"sonst erkennt der STM keinen Stall.")
    if len(HOME_DIR) != N_AX or any(d not in (1, -1) for d in HOME_DIR):
        sys.exit("HOME_DIR braucht 6 Einträge mit +1 oder -1.")

    offsets = load_offsets()
    link = Link(PORT)
    try:
        if not link.wait(lambda s: True, 3.0):
            sys.exit("Keine Zustandsframes empfangen (Port? Firmware? Terminal noch offen?)")
        print("Verbunden:", link.state)
        print(HELP)

        while True:
            try:
                words = input("> ").split()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not words:
                continue
            cmd, args = words[0].lower(), words[1:]
            try:
                if cmd == "zero":
                    cmd_zero(link)
                elif cmd == "enable":
                    cmd_enable(link)
                elif cmd == "run":
                    if len(args) != 1 or not args[0].isdigit() or int(args[0]) >= N_AX:
                        print(f"Aufruf: run <achse 0...{N_AX - 1}>")
                        continue
                    cmd_run(link, int(args[0]), offsets)
                elif cmd == "show":
                    cmd_show(link, offsets)
                elif cmd in ("quit", "exit", "q"):
                    break
                else:
                    print(HELP)
            except TimeoutError as e:
                print("Keine Antwort vom STM:", e)
    finally:
        link.close()


if __name__ == "__main__":
    main()
