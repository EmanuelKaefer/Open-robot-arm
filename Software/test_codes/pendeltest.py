#!/usr/bin/env python3
"""
pendeltest.py - Genauigkeitstest mit echtem Motor über das Binärprotokoll

Testarten (--mode):
  pendel  Hin und her um --amp Schritte, mit kurzem Halt an beiden Enden.
          Am Ende steht der Motor wieder an der Startposition.
  drift   --cycles Fahrten in DIESELBE Richtung um je --step Schritte
          (Standard 3200 = 1 Motorumdrehung), mit Stopp dazwischen.
          Nach N ganzen Umdrehungen muss die Markierung genau wie am Anfang stehen.
  kurz    Viele kurze Fahrten zufälliger Länge mit Stopp, danach zurück zum Start.
          Prüft die Schrittzählung bei vielen Start-/Stoppvorgängen.

Beispiele:
  python pendeltest.py COM5
  python pendeltest.py COM5 --mode drift --cycles 50
  python pendeltest.py COM5 --mode kurz --cycles 300
  python pendeltest.py COM5 --axes 0,1 --amp 6400 --vmax 6000 --accel 12000 --csv log.csv --plot

Vorher: Nucleo resetten, Terminalprogramme schließen, Treiberversorgung (VM) an.
Motoren NIE bei eingeschalteter Treiberversorgung an- oder abstecken.
Abbruch jederzeit mit Strg+C: der Motor wird kontrolliert abgebremst.
"""
import argparse
import csv
import math
import random
import struct
import sys
import threading
import time

import serial

T_MS = 10
T = T_MS / 1000.0
STEPS_PER_REV = 3200                      # 200 Vollschritte x 16 Mikroschritte

# Nachrichtentypen (wie uartconfig.c)
T_SETPOINT, T_CONFIG, T_ENABLE, T_CLEARFAULT, T_SETZERO = 0x01, 0x02, 0x03, 0x04, 0x05
T_STATE, T_DIAG = 0x81, 0x82

ST_DISABLED, ST_IDLE, ST_RUNNING, ST_STOPPING, ST_FAULT = 0, 1, 2, 3, 4
ST_NAMES = {0: "DISABLED", 1: "IDLE", 2: "RUNNING", 3: "STOPPING", 4: "FAULT"}

# Bitwerte wie in motion.h. DRV_OK und STARVED bei Bedarf an deine Definition anpassen!
FL_UNDERRUN, FL_OVERRUN, FL_CFG_OK, FL_JUMP, FL_DRV_OK, FL_STARVED = 1, 2, 4, 8, 16, 32
FLAG_NAMES = [(FL_UNDERRUN, "UNDERRUN"), (FL_OVERRUN, "OVERRUN"), (FL_CFG_OK, "CFG_OK"),
              (FL_JUMP, "JUMP"), (FL_DRV_OK, "DRV_OK"), (FL_STARVED, "STARVED")]
DIAG_NAMES = ["ok", "crc", "len", "tout", "full", "uart", "bad", "rej"]


def flag_str(f):
    s = "|".join(n for b, n in FLAG_NAMES if f & b)
    return s or "-"


def crc16(data):
    c = 0xFFFF
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


# ----------------------------------------------------------------------------
# Verbindung: Senden im Hauptthread, Empfangen und Parsen in einem eigenen Thread
# ----------------------------------------------------------------------------
class Link:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.01)
        self.s.reset_input_buffer()
        self.buf = bytearray()
        self.lock = threading.Lock()
        self.states = []          # (Ankunftszeit, seq, state, free, flags, [6 Positionen])
        self.diag = None          # letzte 8 Zähler
        self.rx_crc_err = 0
        self.seq_gaps = 0
        self.last_seq = None
        self.tx_seq = 0
        self.run = True
        self.th = threading.Thread(target=self._rx, daemon=True)
        self.th.start()

    def close(self):
        self.run = False
        self.th.join(0.5)
        self.s.close()

    def _rx(self):
        while self.run:
            try:
                d = self.s.read(256)
            except serial.SerialException:
                break
            if d:
                self.buf += d
                self._parse()

    def _parse(self):
        while True:
            i = self.buf.find(0xA5)
            if i < 0:
                self.buf.clear()
                return
            if i:
                del self.buf[:i]
            if len(self.buf) < 2:
                return
            n = self.buf[1]
            if n > 64:
                del self.buf[0]
                continue
            if len(self.buf) < n + 6:
                return
            body = bytes(self.buf[1:4 + n])                 # len, type, seq, Nutzdaten
            crc = self.buf[4 + n] | (self.buf[5 + n] << 8)
            if crc16(body) != crc:
                self.rx_crc_err += 1
                del self.buf[0]
                continue
            del self.buf[:n + 6]
            typ, seq, pl = body[1], body[2], body[3:]
            if self.last_seq is not None and ((seq - self.last_seq) & 0xFF) != 1:
                self.seq_gaps += 1
            self.last_seq = seq
            now = time.perf_counter()
            with self.lock:
                if typ == T_STATE and n == 28:
                    st, free, fl, _res, *pos = struct.unpack("<BBBB6i", pl)
                    self.states.append((now, seq, st, free, fl, pos))
                elif typ == T_DIAG and n == 32:
                    self.diag = struct.unpack("<8I", pl)

    def send(self, typ, payload=b""):
        body = bytes([len(payload), typ, self.tx_seq & 0xFF]) + payload
        self.tx_seq += 1
        self.s.write(bytes([0xA5]) + body + struct.pack("<H", crc16(body)))

    def setpoint(self, p):
        self.send(T_SETPOINT, struct.pack("<6i", *[int(x) for x in p]))

    def config(self, vmax, accel):
        self.send(T_CONFIG, struct.pack("<H6f6f", T_MS, *([vmax] * 6), *([accel] * 6)))

    def enable(self, on):
        self.send(T_ENABLE, bytes([1 if on else 0]))

    def clear_fault(self):
        self.send(T_CLEARFAULT)

    def last(self):
        with self.lock:
            return self.states[-1] if self.states else None

    def n_states(self):
        with self.lock:
            return len(self.states)

    def frames_from(self, j0):
        with self.lock:
            return list(self.states[j0:])

    def wait_state(self, want, timeout=2.0):
        t_end = time.time() + timeout
        while time.time() < t_end:
            l = self.last()
            if l and l[2] == want:
                return True
            time.sleep(0.01)
        return False


def describe(st):
    if not st:
        return "keine Daten"
    return f"{ST_NAMES.get(st[2], st[2])} free={st[3]} flags={flag_str(st[4])} pos={st[5]}"


# ----------------------------------------------------------------------------
# Bewegungsprofil: weiche Fahrten (Kosinusrampe) und Haltephasen im 10-ms-Raster
# ----------------------------------------------------------------------------
class Profile:
    def __init__(self, start, axes, v_lim, a_lim):
        self.start = list(start)
        self.axes = axes
        self.v_lim = v_lim
        self.a_lim = a_lim
        self.off = 0                     # aktueller Versatz der getesteten Achsen (Schritte)
        self.cmd = []                    # Sollwerte, je 6 absolute Positionen
        self.marks = []                  # (Index, erwarteter Versatz, Text, Länge der Haltephase)
        self.v_pk = 0.0                  # größte geplante Geschwindigkeit (Schritte/s)
        self.a_pk = 0.0                  # größte geplante Beschleunigung (Schritte/s^2)

    def _push(self, off):
        p = list(self.start)
        for a in self.axes:
            p[a] = self.start[a] + int(round(off))
        self.cmd.append(p)

    def move_time(self, d):
        # Kosinusrampe über d Schritte in t Sekunden:
        #   v_max = pi*d/(2t)   a_max = pi^2*d/(2t^2)
        d = abs(d)
        return max(math.pi * d / (2 * self.v_lim),
                   math.pi * math.sqrt(d / (2 * self.a_lim)),
                   2 * T)

    def move(self, d):
        if d == 0:
            return
        n = max(2, math.ceil(self.move_time(d) / T))
        t = n * T
        self.v_pk = max(self.v_pk, math.pi * abs(d) / (2 * t))
        self.a_pk = max(self.a_pk, math.pi ** 2 * abs(d) / (2 * t * t))
        o0 = self.off
        for i in range(1, n + 1):
            self._push(o0 + d * (1 - math.cos(math.pi * i / n)) / 2)
        self.off = o0 + d

    def dwell(self, sec, label=None):
        n = max(1, round(sec / T))
        for _ in range(n):
            self._push(self.off)
        if label:
            self.marks.append((len(self.cmd) - 1, self.off, label, n))


def build_profile(args, start):
    prof = Profile(start, args.axes, args.vmax * args.margin, args.accel * args.margin)
    prof.dwell(0.3, "Start")
    if args.mode == "pendel":
        for c in range(args.cycles):
            prof.move(+args.amp)
            prof.dwell(args.dwell, f"Zyklus {c + 1} oben")
            prof.move(-args.amp)
            prof.dwell(args.dwell, f"Zyklus {c + 1} unten")
    elif args.mode == "drift":
        for c in range(args.cycles):
            prof.move(+args.step)
            prof.dwell(args.dwell, f"Fahrt {c + 1}")
    elif args.mode == "kurz":
        rng = random.Random(args.seed)
        for c in range(args.cycles):
            d = rng.randint(10, args.short_max) * rng.choice((-1, 1))
            if abs(prof.off + d) > args.amp:          # im Bereich +-amp bleiben
                d = -d
            prof.move(d)
            prof.dwell(0.1, f"Fahrt {c + 1}")
        prof.move(-prof.off)
        prof.dwell(args.dwell, "zurück am Start")
    prof.dwell(0.5, "Ende")
    return prof


# ----------------------------------------------------------------------------
# Senden im 10-ms-Raster. Verspätungen werden nachgeholt, aber höchstens
# max_burst Sollwerte auf einmal; was darüber hinaus fällig ist, wird übersprungen.
# ----------------------------------------------------------------------------
def stream(link, cmd, max_burst=4):
    j0 = link.n_states()
    t0 = time.perf_counter()
    k, skipped, late_events = 0, 0, 0
    while k < len(cmd):
        now = time.perf_counter()
        due = int((now - t0) / T)
        if due < k:
            target = t0 + k * T
            dt = target - now
            if dt > 0.002:
                time.sleep(dt - 0.0015)
            while time.perf_counter() < target:
                pass
            continue
        if due - k > max_burst:
            skipped += due - k - max_burst
            k = due - max_burst
            late_events += 1
        link.setpoint(cmd[k])
        k += 1
    return j0, skipped, late_events


# ----------------------------------------------------------------------------
# Auswertung
# ----------------------------------------------------------------------------
def best_lag(frames, cmd, axis, max_lag=40):
    """Verzögerung in Frames, bei der Ist und Soll am besten übereinstimmen."""
    best = (float("inf"), 0)
    n = len(cmd)
    for lag in range(max_lag):
        e = 0
        for k in range(10, n):
            j = k + lag
            if j >= len(frames):
                break
            e = max(e, abs(frames[j][5][axis] - cmd[k][axis]))
        if e < best[0]:
            best = (e, lag)
    return best[1]


results = []


def check(name, ok, info=""):
    results.append(ok)
    print(f"{'OK  ' if ok else 'FAIL'}  {name}" + (f"  ({info})" if info else ""))


def main(argv=None):
    ap = argparse.ArgumentParser(description="Genauigkeitstest mit echtem Motor")
    ap.add_argument("port")
    ap.add_argument("--mode", choices=["pendel", "drift", "kurz"], default="pendel")
    ap.add_argument("--axes", default="0", help="getestete Achsen, z. B. 0 oder 0,1")
    ap.add_argument("--amp", type=int, default=STEPS_PER_REV, help="Pendelweg bzw. Bereich (Schritte)")
    ap.add_argument("--step", type=int, default=STEPS_PER_REV, help="Fahrtlänge im Drift-Test")
    ap.add_argument("--short-max", type=int, default=400, help="längste kurze Fahrt (Schritte)")
    ap.add_argument("--cycles", type=int, default=10)
    ap.add_argument("--dwell", type=float, default=0.3, help="Haltezeit an den Enden (s)")
    ap.add_argument("--vmax", type=float, default=4000.0, help="Grenze im STM (Schritte/s)")
    ap.add_argument("--accel", type=float, default=8000.0, help="Grenze im STM (Schritte/s^2)")
    ap.add_argument("--margin", type=float, default=0.8, help="Profil nutzt diesen Anteil der Grenzen")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--csv", help="Soll/Ist pro Frame in diese Datei schreiben")
    ap.add_argument("--plot", action="store_true", help="Soll/Ist und Fehler plotten (matplotlib)")
    ap.add_argument("--yes", action="store_true", help="ohne Rückfrage starten")
    args = ap.parse_args(argv)
    args.axes = [int(a) for a in args.axes.split(",")]
    if any(a < 0 or a > 5 for a in args.axes):
        sys.exit("Achsen müssen zwischen 0 und 5 liegen.")

    link = Link(args.port)
    try:
        time.sleep(0.5)
        if not link.last():
            sys.exit("Keine Zustandsframes empfangen (Port? Firmware? Terminal noch offen?)")
        print("Start:", describe(link.last()))

        # --- Ausgangszustand herstellen ---
        if link.last()[2] == ST_FAULT:
            link.clear_fault()
        if link.last()[2] not in (ST_DISABLED, ST_FAULT):
            link.enable(False)
        if not link.wait_state(ST_DISABLED, 3.0):
            sys.exit("STM erreicht DISABLED nicht: " + describe(link.last()))
        link.clear_fault()                                  # klebrige Flags löschen
        time.sleep(0.05)
        if not link.last()[4] & FL_DRV_OK:
            sys.exit("DRV_OK fehlt: TMC-Treiber nicht initialisiert "
                     "(VM-Versorgung an? Achse in TMC_InitAll enthalten?)")

        link.config(args.vmax, args.accel)
        time.sleep(0.1)
        if not link.last()[4] & FL_CFG_OK:
            sys.exit("Konfiguration nicht angenommen: " + describe(link.last()))

        # --- Profil bauen ---
        start = list(link.last()[5])
        prof = build_profile(args, start)
        cmd = prof.cmd
        v_pk, a_pk = prof.v_pk, prof.a_pk
        dur = len(cmd) * T
        print(f"\nTest '{args.mode}' auf Achse(n) {args.axes}: {len(cmd)} Sollwerte, {dur:.1f} s")
        print(f"  Spitze: {v_pk:.0f} Schritte/s ({v_pk / STEPS_PER_REV:.2f} U/s), "
              f"{a_pk:.0f} Schritte/s²   Grenzen im STM: {args.vmax:.0f} / {args.accel:.0f}")
        if args.mode == "pendel":
            print(f"  Pendelweg: {args.amp} Schritte = {args.amp / STEPS_PER_REV:.2f} Motorumdrehungen")
        elif args.mode == "drift":
            print(f"  {args.cycles} x {args.step} Schritte = "
                  f"{args.cycles * args.step / STEPS_PER_REV:.2f} Motorumdrehungen in eine Richtung")
        if not args.yes:
            input("\nWelle markieren (z. B. Klebestreifen als Zeiger), dann Enter zum Start ... ")

        diag0 = link.diag
        link.enable(True)
        if not link.wait_state(ST_IDLE, 2.0):
            sys.exit("Enable abgelehnt: " + describe(link.last()))

        # --- Fahren ---
        try:
            j0, skipped, late = stream(link, cmd)
        except KeyboardInterrupt:
            print("\nAbbruch: kontrolliertes Bremsen ...")
            link.enable(False)
            link.wait_state(ST_DISABLED, 5.0)
            print("Zustand:", describe(link.last()))
            return 1

        time.sleep(0.3)
        link.enable(False)
        stopped = link.wait_state(ST_DISABLED, 5.0)
        time.sleep(0.25)                                    # letztes Diagnoseframe abwarten
        frames = link.frames_from(j0)
        diag1 = link.diag

        # --- Auswertung ---
        print()
        check("Fahrt beendet, Zustand DISABLED", stopped, describe(link.last()))
        fl_all = 0
        for f in frames:
            fl_all |= f[4]
        check("kein JUMP / STARVED / FAULT", not (fl_all & (FL_JUMP | FL_STARVED))
              and all(f[2] != ST_FAULT for f in frames), flag_str(fl_all))
        if fl_all & (FL_UNDERRUN | FL_OVERRUN):
            print(f"      Hinweis: {flag_str(fl_all & (FL_UNDERRUN | FL_OVERRUN))} aufgetreten "
                  f"(PC-Aussetzer, vom STM überbrückt)")
        print(f"      PC-Sendung: {late} Verspätung(en), {skipped} Sollwert(e) übersprungen")

        lag = best_lag(frames, cmd, args.axes[0])
        print(f"      Verzögerung Soll -> Ist: {lag * T_MS} ms")

        for a in args.axes:
            errs = [abs(frames[k + lag][5][a] - cmd[k][a])
                    for k in range(10, len(cmd)) if k + lag < len(frames)]
            e_max = max(errs) if errs else float("nan")
            print(f"      Achse {a}: Folgefehler während der Fahrt max {e_max} Schritte "
                  f"(Info: enthält Schwankungen der Verzögerung, kein Messwert der Genauigkeit)")

            devs = []
            for idx, exp_off, label, n_dw in prof.marks:
                j = idx - n_dw // 3 + lag            # sicher innerhalb der Haltephase
                if j < len(frames):
                    devs.append((frames[j][5][a] - start[a] - exp_off, label))
            worst = max(devs, key=lambda x: abs(x[0])) if devs else (0, "-")
            check(f"Achse {a}: Position an allen {len(devs)} Haltepunkten (+-1)",
                  all(abs(d) <= 1 for d, _ in devs), f"größte Abweichung {worst[0]} bei '{worst[1]}'")

            end_dev = link.last()[5][a] - cmd[-1][a]
            check(f"Achse {a}: Endposition = letzter Sollwert (+-1)", abs(end_dev) <= 1,
                  f"Abweichung {end_dev}")

        if diag0 and diag1:
            d = {n: diag1[i] - diag0[i] for i, n in enumerate(DIAG_NAMES)}
            bad = {n: d[n] for n in ("crc", "len", "tout", "full", "uart", "bad")}
            check("Verbindung sauber", all(v == 0 for v in bad.values()) and link.rx_crc_err == 0,
                  f"MCU {d}, PC-CRC {link.rx_crc_err}, Seq-Lücken {link.seq_gaps}")

        print(f"\n{results.count(True)}/{len(results)} Prüfungen bestanden")

        # --- Mechanische Kontrolle ---
        print("\nDie Software prüft nur, ob der STM die richtige Zahl an Schritten AUSGEGEBEN hat.")
        print("Ob der Motor alle umgesetzt hat, zeigt nur die Markierung:")
        if args.mode == "drift" and args.step % STEPS_PER_REV:
            print("  (Achtung: --step ist kein Vielfaches von 3200, die Markierung steht dann versetzt.)")
        else:
            print("  Sie muss jetzt genau wie am Anfang stehen.")
        print("  1 Mikroschritt = 0,1125°, 1 Vollschritt = 1,8°.")
        print("  Verlorene Schritte zeigen sich typisch in Sprüngen von 4 Vollschritten = 7,2°.")

        # --- CSV / Plot ---
        if args.csv or args.plot:
            rows = []
            for k in range(len(cmd)):
                j = k + lag
                if j >= len(frames):
                    break
                rows.append([k * T] + sum(([cmd[k][a], frames[j][5][a],
                                            frames[j][5][a] - cmd[k][a]] for a in args.axes), []))
            if args.csv:
                with open(args.csv, "w", newline="") as fh:
                    w = csv.writer(fh, delimiter=";")
                    w.writerow(["t_s"] + sum(([f"soll{a}", f"ist{a}", f"fehler{a}"]
                                              for a in args.axes), []))
                    w.writerows(rows)
                print(f"\nCSV geschrieben: {args.csv}")
            if args.plot:
                try:
                    import matplotlib.pyplot as plt
                except ImportError:
                    print("matplotlib nicht installiert (pip install matplotlib)")
                else:
                    t = [r[0] for r in rows]
                    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
                    for i, a in enumerate(args.axes):
                        ax1.plot(t, [r[1 + 3 * i] for r in rows], label=f"Soll {a}")
                        ax1.plot(t, [r[2 + 3 * i] for r in rows], "--", label=f"Ist {a}")
                        ax2.plot(t, [r[3 + 3 * i] for r in rows], label=f"Fehler {a}")
                    ax1.set_ylabel("Position (Schritte)")
                    ax2.set_ylabel("Ist - Soll (Schritte)")
                    ax2.set_xlabel("Zeit (s)")
                    ax1.legend()
                    ax2.legend()
                    plt.show()
        return 0 if all(results) else 2
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())