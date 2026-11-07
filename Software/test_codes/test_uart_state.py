"""Protokolltest beide Richtungen: Zustandsframes (Rate, Seq, CRC) + Empfangszähler im Diagnoseframe."""
import os, time
from robot_proto import *

PORT = "COM10"      # bei dir ggf. anders (Windows: COMx)

link = Link(PORT)
if not link.wait(lambda st: True, 3.0):
    raise SystemExit("Kein Zustandsframe empfangen")

def check(name, before, expect):
    after = link.fresh_diag()
    diff = {k: after[k] - before[k] for k in after if after[k] != before[k]}
    ok = all(diff.get(k, 0) == v for k, v in expect.items())
    print(f"{'OK  ' if ok else 'FAIL'} {name}: {diff}")

# ---------- 1) Nur zuhören: Rate ----------
with link.lock: n0 = len(link.states)
t0 = time.perf_counter()
time.sleep(2.0)
with link.lock: n1 = len(link.states)
print(f"Rate {(n1 - n0) / (time.perf_counter() - t0):.1f} Hz  {link.state}")

# ---------- 2) Gleichzeitig Frames senden, Zähler müssen mitwandern ----------
# Sollwerte im Zustand DISABLED werden gültig empfangen (ok), aber abgelehnt (rej).
c = link.fresh_diag()
for i in range(1000):
    link.send(T_SETPOINT, os.urandom(24)); time.sleep(0.005)
check("1000 Sollwerte", c, {"ok": 1000, "crc": 0, "rej": 1000})

c = link.fresh_diag()
f = bytearray(frame(T_SETPOINT, os.urandom(24)))
f[10] ^= 0x01
link.s.write(bytes(f)); time.sleep(0.05)
link.send(T_SETPOINT, os.urandom(24))
check("gekipptes Bit", c, {"ok": 1, "crc": 1})

c = link.fresh_diag()
link.s.write(frame(T_SETPOINT, os.urandom(24))[:8]); time.sleep(0.1)
link.send(T_SETPOINT, os.urandom(24))
check("abgebrochener Frame", c, {"ok": 1, "tout": 1})

c = link.fresh_diag()
link.s.write(b"\x00\xFF\xA5\xFF"); time.sleep(0.1)
link.send(0x7F, b"\x01")
check("Müll + unbekannter Typ", c, {"len": 1, "ok": 1, "bad": 1})

c = link.fresh_diag()
link.s.write(b"".join(frame(T_SETPOINT, os.urandom(24), i) for i in range(20)))
check("20 am Stück", c, {"ok": 20})

# ---------- Zusammenfassung Rückrichtung ----------
time.sleep(0.1)
with link.lock:
    ts = [s.t for s in link.states]
    print(f"\nFrames: {link.frames}  CRC-Fehler: {link.crc_err}  Seq-Lücken: {link.seq_gaps}")
dts = [b - a for a, b in zip(ts, ts[1:])]
print(f"Zustandsframes: {len(ts)}  Abstand min {min(dts)*1000:.1f} ms  max {max(dts)*1000:.1f} ms  "
      f"Mittel {sum(dts)/len(dts)*1000:.2f} ms")
print("Endstand:", link.fresh_diag())
link.close()
