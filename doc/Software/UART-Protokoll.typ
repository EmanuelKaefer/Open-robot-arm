// UART-Protokoll PC <-> Driver-Board (gleicher Inhalt wie UART-Protokoll.md)

#set document(title: "UART-Protokoll PC ↔ Driver-Board")
#set page(paper: "a4", margin: (x: 2cm, y: 2cm), numbering: "1")
#set text(lang: "de", size: 10pt)
#set par(justify: false)
#set heading(numbering: "1.1")

#set table(stroke: 0.5pt + luma(160), inset: 5pt)
#show table: set text(size: 8.5pt)
#show table.cell.where(y: 0): set text(weight: "bold")
#show raw.where(block: true): set text(size: 8pt)
#show raw.where(block: true): block.with(fill: luma(240), inset: 8pt, radius: 3pt, width: 100%)

#align(center)[
  #text(size: 18pt, weight: "bold")[UART-Protokoll PC ↔ Driver-Board]
]

Stand: 18.11.2026 (Firmware-Branch `feature/homing`). Quelle der Wahrheit ist die Firmware:
`Core/Src/uartconfig.c`, `Core/Src/motion.c`, `Core/Inc/motion.h`, `Core/Src/homing.c`, `Core/Inc/homing.h`. \
PC-Referenzimplementierung: `Software/test_codes/Home/robot_proto.py`.

= Verbindung
- USART2 des Nucleo (virtueller COM-Port des ST-Link), *115200 Baud, 8N1*, kein Handshake.
- Zwischen zwei Bytes eines Frames darf höchstens *50 ms* Pause sein, sonst verwirft der STM den angefangenen Frame (Zähler `tout`).

= Frame-Aufbau
```
A5 | len | type | seq | payload[len] | crc_lo | crc_hi
```
- `A5`: Startbyte. `len`: Länge der Nutzdaten, *0…64*.
- `type`: Frame-Typ (siehe unten). `seq`: laufende Nummer, pro Richtung +1 je Frame (läuft bei 255 über). Der STM wertet die PC-Nummer nicht aus; der PC erkennt an der STM-Nummer verlorene Frames.
- *CRC-16/CCITT-FALSE* (Polynom 0x1021, Startwert 0xFFFF, keine Spiegelung, kein Schluss-XOR) über `len, type, seq, payload`. Prüfwert: `crc16("123456789") = 0x29B1`.
- Alle Mehrbyte-Werte *little-endian*. `int32` = vorzeichenbehaftet, `float` = IEEE 754 32 Bit.
- Beispiel Enable: `A5 01 03 00 01 05 BB`

= Befehle PC → STM

Der STM sendet *keine Quittung*. Ob ein Befehl gewirkt hat, sieht der PC am Zustandsframe (Abschnitt 8.3).

#table(
  columns: (auto, auto, auto, 1.3fr, auto, 2fr),
  table.header([Typ], [Name], [len], [Nutzdaten], [erlaubt in], [Wirkung / Ablehnung]),
  [0x01], [SETPOINT], [24], [6 × int32 Sollposition \[Schritte\], absolut], [IDLE, RUNNING],
  [Kommt in den Sollwert-Puffer (8 Plätze). Puffer voll → verworfen, Flag OVERRUN. Sprung zum vorigen Sollwert größer als `2,5 · vmax · T + 2` Schritte → Flag JUMP, Bremsen, FAULT],
  [0x02], [CONFIG], [50], [uint16 `period_ms`, 6 × float `vmax` \[Schritte/s\], 6 × float `accel` \[Schritte/s²\]], [DISABLED],
  [`period_ms` 2…100 = Takt der Sollwerte. Alle `vmax` und `accel` müssen größer 0 sein (auch für ungenutzte Achsen). Danach Flag `CFG_OK`],
  [0x03], [ENABLE], [1], [uint8: 1 = Enable, 0 = Disable], [1: DISABLED; 0: immer],
  [*1:* braucht `CFG_OK` und `DRV_OK` → IDLE, Puffer leer, Startpunkt = aktuelle Position, Flags gelöscht. *0:* aus IDLE/RUNNING kontrolliert bremsen (STOPPING) → DISABLED; bricht ein laufendes Homing ab],
  [0x04], [CLEARFAULT], [0], [–], [immer],
  [FAULT → DISABLED. Löscht die Flags UNDERRUN, OVERRUN, JUMP, STARVED],
  [0x05], [SETZERO], [0], [–], [DISABLED],
  [Alle 6 Positionen = 0],
  [0x06], [HOMING], [24], [6 × int32 Offsets \[Schritte\] (aus `offsets.yaml`)], [DISABLED, `DRV_OK`],
  [Startet das Homing (Status-Byte Bit 6). Läuft es schon: wird angenommen, startet aber *nicht* neu],
)

Abgelehnte Befehle zählen im Diagnoseframe als `rej`, Frames mit unbekanntem Typ oder falscher Länge als `bad`.

*Während das Homing läuft*, lehnt der STM alles ab außer HOMING (Wiederholung) und ENABLE 0 (Abbruch).

= Meldungen STM → PC

#table(
  columns: (auto, auto, auto, auto, 1fr),
  table.header([Typ], [Name], [len], [Takt], [Nutzdaten]),
  [0x81], [STATE], [28], [10 ms], [uint8 `state`, uint8 `free`, uint8 `flags`, uint8 `home`, 6 × int32 `pos` \[Schritte\]],
  [0x82], [DIAG], [32], [100 ms], [8 × uint32 Zähler: `ok, crc, len, tout, full, uerr, bad, rej`],
)

- `free`: freie Plätze im Sollwert-Puffer (0…8).
- `pos`: vom STM *ausgegebene* Schritte seit SETZERO bzw. Einschalten (kein Encoder). Nach einem Reset ist die Position 0, deshalb nach dem Einschalten immer Homing.
- Ist der UART beim fälligen STATE noch belegt, fällt dieser STATE aus (der nächste kommt 10 ms später). DIAG geht in der nächsten freien Lücke raus.
- `seq` ist ein gemeinsamer Zähler für STATE und DIAG.

DIAG-Zähler (laufen seit dem Einschalten hoch):

#table(
  columns: (auto, 1fr),
  table.header([Zähler], [Bedeutung]),
  [`ok`], [Frames mit gültiger CRC],
  [`crc`], [CRC-Fehler],
  [`len`], [Längenbyte größer 64],
  [`tout`], [Frame unvollständig (mehr als 50 ms Pause zwischen Bytes)],
  [`full`], [Empfangspuffer im STM übergelaufen],
  [`uerr`], [UART-Hardwarefehler (Overrun, Framing, Rauschen)],
  [`bad`], [unbekannter Typ oder falsche Nutzdatenlänge],
  [`rej`], [gültiger Befehl, aber im aktuellen Zustand abgelehnt],
)

= Zustände (`state`)

#table(
  columns: (auto, auto, 1fr),
  table.header([Wert], [Name], [Bedeutung]),
  [0], [DISABLED], [keine Bewegung, Config/SetZero/Homing möglich],
  [1], [IDLE], [freigegeben, wartet auf Sollwerte],
  [2], [RUNNING], [folgt den Sollwerten],
  [3], [STOPPING], [bremst kontrolliert (alle Achsen synchron)],
  [4], [FAULT], [gestoppt nach Fehler, nur CLEARFAULT hilft],
)

```
DISABLED --ENABLE 1--> IDLE --3 Sollwerte im Puffer--> RUNNING
IDLE/RUNNING --ENABLE 0--> STOPPING --> DISABLED
RUNNING --JUMP / STARVED / Treiberfehler--> STOPPING --> FAULT
RUNNING --Stall--> FAULT              (sofort, ohne Bremsrampe, alle Achsen)
FAULT --CLEARFAULT--> DISABLED
```
Die TMC-Treiber sind bestromt (Haltemoment), sobald VM anliegt und sie initialisiert sind – unabhängig vom Zustand.

= Flags (`flags`)

#table(
  columns: (auto, auto, auto, 1fr),
  table.header([Bit], [Wert], [Name], [Bedeutung]),
  [0], [1], [UNDERRUN], [Puffer war während der Fahrt leer, STM hat extrapoliert (nur Info)],
  [1], [2], [OVERRUN], [Sollwert verworfen, Puffer war voll],
  [2], [4], [`CFG_OK`], [gültige Config vorhanden],
  [3], [8], [JUMP], [Sollwertsprung zu groß → FAULT],
  [4], [16], [`DRV_OK`], [alle TMC-Treiber initialisiert (VM an)],
  [5], [32], [STARVED], [mehr als 50 ms keine Sollwerte während der Fahrt → gebremst, FAULT],
)

UNDERRUN, OVERRUN, JUMP und STARVED bleiben stehen bis ENABLE 1 oder CLEARFAULT. \
`CFG_OK` bleibt gesetzt, solange irgendeine gültige Config da ist – ob eine *neue* CONFIG angenommen wurde, zeigt nur `rej`.

= Homing-Status (`home`)

#table(
  columns: (auto, auto, 1fr),
  table.header([Bit], [Wert], [Bedeutung]),
  [0…5], [0x01…0x20], [Achse 0…5 gehomed],
  [6], [0x40], [Homing läuft],
  [7], [0x80], [Homing-Fehler oder abgebrochen],
)

- *Angenommen:* Bit 6 wird 1.
- *Fertig (gehomed):* Bit 6 wechselt von 1 auf 0 *und* Bit 7 = 0. Die Bits 0…5 zeigen, welche Achsen gehomed sind.
- *Fehlgeschlagen / abgebrochen:* Bit 6 = 0 und Bit 7 = 1.
- Ein neues HOMING löscht die alten Bits.

Der Status kommt in jedem STATE (alle 10 ms) – es gibt keine einmalige „gehomed“-Nachricht, die verloren gehen könnte.

= Abläufe für ROS

== Start
+ Auf STATE-Frames warten (Verbindung steht).
+ Auf `DRV_OK` warten (Treiber initialisiert, VM an).
+ Aus DISABLED: CONFIG senden (`period_ms` = Takt, mit dem ROS Sollwerte schickt; `vmax`/`accel` mit ca. 20 % Reserve über der Trajektorie, die der Folgeregler braucht).
+ Homing (Abschnitt 8.4), danach ENABLE 1 und auf IDLE warten.

== Fahren (Sollwerte streamen)
- Absolute Positionen aller 6 Achsen im Takt `period_ms` senden, auch für Achsen, die stehen.
- Ab 3 Sollwerten im Puffer startet die Fahrt (Verzögerung also ca. 3 · `period_ms`). Der STM hält den Puffer bei ca. 4 Sollwerten, indem er sein Tempo minimal anpasst – `free` ≈ 4 ist normal.
- Kurze Aussetzer des PC (bis 50 ms) überbrückt der STM durch Extrapolation (Flag UNDERRUN), verspätete Sollwerte werden nachgeholt.
- *Nach jeder Bewegung mindestens einen Sollwert mit unveränderter Position senden.* Erst dann weiß der STM, dass die Achsen stehen, und der PC darf mit dem Senden pausieren. Hört der Strom mitten in der Bewegung auf, extrapoliert der STM weiter und geht nach 50 ms in STARVED → FAULT.
- Sollwerte müssen glatt sein (Sprunggrenze siehe SETPOINT). Geschwindigkeit und Beschleunigung begrenzt der STM zusätzlich mit `vmax`/`accel`.

== Befehle sicher senden
Erfolg *immer am Zustand* prüfen, nicht an `rej`:

#table(
  columns: (auto, 1fr),
  table.header([Befehl], [Erfolg, wenn im STATE]),
  [ENABLE 1], [`state == IDLE`],
  [ENABLE 0], [`state == DISABLED` (nach STOPPING)],
  [CLEARFAULT], [`state == DISABLED`],
  [SETZERO], [alle `pos == 0`],
  [HOMING], [`home & 0x40`],
  [CONFIG], [`rej` ist nicht gestiegen und `CFG_OK` gesetzt],
)

Vorgehen: `rej` aus einem frischen DIAG merken → Befehl senden → bis ca. 200 ms auf den Zustand warten. Kommt er nicht: `rej` gestiegen → abgelehnt (falscher Zustand, Meldung an den Benutzer); sonst ging der Frame verloren → erneut senden (z. B. max. 3 Versuche). Wiederholen ist bei allen Befehlen ungefährlich. Beispiel: `start_homing()` in `test_codes/Home/home.py`.

Wichtig: nur Frames auswerten, die *nach* dem Senden entstanden sind (ca. 50 ms Marge, siehe `fresh_state()` in `robot_proto.py`).

== Homing
+ Zustand DISABLED und `DRV_OK`.
+ HOMING mit den 6 Offsets aus `offsets.yaml` senden (gemessen mit `test_codes/Home/reference_run.py`, nicht gemessene Achsen = 0), sicher senden wie in Abschnitt 8.3.
+ Warten, bis Bit 6 wieder 0 ist (`wait_homed()`): Bit 7 = 0 → gehomed, sonst Fehler.
+ Abbruch jederzeit mit ENABLE 0 (bremst kontrolliert, danach Bit 7 = 1).

== Fehler
- FAULT → CLEARFAULT → DISABLED → ENABLE 1. Die Position bleibt erhalten; nach einem Stall stimmt sie aber nicht mehr sicher → neu homen.
- Ursache erkennbar: JUMP und STARVED als Flag. *Stall:* Zustand springt direkt von RUNNING auf FAULT (kein STOPPING). *Treiberfehler* (Kurzschluss, Übertemperatur): FAULT über STOPPING ohne Flag.
- Stall wird nur auf den Achsen erkannt, die im STM in `axis` (`main.h`) stehen, und erst ab `min_freq_stall` (2000 Schritte/s, `TMC_monitor.c`).

== Einheiten
- Positionen in *Mikroschritten* (1/16): 3200 Schritte = 1 Motorumdrehung.
- Gelenkwinkel \[rad\] = Schritte / 3200 · 2π / Untersetzung der Achse. Die Untersetzung pro Achse kennt nur ROS.
- `vmax` in Schritte/s, `accel` in Schritte/s².

= Beispiel (Python, `robot_proto.py`)
```python
import time
from robot_proto import Link, sleep_until, IDLE, FL_DRV_OK, HS_ACTIVE

link = Link("COM10")
link.wait(lambda s: s.flags & FL_DRV_OK, 5.0)            # Treiber bereit

# Homing (Offsets aus offsets.yaml)
link.home([18750, 0, 0, 0, 0, 0])
if link.wait(lambda s: s.home & HS_ACTIVE, 0.2):         # angenommen
    print("gehomed" if link.wait_homed(120.0) else "Homing-Fehler")   # bis der Fahrablauf da ist: Timeout

# Fahren: Achse 0 drei Sekunden mit 1000 Schritte/s, dann stehen
link.config(10, [4000.0] * 6, [8000.0] * 6)
link.enable(True)
link.wait(lambda s: s.state == IDLE, 1.0)
p = list(link.state.pos)
t0 = time.perf_counter()
for k in range(1, 311):
    if k <= 300:
        p[0] += 10                                       # 10 Schritte pro 10 ms
    sleep_until(t0 + k * 0.01)
    link.setpoint(p)                                     # letzte 10 Sollwerte: Stillstand melden
link.close()
```

= Noch nicht definiert
- *Greifer-Befehle.*
- *Fehlerursache pro Achse* (Kurzschluss, Übertemperatur, Stall): liegt im STM in `tmc_flags`, wird nicht übertragen.
- *Homing-Fahrablauf* in der Firmware (`Home_Service()` ist noch ein Platzhalter): bis dahin bleibt das Homing nach dem Start auf „läuft“, bis es mit ENABLE 0 abgebrochen wird.
- Software-Endlagen und „nur gehomed fahren“ gibt es im STM noch nicht.
