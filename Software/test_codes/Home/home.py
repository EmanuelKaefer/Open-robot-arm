#!/usr/bin/env python3
"""
home.py - Homing über UART starten (Test der Anbindung)

Schickt T_HOMING mit den Offsets aus offsets.yaml (von reference_run.py) und prüft,
ob der STM den Befehl angenommen hat. Der Fahrablauf im STM folgt im nächsten Schritt.
Bis dahin bleibt das Homing auf "läuft", bis man es mit 'abort' abbricht.

Sicheres Senden (das Protokoll hat keine Quittung):
  1. rej-Zähler aus einem frischen Diagnoseframe merken
  2. T_HOMING senden und bis zu ACK_TIMEOUT auf das Bit "läuft" im Zustandsframe warten
  3. kein Bit: rej gestiegen -> STM hat abgelehnt, sonst Frame verloren -> erneut senden
  Wiederholen ist ungefährlich: läuft das Homing schon, startet der STM nicht neu.

Befehle:
  home     Homing starten
  status   Zustand und Homing-Status anzeigen
  abort    Homing abbrechen (Enable 0, kontrolliert bremsen)
  quit     beenden

Aufruf: python home.py   (braucht pyserial und PyYAML)
"""
import sys

from robot_proto import Link, N_AX, HS_ACTIVE
from reference_run import load_offsets, OFFSET_FILE

# ---------------- Einstellungen ----------------
PORT = "COM10"
ACK_TIMEOUT = 0.2                       # s, bis das Bit "läuft" im Zustandsframe stehen muss
TRIES = 3                               # Sendeversuche

HELP = """Befehle:
  home     Homing mit den Offsets aus offsets.yaml starten
  status   Zustand und Homing-Status anzeigen
  abort    Homing abbrechen (Enable 0)
  quit     beenden"""


def start_homing(link, offsets):
    """T_HOMING senden, bis der STM 'läuft' meldet. Rückgabe True = angenommen."""
    for i in range(1, TRIES + 1):
        rej0 = link.fresh_diag()["rej"]
        link.home(offsets)
        if link.wait(lambda s: s.home & HS_ACTIVE, ACK_TIMEOUT):
            print(f"Homing angenommen (Versuch {i}):", link.state)
            return True
        if link.fresh_diag()["rej"] > rej0:
            print("Homing abgelehnt: STM nicht in DISABLED oder Treiber nicht bereit.")
            print("  Zustand:", link.state)
            return False
        print(f"Keine Antwort auf Versuch {i}, sende erneut ...")
    print("Homing nicht angenommen (keine Antwort vom STM).")
    return False


def cmd_home(link):
    stored = load_offsets()
    if not stored:
        print(f"Keine Offsets in {OFFSET_FILE}: erst reference_run.py ausführen")
        return
    offsets = [stored.get(a, 0) for a in range(N_AX)]     # nicht gemessene Achsen = 0
    print("Offsets:", offsets)
    start_homing(link, offsets)


def cmd_abort(link):
    link.enable(False)
    print("Zustand:", link.fresh_state())


def main():
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
            cmd = words[0].lower()
            try:
                if cmd == "home":
                    cmd_home(link)
                elif cmd == "status":
                    print("Zustand:", link.fresh_state())
                elif cmd == "abort":
                    cmd_abort(link)
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
