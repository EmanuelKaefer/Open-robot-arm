import serial
import time

# --- EINSTELLUNGEN ---
SERIAL_PORT = 'COM10'  # Hier deinen echten COM-Port eintragen
BAUD_RATE = 115200
STEPS = 10000.0       # Wie viele Schritte getestet werden sollen
WAIT_TIME = 5.0       # Wartezeit in Sekunden (muss länger sein als die Fahrt dauert!)


def main():
    print(f"Verbinde mit {SERIAL_PORT}...")
    try:
        # Serielle Verbindung öffnen
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2) # Kurz warten, falls der STM32 durch die Verbindung neu startet
        print("Verbindung hergestellt. Starte Pendel-Test...")
        i=1
        while True:
            if i>=100:
                break
            i+=1
            print("Durchlauf:", i)
            # 1. Vorwärts fahren
            cmd_fwd = f"MOVE J1:{STEPS} J2:{STEPS} J3:0.0 J4:0.0 J5:0.0 J6:0.0\n"
            print(f"Sende: {cmd_fwd.strip()}")
            ser.write(cmd_fwd.encode('utf-8'))
            
            # Warten, bis die Bewegung sicher fertig ist
            time.sleep(WAIT_TIME)
            
            # 2. Rückwärts fahren
            cmd_rev = f"MOVE J1:{-STEPS} J2:{-STEPS} J3:0.0 J4:0.0 J5:0.0 J6:0.0\n"
            print(f"Sende: {cmd_rev.strip()}")
            ser.write(cmd_rev.encode('utf-8'))
            
            # Wieder warten
            time.sleep(WAIT_TIME)

    except KeyboardInterrupt:
        print("\nTest durch Benutzer abgebrochen.")
    except Exception as e:
        print(f"Fehler: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Verbindung geschlossen.")

if __name__ == '__main__':
    main()