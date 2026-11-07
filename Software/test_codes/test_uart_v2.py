import serial, struct, time, os

PORT = "COM10"      # bei dir ggf. anders (Windows: COMx)
s = serial.Serial(PORT, 115200, timeout=0.1)

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

def show(wait=1.2):
    time.sleep(wait)
    print(s.read_all().decode(errors="replace").strip().splitlines()[-1])

# 1) 1000 gültige Frames (24 Byte Nutzdaten, wie ein Sollwert)
for i in range(1000):
    s.write(frame(1, os.urandom(24), i)); time.sleep(0.005)
show()                       # erwartet: ok=1000 (+ bereits gezählte)

# 2) 0xA5 im Nutzdatenbereich
s.write(frame(1, bytes([0xA5]) * 24)); show()          # ok +1

# 3) Ein gekipptes Bit -> CRC-Fehler, danach geht es normal weiter
f = bytearray(frame(1, os.urandom(24)))
f[10] ^= 0x01
s.write(bytes(f)); time.sleep(0.05)
s.write(frame(1, os.urandom(24))); show()              # crc +1, ok +1

# 4) Frame byteweise mit Pausen
for b in frame(2, os.urandom(10)):
    s.write(bytes([b])); time.sleep(0.003)
show()                                                  # ok +1, t2 +1

# 5) Abgebrochener Frame, dann gültiger nach 100 ms (Timeout muss greifen)
s.write(frame(1, os.urandom(24))[:8]); time.sleep(0.1)
s.write(frame(1, os.urandom(24))); show()              # tout +1, ok +1

# 6) Müll mit 0xA5 davor, 100 ms warten, dann gültiger Frame
s.write(b"\x00\xFF\xA5\xFF"); time.sleep(0.1)
s.write(frame(3, b"\x01")); show()                     # len +1, ok +1, t3 +1

# 7) Mehrere Frames direkt hintereinander
s.write(b"".join(frame(1, os.urandom(24), i) for i in range(20))); show()   # ok +20