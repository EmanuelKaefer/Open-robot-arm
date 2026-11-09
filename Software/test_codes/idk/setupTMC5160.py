import spidev
import time

# SPI Setup
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0b11

def read_reg(addr):
    spi.xfer2([addr & 0x7F, 0, 0, 0, 0])
    result = spi.xfer2([addr & 0x7F, 0, 0, 0, 0])
    return (result[1] << 24) | (result[2] << 16) | (result[3] << 8) | result[4]

def write_reg(addr, value):
    data = [
        addr | 0x80,
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 8)  & 0xFF,
        value & 0xFF
    ]
    spi.xfer2(data)

# --- Clear latched faults ---
write_reg(0x01, 0x00000007)
time.sleep(0.1)

# --- Configure TMC5160 ---
write_reg(0x00, 0x00000004)   # GCONF  - STEP/DIR mode
write_reg(0x10, 0x00061008)   # IHOLD_IRUN - IHOLD=8 (25%), IRUN=16 (50%~1.05A), IHOLDDELAY=6
write_reg(0x6C, 0x14010053)   # CHOPCONF - 16 microsteps
write_reg(0x70, 0xC10D0024)   # PWMCONF
time.sleep(0.1)

# --- Verify ---
gstat     = read_reg(0x01)
drvstatus = read_reg(0x6F)
cs_actual = (drvstatus >> 16) & 0x1F

print("TMC5160 Konfiguration abgeschlossen")
print(f"  GSTAT:     0x{gstat:08X}  (OK = 0x00000000)")
print(f"  CS actual: {cs_actual}        (OK = > 0)")
print(f"  Undervoltage: {bool(gstat & (1<<2))}")

if gstat == 0 and cs_actual > 0:
    print("  >>> Treiber bereit - Simulink kann STEP/DIR übernehmen <<<")
else:
    print("  >>> FEHLER - Treiber nicht bereit, Verdrahtung prüfen! <<<")

spi.close()