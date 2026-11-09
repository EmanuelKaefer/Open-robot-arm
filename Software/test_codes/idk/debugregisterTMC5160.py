import spidev
import time

# SPI Setup
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0b11

REGS = {
    "GCONF":      0x00,
    "GSTAT":      0x01,
    "IOIN":       0x04,
    "TSTEP":      0x12,
    "RAMPSTAT":   0x35,
    "XACTUAL":    0x21,
    "VACTUAL":    0x22,
    "CHOPCONF":   0x6C,
    "DRVSTATUS":  0x6F,
}

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

# --- Clear latched flags first ---
write_reg(0x01, 0x00000007)
time.sleep(0.1)

# --- Raw register dump ---
print("=" * 50)
print("TMC5160 Register Dump")
print("=" * 50)
for name, addr in REGS.items():
    val = read_reg(addr)
    print(f"  {name:<12} (0x{addr:02X}): 0x{val:08X}  ({val})")

# --- Decoded diagnostics ---
print()
print("=" * 50)
print("Decoded Diagnostics")
print("=" * 50)

gstat = read_reg(0x01)
print(f"\nGSTAT: 0x{gstat:08X}")
print(f"  reset occurred: {bool(gstat & (1<<0))}")
print(f"  driver error:   {bool(gstat & (1<<1))}")
print(f"  undervoltage:   {bool(gstat & (1<<2))}")  # TRUE = Netzteil zu schwach!

drvstatus = read_reg(0x6F)
print(f"\nDRVSTATUS: 0x{drvstatus:08X}")
print(f"  stst (standstill): {bool(drvstatus & (1<<31))}")
print(f"  stealth:           {bool(drvstatus & (1<<30))}")
print(f"  CS actual:         {(drvstatus >> 16) & 0x1F}   (0 = kein Strom!)")
print(f"  ot  (overtemp):    {bool(drvstatus & (1<<25))}")
print(f"  otpw (temp warn):  {bool(drvstatus & (1<<26))}")
print(f"  s2ga (short A):    {bool(drvstatus & (1<<27))}")
print(f"  s2gb (short B):    {bool(drvstatus & (1<<28))}")
print(f"  ola  (open A):     {bool(drvstatus & (1<<29))}")
print(f"  olb  (open B):     {bool(drvstatus & (1<<30))}")

ioin = read_reg(0x04)
print(f"\nIOIN: 0x{ioin:08X}")
print(f"  STEP pin:  {bool(ioin & (1<<0))}")
print(f"  DIR pin:   {bool(ioin & (1<<1))}")
print(f"  ENN pin:   {bool(ioin & (1<<4))}  (True = Motor DEAKTIVIERT!)")
print(f"  VERSION:   0x{(ioin >> 24) & 0xFF:02X}  (muss 0x30 sein)")

chopconf = read_reg(0x6C)
print(f"\nCHOPCONF: 0x{chopconf:08X}")
print(f"  toff:      {chopconf & 0xF}  (0 = Treiber AUS!)")
print(f"  mres:      {(chopconf >> 24) & 0xF}  (0=256, 4=16, 8=1 Mikroschritt)")

print()
print("=" * 50)
print("Zusammenfassung:")
ok = True
if gstat & (1<<2):
    print("  ❌ UNDERVOLTAGE - Netzteil zu schwach oder VM nicht verbunden!")
    ok = False
if (ioin >> 24) & 0xFF != 0x30:
    print("  ❌ VERSION falsch - SPI Verbindung prüfen!")
    ok = False
if bool(ioin & (1<<4)):
    print("  ❌ ENN Pin HIGH - Motor ist deaktiviert!")
    ok = False
if (chopconf & 0xF) == 0:
    print("  ❌ toff=0 - Treiber in Software ausgeschaltet!")
    ok = False
if ((drvstatus >> 16) & 0x1F) == 0:
    print("  ❌ CS actual=0 - Kein Strom zum Motor!")
    ok = False
if ok:
    print("  ✅ Alles OK - Treiber bereit!")
print("=" * 50)

spi.close()