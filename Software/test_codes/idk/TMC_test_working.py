import spidev
import time
import RPi.GPIO as GPIO

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

# --- Clear any latched faults first ---
write_reg(0x01, 0x00000007)
time.sleep(0.1)

# --- Configure TMC5160 ---
write_reg(0x00, 0x00000004)   # GCONF
write_reg(0x10, 0x00060F10)   # IHOLD_IRUN
write_reg(0x6C, 0x14010053)   # CHOPCONF - 16 microsteps (was 256, now sensible)
write_reg(0x70, 0xC10D0024)   # PWMCONF
time.sleep(0.1)

# --- Verify chip is still healthy ---
gstat = read_reg(0x01)
drvstatus = read_reg(0x6F)
cs_actual = (drvstatus >> 16) & 0x1F
print(f"GSTAT:     0x{gstat:08X}  (should be 0x00000000)")
print(f"DRVSTATUS: 0x{drvstatus:08X}")
print(f"CS actual: {cs_actual}  (should be > 0)")
print(f"Undervoltage: {bool(gstat & (1<<2))}")
print()

# --- GPIO Setup ---
STEP_PIN = 21
DIR_PIN  = 20
GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN,  GPIO.OUT)
GPIO.output(DIR_PIN, 1)

# --- Move motor ---
# 16 microsteps x 200 steps = 3200 pulses per revolution
PULSES = 3200
DELAY  = 0.001  # 1ms = 1 revolution in ~3.2 seconds, slow and visible

print(f"Sending {PULSES} pulses (1 revolution at 16 microsteps)...")
for i in range(PULSES):
    GPIO.output(STEP_PIN, 1)
    time.sleep(DELAY)
    GPIO.output(STEP_PIN, 0)
    time.sleep(DELAY)

print("Done.")

# --- Check for faults after move ---
gstat = read_reg(0x01)
print(f"GSTAT after move: 0x{gstat:08X}  (still 0 = all good)")

GPIO.cleanup()
spi.close()