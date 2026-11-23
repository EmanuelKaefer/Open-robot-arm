# stm_sim – simulated driver board

`stm_sim.py` stands in for the STM32 driver board, so you can test without hardware. It is Linux only (WSL works).

It creates a virtual serial port at **`/tmp/ttySTM`** and speaks the protocol from `doc/Software/UART-Protokoll.md`. Frame encoding, CRC and constants are reused from `../Home/robot_proto.py`. It needs pyserial, only because `robot_proto.py` imports it.

```
python3 stm_sim.py -v
```

Then point any client at `/tmp/ttySTM` instead of the real port:

- **Python scripts:** `Link("/tmp/ttySTM")`. For `home.py` and `reference_run.py`, set `PORT`.
- **ROS:** set `<param name="port">/tmp/ttySTM</param>` for `arm_hardware/ArmSystem`.

Start the simulator **before** the client. The port disappears when the simulator stops (Ctrl+C).

## Options

| Option | Effect |
|---|---|
| `-v` / `-vv` | Log commands and state/flag changes. `-vv` also logs every SETPOINT |
| `--link PATH` | A different port path (default `/tmp/ttySTM`) |
| `--no-drv` | Motor voltage "off": DRV_OK is never set, so ENABLE and HOMING are rejected |
| `--drv-delay S` | Seconds until DRV_OK is set (default 0.5) |
| `--homing-time S` | The homing mock finishes after S seconds (default 3) |
| `--homing-fail` | Homing ends with the error bit set |
| `--homing-never` | Homing never finishes, like the current firmware placeholder |

## What the simulator implements

These behave like the firmware (`uartconfig.c`, `motion.c`, `homing.c`):

- **Receiving:** the receive state machine with the 50 ms inter-byte timeout, and the DIAG counters `ok`, `crc`, `len`, `tout`, `bad`, `rej`.
- **Commands:** every command, accepted or rejected in the same states as the firmware.
  - While homing runs, only HOMING and ENABLE 0 are accepted.
- **Setpoint buffer:**
  - 8 slots; motion starts once 3 are buffered.
  - The tempo adapts so that about 4 stay buffered.
  - OVERRUN when the buffer is full.
- **Motion faults:**
  - JUMP check, against `2.5 · vmax · T + 2` steps.
  - Extrapolation with UNDERRUN; after 50 ms it raises STARVED and goes to FAULT.
  - After a setpoint without motion it simply holds.
- **Sending:** STATE every 10 ms and DIAG every 100 ms, with a shared `seq`.

## Simplifications

- **RUNNING:** the position follows the interpolated setpoint, capped at `vmax`. There is no P-controller and no acceleration limit.
- **STOPPING:** each axis brakes with its own `accel`. The real STM brakes all axes in sync.
- **Homing:** a mock. After `--homing-time` every position is set to 0 and all axes count as homed.
- **Not simulated:** stall, driver faults (short circuit, overtemperature), or a skipped STATE frame when the UART is busy.
