# Motor Driver Board — v2 KiCad Project

Current PCB design for the robot arm controller board. The schematic is organised into separate hierarchical sheets:

- `driver_board.kicad_sch` — Top-level schematic
- `microcontroller.kicad_sch` — STM32F446RET6 microcontroller, clock, decoupling, and programming header
- `Motordrivers.kicad_sch` — Six TMC5160 stepper driver circuits with SPI control
- `power-supply.kicad_sch` — 24 V input, 5 V and 3.3 V regulation
- `peripherals.kicad_sch` — Connectors for motors, encoders, limit switches, and I/O

The PCB layout is in `driver_board.kicad_pcb`. Symbols specific to this project are stored in `open_robot_arm_lib.kicad_sym`.
