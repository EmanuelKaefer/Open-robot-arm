# Motor Driver Board

This folder contains two design iterations of the custom motor driver PCB for the robot arm.

## driver_board_v1/
First prototype PCB designed in KiCad. Single-sheet schematic combining the microcontroller and TMC5160 motor driver circuitry. Used as a proof-of-concept layout.

## driver_board_v2/
Current PCB design. Schematic is split into dedicated sheets:
- `microcontroller.kicad_sch` — STM32F446RET6 connections and decoupling
- `Motordrivers.kicad_sch` — Six TMC5160 stepper driver circuits
- `power-supply.kicad_sch` — Power regulation and supply rails
- `peripherals.kicad_sch` — Connectors and miscellaneous peripherals

Also includes:
- `Firmware/` — STM32CubeIDE firmware project for the STM32F446RET6
- `Pin_Layout_STM32_Robot_PCB.xlsx` — Pin assignment spreadsheet for the PCB
