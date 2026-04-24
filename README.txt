# Robot Arm Project


This repository contains an ongoing project focused on the development of a 6-DOF robotic arm. The project is part of a school project and may later be extended into a larger final thesis project. The goal is to design and build a functional robotic arm while gaining experience in mechanical design, electronics, embedded systems, and software.

## Mechanical

The robotic arm is largely custom built. Most of the mechanical components are designed specifically for this project and manufactured using 3D printing. The design has gone through four iterations (V1–V4), with V4 being the current version. V4 includes a full SolidWorks assembly, 3D-printable parts for all major links (base, shoulder, upperarm, forearm, wrist, gripper), and an exported URDF model used for simulation and control.

## Electrical

The electronics are custom designed for this project, featuring a custom PCB. The board is designed in KiCad and uses an STM32F446RET6 microcontroller along with six TMC5160 stepper motor drivers. The schematic is split into separate sheets for the microcontroller, motor drivers, power supply, and peripherals. The PCB design has gone through two versions, with driver_board_v2 being the current design. Firmware for the STM32 is developed using STM32CubeIDE.

## Software

The software and control algorithms are developed primarily using MATLAB and Simulink. The `robotarm` module includes an inverse kinematics solver, a motion planning script for waypoint-based trajectories, a Simulink model for real-time control (targeting Raspberry Pi hardware), and the URDF robot model with STL meshes. Manual control firmware written in C++ (Arduino/PlatformIO) allows joystick-based control of individual joints with limit switch support and OLED/LCD feedback. Test scripts for configuring and debugging the TMC5160 motor drivers over SPI are available in Python and C++.

## Repository Structure

- `Mechanical/` — SolidWorks design files and 3D-printable parts (V1–V4)
- `Electrical/` — KiCad PCB projects, firmware (STM32CubeIDE), and pin layout
- `Software/` — MATLAB/Simulink control software, C++ manual controller, TMC5160 test code
- `doc/` — Datasheets, pin assignments, and reference data
