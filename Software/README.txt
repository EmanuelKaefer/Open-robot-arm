# Software

This folder contains all software related to controlling the robot arm.

## robotarm/
MATLAB/Simulink-based control software:
- `ik_test.m` — Interactive inverse kinematics solver (prompts for target XYZ coordinates)
- `robot_main.m` — Waypoint-based IK solver for motion planning; calculates joint angles for a sequence of target positions
- `show_robot.mlx` — Live script to visualise the robot model
- `open_robot_arm.slx` — Simulink model for real-time control (Raspberry Pi deployment via Simulink Coder)
- `raspitest.slx` — Simulink test model for Raspberry Pi hardware
- `urdf/` — URDF robot description exported from SolidWorks (V4 assembly)
- `meshes/` — STL mesh files for all robot links used in simulation

## manual_controller/
C++ firmware for joystick-based manual control of the robot arm:
- `arm_steuerung_mit_limits/` — Multi-axis manual control with limit switch support, multiple variants with OLED and LCD display feedback
- `main.cpp` — Entry point for the manual controller

## test_codes/
Test and development scripts for low-level hardware bring-up:
- `Robot-platformio/` — PlatformIO/Arduino C++ project for testing TMC5160 motor drivers via SPI on an Arduino Mega
- `TMC_test_working.py` — Python SPI test script for TMC5160 (confirmed working)
- `setupTMC5160.py` — Python script for TMC5160 initial configuration over SPI
- `debugregisterTMC5160.py` — Python script for reading and debugging TMC5160 registers
- `basics_on_tmc5160/` — Additional TMC5160 bring-up notes and experiments

## config_TMC5160T V1.0 [09.04.2026].docx
TMC5160T register configuration reference document (version 1.0).