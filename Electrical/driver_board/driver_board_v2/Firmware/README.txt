# Firmware for STM32F446RET6

STM32CubeIDE project for the robot arm controller board (driver_board_v2).

The firmware targets the STM32F446RET6 microcontroller and communicates with six TMC5160 stepper motor drivers over SPI. The project is generated with STM32CubeMX; peripheral configuration is stored in `driver_board_software.ioc`.

## Project Structure
- `Core/` — Application code (main.c, interrupt handlers, startup files)
- `Drivers/` — STM32 HAL drivers and CMSIS headers
- `driver_board_software.ioc` — STM32CubeMX configuration file
- `STM32F446RETX_FLASH.ld` / `STM32F446RETX_RAM.ld` — Linker scripts
