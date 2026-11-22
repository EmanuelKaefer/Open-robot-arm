// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
// See LICENSE-SOFTWARE in the project root for details.

#ifndef ARM_HARDWARE__SERIAL_PORT_HPP_
#define ARM_HARDWARE__SERIAL_PORT_HPP_

#include <sys/types.h>  // ssize_t

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace arm_hardware
{

// Thin wrapper around a POSIX serial device (termios).
//
// - No ROS dependency, so it can also be used from small test programs.
// - Non-blocking: read_some() never waits. ArmSystem::read() is called by the
//   controller_manager every 10 ms and must not block the control loop.
// - Link settings used by the firmware (USART2 / ST-Link virtual COM port):
//   115200 baud, 8N1, no handshake.
// - Typical device under WSL2 after "usbipd attach": /dev/ttyACM0
//   (the user must be in group "dialout").
class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();  // closes the port if still open

  // Owns a file descriptor -> not copyable
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // Opens and configures the device. Returns false on any error
  // (device missing, permission denied, unsupported baudrate, ...).
  bool open(const std::string & device, int baudrate);

  // Closes the device. Safe to call when already closed.
  void close();

  bool is_open() const;

  // Writes all bytes of one frame. Returns false on error or short write.
  bool write(const std::vector<uint8_t> & bytes);

  // Reads whatever is currently available, at most max_len bytes, without blocking.
  // Returns the number of bytes read (0 = nothing available) or -1 on error.
  ssize_t read_some(uint8_t * buf, std::size_t max_len);

private:
  int fd_ = -1;  // file descriptor, -1 = closed
};

}  // namespace arm_hardware

#endif  // ARM_HARDWARE__SERIAL_PORT_HPP_
