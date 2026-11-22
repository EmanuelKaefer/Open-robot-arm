// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
// See LICENSE-SOFTWARE in the project root for details.

#include "arm_hardware/serial_port.hpp"

// Headers needed for the implementation (POSIX):
//   <fcntl.h>    ::open(), O_RDWR, O_NOCTTY, O_NONBLOCK
//   <termios.h>  tcgetattr(), cfmakeraw(), cfsetispeed(), cfsetospeed(), tcsetattr(), tcflush()
//   <unistd.h>   ::read(), ::write(), ::close()
//   <cerrno>     errno (EAGAIN / EWOULDBLOCK = "no data" in non-blocking mode, not an error)

namespace arm_hardware
{

SerialPort::~SerialPort()
{
  // TODO: close()
}

bool SerialPort::open(const std::string & /*device*/, int /*baudrate*/)
{
  // TODO:
  // 1. fd_ = ::open(device, O_RDWR | O_NOCTTY | O_NONBLOCK)
  //    - EACCES -> user is not in group "dialout"
  //    - ENOENT -> ST-Link not attached to WSL (usbipd attach --wsl --busid <id>)
  // 2. tcgetattr(), then cfmakeraw(): raw mode, no echo, no line editing,
  //    no CR/LF translation (binary frames contain 0x0A / 0x0D bytes!)
  // 3. Baudrate: map int -> speed_t (115200 -> B115200), return false for anything else
  // 4. 8N1, no flow control: CS8, clear PARENB, CSTOPB, CRTSCTS; set CLOCAL | CREAD;
  //    clear IXON | IXOFF | IXANY
  // 5. VMIN = 0, VTIME = 0 -> ::read() returns immediately
  // 6. tcsetattr(TCSANOW), then tcflush(TCIOFLUSH) to drop stale bytes
  //    (same idea as reset_input_buffer() in robot_proto.py)
  // 7. On any failure: close the fd again and return false
  return false;  // placeholder: fails until implemented
}

void SerialPort::close()
{
  // TODO: if fd_ >= 0 -> ::close(fd_), fd_ = -1
}

bool SerialPort::is_open() const
{
  // TODO: return fd_ >= 0
  return false;  // placeholder
}

bool SerialPort::write(const std::vector<uint8_t> & /*bytes*/)
{
  // TODO:
  // - ::write() the whole buffer. A frame is at most 70 bytes (6 + 64 payload),
  //   the kernel tty buffer takes it in one call; still loop on short writes.
  // - EAGAIN on a full output buffer -> return false (caller logs it)
  // - Never block for long: write() runs inside the 100 Hz control loop.
  return false;  // placeholder: fails until implemented
}

ssize_t SerialPort::read_some(uint8_t * /*buf*/, std::size_t /*max_len*/)
{
  // TODO:
  // - n = ::read(fd_, buf, max_len)
  // - n > 0 -> return n
  // - n < 0 with errno EAGAIN / EWOULDBLOCK -> return 0 (nothing available)
  // - other errors (e.g. device unplugged) -> return -1
  return -1;  // placeholder: fails until implemented
}

}  // namespace arm_hardware
