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
#include <iostream>
#include <fcntl.h>   // ::open(), O_RDWR, O_NOCTTY, O_NONBLOCK
#include <termios.h>  //tcgetattr(), cfmakeraw(), cfsetispeed(), cfsetospeed(), tcsetattr(), tcflush()
#include <unistd.h>   //::read(), ::write(), ::close()
#include <cerrno>     //errno (EAGAIN / EWOULDBLOCK = "no data" in non-blocking mode, not an error)
#include <cstring>    //std::strerror()

namespace arm_hardware
{

namespace
{
// Largest frame of the protocol: 6 bytes overhead + 64 bytes payload
constexpr std::size_t MAX_FRAME_LEN = 6 + 64;
}  // namespace

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & device, int baudrate)
{
  // 1. fd_ = ::open(device, O_RDWR | O_NOCTTY | O_NONBLOCK)
  //    - EACCES -> user is not in group "dialout"
  //    - ENOENT -> ST-Link not attached to WSL (usbipd attach --wsl --busid <id>)
  fd_=::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd_<0)
  {
    const int err = errno;  // save it, the output below may change errno
    std::cerr << "SerialPort: cannot open " << device << ": " << std::strerror(err) << std::endl;
    if (err == EACCES)
    {
      std::cerr << "  -> user is not in group dialout (sudo usermod -aG dialout $USER)" << std::endl;
    }else if (err==ENOENT)
    {
      std::cerr << "  -> ST-Link not attached to WSL (usbipd attach --wsl --busid <id>)"
                << " or stm_sim.py not running" << std::endl;
    }
    return false;
  }
  
  // 2. tcgetattr(), then cfmakeraw(): raw mode, no echo, no line editing,
  //    no CR/LF translation (binary frames contain 0x0A / 0x0D bytes!)
  struct termios tty{};

  if (tcgetattr(fd_, &tty) !=0)
  {
    ::close(fd_);
    fd_=-1;
    return false;
  }
  cfmakeraw(&tty);

  // 3. Baudrate: map int -> speed_t (115200 -> B115200), return false for anything else
  speed_t speed;
  if (baudrate == 115200)
  {
    speed= B115200;
  } else {
    std::cerr << "Baud rate not supported! Try 115200";
    ::close(fd_);
    fd_=-1;
    return false;
  }
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 4. 8N1, no flow control: CS8, clear PARENB, CSTOPB, CRTSCTS; set CLOCAL | CREAD;
  //    clear IXON | IXOFF | IXANY
  tty.c_cflag &= ~PARENB;        // no Priority
  tty.c_cflag &= ~CSTOPB;        // 1 Stoppbit 
  tty.c_cflag &= ~CSIZE;         // delete Databitmasl ...
  tty.c_cflag |= CS8;            // ... set 8 databits
  tty.c_cflag &= ~CRTSCTS;       // no (RTS/CTS)
  tty.c_cflag |= (CLOCAL | CREAD); // Modem-read only
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);

  // 5. VMIN = 0, VTIME = 0 -> ::read() returns immediately
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  // 6. tcsetattr(TCSANOW), then tcflush(TCIOFLUSH) to drop stale bytes
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      ::close(fd_);
      fd_ = -1;
      return false;
  }
  tcflush(fd_, TCIOFLUSH);

  return true;
}

void SerialPort::close()
{
  if (fd_>=0)
  {
    ::close(fd_);
    fd_=-1;
  }
}

bool SerialPort::is_open() const
{
  return fd_ >= 0;
}

bool SerialPort::write(const std::vector<uint8_t> & bytes)
{
  // TODO:
  // - ::write() the whole buffer. A frame is at most 70 bytes (6 + 64 payload)
  //   the kernel tty buffer takes it in one call; still loop on short writes.
  // - EAGAIN on a full output buffer -> return false (caller logs it)
  // - Never block for long: write() runs inside the 100 Hz control loop.
  if (fd_<0 ||  bytes.size() > MAX_FRAME_LEN)
  {
    return false;  
  }
  
  std::size_t total_written = 0;

  while (total_written < bytes.size()) {
    ssize_t n = ::write(fd_, bytes.data() + total_written, bytes.size() - total_written);

    if (n > 0) {
      total_written += static_cast<std::size_t>(n);
    } else if (n < 0) {
      if (errno == EINTR) {
        // got blocked for a small time -->instantly try again
        continue;
      }
      return false;
    } else {
      return false;  // n == 0: nothing written
    }
  }

  return true;

}

ssize_t SerialPort::read_some(uint8_t * buf, std::size_t max_len)
{
  if (fd_ < 0) {
    return -1;
  }

  ssize_t n = ::read(fd_, buf, max_len);

  if (n >= 0) {
    // n > 0: n bytes read
    // n == 0: no data. With VMIN = 0 / VTIME = 0 Linux returns 0 (not EAGAIN)
    //         when nothing is available, even with O_NONBLOCK -> not an error!
    return n;
  }

  if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
    return 0; // no new data--> is not a error!!
  }
  return -1; // real error (e.g. device unplugged)
}

}  // namespace arm_hardware
