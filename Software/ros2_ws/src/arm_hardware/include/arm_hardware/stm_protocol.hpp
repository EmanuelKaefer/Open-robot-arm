// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
// See LICENSE-SOFTWARE in the project root for details.

// Binary UART protocol PC <-> STM32 driver board.
//
// Specification:  doc/Software/UART-Protokoll.md
// Firmware:       Software/driver_board_software/Core/Src/uartconfig.c (source of truth)
// PC reference:   Software/test_codes/Home/robot_proto.py (port this 1:1)
//
// No ROS dependency -> unit tested in test/test_stm_protocol.cpp.

#ifndef ARM_HARDWARE__STM_PROTOCOL_HPP_
#define ARM_HARDWARE__STM_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace arm_hardware::proto
{

// ---------------------------------------------------------------------------
// Frame layout
//
//   A5 | len | type | seq | payload[len] | crc_lo | crc_hi
//
// - CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final XOR)
//   over len, type, seq, payload. Check value: crc16("123456789") = 0x29B1
// - all multi-byte values little-endian, int32 signed, float IEEE 754
// - seq: running number per direction, +1 per frame, wraps at 255.
//   The STM ignores the PC seq; the PC can detect lost STM frames from gaps.
// - the STM drops a partially received frame after a 50 ms gap between bytes
// ---------------------------------------------------------------------------
constexpr uint8_t START_BYTE = 0xA5;
constexpr std::size_t MAX_PAYLOAD = 64;
constexpr std::size_t FRAME_OVERHEAD = 6;  // start + len + type + seq + 2 CRC bytes

constexpr std::size_t N_AX = 6;  // number of axes in every SETPOINT / STATE / CONFIG

// Units: microsteps (1/16) -> 3200 steps = 1 motor revolution.
// joint angle [rad] = steps / 3200 * 2*pi / gear_ratio (gear ratio is only known to ROS)
constexpr double STEPS_PER_MOTOR_REV = 3200.0;

// ---------------------------------------------------------------------------
// Frame types PC -> STM (the STM sends NO acknowledgement, check the STATE frame)
// ---------------------------------------------------------------------------
constexpr uint8_t T_SETPOINT = 0x01;    // 6 x int32 absolute position [steps]
constexpr uint8_t T_CONFIG = 0x02;      // uint16 period_ms, 6 x float vmax, 6 x float accel
constexpr uint8_t T_ENABLE = 0x03;      // uint8: 1 = enable, 0 = disable
constexpr uint8_t T_CLEARFAULT = 0x04;  // no payload
constexpr uint8_t T_SETZERO = 0x05;     // no payload
constexpr uint8_t T_HOMING = 0x06;      // 6 x int32 offsets [steps] (from offsets.yaml)

// Frame types STM -> PC
constexpr uint8_t T_STATE = 0x81;  // every 10 ms
constexpr uint8_t T_DIAG = 0x82;   // every 100 ms

// Payload lengths (a frame with a wrong length is counted as "bad" by the STM)
constexpr std::size_t LEN_SETPOINT = 24;
constexpr std::size_t LEN_CONFIG = 50;
constexpr std::size_t LEN_ENABLE = 1;
constexpr std::size_t LEN_HOMING = 24;
constexpr std::size_t LEN_STATE = 28;
constexpr std::size_t LEN_DIAG = 32;

// ---------------------------------------------------------------------------
// Motion controller state (STATE byte 0)
//
//   DISABLED --ENABLE 1--> IDLE --3 setpoints buffered--> RUNNING
//   IDLE/RUNNING --ENABLE 0--> STOPPING --> DISABLED
//   RUNNING --JUMP / STARVED / driver fault--> STOPPING --> FAULT
//   RUNNING --stall--> FAULT   (immediately, no STOPPING)
//   FAULT --CLEARFAULT--> DISABLED
// ---------------------------------------------------------------------------
enum class McState : uint8_t
{
  DISABLED = 0,  // no motion; CONFIG / SETZERO / HOMING allowed
  IDLE = 1,      // enabled, waiting for setpoints
  RUNNING = 2,   // following setpoints
  STOPPING = 3,  // controlled braking
  FAULT = 4,     // stopped after error, only CLEARFAULT helps
};

// Flags (STATE byte 2)
constexpr uint8_t FL_UNDERRUN = 0x01;  // buffer ran empty while moving, STM extrapolated (info)
constexpr uint8_t FL_OVERRUN = 0x02;   // setpoint dropped, buffer was full
constexpr uint8_t FL_CFG_OK = 0x04;    // valid CONFIG present
constexpr uint8_t FL_JUMP = 0x08;      // setpoint jump > 2.5 * vmax * T + 2 steps -> FAULT
constexpr uint8_t FL_DRV_OK = 0x10;    // TMC drivers initialised (motor voltage VM on)
constexpr uint8_t FL_STARVED = 0x20;   // > 50 ms without setpoints while moving -> FAULT

// Homing status (STATE byte 3)
constexpr uint8_t HS_HOMED_MASK = 0x3F;  // bit 0..5: axis 0..5 homed
constexpr uint8_t HS_ACTIVE = 0x40;      // homing running (set = command accepted)
constexpr uint8_t HS_ERROR = 0x80;       // homing failed or aborted

// ---------------------------------------------------------------------------
// Decoded frames
// ---------------------------------------------------------------------------

// One complete, CRC-checked frame as delivered by FrameParser
struct Frame
{
  uint8_t type = 0;
  uint8_t seq = 0;
  std::vector<uint8_t> payload;
};

// T_STATE payload (28 bytes): state, free, flags, home, 6 x int32 pos
struct StateFrame
{
  McState state = McState::DISABLED;
  uint8_t free = 0;   // free setpoint buffer slots (0..8), ~4 is normal while streaming
  uint8_t flags = 0;  // FL_*
  uint8_t home = 0;   // HS_*
  // Steps output by the STM since SETZERO / power-up (no encoder!).
  // Lags the commanded position by ~3-4 setpoint periods (buffer).
  std::array<int32_t, N_AX> pos{};
};

// T_DIAG payload (32 bytes): 8 x uint32 counters, counting up since power-up
struct DiagFrame
{
  uint32_t ok = 0;    // frames with valid CRC
  uint32_t crc = 0;   // CRC errors
  uint32_t len = 0;   // length byte > 64
  uint32_t tout = 0;  // incomplete frame (> 50 ms gap between bytes)
  uint32_t full = 0;  // STM receive buffer overflow
  uint32_t uerr = 0;  // UART hardware error (overrun, framing, noise)
  uint32_t bad = 0;   // unknown type or wrong payload length
  uint32_t rej = 0;   // valid command, rejected in the current state
};

// ---------------------------------------------------------------------------
// Encoding
// ---------------------------------------------------------------------------

// CRC-16/CCITT-FALSE, see robot_proto.py: crc16()
uint16_t crc16(const uint8_t * data, std::size_t len);

// Complete frame incl. start byte and CRC, see robot_proto.py: frame()
// Example: make_frame(T_ENABLE, payload_enable(true), 0) == A5 01 03 00 01 05 BB
std::vector<uint8_t> make_frame(uint8_t type, const std::vector<uint8_t> & payload, uint8_t seq);

// Payload builders (CLEARFAULT and SETZERO have an empty payload -> no builder needed)
std::vector<uint8_t> payload_setpoint(const std::array<int32_t, N_AX> & steps);
std::vector<uint8_t> payload_config(
  uint16_t period_ms, const std::array<float, N_AX> & vmax, const std::array<float, N_AX> & accel);
std::vector<uint8_t> payload_enable(bool on);
std::vector<uint8_t> payload_homing(const std::array<int32_t, N_AX> & offsets);

// ---------------------------------------------------------------------------
// Decoding
// ---------------------------------------------------------------------------

// Return std::nullopt if the frame has the wrong type or payload length
std::optional<StateFrame> decode_state(const Frame & frame);
std::optional<DiagFrame> decode_diag(const Frame & frame);

// Stream parser for the bytes coming from the serial port, see robot_proto.py: Link._feed()
//
// Usage:  parser.feed(buf, n);  while (auto f = parser.next()) { ... }
class FrameParser
{
public:
  // Append received bytes to the internal buffer
  void feed(const uint8_t * data, std::size_t len);

  // Next complete frame with valid CRC, or std::nullopt if more bytes are needed
  std::optional<Frame> next();

  // Statistics (for logging)
  uint32_t crc_errors() const;  // only counted after the first valid frame (synced)
  uint32_t seq_gaps() const;    // STM seq not previous + 1 -> STM frames were lost

private:
  std::vector<uint8_t> buf_;
  bool synced_ = false;  // garbage before the first valid frame is not counted
  uint32_t crc_errors_ = 0;
  uint32_t seq_gaps_ = 0;
  std::optional<uint8_t> last_seq_;
};

}  // namespace arm_hardware::proto

#endif  // ARM_HARDWARE__STM_PROTOCOL_HPP_
