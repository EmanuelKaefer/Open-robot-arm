// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
// See LICENSE-SOFTWARE in the project root for details.

#include "arm_hardware/stm_protocol.hpp"

// Implementation hints:
// - Little-endian packing: x86 / ARM are little-endian, so std::memcpy of an
//   int32_t / uint32_t / float / uint16_t into the byte vector gives the right layout
//   (same as struct.pack("<...") in robot_proto.py). Needs <cstring>.
// - Keep this file free of ROS includes.

namespace arm_hardware::proto
{

// ---------------------------------------------------------------------------
// Encoding
// ---------------------------------------------------------------------------

uint16_t crc16(const uint8_t * /*data*/, std::size_t /*len*/)
{
  // TODO: port crc16() from robot_proto.py
  // c = 0xFFFF
  // for each byte b:  c ^= b << 8
  //   8 times:  c = (c & 0x8000) ? ((c << 1) ^ 0x1021) : (c << 1)   (keep 16 bit!)
  return 0;  // placeholder
}

std::vector<uint8_t> make_frame(
  uint8_t /*type*/, const std::vector<uint8_t> & /*payload*/, uint8_t /*seq*/)
{
  // TODO: port frame() from robot_proto.py
  // 1. body = [len(payload), type, seq] + payload      (payload must be <= MAX_PAYLOAD)
  // 2. frame = [START_BYTE] + body + crc16(body) as crc_lo, crc_hi
  return {};  // placeholder
}

std::vector<uint8_t> payload_setpoint(const std::array<int32_t, N_AX> & /*steps*/)
{
  // TODO: 6 x int32 LE -> 24 bytes (LEN_SETPOINT)
  return {};  // placeholder
}

std::vector<uint8_t> payload_config(
  uint16_t /*period_ms*/, const std::array<float, N_AX> & /*vmax*/,
  const std::array<float, N_AX> & /*accel*/)
{
  // TODO: uint16 period_ms, 6 x float vmax [steps/s], 6 x float accel [steps/s^2]
  //       -> 2 + 24 + 24 = 50 bytes (LEN_CONFIG), like struct.pack("<H6f6f", ...)
  // STM accepts it only in DISABLED, with period_ms 2..100 and every vmax/accel > 0
  // (also for unused axes).
  return {};  // placeholder
}

std::vector<uint8_t> payload_enable(bool /*on*/)
{
  // TODO: one byte, 1 = enable, 0 = disable
  return {};  // placeholder
}

std::vector<uint8_t> payload_homing(const std::array<int32_t, N_AX> & /*offsets*/)
{
  // TODO: 6 x int32 LE -> 24 bytes (LEN_HOMING), same layout as SETPOINT
  return {};  // placeholder
}

// ---------------------------------------------------------------------------
// Decoding
// ---------------------------------------------------------------------------

std::optional<StateFrame> decode_state(const Frame & /*frame*/)
{
  // TODO:
  // - check type == T_STATE and payload.size() == LEN_STATE
  // - byte 0 state, 1 free, 2 flags, 3 home, bytes 4..27 = 6 x int32 pos
  //   (robot_proto.py: State(t, pl[0], pl[1], pl[2], unpack("<6i", pl[4:]), pl[3]))
  return std::nullopt;  // placeholder
}

std::optional<DiagFrame> decode_diag(const Frame & /*frame*/)
{
  // TODO:
  // - check type == T_DIAG and payload.size() == LEN_DIAG
  // - 8 x uint32 LE in the order ok, crc, len, tout, full, uerr, bad, rej
  return std::nullopt;  // placeholder
}

// ---------------------------------------------------------------------------
// FrameParser
// ---------------------------------------------------------------------------

void FrameParser::feed(const uint8_t * /*data*/, std::size_t /*len*/)
{
  // TODO: append to buf_
}

std::optional<Frame> FrameParser::next()
{
  // TODO: port Link._feed() / _on_frame() from robot_proto.py
  // 1. find START_BYTE in buf_; none -> clear buf_, return nullopt
  // 2. drop everything before it
  // 3. fewer than 2 bytes -> return nullopt (wait for more)
  // 4. n = buf_[1]; n > MAX_PAYLOAD -> drop 1 byte, goto 1
  // 5. fewer than n + FRAME_OVERHEAD bytes -> return nullopt (wait for more)
  // 6. body = buf_[1 .. n+3]; CRC = buf_[n+4] | buf_[n+5] << 8
  //    wrong CRC -> if synced_: ++crc_errors_; drop ONLY the start byte, goto 1
  //    (re-synchronise: the real start byte may be inside the bad "frame")
  // 7. valid: synced_ = true; seq gap check against last_seq_ (++seq_gaps_);
  //    remove n + FRAME_OVERHEAD bytes; return Frame{type, seq, payload}
  return std::nullopt;  // placeholder
}

uint32_t FrameParser::crc_errors() const
{
  return crc_errors_;
}

uint32_t FrameParser::seq_gaps() const
{
  return seq_gaps_;
}

}  // namespace arm_hardware::proto
