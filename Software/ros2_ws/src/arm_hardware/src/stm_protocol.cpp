// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
// See LICENSE-SOFTWARE in the project root for details.

#include "arm_hardware/stm_protocol.hpp"

#include <cstring>    
#include <stdexcept>  
#include <algorithm>  
#include <cstring> 

// Implementation hints:
// - Little-endian packing: x86 / ARM are little-endian, so std::memcpy of an
//   int32_t / uint32_t / float / uint16_t into the byte vector gives the right layout
//   (same as struct.pack("<...") in robot_proto.py). Needs <cstring>.
// - Keep this file free of ROS includes.

namespace arm_hardware::proto
{

// ---------------------------------------------------------------------------
// Helper funktion (only lokal!!)
// ---------------------------------------------------------------------------
namespace
{
//write
template<typename T>
void append(std::vector<uint8_t> & out, T value)
{
  uint8_t bytes[sizeof(T)];
  std::memcpy(bytes, &value, sizeof(T));
  out.insert(out.end(), bytes, bytes + sizeof(T));
}

//read
template<typename T>
T read_le(const std::vector<uint8_t> & buf, std::size_t offset)
{
  T value;
  std::memcpy(&value, buf.data() + offset, sizeof(T));
  return value;
}
}  // namespace

// ---------------------------------------------------------------------------
// Encoding
// ---------------------------------------------------------------------------

uint16_t crc16(const uint8_t * data, std::size_t len)
{
  uint16_t c = 0xFFFF;                               // Startwert
  for (std::size_t i = 0; i < len; ++i) {
    c ^= static_cast<uint16_t>(data[i] << 8);        // Byte ins obere Halbwort
    for (int k = 0; k < 8; ++k) {                    // 8 Bits nacheinander
      if (c & 0x8000) {
        c = static_cast<uint16_t>((c << 1) ^ 0x1021);  // oberstes Bit war 1 -> Polynom
      } else {
        c = static_cast<uint16_t>(c << 1);
      }
    }
  }
  return c;
}

std::vector<uint8_t> make_frame(uint8_t type, const std::vector<uint8_t> & payload, uint8_t seq)
{
  if (payload.size() > MAX_PAYLOAD) {
    throw std::invalid_argument("payload too long");  
  }

  std::vector<uint8_t> frame;
  frame.reserve(payload.size() + FRAME_OVERHEAD);

  frame.push_back(START_BYTE);                              // A5
  frame.push_back(static_cast<uint8_t>(payload.size()));    // len
  frame.push_back(type);                                    // type
  frame.push_back(seq);                                     // seq
  frame.insert(frame.end(), payload.begin(), payload.end());  // payload

  // CRC 
  uint16_t crc = crc16(frame.data() + 1, frame.size() - 1);
  frame.push_back(static_cast<uint8_t>(crc & 0xFF));  // crc_lo
  frame.push_back(static_cast<uint8_t>(crc >> 8));    // crc_hi
  return frame;
}

std::vector<uint8_t> payload_setpoint(const std::array<int32_t, N_AX> & steps)
{
  std::vector<uint8_t> p;
  p.reserve(LEN_SETPOINT);
  for (int32_t s : steps) {
    append(p, s);  // 6 x 4 Bytes = 24
  }
  return p;
}

std::vector<uint8_t> payload_config(
  uint16_t period_ms, const std::array<float, N_AX> & vmax,
  const std::array<float, N_AX> & accel)
{
  std::vector<uint8_t> p;
  p.reserve(LEN_CONFIG);
  append(p, period_ms);  // 2 Bytes
  for (float v : vmax) {
    append(p, v);        // 6 x 4 Bytes
  }
  for (float a : accel) {
    append(p, a);        // 6 x 4 Bytes  -> zusammen 50
  }
  return p;
}

std::vector<uint8_t> payload_enable(bool on)
{
  return {static_cast<uint8_t>(on ? 1 : 0)};
}

std::vector<uint8_t> payload_homing(const std::array<int32_t, N_AX> & offsets)
{
  return payload_setpoint(offsets);
}

// ---------------------------------------------------------------------------
// Decoding
// ---------------------------------------------------------------------------

std::optional<StateFrame> decode_state(const Frame & frame)
{
  if (frame.type != T_STATE || frame.payload.size() != LEN_STATE) {
    return std::nullopt;  // falscher Typ oder Länge -> nicht auswerten
  }

  const auto & pl = frame.payload;
  StateFrame s;
  s.state = static_cast<McState>(pl[0]);  // Byte 0
  s.free = pl[1];                         // Byte 1
  s.flags = pl[2];                        // Byte 2
  s.home = pl[3];                         // Byte 3
  for (std::size_t i = 0; i < N_AX; ++i) {
    s.pos[i] = read_le<int32_t>(pl, 4 + 4 * i);  // Bytes 4..27
  }
  return s;
}

std::optional<DiagFrame> decode_diag(const Frame & frame)
{
  if (frame.type != T_DIAG || frame.payload.size() != LEN_DIAG) {
    return std::nullopt;
  }

  const auto & pl = frame.payload;
  DiagFrame d;
  d.ok   = read_le<uint32_t>(pl, 0);
  d.crc  = read_le<uint32_t>(pl, 4);
  d.len  = read_le<uint32_t>(pl, 8);
  d.tout = read_le<uint32_t>(pl, 12);
  d.full = read_le<uint32_t>(pl, 16);
  d.uerr = read_le<uint32_t>(pl, 20);
  d.bad  = read_le<uint32_t>(pl, 24);
  d.rej  = read_le<uint32_t>(pl, 28);
  return d;
}

// ---------------------------------------------------------------------------
// FrameParser
// ---------------------------------------------------------------------------

void FrameParser::feed(const uint8_t * data, std::size_t len)
{
{
  buf_.insert(buf_.end(), data, data + len);  // einfach hinten anhängen
}
}

std::optional<Frame> FrameParser::next()
{
  // Schleife statt "goto 1": continue = von vorne suchen
  while (true) {
    // 1. Startbyte suchen
    auto it = std::find(buf_.begin(), buf_.end(), START_BYTE);
    if (it == buf_.end()) {
      buf_.clear();  // kein A5 drin -> alles Müll
      return std::nullopt;
    }

    // 2. alles vor dem Startbyte wegwerfen -> buf_[0] ist jetzt A5
    buf_.erase(buf_.begin(), it);

    // 3. Längenbyte schon da?
    if (buf_.size() < 2) {
      return std::nullopt;  // auf mehr Bytes warten
    }

    // 4. Länge plausibel?
    const std::size_t n = buf_[1];
    if (n > MAX_PAYLOAD) {
      buf_.erase(buf_.begin());  // war kein echter Frameanfang
      continue;
    }

    // 5. ganzer Frame da?
    if (buf_.size() < n + FRAME_OVERHEAD) {
      return std::nullopt;  // auf mehr Bytes warten
    }

    // 6. CRC prüfen: über len, type, seq, payload = buf_[1] .. buf_[n+3]
    const uint16_t crc_rx = static_cast<uint16_t>(buf_[n + 4] | (buf_[n + 5] << 8));
    const uint16_t crc_calc = crc16(buf_.data() + 1, n + 3);
    if (crc_rx != crc_calc) {
      if (synced_) {
        ++crc_errors_;  // Müll vor dem ersten gültigen Frame nicht zählen
      }
      buf_.erase(buf_.begin());  // NUR das A5 wegwerfen, echter Anfang kann drin liegen
      continue;
    }

    // 7. gültiger Frame
    Frame f;
    f.type = buf_[2];
    f.seq = buf_[3];
    f.payload.assign(buf_.begin() + 4, buf_.begin() + 4 + static_cast<std::ptrdiff_t>(n));

    synced_ = true;
    if (last_seq_ && f.seq != static_cast<uint8_t>(*last_seq_ + 1)) {
      ++seq_gaps_;  // STM-Frames verloren gegangen
    }
    last_seq_ = f.seq;

    buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(n + FRAME_OVERHEAD));
    return f;
  }
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
