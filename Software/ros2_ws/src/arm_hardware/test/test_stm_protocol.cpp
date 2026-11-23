// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
// See LICENSE-SOFTWARE in the project root for details.

// Unit tests for the protocol layer (no serial port, no ROS, no simulator).
// Reference values: doc/Software/UART-Protokoll.md section 2 and the self test
// at the top of Software/test_codes/Home/robot_proto.py.
//
// Run:  colcon test --packages-select arm_hardware && colcon test-result --verbose
//  or:  ./build/arm_hardware/test_stm_protocol

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

#include "arm_hardware/stm_protocol.hpp"

namespace proto = arm_hardware::proto;

namespace
{

using Bytes = std::vector<uint8_t>;

// A valid STATE frame: state RUNNING, free 4, flags CFG_OK|DRV_OK, home 0x40,
// positions with a negative value
Bytes make_state_frame(uint8_t seq)
{
  Bytes pl = {static_cast<uint8_t>(proto::McState::RUNNING), 4,
    proto::FL_CFG_OK | proto::FL_DRV_OK, proto::HS_ACTIVE};
  const Bytes pos = proto::payload_setpoint({1000, -2000, 3, 0, -1, 2147483647});
  pl.insert(pl.end(), pos.begin(), pos.end());
  return proto::make_frame(proto::T_STATE, pl, seq);
}

// Feed all bytes, collect every frame the parser returns
std::vector<proto::Frame> parse_all(proto::FrameParser & parser, const Bytes & bytes)
{
  parser.feed(bytes.data(), bytes.size());
  std::vector<proto::Frame> frames;
  while (auto f = parser.next()) {
    frames.push_back(*f);
  }
  return frames;
}

}  // namespace

// ---------------------------------------------------------------------------
// Encoding
// ---------------------------------------------------------------------------

TEST(StmProtocol, Crc16CheckValue)
{
  const char * text = "123456789";
  EXPECT_EQ(proto::crc16(reinterpret_cast<const uint8_t *>(text), 9), 0x29B1);
}

TEST(StmProtocol, EnableFrameBytes)
{
  const Bytes expected = {0xA5, 0x01, 0x03, 0x00, 0x01, 0x05, 0xBB};
  EXPECT_EQ(proto::make_frame(proto::T_ENABLE, proto::payload_enable(true), 0), expected);
}

TEST(StmProtocol, PayloadLengths)
{
  std::array<int32_t, proto::N_AX> steps{};
  std::array<float, proto::N_AX> v{};
  v.fill(1.0f);

  EXPECT_EQ(proto::payload_setpoint(steps).size(), proto::LEN_SETPOINT);
  EXPECT_EQ(proto::payload_config(10, v, v).size(), proto::LEN_CONFIG);
  EXPECT_EQ(proto::payload_enable(false).size(), proto::LEN_ENABLE);
  EXPECT_EQ(proto::payload_homing(steps).size(), proto::LEN_HOMING);

  // whole CONFIG frame: 6 bytes overhead + 50 bytes payload
  EXPECT_EQ(
    proto::make_frame(proto::T_CONFIG, proto::payload_config(10, v, v), 0).size(),
    proto::LEN_CONFIG + proto::FRAME_OVERHEAD);
}

TEST(StmProtocol, ConfigPayloadLayout)
{
  // uint16 period, then 6 x float vmax, then 6 x float accel (all little-endian)
  std::array<float, proto::N_AX> vmax{};
  std::array<float, proto::N_AX> accel{};
  vmax.fill(4000.0f);
  accel.fill(8000.0f);
  const Bytes pl = proto::payload_config(10, vmax, accel);
  ASSERT_EQ(pl.size(), proto::LEN_CONFIG);

  EXPECT_EQ(pl[0], 10);
  EXPECT_EQ(pl[1], 0);
  float f = 0;
  std::memcpy(&f, pl.data() + 2, 4);
  EXPECT_FLOAT_EQ(f, 4000.0f);  // vmax[0]
  std::memcpy(&f, pl.data() + 2 + 24, 4);
  EXPECT_FLOAT_EQ(f, 8000.0f);  // accel[0]
}

// ---------------------------------------------------------------------------
// FrameParser
// ---------------------------------------------------------------------------

TEST(StmProtocol, ParserSkipsGarbage)
{
  const Bytes frame = make_state_frame(7);

  // garbage incl. a stray start byte whose "length" points into the real frame
  Bytes bytes = {0x00, 0x13, 0xA5, 0x05, 0xFF};
  bytes.insert(bytes.end(), frame.begin(), frame.end());

  proto::FrameParser parser;
  const auto frames = parse_all(parser, bytes);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].type, proto::T_STATE);
  EXPECT_EQ(frames[0].seq, 7);
  EXPECT_EQ(frames[0].payload.size(), proto::LEN_STATE);
  EXPECT_FALSE(parser.next().has_value());
  EXPECT_EQ(parser.crc_errors(), 0u);  // garbage before the first valid frame is not counted
}

TEST(StmProtocol, ParserHandlesSplitInput)
{
  const Bytes frame = make_state_frame(1);
  const std::size_t cut = 10;

  proto::FrameParser parser;
  parser.feed(frame.data(), cut);
  EXPECT_FALSE(parser.next().has_value());  // frame not complete yet

  parser.feed(frame.data() + cut, frame.size() - cut);
  const auto f = parser.next();
  ASSERT_TRUE(f.has_value());
  EXPECT_EQ(f->type, proto::T_STATE);
  EXPECT_FALSE(parser.next().has_value());
}

TEST(StmProtocol, ParserDropsBadCrc)
{
  const Bytes good0 = make_state_frame(0);
  Bytes bad1 = make_state_frame(1);
  bad1[10] ^= 0x01;  // flip one payload bit -> CRC no longer matches
  const Bytes good2 = make_state_frame(2);

  Bytes bytes = good0;
  bytes.insert(bytes.end(), bad1.begin(), bad1.end());
  bytes.insert(bytes.end(), good2.begin(), good2.end());

  proto::FrameParser parser;
  const auto frames = parse_all(parser, bytes);
  ASSERT_EQ(frames.size(), 2u);  // the bad frame is dropped, the next one is found again
  EXPECT_EQ(frames[0].seq, 0);
  EXPECT_EQ(frames[1].seq, 2);
  EXPECT_EQ(parser.crc_errors(), 1u);
  EXPECT_EQ(parser.seq_gaps(), 1u);  // seq 0 -> 2: one STM frame lost
}

TEST(StmProtocol, ParserSeqWrapIsNoGap)
{
  Bytes bytes = make_state_frame(255);
  const Bytes next = make_state_frame(0);
  bytes.insert(bytes.end(), next.begin(), next.end());

  proto::FrameParser parser;
  EXPECT_EQ(parse_all(parser, bytes).size(), 2u);
  EXPECT_EQ(parser.seq_gaps(), 0u);
}

// ---------------------------------------------------------------------------
// Decoding
// ---------------------------------------------------------------------------

TEST(StmProtocol, DecodeStateFrame)
{
  const Bytes bytes = make_state_frame(3);
  proto::FrameParser parser;
  const auto frames = parse_all(parser, bytes);
  ASSERT_EQ(frames.size(), 1u);

  const auto s = proto::decode_state(frames[0]);
  ASSERT_TRUE(s.has_value());
  EXPECT_EQ(s->state, proto::McState::RUNNING);
  EXPECT_EQ(s->free, 4);
  EXPECT_EQ(s->flags, proto::FL_CFG_OK | proto::FL_DRV_OK);
  EXPECT_EQ(s->home, proto::HS_ACTIVE);
  const std::array<int32_t, proto::N_AX> expected_pos = {1000, -2000, 3, 0, -1, 2147483647};
  EXPECT_EQ(s->pos, expected_pos);

  // wrong length -> not decoded
  proto::Frame short_frame = frames[0];
  short_frame.payload.pop_back();
  EXPECT_FALSE(proto::decode_state(short_frame).has_value());

  // wrong type -> not decoded
  proto::Frame diag_type = frames[0];
  diag_type.type = proto::T_DIAG;
  EXPECT_FALSE(proto::decode_state(diag_type).has_value());
}

TEST(StmProtocol, DecodeDiagFrame)
{
  proto::Frame f;
  f.type = proto::T_DIAG;
  for (uint32_t v : {1u, 2u, 3u, 4u, 5u, 6u, 7u, 0xDEADBEEFu}) {
    uint8_t b[4];
    std::memcpy(b, &v, 4);
    f.payload.insert(f.payload.end(), b, b + 4);
  }

  const auto d = proto::decode_diag(f);
  ASSERT_TRUE(d.has_value());
  EXPECT_EQ(d->ok, 1u);
  EXPECT_EQ(d->crc, 2u);
  EXPECT_EQ(d->len, 3u);
  EXPECT_EQ(d->tout, 4u);
  EXPECT_EQ(d->full, 5u);
  EXPECT_EQ(d->uerr, 6u);
  EXPECT_EQ(d->bad, 7u);
  EXPECT_EQ(d->rej, 0xDEADBEEFu);

  f.payload.pop_back();
  EXPECT_FALSE(proto::decode_diag(f).has_value());
}
