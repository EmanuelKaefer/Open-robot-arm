// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
// See LICENSE-SOFTWARE in the project root for details.

// Unit tests for the protocol layer (no serial port, no ROS).
// Reference values: doc/Software/UART-Protokoll.md section 2 and the self test
// at the top of Software/test_codes/Home/robot_proto.py.
//
// Run:  colcon test --packages-select arm_hardware && colcon test-result --verbose
//
// Every test is skipped until it is written. Remove GTEST_SKIP() when implementing it.

#include <gtest/gtest.h>

#include "arm_hardware/stm_protocol.hpp"

namespace proto = arm_hardware::proto;

TEST(StmProtocol, Crc16CheckValue)
{
  // TODO: crc16 over the 9 ASCII bytes "123456789" must be 0x29B1
  GTEST_SKIP() << "not implemented yet";
}

TEST(StmProtocol, EnableFrameBytes)
{
  // TODO: make_frame(T_ENABLE, payload_enable(true), seq 0) must be exactly
  //       A5 01 03 00 01 05 BB
  GTEST_SKIP() << "not implemented yet";
}

TEST(StmProtocol, PayloadLengths)
{
  // TODO: payload_setpoint -> LEN_SETPOINT (24), payload_config -> LEN_CONFIG (50),
  //       payload_enable -> LEN_ENABLE (1), payload_homing -> LEN_HOMING (24)
  GTEST_SKIP() << "not implemented yet";
}

TEST(StmProtocol, ParserSkipsGarbage)
{
  // TODO: feed some random bytes (also a stray 0xA5) followed by a valid frame
  //       -> next() returns exactly that frame, then std::nullopt
  GTEST_SKIP() << "not implemented yet";
}

TEST(StmProtocol, ParserHandlesSplitInput)
{
  // TODO: feed one valid frame in two parts -> next() returns nullopt after the first
  //       part and the frame after the second part
  GTEST_SKIP() << "not implemented yet";
}

TEST(StmProtocol, ParserDropsBadCrc)
{
  // TODO: flip one payload byte of a valid frame -> next() returns nullopt,
  //       a valid frame fed afterwards is still found (re-synchronisation)
  GTEST_SKIP() << "not implemented yet";
}

TEST(StmProtocol, DecodeStateFrame)
{
  // TODO: build a T_STATE frame by hand (state, free, flags, home, 6 x int32 incl. a
  //       negative value) -> decode_state() returns the same values;
  //       wrong payload length -> std::nullopt
  GTEST_SKIP() << "not implemented yet";
}

TEST(StmProtocol, DecodeDiagFrame)
{
  // TODO: 8 x uint32 in the order ok, crc, len, tout, full, uerr, bad, rej
  GTEST_SKIP() << "not implemented yet";
}
