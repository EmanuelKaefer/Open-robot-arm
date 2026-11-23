// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
// See LICENSE-SOFTWARE in the project root for details.

// Integration tests: SerialPort + stm_protocol against the simulated STM.
//
// The simulator must be running BEFORE the tests start:
//     python3 Software/test_codes/stm_sim/stm_sim.py -v        (creates /tmp/ttySTM)
// Then:
//     colcon test --packages-select arm_hardware && colcon test-result --verbose
//  or ./build/arm_hardware/test_sim_link
//
// Without the simulator every test is skipped (not failed), so `colcon test`
// stays green on machines without it.
//
// The simulator keeps its state between tests (and between test runs!).
// Every test therefore starts with reset_to_disabled() and only uses
// positions relative to the current one.
//
// The helpers poll(), wait_state(), fresh_diag() and send_reliable-like sequences
// are close to what ArmSystem needs later (see arm_system.cpp TODOs).

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "arm_hardware/serial_port.hpp"
#include "arm_hardware/stm_protocol.hpp"

namespace proto = arm_hardware::proto;
using namespace std::chrono_literals;

class SimLink : public ::testing::Test
{
protected:
  using Clock = std::chrono::steady_clock;
  using StateCondition = std::function<bool (const proto::StateFrame &)>;

  static constexpr const char * PORT = "/tmp/ttySTM";
  static constexpr auto MARGIN = 50ms;  // bytes already sent must be on the wire and processed

  void SetUp() override
  {
    if (!std::filesystem::exists(PORT)) {
      GTEST_SKIP() << PORT << " not found - start stm_sim.py first";
    }
    ASSERT_TRUE(port_.open(PORT, 115200));
  }

  void TearDown() override
  {
    port_.close();
  }

  // Read everything available, feed the parser, keep the newest STATE / DIAG
  // with receive time and count the STATE frames.
  void poll()
  {
    uint8_t buf[512];
    ssize_t n;
    while ((n = port_.read_some(buf, sizeof(buf))) > 0) {
      parser_.feed(buf, static_cast<std::size_t>(n));
    }
    ASSERT_GE(n, 0) << "read error on " << PORT;

    while (auto f = parser_.next()) {
      const auto now = Clock::now();
      if (auto s = proto::decode_state(*f)) {
        state_ = *s;
        state_time_ = now;
        ++n_states_;
      } else if (auto d = proto::decode_diag(*f)) {
        diag_ = *d;
        diag_time_ = now;
      }
    }
  }

  // Poll (every ~2 ms) until cond(newest state) is true or timeout.
  bool wait_state(const StateCondition & cond, std::chrono::milliseconds timeout)
  {
    const auto end = Clock::now() + timeout;
    while (Clock::now() < end) {
      poll();
      if (state_ && cond(*state_)) {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    }
    return false;
  }

  // First STATE generated surely after everything sent so far (robot_proto.py: fresh_state)
  std::optional<proto::StateFrame> fresh_state(std::chrono::milliseconds timeout = 300ms)
  {
    const auto t0 = Clock::now() + MARGIN;
    if (!wait_state([&](const proto::StateFrame &) {return state_time_ > t0;}, timeout)) {
      return std::nullopt;
    }
    return state_;
  }

  // First DIAG received after now + margin (DIAG only every 100 ms)
  std::optional<proto::DiagFrame> fresh_diag(std::chrono::milliseconds timeout = 300ms)
  {
    const auto t0 = Clock::now() + MARGIN;
    const auto end = t0 + timeout;
    while (Clock::now() < end) {
      poll();
      if (diag_ && diag_time_ > t0) {
        return diag_;
      }
      std::this_thread::sleep_for(2ms);
    }
    return std::nullopt;
  }

  bool send(uint8_t type, const std::vector<uint8_t> & payload = {})
  {
    return port_.write(proto::make_frame(type, payload, tx_seq_++));
  }

  // Bring the simulator into a known state: CLEARFAULT, ENABLE 0, wait for DISABLED.
  void reset_to_disabled()
  {
    ASSERT_TRUE(send(proto::T_CLEARFAULT));
    ASSERT_TRUE(send(proto::T_ENABLE, proto::payload_enable(false)));
    ASSERT_TRUE(wait_state(
      [](const proto::StateFrame & s) {return s.state == proto::McState::DISABLED;}, 2000ms))
      << "simulator did not reach DISABLED";
    // make sure the flags we look at come from frames after the reset
    ASSERT_TRUE(fresh_state().has_value());
  }

  // Period 10 ms, vmax 4000, accel 8000 for all axes (same as doc section 9)
  void send_default_config()
  {
    std::array<float, proto::N_AX> vmax{};
    std::array<float, proto::N_AX> accel{};
    vmax.fill(4000.0f);
    accel.fill(8000.0f);
    ASSERT_TRUE(send(proto::T_CONFIG, proto::payload_config(10, vmax, accel)));
  }

  // DISABLED -> CONFIG -> ENABLE 1 -> IDLE
  void enable()
  {
    send_default_config();
    ASSERT_TRUE(wait_state(
      [](const proto::StateFrame & s) {return s.flags & proto::FL_CFG_OK;}, 500ms));
    ASSERT_TRUE(send(proto::T_ENABLE, proto::payload_enable(true)));
    ASSERT_TRUE(wait_state(
      [](const proto::StateFrame & s) {return s.state == proto::McState::IDLE;}, 500ms))
      << "ENABLE 1 not accepted (DRV_OK missing? start the sim without --no-drv)";
  }

  arm_hardware::SerialPort port_;
  proto::FrameParser parser_;
  uint8_t tx_seq_ = 0;

  std::optional<proto::StateFrame> state_;
  Clock::time_point state_time_{};
  std::optional<proto::DiagFrame> diag_;
  Clock::time_point diag_time_{};
  int n_states_ = 0;
};

TEST_F(SimLink, ReceivesStateFrames)
{
  const auto end = Clock::now() + 200ms;
  while (Clock::now() < end) {
    poll();
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_GE(n_states_, 5);  // one every 10 ms -> ~20
  EXPECT_EQ(parser_.crc_errors(), 0u);
  EXPECT_EQ(parser_.seq_gaps(), 0u);
}

TEST_F(SimLink, ReceivesDiag)
{
  EXPECT_TRUE(fresh_diag().has_value());
}

TEST_F(SimLink, ConfigAndEnable)
{
  reset_to_disabled();
  const auto d0 = fresh_diag();
  ASSERT_TRUE(d0.has_value());

  enable();

  ASSERT_TRUE(send(proto::T_ENABLE, proto::payload_enable(false)));
  EXPECT_TRUE(wait_state(
    [](const proto::StateFrame & s) {return s.state == proto::McState::DISABLED;}, 1000ms));

  const auto d1 = fresh_diag();
  ASSERT_TRUE(d1.has_value());
  EXPECT_EQ(d1->rej, d0->rej) << "a command was rejected";
  EXPECT_EQ(d1->bad, d0->bad) << "a frame had a wrong type or length";
  EXPECT_EQ(d1->crc, d0->crc) << "the simulator saw CRC errors";
}

TEST_F(SimLink, SetZero)
{
  reset_to_disabled();
  ASSERT_TRUE(send(proto::T_SETZERO));
  EXPECT_TRUE(wait_state(
    [](const proto::StateFrame & s) {
      for (int32_t p : s.pos) {
        if (p != 0) {return false;}
      }
      return true;
    }, 500ms));
}

TEST_F(SimLink, StreamSetpoints)
{
  reset_to_disabled();
  enable();

  const auto start_state = fresh_state();
  ASSERT_TRUE(start_state.has_value());
  std::array<int32_t, proto::N_AX> p = start_state->pos;
  const int32_t start0 = p[0];

  // 300 x +10 steps on axis 0 (1000 steps/s), then 10 standstill setpoints, period 10 ms
  const auto t0 = Clock::now();
  for (int k = 1; k <= 310; ++k) {
    if (k <= 300) {
      p[0] += 10;
    }
    const auto due = t0 + k * 10ms;
    while (Clock::now() < due) {  // keep reading while waiting, the rx buffer must not fill up
      poll();
      std::this_thread::sleep_for(1ms);
    }
    ASSERT_TRUE(send(proto::T_SETPOINT, proto::payload_setpoint(p)));
  }

  // buffer runs empty (~4 periods), then the axis stands at the last setpoint
  ASSERT_TRUE(wait_state(
    [&](const proto::StateFrame & s) {return s.pos[0] == start0 + 3000;}, 1000ms))
    << "pos[0] = " << state_->pos[0] << ", expected " << start0 + 3000;

  const auto s = fresh_state();
  ASSERT_TRUE(s.has_value());
  EXPECT_EQ(s->state, proto::McState::RUNNING);
  EXPECT_FALSE(s->flags & proto::FL_JUMP);
  EXPECT_FALSE(s->flags & proto::FL_STARVED);
  EXPECT_FALSE(s->flags & proto::FL_OVERRUN);
  for (std::size_t a = 1; a < proto::N_AX; ++a) {
    EXPECT_EQ(s->pos[a], start_state->pos[a]) << "axis " << a << " moved";
  }

  ASSERT_TRUE(send(proto::T_ENABLE, proto::payload_enable(false)));
  EXPECT_TRUE(wait_state(
    [](const proto::StateFrame & st) {return st.state == proto::McState::DISABLED;}, 1000ms));
}

TEST_F(SimLink, JumpFaults)
{
  reset_to_disabled();
  enable();

  const auto start_state = fresh_state();
  ASSERT_TRUE(start_state.has_value());
  std::array<int32_t, proto::N_AX> p = start_state->pos;
  p[1] += 5000;  // limit = 2.5 * 4000 * 0.01 + 2 = 102 steps
  ASSERT_TRUE(send(proto::T_SETPOINT, proto::payload_setpoint(p)));

  ASSERT_TRUE(wait_state(
    [](const proto::StateFrame & s) {return s.state == proto::McState::FAULT;}, 1000ms));
  EXPECT_TRUE(state_->flags & proto::FL_JUMP);

  ASSERT_TRUE(send(proto::T_CLEARFAULT));
  ASSERT_TRUE(wait_state(
    [](const proto::StateFrame & s) {return s.state == proto::McState::DISABLED;}, 500ms));
  const auto s = fresh_state();
  ASSERT_TRUE(s.has_value());
  EXPECT_FALSE(s->flags & proto::FL_JUMP);
}
