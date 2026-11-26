// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
// See LICENSE-SOFTWARE in the project root for details.

// Tests for the ros2_control plugin ArmSystem, without controller_manager and pluginlib.
//
// The test drives the plugin like the controller_manager does: initialize -> configure ->
// activate, then read() / "controller sets commands" / write() every 10 ms.
//
// - ArmSystemInit.*: parameter checks in on_init, always run.
// - SimArmSystem.*:  against the simulated STM, skipped if it is not running:
//     python3 Software/test_codes/stm_sim/stm_sim.py -v        (creates /tmp/ttySTM)
//
// The simulator keeps its state between tests. Every activate() brings it into a known
// state (CLEARFAULT / ENABLE 0 / CONFIG / SETZERO / ENABLE 1), so no reset is needed.

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "arm_hardware/arm_system.hpp"

namespace proto = arm_hardware::proto;
using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;

namespace
{

constexpr const char * SIM_PORT = "/tmp/ttySTM";

const std::array<std::string, proto::N_AX> JOINTS = {
  "shoulder_joint1", "shoulder_joint2", "elbow_joint",
  "wrist_turn_joint", "wrist_tilt_joint", "gripper_turn_joint"};

// Same values for every joint: gear 20, 1 rad/s, 2 rad/s^2
constexpr double GEAR = 20.0;
constexpr double STEPS_PER_RAD = proto::STEPS_PER_MOTOR_REV * GEAR / (2.0 * M_PI);  // 10185.9
constexpr double VMAX_STEPS = 1.2 * 1.0 * STEPS_PER_RAD;                           // 12223
constexpr double MAX_STEP = 122.0;  // floor(VMAX_STEPS * 10 ms), rate limit in write()

int32_t rad_to_steps(double rad) {return static_cast<int32_t>(std::lround(rad * STEPS_PER_RAD));}

// Variations of the <ros2_control> tag for the init tests
struct UrdfOpts
{
  std::string port = SIM_PORT;
  std::string period_ms = "10";
  std::size_t n_joints = proto::N_AX;
  std::array<std::string, proto::N_AX> axis{"0", "1", "2", "3", "4", "5"};
  std::string direction0 = "1";  // direction of the first joint
  bool gear_ratio0 = true;       // false: leave out gear_ratio on the first joint
};

std::string make_urdf(const UrdfOpts & o)
{
  std::string links = "<link name=\"base_link\"/>\n";
  std::string control;
  std::string parent = "base_link";
  for (std::size_t i = 0; i < o.n_joints; ++i) {
    const std::string child = "link" + std::to_string(i + 1);
    links +=
      "<link name=\"" + child + "\"/>\n"
      "<joint name=\"" + JOINTS[i] + "\" type=\"revolute\">"
      "<parent link=\"" + parent + "\"/><child link=\"" + child + "\"/>"
      "<origin xyz=\"0 0 0.1\" rpy=\"0 0 0\"/><axis xyz=\"0 0 1\"/>"
      "<limit lower=\"-3.14\" upper=\"3.14\" effort=\"10\" velocity=\"1.0\"/></joint>\n";
    parent = child;

    control +=
      "<joint name=\"" + JOINTS[i] + "\">"
      "<param name=\"axis\">" + o.axis[i] + "</param>" +
      ((i > 0 || o.gear_ratio0) ? "<param name=\"gear_ratio\">20.0</param>" : "") +
      "<param name=\"direction\">" + (i == 0 ? o.direction0 : "1") + "</param>"
      "<param name=\"max_velocity\">1.0</param>"
      "<param name=\"max_acceleration\">2.0</param>"
      "<command_interface name=\"position\"/>"
      "<state_interface name=\"position\"/><state_interface name=\"velocity\"/>"
      "</joint>\n";
  }
  return
    "<?xml version=\"1.0\"?><robot name=\"arm_test\">\n" + links +
    "<ros2_control name=\"ArmSystem\" type=\"system\"><hardware>"
    "<plugin>arm_hardware/ArmSystem</plugin>"
    "<param name=\"port\">" + o.port + "</param>"
    "<param name=\"baudrate\">115200</param>"
    "<param name=\"period_ms\">" + o.period_ms + "</param>"
    "<param name=\"set_zero_on_activate\">true</param>"
    "</hardware>\n" + control + "</ros2_control></robot>";
}

}  // namespace

// =============================================================================
// Fixture: ArmSystem wrapped in hardware_interface::System, like the resource manager does
// =============================================================================
class ArmSystemTest : public ::testing::Test
{
protected:
  using Clock = std::chrono::steady_clock;

  // Returns the lifecycle state id after initialize (UNCONFIGURED = success)
  uint8_t init(const UrdfOpts & opts = {})
  {
    const auto infos = hardware_interface::parse_control_resources_from_urdf(make_urdf(opts));
    auto impl = std::make_unique<arm_hardware::ArmSystem>();
    arm_ = impl.get();
    sys_ = std::make_unique<hardware_interface::System>(std::move(impl));

    hardware_interface::HardwareComponentParams params;
    params.hardware_info = infos.at(0);
    params.logger = rclcpp::get_logger("test_arm_system");
    params.clock = clock_;
    const uint8_t id = sys_->initialize(params).id();
    if (id == State::PRIMARY_STATE_UNCONFIGURED) {
      for (const auto & s : sys_->export_state_interfaces()) {
        states_[s->get_name()] = s;
      }
      for (const auto & c : sys_->export_command_interfaces()) {
        commands_[c->get_name()] = c;
      }
    }
    return id;
  }

  void TearDown() override
  {
    if (!sys_) {
      return;
    }
    if (sys_->get_lifecycle_state().id() == State::PRIMARY_STATE_ACTIVE) {
      sys_->deactivate();
    }
    if (sys_->get_lifecycle_state().id() == State::PRIMARY_STATE_INACTIVE) {
      sys_->cleanup();
    }
  }

  double position(std::size_t j) const
  {
    return states_.at(JOINTS[j] + "/position")->get_optional<double>().value();
  }
  double command(std::size_t j) const
  {
    return commands_.at(JOINTS[j] + "/position")->get_optional<double>().value();
  }
  void set_command(std::size_t j, double v)
  {
    EXPECT_TRUE(commands_.at(JOINTS[j] + "/position")->set_value(v));
  }

  // One control cycle like the controller_manager: read -> controller -> write
  void cycle(const std::function<void()> & controller = {}, bool do_write = true)
  {
    const rclcpp::Time now = clock_->now();
    ASSERT_EQ(sys_->read(now, rclcpp::Duration(10ms)), hardware_interface::return_type::OK);
    if (controller) {
      controller();
    }
    if (do_write) {
      ASSERT_EQ(sys_->write(now, rclcpp::Duration(10ms)), hardware_interface::return_type::OK);
    }
  }

  // n cycles at 100 Hz, controller(k) is called with k = 1..n
  void run(int n, const std::function<void(int)> & controller = {}, bool do_write = true)
  {
    const auto t0 = Clock::now();
    for (int k = 1; k <= n; ++k) {
      std::this_thread::sleep_until(t0 + k * 10ms);
      cycle(controller ? [&] {controller(k);} : std::function<void()>{}, do_write);
    }
  }

  proto::StateFrame stm() const
  {
    const auto s = arm_->stm_state();
    EXPECT_TRUE(s.has_value());
    return s.value_or(proto::StateFrame{});
  }

  rclcpp::Clock::SharedPtr clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  std::unique_ptr<hardware_interface::System> sys_;
  arm_hardware::ArmSystem * arm_ = nullptr;  // owned by sys_
  std::unordered_map<std::string, hardware_interface::StateInterface::ConstSharedPtr> states_;
  std::unordered_map<std::string, hardware_interface::CommandInterface::SharedPtr> commands_;
};

// =============================================================================
// on_init parameter checks (no simulator needed)
// =============================================================================
using ArmSystemInit = ArmSystemTest;

TEST_F(ArmSystemInit, ValidConfig)
{
  EXPECT_EQ(init(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(states_.size(), 2 * proto::N_AX);
  EXPECT_EQ(commands_.size(), proto::N_AX);
}

TEST_F(ArmSystemInit, RejectsMissingGearRatio)
{
  UrdfOpts o;
  o.gear_ratio0 = false;
  EXPECT_NE(init(o), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(ArmSystemInit, RejectsDuplicateAxis)
{
  UrdfOpts o;
  o.axis[5] = "0";
  EXPECT_NE(init(o), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(ArmSystemInit, RejectsAxisOutOfRange)
{
  UrdfOpts o;
  o.axis[5] = "6";
  EXPECT_NE(init(o), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(ArmSystemInit, RejectsWrongJointCount)
{
  UrdfOpts o;
  o.n_joints = 5;
  EXPECT_NE(init(o), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(ArmSystemInit, RejectsBadPeriod)
{
  UrdfOpts o;
  o.period_ms = "1";
  EXPECT_NE(init(o), State::PRIMARY_STATE_UNCONFIGURED);
  o.period_ms = "ten";
  EXPECT_NE(init(o), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(ArmSystemInit, RejectsBadDirection)
{
  UrdfOpts o;
  o.direction0 = "2";
  EXPECT_NE(init(o), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(ArmSystemInit, ConfigureFailsWithoutPort)
{
  UrdfOpts o;
  o.port = "/tmp/does_not_exist_ttySTM";
  ASSERT_EQ(init(o), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_NE(sys_->configure().id(), State::PRIMARY_STATE_INACTIVE);
}

// =============================================================================
// Against the simulated STM
// =============================================================================
class SimArmSystem : public ArmSystemTest
{
protected:
  void SetUp() override
  {
    if (!std::filesystem::exists(SIM_PORT)) {
      GTEST_SKIP() << SIM_PORT << " not found - start stm_sim.py first";
    }
    ASSERT_EQ(init(), State::PRIMARY_STATE_UNCONFIGURED);
  }

  void configure_and_activate()
  {
    ASSERT_EQ(sys_->configure().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(sys_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  }
};

TEST_F(SimArmSystem, ConfigureReadsPosition)
{
  ASSERT_EQ(sys_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  const auto s = stm();
  EXPECT_EQ(s.state, proto::McState::DISABLED) << "configure must not send commands";
  for (std::size_t j = 0; j < proto::N_AX; ++j) {
    EXPECT_NEAR(position(j), s.pos[j] / STEPS_PER_RAD, 1e-9) << JOINTS[j];
  }
}

TEST_F(SimArmSystem, ActivateReachesIdle)
{
  configure_and_activate();
  const auto s = stm();
  EXPECT_EQ(s.state, proto::McState::IDLE);
  EXPECT_TRUE(s.flags & proto::FL_CFG_OK);
  for (std::size_t j = 0; j < proto::N_AX; ++j) {
    EXPECT_EQ(s.pos[j], 0) << "SETZERO, axis " << j;
    EXPECT_DOUBLE_EQ(position(j), 0.0) << JOINTS[j];
    EXPECT_DOUBLE_EQ(command(j), position(j)) << "command must start at the position";
  }
}

TEST_F(SimArmSystem, StreamRamp)
{
  configure_and_activate();

  // shoulder_joint1: 0 -> 0.2 rad in 2 s (0.1 rad/s), then hold 0.5 s
  constexpr double target = 0.2;
  run(250, [&](int k) {set_command(0, target * std::min(k, 200) / 200.0);});

  const auto s = stm();
  EXPECT_EQ(s.state, proto::McState::RUNNING);
  EXPECT_EQ(s.pos[0], rad_to_steps(target));
  for (std::size_t a = 1; a < proto::N_AX; ++a) {
    EXPECT_EQ(s.pos[a], 0) << "axis " << a << " moved";
  }
  EXPECT_FALSE(s.flags & (proto::FL_JUMP | proto::FL_STARVED | proto::FL_OVERRUN))
    << "flags 0x" << std::hex << int(s.flags);
  EXPECT_NEAR(position(0), target, 1.0 / STEPS_PER_RAD);
}

TEST_F(SimArmSystem, NanCommandHolds)
{
  configure_and_activate();
  for (std::size_t j = 0; j < proto::N_AX; ++j) {
    set_command(j, std::numeric_limits<double>::quiet_NaN());
  }
  run(50);

  const auto s = stm();
  EXPECT_TRUE(s.state == proto::McState::IDLE || s.state == proto::McState::RUNNING)
    << "state " << int(s.state);
  for (std::size_t a = 0; a < proto::N_AX; ++a) {
    EXPECT_EQ(s.pos[a], 0) << "axis " << a;
  }
}

TEST_F(SimArmSystem, RateLimitPreventsJump)
{
  configure_and_activate();

  // 1 rad step in one cycle = 10186 steps, the JUMP limit is ~307 steps
  set_command(1, 1.0);
  run(30);
  auto s = stm();
  ASSERT_NE(s.state, proto::McState::FAULT) << "JUMP fault, rate limit did not work";
  EXPECT_GT(s.pos[1], 0);
  EXPECT_LE(s.pos[1], 30 * MAX_STEP) << "faster than vmax";

  // 1 rad at 122 steps / 10 ms takes ~84 cycles, plus buffer delay
  run(120);
  s = stm();
  EXPECT_NE(s.state, proto::McState::FAULT);
  EXPECT_FALSE(s.flags & proto::FL_JUMP);
  EXPECT_EQ(s.pos[1], rad_to_steps(1.0));
}

TEST_F(SimArmSystem, DeactivateDisables)
{
  configure_and_activate();
  run(50, [&](int k) {set_command(2, 0.002 * k);});  // moving at 0.2 rad/s

  EXPECT_EQ(sys_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(stm().state, proto::McState::DISABLED);
}

TEST_F(SimArmSystem, ReactivateAfterFault)
{
  configure_and_activate();

  // Move at 0.5 rad/s, then stop sending in the middle of the motion -> STARVED -> FAULT
  run(50, [&](int k) {set_command(0, 0.005 * k);});
  run(30, {}, /*do_write=*/ false);
  auto s = stm();
  ASSERT_EQ(s.state, proto::McState::FAULT);
  EXPECT_TRUE(s.flags & proto::FL_STARVED);
  const int32_t pos_fault = s.pos[0];
  EXPECT_GT(pos_fault, 0);

  // Recover like "ros2 control set_hardware_component_state ArmSystem inactive / active"
  ASSERT_EQ(sys_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(sys_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  s = stm();
  EXPECT_EQ(s.state, proto::McState::IDLE);
  EXPECT_EQ(s.pos[0], pos_fault) << "no second SETZERO after a FAULT";
  EXPECT_NEAR(position(0), pos_fault / STEPS_PER_RAD, 1e-9);
  EXPECT_DOUBLE_EQ(command(0), position(0));

  // and it drives again
  run(100, [&](int k) {set_command(0, pos_fault / STEPS_PER_RAD - 0.001 * k);});
  run(20);
  s = stm();
  EXPECT_EQ(s.state, proto::McState::RUNNING);
  EXPECT_EQ(s.pos[0], rad_to_steps(pos_fault / STEPS_PER_RAD - 0.1));
}
