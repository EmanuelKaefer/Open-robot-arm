// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
// See LICENSE-SOFTWARE in the project root for details.

// ros2_control hardware interface for the robot arm.
//
// Data flow:
//   RViz -> move_group -> arm_controller (JointTrajectoryController, 100 Hz)
//     -> command interface "<joint>/position" [rad]
//     -> ArmSystem::write() -> SETPOINT [steps] -> UART -> STM
//   STM -> STATE every 10 ms -> ArmSystem::read()
//     -> state interfaces "<joint>/position", "<joint>/velocity"
//     -> joint_state_broadcaster -> /joint_states
//
// Lifecycle of the hardware component vs. STM state:
//   unconfigured --on_configure--> inactive   serial port open, STM is DISABLED
//   inactive     --on_activate-->  active     CONFIG, SETZERO (later: HOMING), ENABLE 1 -> IDLE
//   active       --on_deactivate-> inactive   ENABLE 0 -> STOPPING -> DISABLED
// Homing is only possible in STM DISABLED, i.e. while this component is inactive.
//
// Timing: controller_manager update_rate (ros2_controllers.yaml, 100 Hz) and the
// hardware parameter period_ms (10) MUST match - the STM consumes one setpoint per period.

#ifndef ARM_HARDWARE__ARM_SYSTEM_HPP_
#define ARM_HARDWARE__ARM_SYSTEM_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "arm_hardware/serial_port.hpp"
#include "arm_hardware/stm_protocol.hpp"

namespace arm_hardware
{

class ArmSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmSystem)

  // Read and check parameters from the <ros2_control> tag (no hardware access here)
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  // Open the serial port, wait for STATE frames and DRV_OK
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  // Bring the STM from DISABLED to IDLE (CONFIG, SETZERO, ENABLE 1)
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // ENABLE 0 -> STM brakes and goes to DISABLED
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Close the serial port
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  // ENABLE 0 and close the serial port (Ctrl+C, controller_manager shutdown)
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  // Called every control cycle (100 Hz): parse received bytes, update state interfaces
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Called every control cycle (100 Hz): send one SETPOINT with the commanded positions
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using Clock = std::chrono::steady_clock;
  using StateCondition = std::function<bool (const proto::StateFrame &)>;

  // ---------------------------------------------------------------------------
  // Configuration per joint, filled in on_init() from the <joint> parameters
  // ---------------------------------------------------------------------------
  struct JointCfg
  {
    std::string name;          // e.g. "shoulder_joint1"
    std::size_t axis = 0;      // STM axis index 0..5 (= TMC1..TMC6)
    double gear_ratio = 1.0;   // motor revolutions per joint revolution
    int direction = 1;         // +1 or -1 (motor turns the other way than the URDF axis)
    double steps_per_rad = 0;  // STEPS_PER_MOTOR_REV * gear_ratio / (2*pi)
    float vmax_steps = 0;      // CONFIG vmax [steps/s]    (~20 % above the joint limit)
    float accel_steps = 0;     // CONFIG accel [steps/s^2] (~20 % above the joint limit)
  };

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------

  // Read all available bytes from the serial port, feed the parser and store the
  // newest STATE / DIAG frame with its receive time. Used by read() and by the
  // blocking helpers below (in on_activate() etc. read() is not running yet).
  void poll_rx();

  // Wait (blocking, polls poll_rx()) until cond(state) is true for a STATE frame.
  bool wait_for(const StateCondition & cond, std::chrono::milliseconds timeout);

  // First STATE / DIAG frame that was generated surely AFTER everything sent so far
  // (receive time > now + 50 ms margin), see robot_proto.py: fresh_state(), fresh_diag().
  std::optional<proto::StateFrame> fresh_state(std::chrono::milliseconds timeout);
  std::optional<proto::DiagFrame> fresh_diag(std::chrono::milliseconds timeout);

  // Build a frame with the next tx_seq_ and write it to the serial port.
  bool send(uint8_t type, const std::vector<uint8_t> & payload = {});

  // "Send commands safely" (protocol doc section 8.3, home.py: start_homing()):
  // remember rej -> send -> wait for success(state) -> retry if lost, give up if rejected.
  bool send_reliable(
    uint8_t type, const std::vector<uint8_t> & payload, const StateCondition & success,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(200), int tries = 3);

  // Unit conversion joint angle [rad] <-> STM position [steps]
  int32_t rad_to_steps(const JointCfg & joint, double rad) const;
  double steps_to_rad(const JointCfg & joint, int32_t steps) const;

  // LATER: send HOMING with homing_offsets_ and wait until finished (replaces SETZERO).
  bool start_homing();

  // ---------------------------------------------------------------------------
  // Hardware parameters (<hardware><param .../></hardware>)
  // ---------------------------------------------------------------------------
  std::string port_;                  // "port", e.g. /dev/ttyACM0
  int baudrate_ = 115200;             // "baudrate"
  uint16_t period_ms_ = 10;           // "period_ms", must match controller_manager update_rate
  bool set_zero_on_activate_ = true;  // "set_zero_on_activate": assume the arm is at zero pose
  std::array<int32_t, proto::N_AX> homing_offsets_{};  // LATER: "homing_offsets" (offsets.yaml)

  std::vector<JointCfg> joints_;  // in the order of info_.joints

  // ---------------------------------------------------------------------------
  // Communication
  // ---------------------------------------------------------------------------
  SerialPort serial_;
  proto::FrameParser parser_;
  uint8_t tx_seq_ = 0;  // PC -> STM sequence number, +1 per frame

  // Newest frames from the STM with receive time
  std::optional<proto::StateFrame> last_state_;
  Clock::time_point last_state_time_{};
  std::optional<proto::DiagFrame> last_diag_;
  Clock::time_point last_diag_time_{};

  // Previous STATE (for the velocity estimate in read(), the STM sends no velocity)
  std::optional<proto::StateFrame> prev_state_;
  Clock::time_point prev_state_time_{};

  // Last SETPOINT sent (used when the command is NaN, i.e. no controller active yet)
  std::array<int32_t, proto::N_AX> last_sent_steps_{};
};

}  // namespace arm_hardware

#endif  // ARM_HARDWARE__ARM_SYSTEM_HPP_
