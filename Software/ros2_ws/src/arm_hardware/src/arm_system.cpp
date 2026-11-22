// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
// See LICENSE-SOFTWARE in the project root for details.

#include "arm_hardware/arm_system.hpp"

#include "pluginlib/class_list_macros.hpp"

// Implementation hints (Jazzy hardware_interface 4.x):
// - Do NOT override export_state_interfaces() / export_command_interfaces() (deprecated).
//   The framework creates the interfaces from the <ros2_control> tag. Access them with
//     set_state("<joint>/position", value)
//     get_command<double>("<joint>/position")
//   Interface name constants: hardware_interface::HW_IF_POSITION / HW_IF_VELOCITY
//   ("hardware_interface/types/hardware_interface_type_values.hpp").
// - Logging: get_logger(), throttled with RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), ms, ...)
// - Parameters are strings: info_.hardware_parameters.at("port"),
//   info_.joints[i].parameters.at("gear_ratio") -> std::stod / std::stoi

namespace arm_hardware
{

// =============================================================================
// Lifecycle
// =============================================================================

hardware_interface::CallbackReturn ArmSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & /*params*/)
{
  // TODO:
  // 1. Call the base class first: SystemInterface::on_init(params), return ERROR if it fails.
  //    After that info_ holds the parsed <ros2_control> tag.
  // 2. Hardware parameters: port, baudrate, period_ms (2..100), set_zero_on_activate
  //    (LATER: homing_offsets)
  // 3. Check the joints: exactly proto::N_AX joints, each with
  //    - one command interface "position"
  //    - state interfaces "position" and "velocity"
  //    -> otherwise log the joint name and return ERROR
  // 4. Per joint parameters -> JointCfg: axis (0..5, each used once), gear_ratio, direction,
  //    max_velocity [rad/s], max_acceleration [rad/s^2]
  // 5. Derived values per joint:
  //    steps_per_rad = proto::STEPS_PER_MOTOR_REV * gear_ratio / (2*pi)
  //    vmax_steps    = 1.2 * max_velocity     * steps_per_rad   (20 % reserve, doc 8.1)
  //    accel_steps   = 1.2 * max_acceleration * steps_per_rad
  //    Check: the STM faults with JUMP if a setpoint step is > 2.5 * vmax * T + 2 steps.
  // No hardware access in on_init!
  return hardware_interface::CallbackReturn::ERROR;  // placeholder: fails until implemented
}

hardware_interface::CallbackReturn ArmSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO:
  // 1. serial_.open(port_, baudrate_), on failure log the port name and return ERROR
  // 2. wait_for(any STATE, 1 s)  -> connection is up (doc 8.1 step 1)
  //    timeout -> "no STATE frames (port? firmware? other program using the port?)"
  // 3. wait_for(flags & FL_DRV_OK, 5 s) -> TMC drivers initialised, VM on (doc 8.1 step 2)
  // 4. Set the state interfaces once from the received position (steps_to_rad),
  //    velocity = 0
  return hardware_interface::CallbackReturn::ERROR;  // placeholder: fails until implemented
}

hardware_interface::CallbackReturn ArmSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: start sequence, assumption for now: arm is "homed" = physically at the URDF zero pose.
  // Every command goes through send_reliable(), success is always checked in the STATE frame.
  //
  // 1. state == FAULT      -> CLEARFAULT, success: state == DISABLED
  // 2. state != DISABLED   -> ENABLE 0,   success: state == DISABLED (via STOPPING)
  // 3. CONFIG(period_ms_, vmax_steps[axis], accel_steps[axis])
  //    success: CFG_OK set AND rej did not increase
  //    (CFG_OK stays set from an older config, so rej is the only proof - doc 8.3)
  // 4. if set_zero_on_activate_: SETZERO, success: all pos == 0
  //    LATER: replace by start_homing()
  // 5. ENABLE 1, success: state == IDLE (needs CFG_OK and DRV_OK)
  // 6. Take the position from a fresh STATE:
  //    - set_state("<joint>/position", ...)
  //    - set_command("<joint>/position", same value)   -> first setpoint == current position,
  //      otherwise the STM sees a jump and faults (JUMP)
  //    - last_sent_steps_ = pos
  //    - prev_state_ = reset (velocity estimate starts again)
  // Any step failing -> log which one and return ERROR.
  return hardware_interface::CallbackReturn::ERROR;  // placeholder: fails until implemented
}

hardware_interface::CallbackReturn ArmSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO:
  // 1. ENABLE 0 via send_reliable(), success: state == DISABLED
  //    (STM brakes all axes synchronously, then DISABLED; also aborts a running homing)
  // 2. Return SUCCESS even if it times out, but log an error (we cannot do more here)
  return hardware_interface::CallbackReturn::ERROR;  // placeholder: fails until implemented
}

hardware_interface::CallbackReturn ArmSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: serial_.close()
  return hardware_interface::CallbackReturn::ERROR;  // placeholder: fails until implemented
}

hardware_interface::CallbackReturn ArmSystem::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO:
  // 1. if the port is open: ENABLE 0 (best effort, short timeout) so the arm stops
  // 2. serial_.close()
  return hardware_interface::CallbackReturn::ERROR;  // placeholder: fails until implemented
}

// =============================================================================
// Control loop (100 Hz, must not block)
// =============================================================================

hardware_interface::return_type ArmSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO:
  // 1. poll_rx() - per cycle 0..2 STATE frames arrive, only the newest one counts
  // 2. No new STATE for a while (e.g. > 100 ms) -> warn throttled (cable? STM reset?)
  // 3. Per joint:
  //    position = steps_to_rad(joint, state.pos[joint.axis])
  //    velocity = difference of the last two STATE positions / difference of their receive
  //               times (prev_state_ / prev_state_time_), 0 if there is no previous frame
  //    -> set_state("<joint>/position", ...), set_state("<joint>/velocity", ...)
  //    Note: pos lags the command by ~3-4 periods (setpoint buffer) -> keep
  //    open_loop_control: true in ros2_controllers.yaml.
  // 4. Log throttled:
  //    - state changes, especially -> FAULT
  //      * RUNNING -> FAULT without STOPPING and without flag = stall
  //      * FAULT via STOPPING without flag = driver error (short circuit, overtemperature)
  //    - flags JUMP, STARVED, OVERRUN (UNDERRUN only as debug/info)
  //    - DIAG counters crc, bad, rej, tout, uerr increasing; parser_.crc_errors(), seq_gaps()
  // 5. For now return OK even on FAULT (only log).
  //    LATER: return ERROR on FAULT -> controller_manager calls on_error / deactivates.
  return hardware_interface::return_type::ERROR;  // placeholder: fails until implemented
}

hardware_interface::return_type ArmSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO:
  // 1. Only send while the STM is IDLE or RUNNING (otherwise the STM only counts rej).
  // 2. Per joint: cmd = get_command<double>("<joint>/position")
  //    - cmd is NaN (no controller active yet, Jazzy initialises commands with NaN)
  //      -> keep last_sent_steps_[axis]
  //    - else steps[axis] = rad_to_steps(joint, cmd)
  // 3. send(T_SETPOINT, payload_setpoint(steps)), last_sent_steps_ = steps
  // Send EVERY cycle, also while standing still:
  //    - the STM keeps ~4 setpoints buffered (free ~ 4) by adjusting its tempo
  //    - a setpoint without motion sets "still" in motion.c, so a gap afterwards does
  //      NOT cause STARVED; a gap > 50 ms during motion does (-> FAULT)
  //    - SETPOINT frame = 30 bytes ~ 2.6 ms at 115200 baud -> ~26 % bus load at 100 Hz
  return hardware_interface::return_type::ERROR;  // placeholder: fails until implemented
}

// =============================================================================
// Helpers
// =============================================================================

void ArmSystem::poll_rx()
{
  // TODO:
  // 1. loop: n = serial_.read_some(buf, sizeof buf) until n == 0 (n < 0 -> log error, stop)
  // 2. parser_.feed(buf, n)
  // 3. while (auto f = parser_.next()):
  //    - T_STATE -> decode_state(); prev_state_ = last_state_ (with time); last_state_ = new,
  //                 last_state_time_ = Clock::now()
  //    - T_DIAG  -> decode_diag(); last_diag_ / last_diag_time_
  //    - other types -> ignore (debug log)
}

bool ArmSystem::wait_for(const StateCondition & /*cond*/, std::chrono::milliseconds /*timeout*/)
{
  // TODO: like Link.wait() in robot_proto.py
  // until timeout: poll_rx(); if last_state_ && cond(*last_state_) return true; sleep ~2 ms
  // Only for the lifecycle callbacks - never call this from read() / write()!
  return false;  // placeholder
}

std::optional<proto::StateFrame> ArmSystem::fresh_state(std::chrono::milliseconds /*timeout*/)
{
  // TODO: like Link.fresh_state() in robot_proto.py
  // t0 = Clock::now() + 50 ms margin (bytes already sent must be on the wire and processed)
  // wait until last_state_time_ > t0, return last_state_; timeout -> std::nullopt
  return std::nullopt;  // placeholder
}

std::optional<proto::DiagFrame> ArmSystem::fresh_diag(std::chrono::milliseconds /*timeout*/)
{
  // TODO: like Link.fresh_diag() in robot_proto.py (DIAG comes only every 100 ms,
  // so use a timeout of at least ~250 ms)
  return std::nullopt;  // placeholder
}

bool ArmSystem::send(uint8_t /*type*/, const std::vector<uint8_t> & /*payload*/)
{
  // TODO:
  // 1. frame = proto::make_frame(type, payload, tx_seq_)
  // 2. ++tx_seq_ (wraps at 255 by itself as uint8_t)
  // 3. return serial_.write(frame)
  return false;  // placeholder
}

bool ArmSystem::send_reliable(
  uint8_t /*type*/, const std::vector<uint8_t> & /*payload*/, const StateCondition & /*success*/,
  std::chrono::milliseconds /*timeout*/, int /*tries*/)
{
  // TODO: protocol doc section 8.3, template: start_homing() in test_codes/Home/home.py
  // for try in 1..tries:
  //   1. rej0 = fresh_diag()->rej
  //   2. send(type, payload)
  //   3. wait_for(success, timeout) -> true: log "accepted (try i)", return true
  //   4. fresh_diag()->rej > rej0  -> rejected (wrong state / drivers not ready):
  //                                   log state + flags, return false
  //   5. otherwise the frame was lost -> next try (repeating is harmless for all commands)
  // all tries used -> log "no answer from STM", return false
  return false;  // placeholder
}

int32_t ArmSystem::rad_to_steps(const JointCfg & /*joint*/, double /*rad*/) const
{
  // TODO: std::lround(joint.direction * rad * joint.steps_per_rad)
  return 0;  // placeholder
}

double ArmSystem::steps_to_rad(const JointCfg & /*joint*/, int32_t /*steps*/) const
{
  // TODO: joint.direction * steps / joint.steps_per_rad
  return 0.0;  // placeholder
}

bool ArmSystem::start_homing()
{
  // LATER (not needed for the first milestone):
  // Preconditions: STM DISABLED and DRV_OK (i.e. called from on_activate before ENABLE 1).
  // 1. send_reliable(T_HOMING, payload_homing(homing_offsets_), home & HS_ACTIVE)
  //    -> accepted when bit 6 is set (a repeated HOMING does not restart it)
  // 2. wait_for(!(home & HS_ACTIVE), long timeout e.g. 120 s)   (robot_proto.py: wait_homed())
  // 3. home & HS_ERROR -> failed / aborted -> return false; else bits 0..5 = homed axes
  // Abort is possible anytime with ENABLE 0.
  // NOTE: Home_Service() in the firmware is still a placeholder -> homing never finishes yet.
  return false;  // placeholder
}

}  // namespace arm_hardware

PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmSystem, hardware_interface::SystemInterface)
