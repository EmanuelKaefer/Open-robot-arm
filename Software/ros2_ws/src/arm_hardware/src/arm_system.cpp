// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
// See LICENSE-SOFTWARE in the project root for details.

#include "arm_hardware/arm_system.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

// Status: milestone 5 (README). on_configure connects, on_activate brings the STM to IDLE
// (CONFIG, SETZERO, ENABLE 1), write() streams a SETPOINT every cycle, on_deactivate /
// on_shutdown brake with ENABLE 0. LATER: homing instead of SETZERO, ERROR on FAULT.
//
// Implementation notes (Jazzy hardware_interface 4.x):
// - Do NOT override export_state_interfaces() / export_command_interfaces() (deprecated).
//   The framework creates the interfaces from the <ros2_control> tag. Access them with
//     set_state("<joint>/position", value)
//     get_command<double>("<joint>/position")
// - Parameters are strings: info_.hardware_parameters, info_.joints[i].parameters

namespace arm_hardware
{

using namespace std::chrono_literals;
using hardware_interface::CallbackReturn;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

namespace
{

// The STM sends a STATE frame every 10 ms (its own clock, independent of period_ms)
constexpr double STATE_PERIOD_S = 0.010;

// STATE older than this -> link considered lost (cable? STM reset?)
constexpr auto STATE_TIMEOUT = 100ms;

// Margin for fresh_state() / fresh_diag(): bytes already sent must be on the wire
// and processed by the STM before a frame counts as "fresh"
constexpr auto FRESH_MARGIN = 50ms;

constexpr int LOG_THROTTLE_MS = 1000;

// ---------------------------------------------------------------------------
// Parameter parsing. All functions throw std::runtime_error with a message that
// names the parameter, on_init() catches it and logs it.
// ---------------------------------------------------------------------------
using ParamMap = std::unordered_map<std::string, std::string>;

std::optional<std::string> find_param(const ParamMap & params, const std::string & key)
{
  const auto it = params.find(key);
  if (it == params.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::string require_param(const ParamMap & params, const std::string & key, const std::string & where)
{
  auto v = find_param(params, key);
  if (!v || v->empty()) {
    throw std::runtime_error(where + ": parameter '" + key + "' is missing");
  }
  return *v;
}

double to_double(const std::string & s, const std::string & what)
{
  try {
    std::size_t idx = 0;
    const double v = std::stod(s, &idx);
    if (idx != s.size() || !std::isfinite(v)) {
      throw std::invalid_argument(s);
    }
    return v;
  } catch (const std::logic_error &) {  // invalid_argument, out_of_range
    throw std::runtime_error(what + ": '" + s + "' is not a number");
  }
}

int to_int(const std::string & s, const std::string & what)
{
  try {
    std::size_t idx = 0;
    const int v = std::stoi(s, &idx);
    if (idx != s.size()) {
      throw std::invalid_argument(s);
    }
    return v;
  } catch (const std::logic_error &) {
    throw std::runtime_error(what + ": '" + s + "' is not an integer");
  }
}

bool to_bool(std::string s, const std::string & what)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
  if (s == "true" || s == "1") {
    return true;
  }
  if (s == "false" || s == "0") {
    return false;
  }
  throw std::runtime_error(what + ": '" + s + "' is not true/false");
}

const char * state_name(proto::McState s)
{
  switch (s) {
    case proto::McState::DISABLED: return "DISABLED";
    case proto::McState::IDLE: return "IDLE";
    case proto::McState::RUNNING: return "RUNNING";
    case proto::McState::STOPPING: return "STOPPING";
    case proto::McState::FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

std::string cmd_name(uint8_t type, const std::vector<uint8_t> & payload)
{
  switch (type) {
    case proto::T_SETPOINT: return "SETPOINT";
    case proto::T_CONFIG: return "CONFIG";
    case proto::T_ENABLE: return (!payload.empty() && payload[0]) ? "ENABLE 1" : "ENABLE 0";
    case proto::T_CLEARFAULT: return "CLEARFAULT";
    case proto::T_SETZERO: return "SETZERO";
    case proto::T_HOMING: return "HOMING";
  }
  return "type 0x" + std::to_string(type);
}

bool is_state(const proto::StateFrame & s, proto::McState st) {return s.state == st;}

// Arm is not moving and will not move by itself (target of ENABLE 0 in every state)
bool is_stopped(const proto::StateFrame & s)
{
  return s.state == proto::McState::DISABLED || s.state == proto::McState::FAULT;
}

bool has_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces, const std::string & name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&](const hardware_interface::InterfaceInfo & i) {return i.name == name;});
}

}  // namespace

// =============================================================================
// Lifecycle
// =============================================================================

CallbackReturn ArmSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  // Base class first: parses the <ros2_control> tag into info_ and creates the interfaces
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    // ---- hardware parameters ------------------------------------------------
    const auto & hw = info_.hardware_parameters;
    const std::string where = "hardware '" + info_.name + "'";

    port_ = require_param(hw, "port", where);
    baudrate_ = to_int(find_param(hw, "baudrate").value_or("115200"), where + " baudrate");

    const int period = to_int(find_param(hw, "period_ms").value_or("10"), where + " period_ms");
    if (period < 2 || period > 100) {
      throw std::runtime_error(where + ": period_ms = " + std::to_string(period) +
                               " is out of range 2..100");
    }
    period_ms_ = static_cast<uint16_t>(period);

    // The STM consumes one setpoint per period_ms, write() sends one per control cycle.
    // rw_rate is filled in by the controller_manager (its update_rate), 0 in plain tests.
    if (info_.rw_rate > 0 && info_.rw_rate * period_ms_ != 1000) {
      throw std::runtime_error(
        where + ": period_ms = " + std::to_string(period_ms_) + " does not match the " +
        "controller_manager update_rate of " + std::to_string(info_.rw_rate) +
        " Hz (must be 1000 / period_ms)");
    }

    set_zero_on_activate_ = to_bool(
      find_param(hw, "set_zero_on_activate").value_or("true"), where + " set_zero_on_activate");

    // ---- joints ------------------------------------------------------------
    if (info_.joints.size() != proto::N_AX) {
      throw std::runtime_error(
        where + ": expected " + std::to_string(proto::N_AX) + " joints, got " +
        std::to_string(info_.joints.size()));
    }

    joints_.clear();
    std::array<bool, proto::N_AX> axis_used{};

    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      const auto & ji = info_.joints[i];
      const std::string jwhere = "joint '" + ji.name + "'";

      // Interfaces: command position, state position + velocity
      if (ji.command_interfaces.size() != 1 || ji.command_interfaces[0].name != HW_IF_POSITION) {
        throw std::runtime_error(jwhere + ": needs exactly one command interface 'position'");
      }
      if (ji.state_interfaces.size() != 2 ||
        !has_interface(ji.state_interfaces, HW_IF_POSITION) ||
        !has_interface(ji.state_interfaces, HW_IF_VELOCITY))
      {
        throw std::runtime_error(jwhere + ": needs state interfaces 'position' and 'velocity'");
      }

      JointCfg cfg;
      cfg.name = ji.name;

      const int axis = to_int(
        find_param(ji.parameters, "axis").value_or(std::to_string(i)), jwhere + " axis");
      if (axis < 0 || axis >= static_cast<int>(proto::N_AX)) {
        throw std::runtime_error(jwhere + ": axis " + std::to_string(axis) + " is out of range 0..5");
      }
      if (axis_used[static_cast<std::size_t>(axis)]) {
        throw std::runtime_error(jwhere + ": axis " + std::to_string(axis) + " is used twice");
      }
      axis_used[static_cast<std::size_t>(axis)] = true;
      cfg.axis = static_cast<std::size_t>(axis);

      cfg.direction = to_int(find_param(ji.parameters, "direction").value_or("1"),
                             jwhere + " direction");
      if (cfg.direction != 1 && cfg.direction != -1) {
        throw std::runtime_error(jwhere + ": direction must be 1 or -1");
      }

      // No defaults on purpose: these must be measured / taken from joint_limits.yaml
      cfg.gear_ratio = to_double(require_param(ji.parameters, "gear_ratio", jwhere),
                                 jwhere + " gear_ratio");
      const double max_vel = to_double(require_param(ji.parameters, "max_velocity", jwhere),
                                       jwhere + " max_velocity");
      const double max_acc = to_double(require_param(ji.parameters, "max_acceleration", jwhere),
                                       jwhere + " max_acceleration");
      if (cfg.gear_ratio <= 0.0 || max_vel <= 0.0 || max_acc <= 0.0) {
        throw std::runtime_error(jwhere + ": gear_ratio, max_velocity, max_acceleration must be > 0");
      }

      // Derived values. 20 % reserve above the joint limit (doc 8.1). The STM faults with
      // JUMP if a setpoint step is > 2.5 * vmax * T + 2 steps, the trajectory controller
      // never exceeds max_velocity -> step <= max_vel * T < 2.5 * 1.2 * max_vel * T.
      cfg.steps_per_rad = proto::STEPS_PER_MOTOR_REV * cfg.gear_ratio / (2.0 * M_PI);
      cfg.vmax_steps = static_cast<float>(1.2 * max_vel * cfg.steps_per_rad);
      cfg.accel_steps = static_cast<float>(1.2 * max_acc * cfg.steps_per_rad);

      joints_.push_back(cfg);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "on_init: %s", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_logger(), "port %s, %d baud, period %u ms, set_zero_on_activate %s",
    port_.c_str(), baudrate_, period_ms_, set_zero_on_activate_ ? "true" : "false");
  for (const auto & j : joints_) {
    RCLCPP_INFO(
      get_logger(),
      "  %-20s axis %zu, gear %.2f, dir %+d, %.1f steps/rad, vmax %.0f steps/s, accel %.0f steps/s^2",
      j.name.c_str(), j.axis, j.gear_ratio, j.direction, j.steps_per_rad,
      static_cast<double>(j.vmax_steps), static_cast<double>(j.accel_steps));
  }

  // No hardware access in on_init!
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!serial_.open(port_, baudrate_)) {
    RCLCPP_ERROR(get_logger(), "cannot open serial port %s (reason: see stderr above)",
                 port_.c_str());
    return CallbackReturn::ERROR;
  }

  // Start with a clean receive side (also after cleanup -> configure again)
  parser_ = proto::FrameParser{};
  last_state_.reset();
  last_diag_.reset();
  prev_state_.reset();
  logged_state_.reset();
  logged_flags_ = 0;
  logged_diag_.reset();
  logged_crc_errors_ = 0;
  logged_seq_gaps_ = 0;
  zeroed_ = false;

  // 1. Connection is up (doc 8.1 step 1)
  if (!wait_for([](const proto::StateFrame &) {return true;}, 1000ms)) {
    RCLCPP_ERROR(
      get_logger(),
      "no STATE frames on %s (port? firmware? other program using the port?)", port_.c_str());
    serial_.close();
    return CallbackReturn::ERROR;
  }

  // 2. TMC drivers initialised, VM on (doc 8.1 step 2)
  if (!wait_for([](const proto::StateFrame & s) {return s.flags & proto::FL_DRV_OK;}, 5000ms)) {
    RCLCPP_ERROR(
      get_logger(), "TMC drivers not ready, DRV_OK missing (motor voltage VM on?), flags 0x%02x",
      last_state_->flags);
    serial_.close();
    return CallbackReturn::ERROR;
  }

  // 3. Initial state interfaces from the received position
  const auto & s = *last_state_;
  for (const auto & j : joints_) {
    set_state(j.name + "/" + HW_IF_POSITION, steps_to_rad(j, s.pos[j.axis]));
    set_state(j.name + "/" + HW_IF_VELOCITY, 0.0);
  }
  last_sent_steps_ = s.pos;

  RCLCPP_INFO(
    get_logger(), "connected to STM on %s: state %s, flags 0x%02x, home 0x%02x",
    port_.c_str(), state_name(s.state), s.flags, s.home);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Start sequence (doc 8.1 / 8.5). Assumption until homing exists: at the first activation
  // after on_configure the arm is physically at the URDF zero pose (set_zero_on_activate).
  // Every command goes through send_reliable(), success is always checked in the STATE frame.
  const auto fail = [this](const char * step) {
      RCLCPP_ERROR(get_logger(), "on_activate failed at step: %s", step);
      disable_best_effort(500ms);  // do not leave the STM in IDLE
      return CallbackReturn::ERROR;
    };
  const auto disabled = [](const proto::StateFrame & s) {
      return is_state(s, proto::McState::DISABLED);
    };

  if (!fresh_state(500ms)) {
    return fail("waiting for a fresh STATE");
  }

  // 1. A running STOPPING ends in DISABLED or (after JUMP / STARVED) in FAULT -> wait for it
  if (is_state(*last_state_, proto::McState::STOPPING) &&
    !wait_for([](const proto::StateFrame & s) {return !is_state(s, proto::McState::STOPPING);},
    2000ms))
  {
    return fail("waiting for STOPPING to finish");
  }

  // 2. FAULT -> CLEARFAULT -> DISABLED (the position is kept)
  if (is_state(*last_state_, proto::McState::FAULT)) {
    RCLCPP_WARN(
      get_logger(), "STM is in FAULT (flags 0x%02x), sending CLEARFAULT", last_state_->flags);
    if (!send_reliable(proto::T_CLEARFAULT, {}, disabled)) {
      return fail("CLEARFAULT");
    }
  }

  // 3. IDLE / RUNNING (e.g. left over from an earlier session) -> ENABLE 0 -> DISABLED
  if (!is_state(*last_state_, proto::McState::DISABLED) &&
    !send_reliable(proto::T_ENABLE, proto::payload_enable(false), disabled, 2000ms))
  {
    return fail("ENABLE 0");
  }

  // 4. CONFIG (only accepted in DISABLED)
  if (!send_config()) {
    return fail("CONFIG");
  }

  // 5. SETZERO, only once per on_configure: after a FAULT the arm is NOT at the zero pose,
  //    the STM position is still the best we know. LATER: start_homing() instead.
  if (set_zero_on_activate_ && !zeroed_) {
    const auto all_zero = [](const proto::StateFrame & s) {
        return std::all_of(s.pos.begin(), s.pos.end(), [](int32_t p) {return p == 0;});
      };
    if (!send_reliable(proto::T_SETZERO, {}, all_zero)) {
      return fail("SETZERO");
    }
    zeroed_ = true;
  } else if (set_zero_on_activate_) {
    RCLCPP_INFO(get_logger(), "already zeroed since on_configure, keeping the STM position");
  }

  // 6. ENABLE 1 -> IDLE (needs CFG_OK and DRV_OK)
  if (!send_reliable(
      proto::T_ENABLE, proto::payload_enable(true),
      [](const proto::StateFrame & s) {return is_state(s, proto::McState::IDLE);}))
  {
    return fail("ENABLE 1");
  }

  // 7. Start from the current position: first setpoint == current position, otherwise the
  //    STM sees a jump (JUMP -> FAULT). Controllers that start later read these values.
  const auto s = fresh_state(500ms);
  if (!s) {
    return fail("waiting for a fresh STATE after ENABLE 1");
  }
  for (const auto & j : joints_) {
    const double pos = steps_to_rad(j, s->pos[j.axis]);
    set_state(j.name + "/" + HW_IF_POSITION, pos);
    set_state(j.name + "/" + HW_IF_VELOCITY, 0.0);
    set_command(j.name + "/" + HW_IF_POSITION, pos);
  }
  last_sent_steps_ = s->pos;
  prev_state_.reset();  // velocity estimate starts again

  RCLCPP_INFO(get_logger(), "active: STM is IDLE, streaming setpoints");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // ENABLE 0: the STM brakes all axes, then DISABLED (also aborts a running homing).
  // In FAULT the arm already stands, ENABLE 0 does not change that state.
  if (last_state_ && is_stopped(*last_state_)) {
    return CallbackReturn::SUCCESS;
  }
  // Send ENABLE 0 AT ONCE, not via send_reliable(): write() is no longer called, and while
  // moving the STM goes STARVED -> FAULT after 50 ms without setpoints. send_reliable()
  // first waits up to ~150 ms for a fresh DIAG - too long. It is only the fallback.
  bool stopped = send(proto::T_ENABLE, proto::payload_enable(false)) &&
    wait_for(is_stopped, 2000ms);
  if (!stopped) {
    stopped = send_reliable(proto::T_ENABLE, proto::payload_enable(false), is_stopped, 2000ms);
  }
  if (!stopped) {
    // Nothing more we can do here, but it must be visible
    RCLCPP_ERROR(get_logger(), "STM did not confirm ENABLE 0 - check the arm!");
  } else if (is_state(*last_state_, proto::McState::FAULT)) {
    RCLCPP_WARN(get_logger(), "deactivated, but the STM ended in FAULT (flags 0x%02x)",
                last_state_->flags);
  } else {
    RCLCPP_INFO(get_logger(), "deactivated: STM is DISABLED");
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmSystem::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  serial_.close();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmSystem::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Ctrl+C / controller_manager shutdown: stop the arm (best effort, short timeout)
  if (serial_.is_open() && !disable_best_effort(500ms)) {
    RCLCPP_WARN(get_logger(), "shutdown: STM did not confirm ENABLE 0");
  }
  serial_.close();
  return CallbackReturn::SUCCESS;
}

// =============================================================================
// Control loop (100 Hz, must not block)
// =============================================================================

hardware_interface::return_type ArmSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Per cycle 0..2 STATE frames arrive, only the newest one counts
  poll_rx();

  if (!last_state_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), LOG_THROTTLE_MS, "no STATE frame yet");
    return hardware_interface::return_type::OK;
  }

  const auto age = Clock::now() - last_state_time_;
  if (age > STATE_TIMEOUT) {
    // Keep the old position, it is the best we know. Velocity unknown -> 0.
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), LOG_THROTTLE_MS,
      "no STATE frame for %lld ms (cable? STM reset?)",
      static_cast<long long>(std::chrono::duration_cast<std::chrono::milliseconds>(age).count()));
    for (const auto & j : joints_) {
      set_state(j.name + "/" + HW_IF_VELOCITY, 0.0);
    }
    return hardware_interface::return_type::OK;
  }

  // Velocity estimate (the STM sends no velocity): difference of the last two STATE
  // positions. The STM sends STATE exactly every 10 ms on its own clock, the receive
  // times on the PC jitter (often 2 frames arrive in one read). So dt is the receive time
  // difference rounded to whole STATE periods (>= 1); one lost frame -> 2 periods.
  double dt = 0.0;
  if (prev_state_) {
    const double recv_dt =
      std::chrono::duration<double>(last_state_time_ - prev_state_time_).count();
    dt = std::max(1.0, std::round(recv_dt / STATE_PERIOD_S)) * STATE_PERIOD_S;
  }

  // Note: pos lags the command by ~3-4 periods (setpoint buffer) -> keep
  // open_loop_control: true in ros2_controllers.yaml.
  const auto & s = *last_state_;
  for (const auto & j : joints_) {
    const double pos = steps_to_rad(j, s.pos[j.axis]);
    const double vel = prev_state_ ? (pos - steps_to_rad(j, prev_state_->pos[j.axis])) / dt : 0.0;
    set_state(j.name + "/" + HW_IF_POSITION, pos);
    set_state(j.name + "/" + HW_IF_VELOCITY, vel);
  }

  log_changes();

  // For now OK even on FAULT (only logged).
  // LATER: return ERROR on FAULT -> controller_manager calls on_error / deactivates.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Send EVERY cycle, also while standing still (doc 8.2):
  //    - the STM keeps ~4 setpoints buffered (free ~ 4) by adjusting its tempo
  //    - a setpoint without motion sets "still" in motion.c, so a gap afterwards does
  //      NOT cause STARVED; a gap > 50 ms during motion does (-> FAULT)
  //    - SETPOINT frame = 30 bytes ~ 2.6 ms at 115200 baud -> ~26 % bus load at 100 Hz

  // Only IDLE / RUNNING accept setpoints, otherwise the STM only counts rej.
  // FAULT is logged in read(); recover with deactivate + activate.
  if (!last_state_ ||
    !(is_state(*last_state_, proto::McState::IDLE) ||
    is_state(*last_state_, proto::McState::RUNNING)))
  {
    return hardware_interface::return_type::OK;
  }

  const double period_s = period_ms_ / 1000.0;
  std::array<int32_t, proto::N_AX> steps = last_sent_steps_;

  for (const auto & j : joints_) {
    const double prev = static_cast<double>(last_sent_steps_[j.axis]);
    const double cmd = get_command<double>(j.name + "/" + HW_IF_POSITION);

    // NaN (no controller active yet, Jazzy initialises commands with NaN) or Inf -> hold
    const double target = std::isfinite(cmd) ? j.direction * cmd * j.steps_per_rad : prev;

    // Rate limit: at most the STM vmax per period. The STM faults with JUMP above
    // 2.5 * vmax * T + 2 steps, so a wrong command (step input, bad trajectory) only
    // makes the axis move at vmax instead of stopping the whole arm. Computed in double,
    // so a huge command cannot overflow int32.
    const double max_step = std::max(1.0, std::floor(j.vmax_steps * period_s));
    const double limited = std::clamp(target, prev - max_step, prev + max_step);
    if (limited != target) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), LOG_THROTTLE_MS,
        "command on %s faster than vmax (%.0f steps/s), limited", j.name.c_str(),
        static_cast<double>(j.vmax_steps));
    }
    steps[j.axis] = static_cast<int32_t>(std::lround(limited));
  }

  if (!send(proto::T_SETPOINT, proto::payload_setpoint(steps))) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), LOG_THROTTLE_MS,
      "SETPOINT write failed on %s (output buffer full? device unplugged?)", port_.c_str());
    return hardware_interface::return_type::OK;
  }
  last_sent_steps_ = steps;
  return hardware_interface::return_type::OK;
}

// =============================================================================
// Helpers
// =============================================================================

void ArmSystem::poll_rx()
{
  uint8_t buf[512];
  ssize_t n;
  while ((n = serial_.read_some(buf, sizeof(buf))) > 0) {
    parser_.feed(buf, static_cast<std::size_t>(n));
  }
  if (n < 0) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), LOG_THROTTLE_MS,
      "read error on %s (device unplugged?)", port_.c_str());
  }

  while (auto f = parser_.next()) {
    const auto now = Clock::now();
    if (f->type == proto::T_STATE) {
      if (auto s = proto::decode_state(*f)) {
        prev_state_ = last_state_;
        prev_state_time_ = last_state_time_;
        last_state_ = *s;
        last_state_time_ = now;
      }
    } else if (f->type == proto::T_DIAG) {
      if (auto d = proto::decode_diag(*f)) {
        last_diag_ = *d;
        last_diag_time_ = now;
      }
    } else {
      RCLCPP_DEBUG(get_logger(), "ignoring frame type 0x%02x", f->type);
    }
  }
}

void ArmSystem::log_changes()
{
  const auto & s = *last_state_;

  // ---- state changes ---------------------------------------------------------
  if (!logged_state_ || *logged_state_ != s.state) {
    const char * from = logged_state_ ? state_name(*logged_state_) : "-";
    if (s.state == proto::McState::FAULT) {
      constexpr uint8_t fault_flags = proto::FL_JUMP | proto::FL_STARVED;
      const char * hint = "";
      if (!(s.flags & fault_flags)) {
        hint = (logged_state_ == proto::McState::STOPPING) ?
          " (via STOPPING without flag = driver error: short circuit, overtemperature?)" :
          " (without STOPPING and without flag = stall)";
      }
      RCLCPP_ERROR(
        get_logger(), "STM state %s -> FAULT, flags 0x%02x%s", from, s.flags, hint);
    } else {
      RCLCPP_INFO(get_logger(), "STM state %s -> %s", from, state_name(s.state));
    }
    logged_state_ = s.state;
  }

  // ---- flags that just got set ------------------------------------------------
  const uint8_t new_flags = static_cast<uint8_t>(s.flags & ~logged_flags_);
  if (new_flags & proto::FL_JUMP) {
    RCLCPP_WARN(get_logger(), "flag JUMP: setpoint step too large -> FAULT");
  }
  if (new_flags & proto::FL_STARVED) {
    RCLCPP_WARN(get_logger(), "flag STARVED: > 50 ms without setpoints while moving -> FAULT");
  }
  if (new_flags & proto::FL_OVERRUN) {
    RCLCPP_WARN(get_logger(), "flag OVERRUN: setpoint dropped, STM buffer was full");
  }
  if (new_flags & proto::FL_UNDERRUN) {
    RCLCPP_DEBUG(get_logger(), "flag UNDERRUN: STM buffer ran empty, extrapolated");
  }
  if ((logged_flags_ & proto::FL_DRV_OK) && !(s.flags & proto::FL_DRV_OK)) {
    RCLCPP_WARN(get_logger(), "flag DRV_OK lost: TMC drivers not ready (motor voltage?)");
  }
  logged_flags_ = s.flags;

  // ---- error counters ----------------------------------------------------------
  if (last_diag_) {
    const auto & d = *last_diag_;
    if (logged_diag_) {
      const auto & o = *logged_diag_;
      if (d.crc > o.crc || d.bad > o.bad || d.rej > o.rej || d.tout > o.tout || d.uerr > o.uerr) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), LOG_THROTTLE_MS,
          "STM rx errors: crc %u, bad %u, rej %u, tout %u, uerr %u",
          d.crc, d.bad, d.rej, d.tout, d.uerr);
      }
    }
    logged_diag_ = d;  // first DIAG: counters since power-up are only the baseline
  }

  const uint32_t crc = parser_.crc_errors();
  const uint32_t gaps = parser_.seq_gaps();
  if (crc > logged_crc_errors_ || gaps > logged_seq_gaps_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), LOG_THROTTLE_MS,
      "PC rx errors: crc %u, seq gaps %u (lost STM frames)", crc, gaps);
    logged_crc_errors_ = crc;
    logged_seq_gaps_ = gaps;
  }
}

bool ArmSystem::wait_for(const StateCondition & cond, std::chrono::milliseconds timeout)
{
  // Only for the lifecycle callbacks - never call this from read() / write()!
  const auto end = Clock::now() + timeout;
  while (Clock::now() < end) {
    poll_rx();
    if (last_state_ && cond(*last_state_)) {
      return true;
    }
    std::this_thread::sleep_for(2ms);
  }
  return false;
}

std::optional<proto::StateFrame> ArmSystem::fresh_state(std::chrono::milliseconds timeout)
{
  const auto t0 = Clock::now() + FRESH_MARGIN;
  if (!wait_for([&](const proto::StateFrame &) {return last_state_time_ > t0;}, timeout)) {
    return std::nullopt;
  }
  return last_state_;
}

std::optional<proto::DiagFrame> ArmSystem::fresh_diag(std::chrono::milliseconds timeout)
{
  // DIAG comes only every 100 ms -> use a timeout of at least ~250 ms
  const auto t0 = Clock::now() + FRESH_MARGIN;
  const auto end = t0 + timeout;
  while (Clock::now() < end) {
    poll_rx();
    if (last_diag_ && last_diag_time_ > t0) {
      return last_diag_;
    }
    std::this_thread::sleep_for(2ms);
  }
  return std::nullopt;
}

bool ArmSystem::send(uint8_t type, const std::vector<uint8_t> & payload)
{
  // tx_seq_ wraps at 255 by itself (uint8_t)
  return serial_.write(proto::make_frame(type, payload, tx_seq_++));
}

bool ArmSystem::send_reliable(
  uint8_t type, const std::vector<uint8_t> & payload, const StateCondition & success,
  std::chrono::milliseconds timeout, int tries)
{
  // Protocol doc section 8.3, template: start_homing() in test_codes/Home/home.py.
  // The STM sends no acknowledgement: success is seen in STATE, rejection in DIAG rej.
  const std::string name = cmd_name(type, payload);

  for (int i = 1; i <= tries; ++i) {
    const auto d0 = fresh_diag(300ms);
    if (!d0) {
      RCLCPP_ERROR(get_logger(), "%s: no DIAG frame from the STM", name.c_str());
      return false;
    }
    if (!send(type, payload)) {
      RCLCPP_ERROR(get_logger(), "%s: serial write failed", name.c_str());
      return false;
    }
    if (wait_for(success, timeout)) {
      RCLCPP_INFO(get_logger(), "%s accepted (try %d)", name.c_str(), i);
      return true;
    }
    const auto d1 = fresh_diag(300ms);
    if (d1 && d1->rej > d0->rej) {
      // Wrong state or drivers not ready
      RCLCPP_ERROR(
        get_logger(), "%s rejected by the STM: state %s, flags 0x%02x", name.c_str(),
        state_name(last_state_->state), last_state_->flags);
      return false;
    }
    // A slow transition (e.g. STOPPING) may have finished while waiting for DIAG
    if (last_state_ && success(*last_state_)) {
      RCLCPP_INFO(get_logger(), "%s accepted (try %d)", name.c_str(), i);
      return true;
    }
    // Frame was lost -> repeat (harmless for all commands)
    RCLCPP_WARN(get_logger(), "%s: no answer (try %d/%d)", name.c_str(), i, tries);
  }
  RCLCPP_ERROR(get_logger(), "%s: no answer from the STM after %d tries", name.c_str(), tries);
  return false;
}

bool ArmSystem::send_config()
{
  std::array<float, proto::N_AX> vmax{};
  std::array<float, proto::N_AX> accel{};
  for (const auto & j : joints_) {
    vmax[j.axis] = j.vmax_steps;
    accel[j.axis] = j.accel_steps;
  }
  const auto payload = proto::payload_config(period_ms_, vmax, accel);

  constexpr int tries = 3;
  for (int i = 1; i <= tries; ++i) {
    const auto d0 = fresh_diag(300ms);
    if (!d0) {
      RCLCPP_ERROR(get_logger(), "CONFIG: no DIAG frame from the STM");
      return false;
    }
    if (!send(proto::T_CONFIG, payload)) {
      RCLCPP_ERROR(get_logger(), "CONFIG: serial write failed");
      return false;
    }
    const auto d1 = fresh_diag(300ms);
    if (!d1) {
      RCLCPP_ERROR(get_logger(), "CONFIG: no DIAG frame from the STM");
      return false;
    }
    if (d1->rej > d0->rej) {
      RCLCPP_ERROR(
        get_logger(), "CONFIG rejected by the STM (state %s, must be DISABLED; values > 0?)",
        state_name(last_state_->state));
      return false;
    }
    // ok counts every valid frame the STM received. Nothing else is sent meanwhile,
    // so ok increased = our CONFIG arrived, rej unchanged = it was accepted.
    if (d1->ok > d0->ok && (last_state_->flags & proto::FL_CFG_OK)) {
      RCLCPP_INFO(get_logger(), "CONFIG accepted (try %d)", i);
      return true;
    }
    RCLCPP_WARN(get_logger(), "CONFIG: no answer (try %d/%d)", i, tries);
  }
  RCLCPP_ERROR(get_logger(), "CONFIG: no answer from the STM after %d tries", tries);
  return false;
}

bool ArmSystem::disable_best_effort(std::chrono::milliseconds timeout)
{
  if (!serial_.is_open()) {
    return false;
  }
  if (last_state_ && is_stopped(*last_state_)) {
    return true;
  }
  send(proto::T_ENABLE, proto::payload_enable(false));
  return wait_for(is_stopped, timeout);
}

int32_t ArmSystem::rad_to_steps(const JointCfg & joint, double rad) const
{
  return static_cast<int32_t>(std::lround(joint.direction * rad * joint.steps_per_rad));
}

double ArmSystem::steps_to_rad(const JointCfg & joint, int32_t steps) const
{
  return joint.direction * static_cast<double>(steps) / joint.steps_per_rad;
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
