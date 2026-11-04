// Copyright (c) 2025 BrainCo
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "revo3_driver/revo3_hand_api.hpp"

namespace revo3_driver
{

/// @brief ROS2 ros2_control SystemInterface for the Revo3 21-DoF dexterous hand.
///
/// Command interfaces per joint (5):
///   - position  (radians → converted to degrees for SDK)
///   - velocity  (rad/s  → converted to rpm for SDK)
///   - effort    (mA, torque feedforward)
///   - kp        (dimensionless)
///   - kd        (dimensionless)
///
/// State interfaces per joint (4):
///   - position  (radians, from SDK degrees)
///   - velocity  (rad/s,   from SDK rpm)
///   - current   (A,       from SDK mA)
///   - motor_state (raw SDK status bitmask)
class Revo3HandHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Revo3HandHardware)

  // ── Lifecycle ───────────────────────────────────────────────────────────
  auto on_init(const hardware_interface::HardwareInfo & info)
    -> hardware_interface::CallbackReturn override;

  auto on_configure(const rclcpp_lifecycle::State & previous_state)
    -> hardware_interface::CallbackReturn override;

  auto on_cleanup(const rclcpp_lifecycle::State & previous_state)
    -> hardware_interface::CallbackReturn override;

  auto on_activate(const rclcpp_lifecycle::State & previous_state)
    -> hardware_interface::CallbackReturn override;

  auto on_deactivate(const rclcpp_lifecycle::State & previous_state)
    -> hardware_interface::CallbackReturn override;

  auto on_shutdown(const rclcpp_lifecycle::State & previous_state)
    -> hardware_interface::CallbackReturn override;

  auto on_error(const rclcpp_lifecycle::State & previous_state)
    -> hardware_interface::CallbackReturn override;

  // ── Hardware interfaces ─────────────────────────────────────────────────
  auto export_state_interfaces()
    -> std::vector<hardware_interface::StateInterface> override;

  auto export_command_interfaces()
    -> std::vector<hardware_interface::CommandInterface> override;

  // ── Control loop ────────────────────────────────────────────────────────
  auto read(const rclcpp::Time & time, const rclcpp::Duration & period)
    -> hardware_interface::return_type override;

  auto write(const rclcpp::Time & time, const rclcpp::Duration & period)
    -> hardware_interface::return_type override;

private:
  // ── SDK API handle ──────────────────────────────────────────────────────
  std::unique_ptr<Revo3Api> api_;

  // ── Driver config parsed from URDF ros2_control params ─────────────────
  Revo3Api::DriverConfig driver_config_;

  // ── Joint order (matches SDK index 0-20) ───────────────────────────────
  //    Populated from hardware_interface::HardwareInfo::joints
  std::vector<std::string> joint_names_;  // size == kJointCount

  // ── Command buffers (ROS2 units) ────────────────────────────────────────
  std::array<double, kJointCount> hw_cmd_positions_{};   // radians
  std::array<double, kJointCount> hw_cmd_velocities_{};  // rad/s
  std::array<double, kJointCount> hw_cmd_efforts_{};     // mA (torque_ff)
  std::array<double, kJointCount> hw_cmd_kp_{};
  std::array<double, kJointCount> hw_cmd_kd_{};

  // ── State buffers (ROS2 units) ──────────────────────────────────────────
  std::array<double, kJointCount> hw_state_positions_{};   // radians
  std::array<double, kJointCount> hw_state_velocities_{};  // rad/s
  std::array<double, kJointCount> hw_state_currents_{};      // A
  std::array<double, kJointCount> hw_state_motor_states_{};  // raw status bitmask

  // ── Read throttling ─────────────────────────────────────────────────────
  // Modbus is half-duplex; doing read + write every controller tick doubles
  // the bus traffic. JTC runs open-loop (open_loop_control: true) and joint
  // states are published at 50 Hz, so reading every Nth write tick is
  // sufficient and frees the bus for write commands.
  std::size_t read_decimation_{1};   // read every N writes (1 = every tick)
  std::size_t read_tick_counter_{0};

  // ── Helper ──────────────────────────────────────────────────────────────
  auto parse_driver_config(const hardware_interface::HardwareInfo & info)
    -> hardware_interface::CallbackReturn;
};

}  // namespace revo3_driver
