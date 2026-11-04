// Copyright (c) 2025 BrainCo
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <functional>
#include <memory>
#include <optional>

#include "revo3_driver/revo3_hand_api.hpp"
#include "revo3_driver/sdk_helpers.hpp"
#include "stark-sdk.h"

namespace revo3_driver
{

/// Modbus transport session for Revo3 (RS-485 / BC Revo3 SDK).
///
/// Lifecycle:
///   open()  → connects via Modbus
///   close() → disconnects and releases handle
class ModbusSession
{
public:
  explicit ModbusSession(Revo3Api::DriverConfig & config);
  ~ModbusSession() = default;

  ModbusSession(const ModbusSession &)            = delete;
  ModbusSession & operator=(const ModbusSession &) = delete;

  // ── Lifecycle ─────────────────────────────────────────────────────────
  [[nodiscard]] auto open()  -> bool;
  auto               close() -> void;
  [[nodiscard]] auto is_open() const -> bool { return handle_ != nullptr; }

  // ── Device access ──────────────────────────────────────────────────────
  [[nodiscard]] auto handler() const -> DeviceHandler * { return handle_.get(); }

  [[nodiscard]] auto resolved_connection() const
    -> std::optional<Revo3Api::ConnectionInfo>
  {
    return resolved_connection_;
  }

  // ── Communication helpers ──────────────────────────────────────────────
  [[nodiscard]] auto fetch_device_info(
    uint8_t slave_id, Revo3Api::DeviceInfoData & out) const -> bool;

  [[nodiscard]] auto get_motor_status(uint8_t slave_id) const
    -> std::optional<Revo3Api::MotorStatus>;

  [[nodiscard]] auto get_motor_positions(uint8_t slave_id) const
    -> std::optional<std::array<float, kMotorCount>>;

  [[nodiscard]] auto get_motor_velocities(uint8_t slave_id) const
    -> std::optional<std::array<float, kMotorCount>>;

  [[nodiscard]] auto get_motor_currents(uint8_t slave_id) const
    -> std::optional<std::array<float, kMotorCount>>;

  [[nodiscard]] auto get_motor_errors(uint8_t slave_id) const
    -> std::optional<std::array<uint16_t, kMotorCount>>;

  [[nodiscard]] auto send_mit_command(
    uint8_t slave_id, const Revo3Api::MitCommand & cmd) -> bool;

  [[nodiscard]] auto send_position_command(
    uint8_t slave_id,
    const std::array<float, kJointCount> & positions_deg) -> bool;

private:
  Revo3Api::DriverConfig & config_;

  // DeviceHandler is owned via modbus_close custom deleter
  struct ModbusCloseDeleter
  {
    void operator()(DeviceHandler * h) const;
  };
  std::unique_ptr<DeviceHandler, ModbusCloseDeleter> handle_;

  std::optional<Revo3Api::ConnectionInfo> resolved_connection_;
};

}  // namespace revo3_driver
