// Copyright (c) 2025 BrainCo
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

// Forward declaration for Stark SDK opaque type
struct DeviceHandler;

namespace revo3_driver
{

static constexpr std::size_t kJointCount{21};
static constexpr std::size_t kMotorCount{23};

enum class Revo3LogLevel : uint8_t
{
  kError = 0,
  kWarn  = 1,
  kInfo  = 2,
  kDebug = 3,
  kTrace = 4,
};

auto log_level_to_string(Revo3LogLevel level) -> std::string;

// ---------------------------------------------------------------------------
// Revo3Api – thin C++ wrapper around BC Revo3 SDK Modbus transport
// ---------------------------------------------------------------------------
class Revo3Api
{
public:
  // ── Nested config types ─────────────────────────────────────────────────
  struct ModbusConfig
  {
    std::string  port{"/dev/ttyUSB0"};
    uint32_t     baudrate{5000000};   // Revo3 default: 5 Mbps
    bool         auto_detect{false};
    bool         auto_detect_quick{true};
    std::string  auto_detect_port;    // hint; empty = scan all
  };

  struct DriverConfig
  {
    uint8_t      slave_id{1};
    Revo3LogLevel log_level{Revo3LogLevel::kInfo};
    ModbusConfig modbus{};
  };

  // ── Connection info returned after successful open ──────────────────────
  struct ConnectionInfo
  {
    std::string port;
    uint32_t    baudrate{0};
    uint8_t     slave_id{0};
  };

  // ── Device identification ───────────────────────────────────────────────
  struct DeviceInfoData
  {
    uint8_t     sku_type{0};
    uint8_t     hardware_type{0};
    std::string serial_number;
    std::string firmware_version;
    std::string hardware_version;
  };

  // ── Motor status (21 joints) ────────────────────────────────────────────
  // positions  – degrees
  // velocities – rpm
  // currents   – mA
  // statuses   – raw SDK motor status bitmask
  struct MotorStatus
  {
    std::array<float, kJointCount> positions{};
    std::array<float, kJointCount> velocities{};
    std::array<float, kJointCount> currents{};
    std::array<uint16_t, kJointCount> statuses{};
  };

  // ── MIT command (all 21 joints) ─────────────────────────────────────────
  // positions  – degrees
  // velocities – rpm
  // torques    – mA (feedforward)
  // kp, kd     – dimensionless
  struct MitCommand
  {
    std::array<float, kJointCount> kp{};
    std::array<float, kJointCount> kd{};
    std::array<float, kJointCount> positions{};
    std::array<float, kJointCount> velocities{};
    std::array<float, kJointCount> torques{};
  };

  // ── Lifecycle ───────────────────────────────────────────────────────────
  Revo3Api();
  explicit Revo3Api(const DriverConfig & config);
  ~Revo3Api();

  Revo3Api(const Revo3Api &)            = delete;
  Revo3Api & operator=(const Revo3Api &) = delete;
  Revo3Api(Revo3Api &&) noexcept        = default;
  Revo3Api & operator=(Revo3Api &&) noexcept = default;

  auto configure(const DriverConfig & config) -> void;

  auto open()     -> bool;
  auto close()    -> void;
  [[nodiscard]] auto is_open() const -> bool;

  // ── Communication ───────────────────────────────────────────────────────
  auto fetch_device_info(uint8_t slave_id, DeviceInfoData & out) const -> bool;

  [[nodiscard]] auto get_motor_status(uint8_t slave_id) const
    -> std::optional<MotorStatus>;

  [[nodiscard]] auto get_motor_positions(uint8_t slave_id) const
    -> std::optional<std::array<float, kMotorCount>>;

  [[nodiscard]] auto get_motor_velocities(uint8_t slave_id) const
    -> std::optional<std::array<float, kMotorCount>>;

  [[nodiscard]] auto get_motor_currents(uint8_t slave_id) const
    -> std::optional<std::array<float, kMotorCount>>;

  [[nodiscard]] auto get_motor_errors(uint8_t slave_id) const
    -> std::optional<std::array<uint16_t, kMotorCount>>;

  [[nodiscard]] auto send_mit_command(uint8_t slave_id, const MitCommand & cmd) -> bool;

  // Fallback: position-only (mode=0) via multi_joint_control (0.1 deg resolution)
  [[nodiscard]] auto send_position_command(
    uint8_t slave_id, const std::array<float, kJointCount> & positions_deg) -> bool;

  [[nodiscard]] auto resolved_connection() const -> std::optional<ConnectionInfo>;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace revo3_driver
