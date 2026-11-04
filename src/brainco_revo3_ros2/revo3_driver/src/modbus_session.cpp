// Copyright (c) 2025 BrainCo
// SPDX-License-Identifier: Apache-2.0

#include "revo3_driver/modbus_session.hpp"

#include <algorithm>
#include <regex>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "revo3_driver/logger_macros.hpp"
#include "revo3_driver/sdk_helpers.hpp"
#include "stark-sdk.h"

namespace revo3_driver
{

// ── ModbusCloseDeleter ──────────────────────────────────────────────────────

void ModbusSession::ModbusCloseDeleter::operator()(DeviceHandler * h) const
{
  if (h)
  {
    ::modbus_close(h);
  }
}

// ── Constructor ─────────────────────────────────────────────────────────────

ModbusSession::ModbusSession(Revo3Api::DriverConfig & config)
: config_(config), handle_(nullptr)
{
}

// ── open ────────────────────────────────────────────────────────────────────

namespace
{
/// Discover all hardware serial ports in /dev/ (ttyUSB*, ttyACM*, ttyCH343USB*, ttyS*, etc.).
/// Excludes virtual consoles (tty0-tty63) by matching tty[A-Z]+ prefix.
/// When hint is non-empty, only serial device paths with that prefix are scanned.
std::vector<std::string> scan_serial_ports(const std::string & port_prefix = "")
{
  std::vector<std::string> ports;
  const std::regex serial_re(R"(^tty[A-Z])");  // starts with "tty" + uppercase letter
  std::error_code ec;

  for (const auto & entry : std::filesystem::directory_iterator("/dev", ec))
  {
    if (ec) { break; }
    if (!entry.is_character_file(ec)) { continue; }
    const std::string name = entry.path().filename().string();
    const std::string path = entry.path().string();
    if (std::regex_search(name, serial_re) &&
        (port_prefix.empty() || path.rfind(port_prefix, 0) == 0))
    {
      ports.push_back(path);
    }
  }

  std::sort(ports.begin(), ports.end());
  return ports;
}

struct DetectedDeviceListDeleter
{
  void operator()(CDetectedDeviceList * list) const
  {
    if (list)
    {
      ::free_detected_device_list(list);
    }
  }
};

using DetectedDeviceListPtr = std::unique_ptr<CDetectedDeviceList, DetectedDeviceListDeleter>;

auto detect_modbus_revo3(
  const char * port,
  uint8_t requested_slave_id) -> std::optional<Revo3Api::ConnectionInfo>
{
  DetectedDeviceListPtr list{
    ::stark_auto_detect(true, port, STARK_PROTOCOL_TYPE_MODBUS)};
  if (!list || list->count == 0 || !list->devices)
  {
    return std::nullopt;
  }

  for (std::uintptr_t i = 0; i < list->count; ++i)
  {
    const CDetectedDevice & device = list->devices[i];
    if (device.protocol != STARK_PROTOCOL_TYPE_MODBUS)
    {
      continue;
    }
    if (requested_slave_id != 0 && device.slave_id != requested_slave_id)
    {
      continue;
    }

    Revo3Api::ConnectionInfo connection{};
    connection.port = device.port_name ? device.port_name : std::string{};
    connection.baudrate = device.baudrate;
    connection.slave_id = device.slave_id;
    return connection;
  }

  return std::nullopt;
}
}  // namespace

bool ModbusSession::open()
{
  close();

  Revo3Api::ConnectionInfo connection{};
  connection.slave_id = config_.slave_id;

  if (config_.modbus.auto_detect)
  {
    const uint8_t requested_slave_id = config_.slave_id;

    REVO3_LOG_INFO(
      "Auto-detecting Revo3 device (requested slave_id: %u, quick: %s)",
      requested_slave_id, config_.modbus.auto_detect_quick ? "true" : "false");

    // Only auto_detect_port restricts discovery. The configured port field is
    // ignored in auto-detect mode; an empty hint must scan all serial ports.
    // Resolve symlinks because the SDK may not follow them natively.
    const char * hint = nullptr;
    std::string  hint_holder;
    if (!config_.modbus.auto_detect_port.empty()) {
      hint_holder = config_.modbus.auto_detect_port;
    }
    if (!hint_holder.empty()) {
      // Resolve symlinks (e.g. /dev/revo3_hand_left -> /dev/ttyCH343USB1)
      if (std::filesystem::is_symlink(hint_holder)) {
        hint_holder = std::filesystem::canonical(hint_holder).string();
      }
      hint = hint_holder.c_str();
    }

    std::optional<Revo3Api::ConnectionInfo> best_connection =
      detect_modbus_revo3(hint, requested_slave_id);

    // --- Port scan fallback ---
    if (!best_connection)
    {
      auto ports = scan_serial_ports(hint_holder);
      for (const auto & port : ports)
      {
        best_connection = detect_modbus_revo3(port.c_str(), requested_slave_id);
        if (best_connection)
        {
          REVO3_LOG_INFO(
            "Port scan found matching device: port=%s slave_id=%u",
            best_connection->port.c_str(), best_connection->slave_id);
          break;
        }
      }
    }

    if (!best_connection)
    {
      REVO3_LOG_ERROR(
        "Auto-detection failed: no Revo3 device found with slave_id=%u. "
        "Check USB cable, device power, and serial port permissions.",
        requested_slave_id);
      return false;
    }

    connection = *best_connection;
    config_.slave_id = best_connection->slave_id;
  }
  else
  {
    connection.port    = config_.modbus.port;
    connection.baudrate = config_.modbus.baudrate;

    if (!std::filesystem::exists(connection.port))
    {
      REVO3_LOG_ERROR(
        "Port %s does not exist. Check device connection or enable auto_detect.",
        connection.port.c_str());
      return false;
    }
    REVO3_LOG_INFO(
      "Using manual port: port=%s baudrate=%u slave_id=%u",
      connection.port.c_str(), connection.baudrate, connection.slave_id);
  }

  DeviceHandler * raw = ::modbus_open(connection.port.c_str(), connection.baudrate);
  if (!raw)
  {
    REVO3_LOG_ERROR(
      "modbus_open() failed: port=%s baudrate=%u. "
      "Device may already be in use or permission denied.",
      connection.port.c_str(), connection.baudrate);
    return false;
  }

  handle_.reset(raw);
  ::stark_set_hardware_type(handle_.get(), connection.slave_id, STARK_HARDWARE_TYPE_REVO3_ULTRA);
  resolved_connection_ = connection;

  REVO3_LOG_INFO(
    "Revo3 connected: port=%s baudrate=%u slave_id=%u",
    connection.port.c_str(), connection.baudrate, connection.slave_id);
  return true;
}

// ── close ────────────────────────────────────────────────────────────────────

void ModbusSession::close()
{
  handle_.reset();
  resolved_connection_.reset();
}

// ── fetch_device_info ────────────────────────────────────────────────────────

bool ModbusSession::fetch_device_info(uint8_t slave_id, Revo3Api::DeviceInfoData & out) const
{
  if (!handle_)
  {
    REVO3_LOG_ERROR("fetch_device_info: not connected");
    return false;
  }

  DeviceInfoPtr info{::stark_get_device_info(handle_.get(), slave_id)};
  (void)info;
  if (!info)
  {
    return false;
  }

  out.sku_type           = static_cast<uint8_t>(info->sku_type);
  out.hardware_type      = static_cast<uint8_t>(info->hardware_type);
  out.serial_number      = info->serial_number      ? info->serial_number      : std::string{};
  out.firmware_version   = info->firmware_version   ? info->firmware_version   : std::string{};
  out.hardware_version   =
    [&]() -> std::string
    {
      const char * hv = ::revo3_get_hardware_version(handle_.get(), slave_id);
      if (hv && hv[0])
      {
        std::string hardware_version{hv};
        ::free_string(hv);
        return hardware_version;
      }
      if (hv)
      {
        ::free_string(hv);
      }
      return std::string{};
    }();
  return true;
}

// ── get_motor_status ─────────────────────────────────────────────────────────

std::optional<Revo3Api::MotorStatus> ModbusSession::get_motor_status(uint8_t slave_id) const
{
  if (!handle_)
  {
    return std::nullopt;
  }

  V3MotorStatusPtr raw{::revo3_get_motor_status_data(handle_.get(), slave_id)};
  if (!raw)
  {
    return std::nullopt;
  }

  Revo3Api::MotorStatus status{};
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    status.positions[i]  = raw->positions[i];   // degrees
    status.velocities[i] = raw->velocities[i];  // rpm
    status.currents[i]   = raw->currents[i];    // mA
    status.statuses[i]   = raw->statuses[i];    // raw status bitmask
  }
  return status;
}

std::optional<std::array<float, kMotorCount>> ModbusSession::get_motor_positions(uint8_t slave_id) const
{
  if (!handle_)
  {
    return std::nullopt;
  }

  std::array<float, kMotorCount> positions{};
  if (::revo3_get_all_motor_positions(handle_.get(), slave_id, positions.data()) != 0)
  {
    return std::nullopt;
  }
  return positions;
}

std::optional<std::array<float, kMotorCount>> ModbusSession::get_motor_velocities(uint8_t slave_id) const
{
  if (!handle_)
  {
    return std::nullopt;
  }

  std::array<float, kMotorCount> velocities{};
  if (::revo3_get_all_motor_velocities(handle_.get(), slave_id, velocities.data()) != 0)
  {
    return std::nullopt;
  }
  return velocities;
}

std::optional<std::array<float, kMotorCount>> ModbusSession::get_motor_currents(uint8_t slave_id) const
{
  if (!handle_)
  {
    return std::nullopt;
  }

  std::array<float, kMotorCount> currents{};
  if (::revo3_get_all_motor_currents(handle_.get(), slave_id, currents.data()) != 0)
  {
    return std::nullopt;
  }
  return currents;
}

std::optional<std::array<uint16_t, kMotorCount>> ModbusSession::get_motor_errors(uint8_t slave_id) const
{
  if (!handle_)
  {
    return std::nullopt;
  }

  std::array<uint16_t, kMotorCount> errors{};
  if (::revo3_get_all_motor_errors(handle_.get(), slave_id, errors.data()) != 0)
  {
    return std::nullopt;
  }
  return errors;
}

// ── send_mit_command ─────────────────────────────────────────────────────────

bool ModbusSession::send_mit_command(uint8_t slave_id, const Revo3Api::MitCommand & cmd)
{
  if (!handle_)
  {
    return false;
  }

  ::revo3_hand_mit_control_without_retry(
    handle_.get(), slave_id,
    cmd.kp.data(),
    cmd.kd.data(),
    cmd.positions.data(),
    cmd.velocities.data(),
    cmd.torques.data());
  return true;
}

// ── send_position_command ────────────────────────────────────────────────────

bool ModbusSession::send_position_command(
  uint8_t slave_id,
  const std::array<float, kJointCount> & positions_deg)
{
  if (!handle_)
  {
    return false;
  }

  std::array<float, kJointCount> params{};
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    params[i] = std::max(0.0f, positions_deg[i]);
  }

  ::revo3_multi_joint_control(handle_.get(), slave_id, 0 /*mode=position*/, params.data());
  return true;
}

}  // namespace revo3_driver
