// Copyright (c) 2025 BrainCo
// SPDX-License-Identifier: Apache-2.0

#include "revo3_driver/revo3_hand_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "revo3_driver/logger_macros.hpp"
#include "revo3_driver/mit_gains.hpp"

// Unit conversion constants
namespace
{
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kRpmToRadS = 2.0 * M_PI / 60.0;
constexpr double kRadSToRpm = 60.0 / (2.0 * M_PI);
constexpr double kMilliampToAmp = 1.0 / 1000.0;
}  // namespace

namespace revo3_driver
{

// ── Helpers ──────────────────────────────────────────────────────────────────

namespace
{
/// Parse a string parameter from HardwareInfo, returning default_val if absent.
std::string get_param( 
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  const std::string & default_val = "")
{
  auto it = info.hardware_parameters.find(key);
  return (it != info.hardware_parameters.end()) ? it->second : default_val;
}

std::string to_lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  return s;
}

Revo3LogLevel parse_log_level(const std::string & s)
{
  const auto sl = to_lower(s);
  if (sl == "trace")  return Revo3LogLevel::kTrace;
  if (sl == "debug")  return Revo3LogLevel::kDebug;
  if (sl == "warn")   return Revo3LogLevel::kWarn;
  if (sl == "error")  return Revo3LogLevel::kError;
  return Revo3LogLevel::kInfo;
}
}  // namespace

// ── on_init ──────────────────────────────────────────────────────────────────

auto Revo3HandHardware::on_init(const hardware_interface::HardwareInfo & info)
  -> hardware_interface::CallbackReturn
{
  REVO3_LOG_INFO("on_init invoked");

  auto base = hardware_interface::SystemInterface::on_init(info);
  if (base != hardware_interface::CallbackReturn::SUCCESS)
  {
    REVO3_LOG_ERROR("SystemInterface::on_init failed");
    return base;
  }

  if (parse_driver_config(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Validate joint count
  if (info_.joints.size() != kJointCount)
  {
    REVO3_LOG_ERROR(
      "Expected %zu joints but URDF defines %zu",
      kJointCount, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.resize(kJointCount);
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    joint_names_[i] = info_.joints[i].name;
    REVO3_LOG_INFO("  Joint[%zu] = %s", i, joint_names_[i].c_str());
  }

  // Default command buffers to zero / neutral
  hw_cmd_positions_.fill(0.0);
  hw_cmd_velocities_.fill(0.0);
  hw_cmd_efforts_.fill(0.0);
  hw_cmd_kp_.fill(kFallbackKp);
  hw_cmd_kd_.fill(kFallbackKd);

  hw_state_positions_.fill(0.0);
  hw_state_velocities_.fill(0.0);
  hw_state_currents_.fill(0.0);
  hw_state_motor_states_.fill(0.0);

  REVO3_LOG_INFO(
    "Revo3HandHardware initialised (slave_id=%u, port=%s, baudrate=%u, auto_detect=%s)",
    driver_config_.slave_id,
    driver_config_.modbus.port.c_str(),
    driver_config_.modbus.baudrate,
    driver_config_.modbus.auto_detect ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── parse_driver_config ───────────────────────────────────────────────────────

auto Revo3HandHardware::parse_driver_config(const hardware_interface::HardwareInfo & info)
  -> hardware_interface::CallbackReturn
{
  const auto slave_id_str = get_param(info, "slave_id", "1");
  const auto port_str     = get_param(info, "port", "/dev/ttyUSB0");
  const auto baudrate_str = get_param(info, "baudrate", "5000000");
  const auto log_level_str= get_param(info, "log_level", "info");
  const auto auto_detect_str       = get_param(info, "auto_detect", "false");
  const auto auto_detect_quick_str = get_param(info, "auto_detect_quick", "true");
  const auto auto_detect_port_str  = get_param(info, "auto_detect_port", "");
  // Throttle the Modbus read() so the half-duplex bus has bandwidth left for
  // write(). 1 = read every tick (legacy behaviour). 4 at update_rate=200 Hz
  // gives ~50 Hz state, which matches state_publish_rate and is plenty.
  const auto read_decimation_str = get_param(info, "read_decimation", "4");

  try
  {
    driver_config_.slave_id         = static_cast<uint8_t>(std::stoul(slave_id_str));
    driver_config_.modbus.baudrate  = static_cast<uint32_t>(std::stoul(baudrate_str));
    read_decimation_                = std::max<std::size_t>(1, std::stoul(read_decimation_str));
  }
  catch (const std::exception & e)
  {
    REVO3_LOG_ERROR("Failed to parse numeric URDF parameters: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  driver_config_.modbus.port              = port_str;
  driver_config_.log_level               = parse_log_level(log_level_str);
  driver_config_.modbus.auto_detect       = (to_lower(auto_detect_str) == "true");
  driver_config_.modbus.auto_detect_quick = (to_lower(auto_detect_quick_str) != "false");
  driver_config_.modbus.auto_detect_port  = auto_detect_port_str;

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_configure ─────────────────────────────────────────────────────────────

auto Revo3HandHardware::on_configure(const rclcpp_lifecycle::State & /*prev*/)
  -> hardware_interface::CallbackReturn
{
  REVO3_LOG_INFO("on_configure invoked");

  hw_cmd_positions_.fill(0.0);
  hw_cmd_velocities_.fill(0.0);
  hw_cmd_efforts_.fill(0.0);
  hw_cmd_kp_.fill(kFallbackKp);
  hw_cmd_kd_.fill(kFallbackKd);
  hw_state_positions_.fill(0.0);
  hw_state_velocities_.fill(0.0);
  hw_state_currents_.fill(0.0);
  hw_state_motor_states_.fill(0.0);

  api_ = std::make_unique<Revo3Api>(driver_config_);
  if (!api_->open())
  {
    REVO3_LOG_ERROR(
      "Failed to open Revo3 connection. "
      "Check device cable, power, and serial port permissions.");
    api_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Fetch and log hardware version for diagnostics
  Revo3Api::DeviceInfoData dev_info{};
  if (api_->fetch_device_info(driver_config_.slave_id, dev_info))
  {
    REVO3_LOG_INFO(
      "Revo3 device info: hw_ver=%s fw_ver=%s sn=%s",
      dev_info.hardware_version.c_str(),
      dev_info.firmware_version.c_str(),
      dev_info.serial_number.c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_cleanup ───────────────────────────────────────────────────────────────

auto Revo3HandHardware::on_cleanup(const rclcpp_lifecycle::State & /*prev*/)
  -> hardware_interface::CallbackReturn
{
  REVO3_LOG_INFO("on_cleanup invoked");
  if (api_)
  {
    api_->close();
    api_.reset();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_activate ──────────────────────────────────────────────────────────────

auto Revo3HandHardware::on_activate(const rclcpp_lifecycle::State & /*prev*/)
  -> hardware_interface::CallbackReturn
{
  REVO3_LOG_INFO("on_activate invoked");
  if (!api_ || !api_->is_open())
  {
    REVO3_LOG_ERROR("Cannot activate: API not connected");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Seed position commands from current state to avoid jerk on activation
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_cmd_positions_[i] = hw_state_positions_[i];
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_deactivate ────────────────────────────────────────────────────────────

auto Revo3HandHardware::on_deactivate(const rclcpp_lifecycle::State & /*prev*/)
  -> hardware_interface::CallbackReturn
{
  REVO3_LOG_INFO("on_deactivate invoked");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_shutdown ──────────────────────────────────────────────────────────────

auto Revo3HandHardware::on_shutdown(const rclcpp_lifecycle::State & /*prev*/)
  -> hardware_interface::CallbackReturn
{
  REVO3_LOG_INFO("on_shutdown invoked");
  if (api_)
  {
    api_->close();
    api_.reset();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_error ─────────────────────────────────────────────────────────────────

auto Revo3HandHardware::on_error(const rclcpp_lifecycle::State & /*prev*/)
  -> hardware_interface::CallbackReturn
{
  REVO3_LOG_ERROR("on_error invoked – closing connection");
  if (api_)
  {
    api_->close();
    api_.reset();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── export_state_interfaces ──────────────────────────────────────────────────

auto Revo3HandHardware::export_state_interfaces()
  -> std::vector<hardware_interface::StateInterface>
{
  REVO3_LOG_INFO("export_state_interfaces invoked");
  std::vector<hardware_interface::StateInterface> ifaces;
  ifaces.reserve(kJointCount * 4);

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    const auto & name = joint_names_[i];
    ifaces.emplace_back(name, hardware_interface::HW_IF_POSITION,  &hw_state_positions_[i]);
    ifaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY,  &hw_state_velocities_[i]);
    ifaces.emplace_back(name, "current",                           &hw_state_currents_[i]);
    ifaces.emplace_back(name, "motor_state",                       &hw_state_motor_states_[i]);
  }
  return ifaces;
}

// ── export_command_interfaces ────────────────────────────────────────────────

auto Revo3HandHardware::export_command_interfaces()
  -> std::vector<hardware_interface::CommandInterface>
{
  REVO3_LOG_INFO("export_command_interfaces invoked");
  std::vector<hardware_interface::CommandInterface> ifaces;
  ifaces.reserve(kJointCount * 5);

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    const auto & name = joint_names_[i];
    ifaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &hw_cmd_positions_[i]);
    ifaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_velocities_[i]);
    ifaces.emplace_back(name, hardware_interface::HW_IF_EFFORT,   &hw_cmd_efforts_[i]);
    ifaces.emplace_back(name, "kp",                               &hw_cmd_kp_[i]);
    ifaces.emplace_back(name, "kd",                               &hw_cmd_kd_[i]);
  }
  return ifaces;
}

// ── read ─────────────────────────────────────────────────────────────────────

auto Revo3HandHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  -> hardware_interface::return_type
{
  if (!api_ || !api_->is_open())
  {
    REVO3_LOG_WARN("read: API not connected");
    return hardware_interface::return_type::ERROR;
  }

  // Throttle: only actually hit the bus every Nth tick. Modbus is half-duplex
  // and a single get_motor_status round-trip costs several ms; doing it every
  // tick at 200+ Hz starves the write path and produces choppy motion.
  if (read_decimation_ > 1 && (read_tick_counter_++ % read_decimation_) != 0)
  {
    return hardware_interface::return_type::OK;
  }

  // One Modbus read returns the first 21 exported hand joints plus wrist motors.
  // The ROS2 control surface intentionally exposes only hand joint indices 0-20.
  auto status = api_->get_motor_status(driver_config_.slave_id);
  if (!status)
  {
    REVO3_LOG_WARN("read: get_motor_status returned no data");
    return hardware_interface::return_type::ERROR;
  }

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_state_positions_[i] = static_cast<double>(status->positions[i]) * kDegToRad;
    hw_state_velocities_[i] = static_cast<double>(status->velocities[i]) * kRpmToRadS;
    hw_state_currents_[i] = static_cast<double>(status->currents[i]) * kMilliampToAmp;
    hw_state_motor_states_[i] = static_cast<double>(status->statuses[i]);
  }
  return hardware_interface::return_type::OK;
}

// ── write ────────────────────────────────────────────────────────────────────

auto Revo3HandHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  -> hardware_interface::return_type
{
  if (!api_ || !api_->is_open())
  {
    REVO3_LOG_WARN("write: API not connected");
    return hardware_interface::return_type::ERROR;
  }

  // Build MIT command: convert ROS2 units → SDK units
  Revo3Api::MitCommand cmd{};
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    const double kp = mit_gain_or_default(hw_cmd_kp_[i], kFallbackKp);
    const double kd = mit_gain_or_default(hw_cmd_kd_[i], kFallbackKd);
    cmd.kp[i]         = static_cast<float>(kp);
    cmd.kd[i]         = static_cast<float>(kd);
    cmd.positions[i]  = static_cast<float>(hw_cmd_positions_[i]  * kRadToDeg);
    cmd.velocities[i] = static_cast<float>(hw_cmd_velocities_[i] * kRadSToRpm);
    cmd.torques[i]    = static_cast<float>(hw_cmd_efforts_[i]);   // mA
  }

  if (!api_->send_mit_command(driver_config_.slave_id, cmd))
  {
    REVO3_LOG_WARN("write: send_mit_command failed");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace revo3_driver

PLUGINLIB_EXPORT_CLASS(revo3_driver::Revo3HandHardware, hardware_interface::SystemInterface)
