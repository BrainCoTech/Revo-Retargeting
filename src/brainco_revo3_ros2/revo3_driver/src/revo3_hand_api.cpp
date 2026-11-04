// Copyright (c) 2025 BrainCo
// SPDX-License-Identifier: Apache-2.0

#include "revo3_driver/revo3_hand_api.hpp"

#include <memory>
#include <stdexcept>

#include "revo3_driver/modbus_session.hpp"
#include "revo3_driver/sdk_helpers.hpp"
#include "stark-sdk.h"

namespace revo3_driver
{

auto log_level_to_string(Revo3LogLevel level) -> std::string
{
  switch (level)
  {
    case Revo3LogLevel::kError:  return "error";
    case Revo3LogLevel::kWarn:   return "warn";
    case Revo3LogLevel::kInfo:   return "info";
    case Revo3LogLevel::kDebug:  return "debug";
    case Revo3LogLevel::kTrace:  return "trace";
  }
  return "info";
}

// ── Pimpl ───────────────────────────────────────────────────────────────────

struct Revo3Api::Impl
{
  DriverConfig               config{};
  std::unique_ptr<ModbusSession> session;

  void rebuild_session()
  {
    session = std::make_unique<ModbusSession>(config);
  }
};

// ── Constructors / destructor ────────────────────────────────────────────────

Revo3Api::Revo3Api() : impl_(std::make_unique<Impl>())
{
  configure(DriverConfig{});
}

Revo3Api::Revo3Api(const DriverConfig & config) : impl_(std::make_unique<Impl>())
{
  configure(config);
}

Revo3Api::~Revo3Api() = default;

// ── configure ───────────────────────────────────────────────────────────────

auto Revo3Api::configure(const DriverConfig & config) -> void
{
  if (!impl_)
  {
    impl_ = std::make_unique<Impl>();
  }

  close();
  impl_->config = config;

  // Revo3 always uses Modbus; set up SDK logging
  ::init_logging(to_sdk_log_level(config.log_level));

  impl_->rebuild_session();
}

// ── open / close / is_open ────────────────────────────────────────────────

auto Revo3Api::open() -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
  }
  return impl_->session->open();
}

auto Revo3Api::close() -> void
{
  if (impl_ && impl_->session)
  {
    impl_->session->close();
  }
}

auto Revo3Api::is_open() const -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
  }
  return impl_->session->is_open();
}

// ── Device info ──────────────────────────────────────────────────────────────

auto Revo3Api::fetch_device_info(uint8_t slave_id, DeviceInfoData & out) const -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
  }
  return impl_->session->fetch_device_info(slave_id, out);
}

// ── Motor status ─────────────────────────────────────────────────────────────

auto Revo3Api::get_motor_status(uint8_t slave_id) const -> std::optional<MotorStatus>
{
  if (!impl_ || !impl_->session)
  {
    return std::nullopt;
  }
  return impl_->session->get_motor_status(slave_id);
}

auto Revo3Api::get_motor_positions(uint8_t slave_id) const
  -> std::optional<std::array<float, kMotorCount>>
{
  if (!impl_ || !impl_->session)
  {
    return std::nullopt;
  }
  return impl_->session->get_motor_positions(slave_id);
}

auto Revo3Api::get_motor_velocities(uint8_t slave_id) const
  -> std::optional<std::array<float, kMotorCount>>
{
  if (!impl_ || !impl_->session)
  {
    return std::nullopt;
  }
  return impl_->session->get_motor_velocities(slave_id);
}

auto Revo3Api::get_motor_currents(uint8_t slave_id) const
  -> std::optional<std::array<float, kMotorCount>>
{
  if (!impl_ || !impl_->session)
  {
    return std::nullopt;
  }
  return impl_->session->get_motor_currents(slave_id);
}

auto Revo3Api::get_motor_errors(uint8_t slave_id) const
  -> std::optional<std::array<uint16_t, kMotorCount>>
{
  if (!impl_ || !impl_->session)
  {
    return std::nullopt;
  }
  return impl_->session->get_motor_errors(slave_id);
}

// ── MIT command ──────────────────────────────────────────────────────────────

auto Revo3Api::send_mit_command(uint8_t slave_id, const MitCommand & cmd) -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
  }
  return impl_->session->send_mit_command(slave_id, cmd);
}

// ── Position-only command ────────────────────────────────────────────────────

auto Revo3Api::send_position_command(
  uint8_t slave_id, const std::array<float, kJointCount> & positions_deg) -> bool
{
  if (!impl_ || !impl_->session)
  {
    return false;
  }
  return impl_->session->send_position_command(slave_id, positions_deg);
}

// ── Resolved connection ──────────────────────────────────────────────────────

auto Revo3Api::resolved_connection() const -> std::optional<ConnectionInfo>
{
  if (!impl_ || !impl_->session)
  {
    return std::nullopt;
  }
  return impl_->session->resolved_connection();
}

}  // namespace revo3_driver
