// Copyright (c) 2025 BrainCo
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>

#include "revo3_driver/revo3_hand_api.hpp"
#include "stark-sdk.h"

namespace revo3_driver
{

// ── Custom deleters for Revo3 SDK heap objects ─────────────────────────────

struct DeviceInfoDeleter
{
  void operator()(CDeviceInfo * info) const;
};

/// Deleter for CRevo3MotorStatusData returned by revo3_get_motor_status_data().
struct V3MotorStatusDeleter
{
  void operator()(CRevo3MotorStatusData * data) const;
};

// ── Smart pointer aliases ──────────────────────────────────────────────────

using DeviceInfoPtr      = std::unique_ptr<CDeviceInfo,        DeviceInfoDeleter>;
using V3MotorStatusPtr   = std::unique_ptr<CRevo3MotorStatusData, V3MotorStatusDeleter>;

// ── SDK log-level converter ────────────────────────────────────────────────

/// Convert revo3_driver::Revo3LogLevel to Revo3 SDK LogLevel enum.
auto to_sdk_log_level(Revo3LogLevel level) -> LogLevel;

}  // namespace revo3_driver
