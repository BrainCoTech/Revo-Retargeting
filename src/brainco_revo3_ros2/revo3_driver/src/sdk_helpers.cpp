// Copyright (c) 2025 BrainCo
// SPDX-License-Identifier: Apache-2.0

#include "revo3_driver/sdk_helpers.hpp"

#include "stark-sdk.h"

namespace revo3_driver
{

void DeviceInfoDeleter::operator()(CDeviceInfo * info) const
{
  if (info)
  {
    ::free_device_info(info);
  }
}

void V3MotorStatusDeleter::operator()(CRevo3MotorStatusData * data) const
{
  if (data)
  {
    ::free_revo3_motor_status_data(data);
  }
}

auto to_sdk_log_level(Revo3LogLevel level) -> LogLevel
{
  switch (level)
  {
    case Revo3LogLevel::kError:
      return LOG_LEVEL_ERROR;
    case Revo3LogLevel::kWarn:
      return LOG_LEVEL_WARN;
    case Revo3LogLevel::kInfo:
      return LOG_LEVEL_INFO;
    case Revo3LogLevel::kDebug:
      return LOG_LEVEL_DEBUG;
    case Revo3LogLevel::kTrace:
      return LOG_LEVEL_TRACE;
  }
  return LOG_LEVEL_INFO;
}

}  // namespace revo3_driver
