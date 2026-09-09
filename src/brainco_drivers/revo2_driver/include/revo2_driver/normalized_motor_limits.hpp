#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace revo2_driver
{
// Firmware limits, in SDK motor order, used only by the opt-in normalized
// position profile. Never substitute nominal URDF limits for a failed read.
struct NormalizedMotorLimits
{
  std::array<double, 6> min_deg{};
  std::array<double, 6> max_deg{};
  std::array<double, 6> max_speed_deg_s{};
  static constexpr double deg_to_rad = 0.017453292519943295;

  bool valid() const
  {
    for (std::size_t i = 0; i < 6; ++i) {
      if (!std::isfinite(min_deg[i]) || !std::isfinite(max_deg[i]) ||
        !std::isfinite(max_speed_deg_s[i]) || min_deg[i] < 0.0 ||
        max_deg[i] <= min_deg[i] || max_deg[i] > 180.0 ||
        max_speed_deg_s[i] <= 0.0 || max_speed_deg_s[i] > 2000.0)
      {
        return false;
      }
    }
    return true;
  }

  double position_rad(std::size_t motor, double normalized) const
  {
    if (!std::isfinite(normalized) || normalized < 0.0 || normalized > 1000.0) {
      throw std::invalid_argument("Motor position outside normalized 0..1000");
    }
    return (min_deg[motor] + normalized * (max_deg[motor] - min_deg[motor]) / 1000.0)
           * deg_to_rad;
  }

  double position_command(std::size_t motor, double radians) const
  {
    if (!std::isfinite(radians)) {
      throw std::invalid_argument("Non-finite position command");
    }
    return std::clamp((radians / deg_to_rad - min_deg[motor]) * 1000.0 /
                     (max_deg[motor] - min_deg[motor]), 0.0, 1000.0);
  }

  double velocity_rad_s(std::size_t motor, double normalized) const
  {
    return normalized * max_speed_deg_s[motor] * deg_to_rad / 1000.0;
  }
};
}  // namespace revo2_driver
