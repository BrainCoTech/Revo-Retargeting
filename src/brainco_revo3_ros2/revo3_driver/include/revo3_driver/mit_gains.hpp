#pragma once

#include <cmath>

namespace revo3_driver
{
// Initialize unclaimed gain interfaces for position-only controllers. Explicit
// zero commands from MIT controllers must remain zero to release motor torque.
constexpr double kFallbackKp = 1.0;
constexpr double kFallbackKd = 0.1;

inline double mit_gain_or_default(double gain, double fallback)
{
  return std::isfinite(gain) && gain >= 0.0 ? gain : fallback;
}
}  // namespace revo3_driver
