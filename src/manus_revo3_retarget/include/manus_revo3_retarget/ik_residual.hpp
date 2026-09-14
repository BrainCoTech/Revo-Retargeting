#pragma once
#include <cmath>
#include <stdexcept>
namespace manus_revo3_retarget {
inline double residual_row_scale(double objective_weight, double sigma)
{
  if (!std::isfinite(objective_weight) || objective_weight < 0.0 || !std::isfinite(sigma) || sigma <= 0.0) {
    throw std::invalid_argument("IK weight must be finite/nonnegative and sigma finite/positive");
  }
  return std::sqrt(objective_weight) / sigma;
}
}  // namespace manus_revo3_retarget
