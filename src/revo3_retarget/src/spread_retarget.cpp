#include "manus_revo3_retarget/spread_retarget.hpp"

namespace manus_revo3_retarget
{

void SpreadRetarget::set_config(const SpreadConfig & config)
{
  config_ = config;
}

void SpreadRetarget::apply(const Ergonomics & ergonomics, JointArray & q) const
{
  const double middle_ref = ergonomic_value(ergonomics, "MiddleSpread", 0.0);
  const double middle_value_deg = config_.middle_dynamic ?
    (middle_ref - config_.middle_offset_deg) * config_.middle_scale :
    -config_.middle_offset_deg;

  double index_value_deg = (ergonomic_value(ergonomics, "IndexSpread", 0.0) - middle_ref -
    config_.index_offset_deg) * config_.index_scale;
  double ring_value_deg = (ergonomic_value(ergonomics, "RingSpread", 0.0) - middle_ref -
    config_.ring_offset_deg) * config_.ring_scale;
  double pinky_value_deg = (ergonomic_value(ergonomics, "PinkySpread", 0.0) - middle_ref -
    config_.pinky_offset_deg) * config_.pinky_scale;

  if (ring_value_deg > 0.0) {
    ring_value_deg *= config_.ring_forward_scale;
  } else if (ring_value_deg < 0.0) {
    ring_value_deg *= config_.ring_backward_scale;
  }

  q[IndexMPR] = deg_to_rad(config_.finger_spread_sign * index_value_deg);
  q[MiddleMPR] = deg_to_rad(config_.finger_spread_sign * middle_value_deg);
  q[RingMPR] = deg_to_rad(config_.finger_spread_sign * ring_value_deg);
  q[LittleMPR] = deg_to_rad(config_.finger_spread_sign * pinky_value_deg);
}

}  // namespace manus_revo3_retarget
