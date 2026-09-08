#include "manus_revo3_retarget/spread_retarget.hpp"

namespace manus_revo3_retarget
{

void SpreadRetarget::set_config(const SpreadConfig & config)
{
  config_ = config;
}

void SpreadRetarget::apply(const JointPositions & joints, JointArray & q) const
{
  const auto angle = [&](const std::string & finger) {
      return joints.at(finger + "_" + config_.joint_suffix);
    };
  const double middle = angle("middle");
  const double reference = config_.relative_to_middle ? middle : 0.0;
  const double middle_value = config_.middle_dynamic ?
    (middle - deg_to_rad(config_.middle_offset_deg)) * config_.middle_scale :
    -deg_to_rad(config_.middle_offset_deg);

  const double index_value = (angle("index") - reference -
    deg_to_rad(config_.index_offset_deg)) * config_.index_scale;
  double ring_value = (angle("ring") - reference -
    deg_to_rad(config_.ring_offset_deg)) * config_.ring_scale;
  const double little_value = (angle("little") - reference -
    deg_to_rad(config_.pinky_offset_deg)) * config_.pinky_scale;

  if (ring_value > 0.0) {
    ring_value *= config_.ring_forward_scale;
  } else if (ring_value < 0.0) {
    ring_value *= config_.ring_backward_scale;
  }

  q[IndexMPR] = config_.finger_spread_sign * index_value;
  q[MiddleMPR] = config_.finger_spread_sign * middle_value;
  q[RingMPR] = config_.finger_spread_sign * ring_value;
  q[LittleMPR] = config_.finger_spread_sign * little_value;
}

}  // namespace manus_revo3_retarget
