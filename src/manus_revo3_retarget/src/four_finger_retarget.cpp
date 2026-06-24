#include "manus_revo3_retarget_cpp/four_finger_retarget.hpp"

namespace manus_revo3_retarget_cpp
{

void FourFingerRetarget::set_config(const FourFingerConfig & config)
{
  config_ = config;
}

void FourFingerRetarget::apply(const Ergonomics & ergonomics, JointArray & q) const
{
  auto flex = [&](const std::string & name) {
    return ergonomic_value(ergonomics, name, 0.0);
  };

  q[IndexMCP] = deg_to_rad(flex("IndexMCPStretch") * config_.index_angle_scale * config_.four_finger_mcp_scale);
  q[IndexPIP] = deg_to_rad(flex("IndexPIPStretch") * config_.index_angle_scale);
  q[IndexDIP] = deg_to_rad(flex("IndexDIPStretch") * config_.index_angle_scale);

  q[MiddleMCP] = deg_to_rad(flex("MiddleMCPStretch") * config_.all_finger_angle_scale * config_.four_finger_mcp_scale);
  q[MiddlePIP] = deg_to_rad(flex("MiddlePIPStretch") * config_.all_finger_angle_scale);
  q[MiddleDIP] = deg_to_rad(flex("MiddleDIPStretch") * config_.all_finger_angle_scale * config_.middle_ring_dip_scale);

  q[RingMCP] = deg_to_rad(flex("RingMCPStretch") * config_.all_finger_angle_scale * config_.four_finger_mcp_scale);
  q[RingPIP] = deg_to_rad(flex("RingPIPStretch") * config_.all_finger_angle_scale);
  q[RingDIP] = deg_to_rad(flex("RingDIPStretch") * config_.all_finger_angle_scale * config_.middle_ring_dip_scale);

  q[LittleMCP] = deg_to_rad(flex("PinkyMCPStretch") * config_.pinky_angle_scale * config_.pinky_mcp_scale *
    config_.four_finger_mcp_scale);
  q[LittlePIP] = deg_to_rad(flex("PinkyPIPStretch") * config_.pinky_angle_scale * config_.pinky_dip_pip_scale);
  q[LittleDIP] = deg_to_rad(flex("PinkyDIPStretch") * config_.pinky_angle_scale * config_.pinky_dip_pip_scale);
}

}  // namespace manus_revo3_retarget_cpp
