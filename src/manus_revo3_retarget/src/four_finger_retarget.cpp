#include "manus_revo3_retarget/four_finger_retarget.hpp"

namespace manus_revo3_retarget
{

void FourFingerRetarget::set_config(const FourFingerConfig & config)
{
  config_ = config;
}

void FourFingerRetarget::apply(const JointPositions & joints, JointArray & q) const
{
  auto flex = [&](const std::string & name) {
    return joints.at(name);
  };

  q[IndexMCP] = flex("index_mcp") * config_.index_angle_scale * config_.four_finger_mcp_scale;
  q[IndexPIP] = flex("index_pip") * config_.index_angle_scale;
  q[IndexDIP] = flex("index_dip") * config_.index_angle_scale;

  q[MiddleMCP] = flex("middle_mcp") * config_.all_finger_angle_scale * config_.four_finger_mcp_scale;
  q[MiddlePIP] = flex("middle_pip") * config_.all_finger_angle_scale;
  q[MiddleDIP] = flex("middle_dip") * config_.all_finger_angle_scale * config_.middle_ring_dip_scale;

  q[RingMCP] = flex("ring_mcp") * config_.all_finger_angle_scale * config_.four_finger_mcp_scale;
  q[RingPIP] = flex("ring_pip") * config_.all_finger_angle_scale;
  q[RingDIP] = flex("ring_dip") * config_.all_finger_angle_scale * config_.middle_ring_dip_scale;

  q[LittleMCP] = flex("little_mcp") * config_.pinky_angle_scale * config_.pinky_mcp_scale *
    config_.four_finger_mcp_scale;
  q[LittlePIP] = flex("little_pip") * config_.pinky_angle_scale * config_.pinky_dip_pip_scale;
  q[LittleDIP] = flex("little_dip") * config_.pinky_angle_scale * config_.pinky_dip_pip_scale;
}

}  // namespace manus_revo3_retarget
