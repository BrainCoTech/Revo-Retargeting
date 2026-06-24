#pragma once

#include "manus_revo3_retarget_cpp/retarget_common.hpp"

namespace manus_revo3_retarget_cpp
{

struct FourFingerConfig
{
  double index_angle_scale{1.0};
  double four_finger_mcp_scale{1.0};
  double middle_ring_dip_scale{1.0};
  double pinky_angle_scale{1.0};
  double pinky_dip_pip_scale{1.0};
  double pinky_mcp_scale{1.0};
  double all_finger_angle_scale{1.0};
};

class FourFingerRetarget
{
public:
  void set_config(const FourFingerConfig & config);
  void apply(const Ergonomics & ergonomics, JointArray & q) const;

private:
  FourFingerConfig config_;
};

}  // namespace manus_revo3_retarget_cpp
