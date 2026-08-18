#pragma once

#include "manus_revo3_retarget/retarget_common.hpp"

namespace manus_revo3_retarget
{

struct SpreadConfig
{
  double index_offset_deg{0.0};
  double middle_offset_deg{0.0};
  double ring_offset_deg{0.0};
  double pinky_offset_deg{0.0};
  double index_scale{1.0};
  double middle_scale{1.0};
  double ring_scale{1.0};
  double pinky_scale{1.0};
  bool middle_dynamic{false};
  double ring_forward_scale{1.0};
  double ring_backward_scale{1.0};
  double finger_spread_sign{-1.0};
};

class SpreadRetarget
{
public:
  void set_config(const SpreadConfig & config);
  void apply(const Ergonomics & ergonomics, JointArray & q) const;

private:
  SpreadConfig config_;
};

}  // namespace manus_revo3_retarget
