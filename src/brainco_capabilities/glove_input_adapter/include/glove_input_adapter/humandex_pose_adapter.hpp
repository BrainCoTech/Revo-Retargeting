#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <manus_ros2_msgs/msg/manus_glove.hpp>

namespace glove_input_adapter
{

double mapAbsoluteJointToTarget(
  double position_rad,
  double open_position_rad,
  double closed_position_rad,
  double target_max);

struct HumandexAdapterConfig
{
  std::string side{"right"};
  int32_t glove_id{1};
};

class HumandexPoseAdapter
{
public:
  explicit HumandexPoseAdapter(HumandexAdapterConfig config);

  static std::vector<geometry_msgs::msg::Pose> selectHandPoses(
    const std::vector<geometry_msgs::msg::Pose> & poses,
    const std::string & input_hand_mode,
    const std::string & side);

  bool validateInput(
    const std::vector<geometry_msgs::msg::Pose> & fingertip_poses,
    std::string * reason = nullptr) const;

  manus_ros2_msgs::msg::ManusGlove adapt(
    const std::vector<geometry_msgs::msg::Pose> & fingertip_poses) const;

private:
  HumandexAdapterConfig config_;
};

}  // namespace glove_input_adapter
