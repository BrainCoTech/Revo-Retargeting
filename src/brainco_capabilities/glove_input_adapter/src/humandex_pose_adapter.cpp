#include "glove_input_adapter/humandex_pose_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <manus_ros2_msgs/msg/manus_raw_node.hpp>

namespace glove_input_adapter
{

double mapAbsoluteJointToTarget(
  double position_rad,
  double open_position_rad,
  double closed_position_rad,
  double target_max)
{
  if (
    !std::isfinite(position_rad) || !std::isfinite(open_position_rad) ||
    !std::isfinite(closed_position_rad) || !std::isfinite(target_max))
  {
    throw std::invalid_argument("absolute joint mapping values must be finite");
  }
  const double span_rad = closed_position_rad - open_position_rad;
  if (std::abs(span_rad) < 1e-9) {
    throw std::invalid_argument("absolute open and closed joint positions must differ");
  }
  if (target_max <= 0.0) {
    throw std::invalid_argument("absolute target maximum must be positive");
  }

  const double normalized = std::clamp(
    (position_rad - open_position_rad) / span_rad, 0.0, 1.0);
  return normalized * target_max;
}

namespace
{

constexpr std::size_t kFingertipCount = 5U;
// HumanDex per-hand input: index, middle, ring, little, thumb.
// Compatibility output: thumb, index, middle, ring, pinky.
constexpr std::array<std::size_t, kFingertipCount> kOutputSourceIndices{4, 0, 1, 2, 3};
constexpr std::array<int32_t, kFingertipCount> kManusNodeIds{4, 9, 14, 19, 24};
constexpr std::array<const char *, kFingertipCount> kChainTypes{
  "thumb", "index", "middle", "ring", "pinky"};

bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) &&
         std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) &&
         std::isfinite(pose.orientation.w) &&
         std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z);
}

manus_ros2_msgs::msg::ManusRawNode rawNode(
  int32_t node_id,
  int32_t parent_node_id,
  const std::string & joint_type,
  const std::string & chain_type,
  const geometry_msgs::msg::Pose & pose)
{
  manus_ros2_msgs::msg::ManusRawNode result;
  result.node_id = node_id;
  result.parent_node_id = parent_node_id;
  result.joint_type = joint_type;
  result.chain_type = chain_type;
  result.pose = pose;
  return result;
}

geometry_msgs::msg::Pose compatibilityPose(const geometry_msgs::msg::Pose & source)
{
  // retarget_node converts legacy Manus positions as [-raw.y, -raw.x, raw.z].
  // Apply its inverse so the resulting retarget position remains exactly in
  // the HumanDex palm-local URDF frame.
  geometry_msgs::msg::Pose result;
  result.position.x = -source.position.y;
  result.position.y = -source.position.x;
  result.position.z = source.position.z;
  // The legacy axis map contains a reflection and cannot be represented by a
  // quaternion. Retargeting currently consumes fingertip position only.
  result.orientation.w = 1.0;
  return result;
}

}  // namespace

HumandexPoseAdapter::HumandexPoseAdapter(HumandexAdapterConfig config)
: config_(std::move(config))
{
  if (config_.side != "left" && config_.side != "right") {
    throw std::invalid_argument("side must be left or right");
  }
}

std::vector<geometry_msgs::msg::Pose> HumandexPoseAdapter::selectHandPoses(
  const std::vector<geometry_msgs::msg::Pose> & poses,
  const std::string & input_hand_mode,
  const std::string & side)
{
  if (side != "left" && side != "right") {
    throw std::invalid_argument("side must be left or right");
  }
  if (input_hand_mode != "left" && input_hand_mode != "right" && input_hand_mode != "both") {
    throw std::invalid_argument("input_hand_mode must be left, right, or both");
  }
  if (input_hand_mode != "both" && input_hand_mode != side) {
    throw std::invalid_argument("side is not present in input_hand_mode");
  }

  const std::size_t expected_count = input_hand_mode == "both" ? 10U : kFingertipCount;
  if (poses.size() != expected_count) {
    throw std::invalid_argument(
            "HumanDex PoseArray count does not match input_hand_mode (expected " +
            std::to_string(expected_count) + ", got " + std::to_string(poses.size()) + ")");
  }

  const std::size_t begin = input_hand_mode == "both" && side == "right" ? 5U : 0U;
  return std::vector<geometry_msgs::msg::Pose>(
    poses.begin() + static_cast<std::ptrdiff_t>(begin),
    poses.begin() + static_cast<std::ptrdiff_t>(begin + kFingertipCount));
}

bool HumandexPoseAdapter::validateInput(
  const std::vector<geometry_msgs::msg::Pose> & fingertip_poses,
  std::string * reason) const
{
  if (fingertip_poses.size() != kFingertipCount) {
    if (reason != nullptr) {
      *reason = "expected five HumanDex fingertip poses";
    }
    return false;
  }
  for (const auto & pose : fingertip_poses) {
    if (!finitePose(pose)) {
      if (reason != nullptr) {
        *reason = "a HumanDex fingertip pose contains non-finite values";
      }
      return false;
    }
  }
  return true;
}

manus_ros2_msgs::msg::ManusGlove HumandexPoseAdapter::adapt(
  const std::vector<geometry_msgs::msg::Pose> & fingertip_poses) const
{
  std::string reason;
  if (!validateInput(fingertip_poses, &reason)) {
    throw std::invalid_argument(reason);
  }

  manus_ros2_msgs::msg::ManusGlove result;
  result.glove_id = config_.glove_id;
  result.side = config_.side;
  result.raw_nodes.reserve(kFingertipCount + 1U);
  result.raw_sensor.reserve(kFingertipCount + 1U);

  geometry_msgs::msg::Pose palm_pose;
  palm_pose.orientation.w = 1.0;
  result.raw_nodes.push_back(rawNode(0, -1, "palm", "palm", palm_pose));
  result.raw_sensor.push_back(palm_pose);

  for (std::size_t index = 0; index < kFingertipCount; ++index) {
    const auto pose = compatibilityPose(fingertip_poses[kOutputSourceIndices[index]]);
    result.raw_nodes.push_back(
      rawNode(kManusNodeIds[index], 0, "tip", kChainTypes[index], pose));
    result.raw_sensor.push_back(pose);
  }

  result.raw_node_count = static_cast<int32_t>(result.raw_nodes.size());
  result.ergonomics_count = 0;
  result.raw_sensor_count = static_cast<int32_t>(result.raw_sensor.size());
  result.raw_sensor_orientation = palm_pose.orientation;
  return result;
}

}  // namespace glove_input_adapter
