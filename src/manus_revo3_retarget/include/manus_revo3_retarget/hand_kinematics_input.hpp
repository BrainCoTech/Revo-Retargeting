#pragma once

#include "hand_teleop_msgs/msg/hand_kinematics.hpp"
#include "manus_revo3_retarget/thumb_retarget.hpp"

namespace manus_revo3_retarget
{

struct HandObservation
{
  JointPositions joints;
  HandLandmarks landmarks;
};

// Validate a complete frame before replacing the last usable observation.
inline std::optional<HandObservation> parse_hand_kinematics(
  const hand_teleop_msgs::msg::HandKinematics & msg,
  const std::string & side,
  const std::string & expected_frame,
  const std::string & spread_joint_suffix,
  std::string & error)
{
  using Message = hand_teleop_msgs::msg::HandKinematics;
  const auto reject = [&](const std::string & reason) -> std::optional<HandObservation> {
      error = reason;
      return std::nullopt;
    };
  if (msg.side != (side == "left" ? Message::LEFT : Message::RIGHT)) {
    return reject("side does not match input topic");
  }
  if (msg.header.frame_id != expected_frame) {
    return reject("expected frame " + expected_frame + ", received " + msg.header.frame_id);
  }
  if (msg.header.stamp.sec < 0 ||
    (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0) ||
    msg.header.stamp.nanosec >= 1000000000u)
  {
    return reject("invalid source timestamp");
  }
  if (msg.joint_names.size() != msg.joint_positions_rad.size() ||
    msg.landmark_names.size() != msg.landmarks_m.size())
  {
    return reject("name/value array lengths differ");
  }
  HandObservation result;
  for (std::size_t i = 0; i < msg.joint_names.size(); ++i) {
    const auto & name = msg.joint_names[i];
    const double value = msg.joint_positions_rad[i];
    if (name.empty() || !std::isfinite(value) || !result.joints.emplace(name, value).second) {
      return reject("empty, duplicate, or non-finite joint: " + name);
    }
  }
  for (std::size_t i = 0; i < msg.landmark_names.size(); ++i) {
    const auto & name = msg.landmark_names[i];
    const auto & point = msg.landmarks_m[i];
    const Eigen::Vector3d value(point.x, point.y, point.z);
    if (name.empty() || !value.allFinite() || !result.landmarks.emplace(name, value).second) {
      return reject("empty, duplicate, or non-finite landmark: " + name);
    }
  }
  for (const auto * finger : {"index", "middle", "ring", "little"}) {
    for (const std::string & joint : {std::string("mcp"), std::string("pip"),
        std::string("dip"), spread_joint_suffix})
    {
      const std::string name = std::string(finger) + "_" + joint;
      if (result.joints.count(name) == 0) {
        return reject("missing joint: " + name);
      }
    }
  }
  for (const auto * name : {"thumb_tip"}) {
    if (result.landmarks.count(name) == 0) {
      return reject(std::string("missing landmark: ") + name);
    }
  }
  error.clear();
  return result;
}

}  // namespace manus_revo3_retarget
