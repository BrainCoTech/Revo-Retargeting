#include <atomic>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <mutex>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <manus_ros2_msgs/msg/manus_ergonomics.hpp>
#include <manus_ros2_msgs/msg/manus_glove.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "glove_input_adapter/humandex_pose_adapter.hpp"

namespace glove_input_adapter
{
namespace
{

constexpr std::array<const char *, 4U> kHumandexFourFingers{
  "index", "middle", "ring", "little"};
constexpr std::array<const char *, 4U> kManusFourFingers{
  "Index", "Middle", "Ring", "Pinky"};
constexpr std::array<const char *, 3U> kStretchJoints{"MCP", "PIP", "DIP"};

}  // namespace

class HumandexPoseAdapterNode final : public rclcpp::Node
{
public:
  HumandexPoseAdapterNode()
  : Node("humandex_pose_adapter_node")
  {
    side_ = declare_parameter<std::string>("side", "right");
    input_hand_mode_ = declare_parameter<std::string>("input_hand_mode", "");
    if (input_hand_mode_.empty()) {
      input_hand_mode_ = side_;
    }

    int64_t glove_id = declare_parameter<int64_t>("glove_id", -1);
    if (glove_id < 0) {
      glove_id = side_ == "left" ? 0 : 1;
    }
    input_topic_ = declare_parameter<std::string>("input_topic", "/humandex_eef_pose");
    joint_topic_ = declare_parameter<std::string>("joint_topic", "");
    if (joint_topic_.empty()) {
      joint_topic_ = "/humandex_" + side_ + "/joint_states";
    }
    output_topic_ = declare_parameter<std::string>("output_topic", "");
    if (output_topic_.empty()) {
      output_topic_ = side_ == "left" ? "/manus_glove_0" : "/manus_glove_1";
    }
    expected_frame_id_ = declare_parameter<std::string>(
      "expected_frame_id", "hand_base_link_local");
    ergonomics_enabled_ = declare_parameter<bool>("ergonomics_enabled", true);
    absolute_source_joint_ = declare_parameter<std::string>("absolute_source_joint", "PIP");
    absolute_open_rad_ = declare_parameter<double>(
      "absolute_open_rad", 0.20943951023931956);
    absolute_closed_rad_ = declare_parameter<double>(
      "absolute_closed_rad", -0.20943951023931956);
    absolute_target_max_deg_ = declare_parameter<double>(
      "absolute_target_max_deg", 84.00134234412998);

    if (
      absolute_source_joint_ != "MPR" && absolute_source_joint_ != "MCP" &&
      absolute_source_joint_ != "PIP" && absolute_source_joint_ != "DIP")
    {
      throw std::invalid_argument(
              "absolute_source_joint must be MPR, MCP, PIP, or DIP");
    }
    // Validate the fixed affine mapping during startup. It is deliberately
    // independent of the first received frame.
    (void)mapAbsoluteJointToTarget(
      absolute_open_rad_, absolute_open_rad_, absolute_closed_rad_,
      absolute_target_max_deg_);

    HumandexAdapterConfig config;
    config.side = side_;
    config.glove_id = static_cast<int32_t>(glove_id);
    adapter_ = std::make_unique<HumandexPoseAdapter>(config);

    publisher_ = create_publisher<manus_ros2_msgs::msg::ManusGlove>(output_topic_, 10);
    subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      input_topic_, 10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr message) {handlePoses(*message);});
    if (ergonomics_enabled_) {
      joint_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_topic_, 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr message) {handleJoints(*message);});
    }

    RCLCPP_INFO(
      get_logger(),
      "HumanDex pose adapter started: side=%s input_hand_mode=%s input=%s joint=%s "
      "output=%s ergonomics=%s absolute_source=%s open=%.6f rad closed=%.6f rad "
      "target_max=%.3f deg",
      side_.c_str(), input_hand_mode_.c_str(), input_topic_.c_str(), joint_topic_.c_str(),
      output_topic_.c_str(), ergonomics_enabled_ ? "true" : "false",
      absolute_source_joint_.c_str(), absolute_open_rad_, absolute_closed_rad_,
      absolute_target_max_deg_);
  }

private:
  void handleJoints(const sensor_msgs::msg::JointState & message)
  {
    if (message.position.size() < message.name.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping HumanDex JointState: position count %zu < name count %zu",
        message.position.size(), message.name.size());
      return;
    }

    std::unordered_map<std::string, double> positions;
    positions.reserve(message.name.size());
    for (std::size_t i = 0; i < message.name.size(); ++i) {
      const double value = message.position[i];
      if (!std::isfinite(value)) {
        continue;
      }
      positions[message.name[i]] = value;
    }

    {
      std::lock_guard<std::mutex> lock(joint_mutex_);
      latest_joint_positions_ = std::move(positions);
      has_joint_positions_ = true;
    }
  }

  double flexDegForFinger(
    const std::string & humandex_finger,
    const std::unordered_map<std::string, double> & latest) const
  {
    const std::string name =
      side_ + "_" + humandex_finger + "_" + absolute_source_joint_ + "_joint";
    const auto latest_it = latest.find(name);
    if (latest_it == latest.end()) {
      return 0.0;
    }
    return mapAbsoluteJointToTarget(
      latest_it->second,
      absolute_open_rad_,
      absolute_closed_rad_,
      absolute_target_max_deg_);
  }

  std::vector<manus_ros2_msgs::msg::ManusErgonomics> buildErgonomics() const
  {
    std::unordered_map<std::string, double> latest;
    {
      std::lock_guard<std::mutex> lock(joint_mutex_);
      if (!has_joint_positions_) {
        return {};
      }
      latest = latest_joint_positions_;
    }

    std::vector<manus_ros2_msgs::msg::ManusErgonomics> ergonomics;
    ergonomics.reserve(kHumandexFourFingers.size() * kStretchJoints.size());
    for (std::size_t finger_index = 0; finger_index < kHumandexFourFingers.size(); ++finger_index) {
      const double flex_deg = flexDegForFinger(kHumandexFourFingers[finger_index], latest);
      for (const char * joint : kStretchJoints) {
        manus_ros2_msgs::msg::ManusErgonomics item;
        item.type = std::string(kManusFourFingers[finger_index]) + joint + "Stretch";
        item.value = static_cast<float>(flex_deg);
        ergonomics.push_back(item);
      }
    }
    return ergonomics;
  }

  void handlePoses(const geometry_msgs::msg::PoseArray & message)
  {
    ++received_frame_count_;
    if (!expected_frame_id_.empty() && message.header.frame_id != expected_frame_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping HumanDex PoseArray with frame_id='%s'; expected '%s'",
        message.header.frame_id.c_str(), expected_frame_id_.c_str());
      return;
    }

    std::vector<geometry_msgs::msg::Pose> fingertip_poses;
    try {
      fingertip_poses = HumandexPoseAdapter::selectHandPoses(
        message.poses, input_hand_mode_, side_);
    } catch (const std::invalid_argument & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Dropping HumanDex PoseArray: %s", error.what());
      return;
    }

    std::string reason;
    if (!adapter_->validateInput(fingertip_poses, &reason)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Dropping invalid HumanDex frame: %s", reason.c_str());
      return;
    }

    auto output = adapter_->adapt(fingertip_poses);
    if (ergonomics_enabled_) {
      output.ergonomics = buildErgonomics();
      output.ergonomics_count = static_cast<int32_t>(output.ergonomics.size());
    }
    publisher_->publish(output);
    const uint64_t count = ++published_frame_count_;
    if (count == 1U || count % 500U == 0U) {
      RCLCPP_INFO(
        get_logger(), "Published %lu adapted HumanDex frames (received=%lu, ergonomics=%zu)",
        static_cast<unsigned long>(count),
        static_cast<unsigned long>(received_frame_count_.load()),
        output.ergonomics.size());
    }
  }

  std::string side_;
  std::string input_hand_mode_;
  std::string input_topic_;
  std::string joint_topic_;
  std::string output_topic_;
  std::string expected_frame_id_;
  bool ergonomics_enabled_{true};
  std::string absolute_source_joint_{"PIP"};
  double absolute_open_rad_{0.20943951023931956};
  double absolute_closed_rad_{-0.20943951023931956};
  double absolute_target_max_deg_{84.00134234412998};
  std::unique_ptr<HumandexPoseAdapter> adapter_;
  rclcpp::Publisher<manus_ros2_msgs::msg::ManusGlove>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  mutable std::mutex joint_mutex_;
  std::unordered_map<std::string, double> latest_joint_positions_;
  bool has_joint_positions_{false};
  std::atomic<uint64_t> received_frame_count_{0};
  std::atomic<uint64_t> published_frame_count_{0};
};

}  // namespace glove_input_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<glove_input_adapter::HumandexPoseAdapterNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("humandex_pose_adapter_node"), "%s", error.what());
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("humandex_pose_adapter_node"), "Unknown fatal error");
  }
  rclcpp::shutdown();
  return 0;
}
