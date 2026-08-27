#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "glove_input_adapter/humandex_pose_adapter.hpp"

namespace glove_input_adapter
{
namespace
{

constexpr std::array<const char *, 4U> kHumanFourFingers{
  "index", "middle", "ring", "little"};
constexpr std::array<const char *, 4U> kRevoFourFingers{
  "index", "middle", "ring", "pinky"};

}  // namespace

class HumandexAbsoluteRevo2Node final : public rclcpp::Node
{
public:
  HumandexAbsoluteRevo2Node()
  : Node("humandex_absolute_revo2_node")
  {
    side_ = declare_parameter<std::string>("side", "left");
    if (side_ != "left" && side_ != "right") {
      throw std::invalid_argument("side must be left or right");
    }
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/humandex_" + side_ + "/joint_states");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/revo2_" + side_ + "/revo2_pid_controller/target_joint_states");

    four_source_joint_ = declare_parameter<std::string>("four_source_joint", "PIP");
    four_open_rad_ = declare_parameter<double>("four_open_rad", 0.20943951023931956);
    four_closed_rad_ = declare_parameter<double>("four_closed_rad", -0.20943951023931956);
    four_target_max_rad_ = declare_parameter<double>("four_target_max_rad", 1.4661);

    thumb_prox_source_joint_ = declare_parameter<std::string>(
      "thumb_prox_source_joint", "CMR");
    thumb_prox_open_rad_ = declare_parameter<double>(
      "thumb_prox_open_rad", -0.20943951023931956);
    thumb_prox_closed_rad_ = declare_parameter<double>(
      "thumb_prox_closed_rad", 0.20943951023931956);
    thumb_prox_target_max_rad_ = declare_parameter<double>(
      "thumb_prox_target_max_rad", 1.0472);

    thumb_meta_source_joint_ = declare_parameter<std::string>(
      "thumb_meta_source_joint", "CMP");
    thumb_meta_open_rad_ = declare_parameter<double>(
      "thumb_meta_open_rad", -0.10471975511965977);
    thumb_meta_closed_rad_ = declare_parameter<double>(
      "thumb_meta_closed_rad", -0.41887902047863906);
    thumb_meta_target_max_rad_ = declare_parameter<double>(
      "thumb_meta_target_max_rad", 1.5184);

    validateMapping(
      four_open_rad_, four_closed_rad_, four_target_max_rad_, "four-finger");
    validateMapping(
      thumb_prox_open_rad_, thumb_prox_closed_rad_,
      thumb_prox_target_max_rad_, "thumb proximal");
    validateMapping(
      thumb_meta_open_rad_, thumb_meta_closed_rad_,
      thumb_meta_target_max_rad_, "thumb metacarpal");

    publisher_ = create_publisher<sensor_msgs::msg::JointState>(output_topic_, 20);
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      input_topic_, 20,
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {handleJoints(*message);});

    RCLCPP_INFO(
      get_logger(),
      "HumanDex absolute Revo2 mapping: side=%s input=%s output=%s; "
      "four %s %.6f->%.6f rad; thumb_prox %s %.6f->%.6f rad; "
      "thumb_meta %s %.6f->%.6f rad",
      side_.c_str(), input_topic_.c_str(), output_topic_.c_str(),
      four_source_joint_.c_str(), four_open_rad_, four_closed_rad_,
      thumb_prox_source_joint_.c_str(), thumb_prox_open_rad_, thumb_prox_closed_rad_,
      thumb_meta_source_joint_.c_str(), thumb_meta_open_rad_, thumb_meta_closed_rad_);
  }

private:
  static void validateMapping(double open_rad, double closed_rad, double target_max, const char * label)
  {
    try {
      (void)mapAbsoluteJointToTarget(open_rad, open_rad, closed_rad, target_max);
    } catch (const std::invalid_argument & error) {
      throw std::invalid_argument(std::string(label) + " mapping: " + error.what());
    }
  }

  bool sourceValue(
    const std::unordered_map<std::string, double> & positions,
    const std::string & name,
    double * value)
  {
    const auto iterator = positions.find(name);
    if (iterator == positions.end() || !std::isfinite(iterator->second)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping HumanDex frame: missing finite source joint %s", name.c_str());
      return false;
    }
    *value = iterator->second;
    return true;
  }

  void handleJoints(const sensor_msgs::msg::JointState & input)
  {
    if (input.position.size() < input.name.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping HumanDex JointState: position count %zu < name count %zu",
        input.position.size(), input.name.size());
      return;
    }
    std::unordered_map<std::string, double> positions;
    positions.reserve(input.name.size());
    for (std::size_t index = 0; index < input.name.size(); ++index) {
      positions[input.name[index]] = input.position[index];
    }

    double thumb_prox = 0.0;
    double thumb_meta = 0.0;
    const std::string thumb_prox_name =
      side_ + "_thumb_" + thumb_prox_source_joint_ + "_joint";
    const std::string thumb_meta_name =
      side_ + "_thumb_" + thumb_meta_source_joint_ + "_joint";
    if (
      !sourceValue(positions, thumb_prox_name, &thumb_prox) ||
      !sourceValue(positions, thumb_meta_name, &thumb_meta))
    {
      return;
    }

    std::array<double, 4U> four_values{};
    for (std::size_t index = 0; index < kHumanFourFingers.size(); ++index) {
      const std::string source_name =
        side_ + "_" + kHumanFourFingers[index] + "_" + four_source_joint_ + "_joint";
      if (!sourceValue(positions, source_name, &four_values[index])) {
        return;
      }
    }

    sensor_msgs::msg::JointState output;
    output.header = input.header;
    output.name.reserve(6U);
    output.position.reserve(6U);
    output.name.push_back(side_ + "_thumb_proximal_joint");
    output.position.push_back(mapAbsoluteJointToTarget(
      thumb_prox, thumb_prox_open_rad_, thumb_prox_closed_rad_,
      thumb_prox_target_max_rad_));
    output.name.push_back(side_ + "_thumb_metacarpal_joint");
    output.position.push_back(mapAbsoluteJointToTarget(
      thumb_meta, thumb_meta_open_rad_, thumb_meta_closed_rad_,
      thumb_meta_target_max_rad_));
    for (std::size_t index = 0; index < kRevoFourFingers.size(); ++index) {
      output.name.push_back(
        side_ + "_" + kRevoFourFingers[index] + "_proximal_joint");
      output.position.push_back(mapAbsoluteJointToTarget(
        four_values[index], four_open_rad_, four_closed_rad_, four_target_max_rad_));
    }
    publisher_->publish(output);

    const auto count = ++published_frames_;
    if (count == 1U || count % 500U == 0U) {
      RCLCPP_INFO(
        get_logger(), "Published %lu absolute Revo2 target frames",
        static_cast<unsigned long>(count));
    }
  }

  std::string side_;
  std::string input_topic_;
  std::string output_topic_;
  std::string four_source_joint_;
  double four_open_rad_{0.0};
  double four_closed_rad_{0.0};
  double four_target_max_rad_{0.0};
  std::string thumb_prox_source_joint_;
  double thumb_prox_open_rad_{0.0};
  double thumb_prox_closed_rad_{0.0};
  double thumb_prox_target_max_rad_{0.0};
  std::string thumb_meta_source_joint_;
  double thumb_meta_open_rad_{0.0};
  double thumb_meta_closed_rad_{0.0};
  double thumb_meta_target_max_rad_{0.0};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::atomic<uint64_t> published_frames_{0};
};

}  // namespace glove_input_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<glove_input_adapter::HumandexAbsoluteRevo2Node>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("humandex_absolute_revo2_node"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
