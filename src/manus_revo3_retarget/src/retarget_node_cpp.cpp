#include <array>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "manus_ros2_msgs/msg/manus_glove.hpp"
#include "rclcpp/rclcpp.hpp"
#include "revo3_mit_controller_msgs/msg/revo3_mit_command.hpp"

#include <dlfcn.h>

#include "manus_revo3_retarget_cpp/four_finger_retarget.hpp"
#include "manus_revo3_retarget_cpp/spread_retarget.hpp"
#include "manus_revo3_retarget_cpp/thumb_retarget.hpp"

namespace manus_revo3_retarget_cpp
{

using ManusGlove = manus_ros2_msgs::msg::ManusGlove;
using Revo3MITCommand = revo3_mit_controller_msgs::msg::Revo3MITCommand;

class ThumbPlugin
{
public:
  explicit ThumbPlugin(const std::string & library_path)
  {
    library_ = dlopen(library_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (library_ == nullptr) {
      throw std::runtime_error("dlopen failed for " + library_path + ": " + dlerror_string());
    }
    create_ = load_symbol<CreateFn>("manus_revo3_thumb_create");
    destroy_ = load_symbol<DestroyFn>("manus_revo3_thumb_destroy");
    initialize_ = load_symbol<InitializeFn>("manus_revo3_thumb_initialize");
    set_config_ = load_symbol<SetConfigFn>("manus_revo3_thumb_set_config");
    apply_ = load_symbol<ApplyFn>("manus_revo3_thumb_apply");
    last_iteration_count_ = load_symbol<LastIterationCountFn>("manus_revo3_thumb_last_iteration_count");
    handle_ = create_();
    if (handle_ == nullptr) {
      throw std::runtime_error("thumb plugin create returned null");
    }
  }

  ~ThumbPlugin()
  {
    if (handle_ != nullptr && destroy_ != nullptr) {
      destroy_(handle_);
    }
    if (library_ != nullptr) {
      dlclose(library_);
    }
  }

  ThumbPlugin(const ThumbPlugin &) = delete;
  ThumbPlugin & operator=(const ThumbPlugin &) = delete;

  bool initialize(const std::string & model_base, const std::string & side, std::string * error)
  {
    return initialize_(handle_, model_base.c_str(), side.c_str(), error);
  }

  void set_config(const ThumbConfig & config)
  {
    set_config_(handle_, &config);
  }

  void apply(const Ergonomics & ergonomics, const ManusKeypoints & keypoints, JointArray & q)
  {
    apply_(handle_, &ergonomics, &keypoints, &q);
  }

  int last_iteration_count() const
  {
    return last_iteration_count_(handle_);
  }

private:
  using CreateFn = void * (*)();
  using DestroyFn = void (*)(void *);
  using InitializeFn = bool (*)(void *, const char *, const char *, std::string *);
  using SetConfigFn = void (*)(void *, const ThumbConfig *);
  using ApplyFn = void (*)(void *, const Ergonomics *, const ManusKeypoints *, JointArray *);
  using LastIterationCountFn = int (*)(void *);

  static std::string dlerror_string()
  {
    const char * error = dlerror();
    return error != nullptr ? std::string(error) : std::string("unknown dlopen/dlsym error");
  }

  template<typename T>
  T load_symbol(const char * name)
  {
    dlerror();
    void * symbol = dlsym(library_, name);
    const char * error = dlerror();
    if (error != nullptr || symbol == nullptr) {
      throw std::runtime_error(std::string("dlsym failed for ") + name + ": " + dlerror_string());
    }
    return reinterpret_cast<T>(symbol);
  }

  void * library_{nullptr};
  void * handle_{nullptr};
  CreateFn create_{nullptr};
  DestroyFn destroy_{nullptr};
  InitializeFn initialize_{nullptr};
  SetConfigFn set_config_{nullptr};
  ApplyFn apply_{nullptr};
  LastIterationCountFn last_iteration_count_{nullptr};
};

struct SideState
{
  std::string side;
  std::vector<std::string> names;
  rclcpp::Publisher<Revo3MITCommand>::SharedPtr command_pub;
  rclcpp::Publisher<Revo3MITCommand>::SharedPtr target_pub;
  std::mutex mutex;
  std::optional<Revo3MITCommand> latest;
  FourFingerRetarget four_finger;
  SpreadRetarget spread;
  std::unique_ptr<ThumbPlugin> thumb;
};

class RetargetNodeCpp : public rclcpp::Node
{
public:
  explicit RetargetNodeCpp(const rclcpp::NodeOptions & options)
  : Node("manus_revo3_retarget", options)
  {
    hand_mode_ = string_param("hand_mode", "both");
    use_revo3_namespace_ = bool_param("use_revo3_namespace", true);
    command_topic_suffix_ = string_param("command_topic_suffix", "joint_forward_mit_controller/commands");
    target_topic_suffix_ = string_param("retarget_target_topic_suffix", "joint_forward_mit_controller/retarget_targets");
    mit_command_publish_hz_ = double_param("mit_command_publish_hz", 200.0);
    mit_default_kp_ = double_param("mit_default_kp", 0.4);
    mit_default_kd_ = double_param("mit_default_kd", 0.05);

    if (hand_mode_ != "left" && hand_mode_ != "right" && hand_mode_ != "both") {
      throw std::runtime_error("hand_mode must be left, right, or both");
    }

    if (hand_mode_ == "left" || hand_mode_ == "both") {
      left_ = create_side("left");
    }
    if (hand_mode_ == "right" || hand_mode_ == "both") {
      right_ = create_side("right");
    }

    sub_0_ = create_subscription<ManusGlove>(
      "/manus_glove_0", 10, [this](ManusGlove::SharedPtr msg) { on_glove(*msg); });
    sub_1_ = create_subscription<ManusGlove>(
      "/manus_glove_1", 10, [this](ManusGlove::SharedPtr msg) { on_glove(*msg); });

    const double period_s = 1.0 / std::max(1.0, mit_command_publish_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s)),
      [this]() { publish_latest(); });

    RCLCPP_INFO(get_logger(), "C++ retarget node ready hand_mode=%s command_hz=%.1f", hand_mode_.c_str(),
      mit_command_publish_hz_);
  }

private:
  std::shared_ptr<SideState> create_side(const std::string & side)
  {
    auto state = std::make_shared<SideState>();
    state->side = side;
    state->names = joint_names(side);
    state->command_pub = create_publisher<Revo3MITCommand>(command_topic(side), 10);
    state->target_pub = create_publisher<Revo3MITCommand>(target_topic(side), 10);

    state->four_finger.set_config(load_four_finger_config(side));
    state->spread.set_config(load_spread_config(side));
    state->thumb = std::make_unique<ThumbPlugin>(thumb_plugin_path());
    state->thumb->set_config(load_thumb_config(side));
    const std::string model_base = model_base_path();
    RCLCPP_INFO(get_logger(), "C++ %s thumb Pinocchio description base: %s", side.c_str(), model_base.c_str());
    std::string thumb_error;
    if (!state->thumb->initialize(model_base, side, &thumb_error)) {
      throw std::runtime_error("failed to initialize thumb Pinocchio IK for " + side + ": " + thumb_error);
    }
    RCLCPP_INFO(get_logger(), "C++ %s thumb Pinocchio IK initialized", side.c_str());

    RCLCPP_INFO(get_logger(), "C++ %s retarget -> %s", side.c_str(), command_topic(side).c_str());
    return state;
  }

  FourFingerConfig load_four_finger_config(const std::string & side)
  {
    const std::string p = "legacy_" + side + "_physical_";
    FourFingerConfig cfg;
    cfg.index_angle_scale = double_param(p + "index_angle_scale", 1.0);
    cfg.four_finger_mcp_scale = double_param(p + "four_finger_mcp_scale", 1.0);
    cfg.middle_ring_dip_scale = double_param(p + "middle_ring_dip_scale", 1.0);
    cfg.pinky_angle_scale = double_param(p + "pinky_angle_scale", 1.0);
    cfg.pinky_dip_pip_scale = double_param(p + "pinky_dip_pip_scale", 1.0);
    cfg.pinky_mcp_scale = double_param(p + "pinky_mcp_scale", 1.0);
    cfg.all_finger_angle_scale = double_param(p + "all_finger_angle_scale", 1.0);
    return cfg;
  }

  SpreadConfig load_spread_config(const std::string & side)
  {
    const std::string p = "legacy_" + side + "_physical_";
    SpreadConfig cfg;
    cfg.index_offset_deg = double_param(p + "index_spread_offset_deg", 0.0);
    cfg.middle_offset_deg = double_param(p + "middle_spread_offset_deg", 0.0);
    cfg.ring_offset_deg = double_param(p + "ring_spread_offset_deg", 0.0);
    cfg.pinky_offset_deg = double_param(p + "pinky_spread_offset_deg", 0.0);
    cfg.index_scale = double_param(p + "index_spread_scale", 1.0);
    cfg.middle_scale = double_param(p + "middle_spread_scale", 1.0);
    cfg.ring_scale = double_param(p + "ring_spread_scale", 1.0);
    cfg.pinky_scale = double_param(p + "pinky_spread_scale", 1.0);
    cfg.middle_dynamic = bool_param(p + "middle_spread_dynamic", false);
    cfg.ring_forward_scale = double_param(p + "ring_spread_forward_scale", 1.0);
    cfg.ring_backward_scale = double_param(p + "ring_spread_backward_scale", 1.0);
    cfg.finger_spread_sign = -1.0;
    return cfg;
  }

  ThumbConfig load_thumb_config(const std::string & side)
  {
    const std::string p = "legacy_" + side + "_physical_";
    ThumbConfig cfg;
    cfg.joint_offset_deg = double_param(p + "thumb_joint_offset_deg", 0.0);
    cfg.cmp_offset_deg = double_param(side + "_thumb_cmp_offset_deg_physical", 0.0);
    cfg.cmp_scale = double_param(side + "_thumb_cmp_scale_physical", 1.0);
    cfg.cmr_offset_deg = double_param(p + "thumb_cmr_offset_deg", 0.0);
    cfg.mcp_offset_deg = double_param(p + "thumb_mcp_offset_deg", 0.0);
    cfg.mcp_scale = double_param(p + "thumb_mcp_scale", 1.0);
    cfg.pip_scale = double_param(p + "thumb_pip_scale", 1.0);
    cfg.dip_scale = double_param(p + "thumb_dip_scale", 1.0);
    cfg.spread_sign = side == "left" ? 1.0 : -1.0;
    cfg.manus_out_y_sign = -1.0;
    cfg.reach_scale = double_param(p + "thumb_reach_scale", 1.0);
    cfg.ik_position_scale = double_param(p + "thumb_ik_position_scale", 1.0);
    cfg.pip_ik_scale = double_param(p + "thumb_pip_ik_scale", 1.0);
    cfg.dip_ik_scale = double_param(p + "thumb_dip_ik_scale", 1.0);
    cfg.ema_prev = side == "left" ? 0.9 : 0.4;
    cfg.ema_cur = side == "left" ? 0.1 : 0.6;
    cfg.ik_posture_weight = double_param("thumb_ik_posture_weight", 0.1);
    cfg.ik_smooth_weight = double_param("thumb_ik_smooth_weight", 0.1);
    cfg.ik_max_iterations = int_param("thumb_ik_max_iterations", 15);
    cfg.ik_max_step_rad = deg_to_rad(double_param("thumb_ik_max_step_deg", 3.0));
    cfg.ik_max_frame_delta_rad = deg_to_rad(double_param("thumb_ik_max_frame_delta_deg", 6.0));
    cfg.ik_damping = double_param("thumb_ik_damping", 0.02);
    cfg.ik_step_size = double_param("thumb_ik_step_size", 0.30);
    cfg.ik_tolerance = double_param("thumb_ik_tolerance", 5e-4);
    return cfg;
  }

  void on_glove(const ManusGlove & msg)
  {
    std::string side = msg.side;
    for (auto & ch : side) {
      ch = static_cast<char>(std::tolower(ch));
    }

    std::shared_ptr<SideState> state;
    if ((side == "left" || side == "l") && left_) {
      state = left_;
    } else if ((side == "right" || side == "r") && right_) {
      state = right_;
    } else {
      return;
    }

    state->four_finger.set_config(load_four_finger_config(state->side));
    state->spread.set_config(load_spread_config(state->side));
    state->thumb->set_config(load_thumb_config(state->side));

    Ergonomics ergonomics;
    ergonomics.reserve(msg.ergonomics.size());
    for (const auto & item : msg.ergonomics) {
      ergonomics[item.type] = static_cast<double>(item.value);
    }

    ManusKeypoints keypoints;
    for (const auto & raw_node : msg.raw_nodes) {
      const int node_id = raw_node.node_id;
      if (node_id < 0 || node_id >= static_cast<int>(keypoints.size())) {
        continue;
      }
      const auto & pos = raw_node.pose.position;
      keypoints[static_cast<std::size_t>(node_id)] = Eigen::Vector3d(
        static_cast<double>(pos.x), static_cast<double>(pos.y), static_cast<double>(pos.z));
    }

    JointArray q{};
    q.fill(0.0);
    state->four_finger.apply(ergonomics, q);
    state->spread.apply(ergonomics, q);
    state->thumb->apply(ergonomics, keypoints, q);

    Revo3MITCommand out;
    out.header.stamp = now();
    out.joint_names = state->names;
    out.position.assign(q.begin(), q.end());
    apply_output_calibration(state->side, state->names, out.position);
    out.velocity.assign(state->names.size(), 0.0);
    out.effort.assign(state->names.size(), 0.0);
    mit_default_kp_ = double_param("mit_default_kp", mit_default_kp_);
    mit_default_kd_ = double_param("mit_default_kd", mit_default_kd_);
    out.kp.assign(state->names.size(), mit_default_kp_);
    out.kd.assign(state->names.size(), mit_default_kd_);

    state->target_pub->publish(out);
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->latest = out;
    }
  }

  void publish_latest()
  {
    publish_latest(left_);
    publish_latest(right_);
  }

  void publish_latest(const std::shared_ptr<SideState> & state)
  {
    if (!state) {
      return;
    }
    std::optional<Revo3MITCommand> msg;
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      msg = state->latest;
    }
    if (!msg) {
      return;
    }
    msg->header.stamp = now();
    state->command_pub->publish(*msg);
  }

  void apply_output_calibration(
    const std::string & side,
    const std::vector<std::string> & names,
    std::vector<double> & positions)
  {
    const std::string prefix = side + "_";
    for (std::size_t i = 0; i < names.size() && i < positions.size(); ++i) {
      std::string suffix = names[i];
      if (suffix.rfind(prefix, 0) == 0) {
        suffix = suffix.substr(prefix.size());
      }
      const double scale = double_param("physical_" + side + "_" + suffix + "_scale", 1.0);
      const double offset = deg_to_rad(double_param("physical_" + side + "_" + suffix + "_offset_deg", 0.0));
      positions[i] = positions[i] * scale + offset;
    }
  }

  std::string command_topic(const std::string & side) const
  {
    return topic_for(side, command_topic_suffix_, "joint_forward_mit_controller/commands");
  }

  std::string target_topic(const std::string & side) const
  {
    return topic_for(side, target_topic_suffix_, "joint_forward_mit_controller/retarget_targets");
  }

  std::string topic_for(const std::string & side, std::string suffix, const std::string & fallback) const
  {
    if (suffix.empty()) {
      suffix = fallback;
    }
    while (!suffix.empty() && suffix.front() == '/') {
      suffix.erase(suffix.begin());
    }
    if (use_revo3_namespace_) {
      return "/revo3_" + side + "/" + suffix;
    }
    return "/" + suffix;
  }

  double double_param(const std::string & name, double fallback)
  {
    if (!has_parameter(name)) {
      declare_parameter<double>(name, fallback);
    }
    double value = fallback;
    get_parameter(name, value);
    return finite_or(value, fallback);
  }

  bool bool_param(const std::string & name, bool fallback)
  {
    if (!has_parameter(name)) {
      declare_parameter<bool>(name, fallback);
    }
    bool value = fallback;
    get_parameter(name, value);
    return value;
  }

  int int_param(const std::string & name, int fallback)
  {
    if (!has_parameter(name)) {
      declare_parameter<int>(name, fallback);
    }
    int value = fallback;
    get_parameter(name, value);
    return value;
  }

  std::string string_param(const std::string & name, const std::string & fallback)
  {
    if (!has_parameter(name)) {
      declare_parameter<std::string>(name, fallback);
    }
    std::string value = fallback;
    get_parameter(name, value);
    return value.empty() ? fallback : value;
  }

  std::string model_base_path()
  {
    const char * env_path = std::getenv("REVO3_MODEL_PATH");
    if (env_path != nullptr && std::string(env_path).size() > 0) {
      return std::string(env_path);
    }
    return ament_index_cpp::get_package_share_directory("revo3_description");
  }

  std::string thumb_plugin_path()
  {
    return (std::filesystem::path(ament_index_cpp::get_package_prefix("manus_revo3_retarget")) / "lib" /
      "libmanus_revo3_retarget_thumb_pinocchio.so").string();
  }

  std::string hand_mode_;
  bool use_revo3_namespace_{true};
  std::string command_topic_suffix_;
  std::string target_topic_suffix_;
  double mit_command_publish_hz_{200.0};
  double mit_default_kp_{0.4};
  double mit_default_kd_{0.05};
  std::shared_ptr<SideState> left_;
  std::shared_ptr<SideState> right_;
  rclcpp::Subscription<ManusGlove>::SharedPtr sub_0_;
  rclcpp::Subscription<ManusGlove>::SharedPtr sub_1_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace manus_revo3_retarget_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.enable_rosout(false);
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  auto node = std::make_shared<manus_revo3_retarget_cpp::RetargetNodeCpp>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
