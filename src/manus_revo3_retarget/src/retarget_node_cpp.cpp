#include <array>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "hand_teleop_msgs/msg/hand_kinematics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "revo3_mit_controller_msgs/msg/revo3_mit_command.hpp"

#include <dlfcn.h>

#include "manus_revo3_retarget/four_finger_retarget.hpp"
#include "manus_revo3_retarget/hand_kinematics_input.hpp"
#include "manus_revo3_retarget/spread_retarget.hpp"
#include "manus_revo3_retarget/thumb_retarget.hpp"

namespace manus_revo3_retarget
{

using HandKinematics = hand_teleop_msgs::msg::HandKinematics;
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
    diagnostics_ = load_symbol<DiagnosticsFn>("manus_revo3_thumb_diagnostics");
    limits_ = load_symbol<LimitsFn>("manus_revo3_thumb_joint_limits");
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

  void apply(const JointPositions & joints, const HandLandmarks & landmarks, JointArray & q)
  {
    apply_(handle_, &joints, &landmarks, &q);
  }

  void joint_limits(const std::string & side, JointArray & lower, JointArray & upper) const
  { limits_(handle_, side.c_str(), &lower, &upper); }

  ThumbDiagnostics diagnostics() const
  { ThumbDiagnostics out; diagnostics_(handle_, &out); return out; }

  int last_iteration_count() const
  {
    return last_iteration_count_(handle_);
  }

private:
  using DiagnosticsFn = void (*)(void *, ThumbDiagnostics *);
  DiagnosticsFn diagnostics_{nullptr};
  using LimitsFn = void (*)(void *, const char *, JointArray *, JointArray *);
  LimitsFn limits_{nullptr};
  using CreateFn = void * (*)();
  using DestroyFn = void (*)(void *);
  using InitializeFn = bool (*)(void *, const char *, const char *, std::string *);
  using SetConfigFn = void (*)(void *, const ThumbConfig *);
  using ApplyFn = void (*)(void *, const JointPositions *, const HandLandmarks *, JointArray *);
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
  std::optional<Revo3MITCommand> latest_target;
  std::vector<double> start_position;
  std::vector<double> target_position;
  std::vector<double> target_velocity;
  rclcpp::Time start_time;
  rclcpp::Time end_time;
  rclcpp::Time last_target_time;
  bool has_segment{false};
  bool has_last_target{false};
  JointArray lower;
  JointArray upper;
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
    for (const auto & name : list_parameters({}, 100).names) {
      const bool obsolete = name == "thumb_ik_max_frame_delta_deg" || name == "thumb_ik_tolerance" ||
        name.find("_thumb_cmp_scale_physical") != std::string::npos ||
        name.find("_thumb_cmp_offset_deg_physical") != std::string::npos ||
        (name.rfind("legacy_", 0) == 0 && (
          name.find("_thumb_joint_offset_deg") != std::string::npos ||
          name.find("_thumb_cmr_offset_deg") != std::string::npos ||
          name.find("_thumb_mcp_offset_deg") != std::string::npos ||
          name.find("_thumb_mcp_scale") != std::string::npos ||
          name.find("_thumb_pip_scale") != std::string::npos ||
          name.find("_thumb_dip_scale") != std::string::npos ||
          name.find("_thumb_pip_ik_scale") != std::string::npos ||
          name.find("_thumb_dip_ik_scale") != std::string::npos ||
          name.find("_thumb_reach_scale") != std::string::npos ||
          name.find("_thumb_ema_") != std::string::npos));
      if (obsolete) { throw std::runtime_error("Removed parameter: " + name + "; migrate the old YAML layers first"); }
    }
    hand_mode_ = string_param("hand_mode", "both");
    use_revo3_namespace_ = bool_param("use_revo3_namespace", true);
    command_topic_suffix_ = string_param("command_topic_suffix", "joint_forward_mit_controller/commands");
    target_topic_suffix_ = string_param("retarget_target_topic_suffix", "joint_forward_mit_controller/retarget_targets");
    mit_command_publish_hz_ = double_param("mit_command_publish_hz", 200.0);
    mit_velocity_feedforward_enabled_ = bool_param("mit_velocity_feedforward_enabled", true);
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

    if (left_) {
      left_input_ = create_subscription<HandKinematics>(
        string_param("left_input_topic", "/hand_kinematics/left"), 20,
        [this](HandKinematics::SharedPtr msg) { on_hand_kinematics(*msg, left_); });
    }
    if (right_) {
      right_input_ = create_subscription<HandKinematics>(
        string_param("right_input_topic", "/hand_kinematics/right"), 20,
        [this](HandKinematics::SharedPtr msg) { on_hand_kinematics(*msg, right_); });
    }

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
    state->thumb->joint_limits(side, state->lower, state->upper);
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
    cfg.joint_suffix = string_param("spread_joint_suffix", "spread");
    cfg.relative_to_middle = bool_param("spread_relative_to_middle", true);
    cfg.finger_spread_sign = double_param("finger_spread_sign", -1.0);
    return cfg;
  }

  ThumbConfig load_thumb_config(const std::string & side)
  {
    const std::string p = "legacy_" + side + "_physical_";
    ThumbConfig cfg;
    cfg.cmr_joint_name = string_param("thumb_cmr_joint_name", "thumb_mcp_spread");
    cfg.spread_sign = double_param(side + "_thumb_cmr_input_sign", side == "left" ? 1.0 : -1.0);
    cfg.ik_position_scale = double_param(p + "thumb_ik_position_scale", 1.0);
    cfg.position_sigma_m = double_param("thumb_ik_position_sigma_m", 0.01);
    cfg.posture_sigma_rad = deg_to_rad(double_param("thumb_ik_posture_sigma_deg", 10.0));
    cfg.smooth_sigma_rad = deg_to_rad(double_param("thumb_ik_smooth_sigma_deg", 10.0));
    cfg.tip_weight = double_param("thumb_ik_tip_weight", 0.0004);
    cfg.pip_weight = double_param(side + "_thumb_ik_pip_weight", 0.000001);
    cfg.dip_weight = double_param(side + "_thumb_ik_dip_weight", 0.000001);
    const std::array<std::string, 5> posture_names = {"cmp", "cmr", "mcp", "pip", "dip"};
    for (std::size_t i = 0; i < posture_names.size(); ++i) {
      cfg.posture_joint_weights[i] = double_param("thumb_ik_posture_" + posture_names[i] + "_weight", cfg.posture_joint_weights[i]);
    }
    cfg.ik_posture_weight = double_param("thumb_ik_posture_weight", 0.00030461741978670857);
    cfg.ik_smooth_weight = double_param("thumb_ik_smooth_weight", 0.00030461741978670857);
    cfg.ik_max_iterations = int_param("thumb_ik_max_iterations", 15);
    cfg.ik_max_step_rad = deg_to_rad(double_param("thumb_ik_max_step_deg", 3.0));
    cfg.ik_damping = double_param("thumb_ik_damping", 0.02);
    cfg.ik_step_size = double_param("thumb_ik_step_size", 0.30);
    cfg.ik_tolerance = double_param("thumb_ik_normalized_tolerance", 5e-4);
    return cfg;
  }

  void on_hand_kinematics(
    const HandKinematics & msg, const std::shared_ptr<SideState> & state)
  {
    const auto spread_config = load_spread_config(state->side);
    std::string error;
    const auto observation = parse_hand_kinematics(
      msg, state->side,
      string_param(state->side + "_input_frame", "hand_retarget_" + state->side),
      spread_config.joint_suffix, error);
    if (!observation) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Dropping %s HandKinematics: %s", state->side.c_str(), error.c_str());
      return;
    }
    state->four_finger.set_config(load_four_finger_config(state->side));
    state->spread.set_config(spread_config);
    state->thumb->set_config(load_thumb_config(state->side));
    const auto & joints = observation->joints;
    const auto & landmarks = observation->landmarks;

    JointArray q{};
    q.fill(0.0);
    state->four_finger.apply(joints, q);
    state->spread.apply(joints, q);
    state->thumb->apply(joints, landmarks, q);
    const auto diagnostics = state->thumb->diagnostics();
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
      "%s thumb IK: tip_error_m=%.6f posture_rms_rad=%.6f normalized_residual=%.6f iterations=%d",
      state->side.c_str(), diagnostics.tip_error_m, diagnostics.posture_error_rad,
      diagnostics.normalized_residual, state->thumb->last_iteration_count());

    Revo3MITCommand out;
    out.header.stamp = now();
    out.joint_names = state->names;
    out.position.assign(q.begin(), q.end());
    apply_output_calibration(*state, out.position);
    out.velocity.assign(state->names.size(), 0.0);
    out.effort.assign(state->names.size(), 0.0);
    mit_velocity_feedforward_enabled_ = bool_param(
      "mit_velocity_feedforward_enabled", mit_velocity_feedforward_enabled_);
    mit_default_kp_ = double_param("mit_default_kp", mit_default_kp_);
    mit_default_kd_ = double_param("mit_default_kd", mit_default_kd_);
    out.kp = mit_gains_for_joints(state->names, "kp", mit_default_kp_);
    out.kd = mit_gains_for_joints(state->names, "kd", mit_default_kd_);

    update_interpolation_target(state, out);
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (state->latest_target) {
        out = *state->latest_target;
      }
    }
    state->target_pub->publish(out);
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
      msg = sample_command_locked(*state, now());
    }
    if (!msg) {
      return;
    }
    state->command_pub->publish(*msg);
  }

  void update_interpolation_target(
    const std::shared_ptr<SideState> & state,
    Revo3MITCommand & target)
  {
    const rclcpp::Time target_time(target.header.stamp);
    std::lock_guard<std::mutex> lock(state->mutex);

    const std::vector<double> current_position = sample_position_locked(*state, target_time);
    const std::vector<double> previous_target = state->target_position;
    const bool compatible =
      state->has_segment &&
      state->latest_target &&
      state->latest_target->joint_names == target.joint_names &&
      previous_target.size() == target.position.size();

    double duration_s = default_interpolation_duration_s();
    std::vector<double> target_velocity(target.position.size(), 0.0);
    if (state->has_last_target) {
      const double interval_s = (target_time - state->last_target_time).seconds();
      if (std::isfinite(interval_s) && interval_s > min_duration_s()) {
        duration_s = interval_s;
        if (compatible) {
          for (std::size_t i = 0; i < target.position.size(); ++i) {
            target_velocity[i] = (target.position[i] - previous_target[i]) / interval_s;
          }
        }
      }
    }

    state->start_position = compatible ? current_position : target.position;
    state->target_position = target.position;
    state->target_velocity = target_velocity;
    state->start_time = target_time;
    state->end_time = target_time + rclcpp::Duration::from_seconds(std::max(duration_s, min_duration_s()));
    state->last_target_time = target_time;
    state->has_last_target = true;
    state->has_segment = true;

    target.velocity = velocity_feedforward_enabled() ?
      target_velocity : std::vector<double>(target.position.size(), 0.0);
    state->latest_target = target;
  }

  std::optional<Revo3MITCommand> sample_command_locked(SideState & state, const rclcpp::Time & sample_time)
  {
    if (!state.has_segment || !state.latest_target) {
      return std::nullopt;
    }
    Revo3MITCommand out = *state.latest_target;
    out.header.stamp = sample_time;
    out.position = sample_position_locked(state, sample_time);
    out.velocity = sample_velocity_locked(state);
    return out;
  }

  std::vector<double> sample_position_locked(const SideState & state, const rclcpp::Time & sample_time) const
  {
    if (!state.has_segment || state.start_position.size() != state.target_position.size()) {
      return state.target_position;
    }
    const double duration_s = (state.end_time - state.start_time).seconds();
    if (duration_s <= min_duration_s() || sample_time >= state.end_time) {
      return state.target_position;
    }
    const double alpha = std::clamp((sample_time - state.start_time).seconds() / duration_s, 0.0, 1.0);
    std::vector<double> position(state.target_position.size(), 0.0);
    for (std::size_t i = 0; i < position.size(); ++i) {
      position[i] = state.start_position[i] + alpha * (state.target_position[i] - state.start_position[i]);
    }
    return position;
  }

  std::vector<double> sample_velocity_locked(const SideState & state) const
  {
    if (!state.has_segment) {
      return std::vector<double>(state.target_position.size(), 0.0);
    }
    if (!velocity_feedforward_enabled()) {
      return std::vector<double>(state.target_position.size(), 0.0);
    }
    if (state.target_velocity.size() == state.target_position.size()) {
      return state.target_velocity;
    }
    return std::vector<double>(state.target_position.size(), 0.0);
  }

  static double min_duration_s()
  {
    return 1e-6;
  }

  static double default_interpolation_duration_s()
  {
    return 1.0 / 60.0;
  }

  bool velocity_feedforward_enabled() const
  {
    bool enabled = mit_velocity_feedforward_enabled_;
    if (has_parameter("mit_velocity_feedforward_enabled")) {
      get_parameter("mit_velocity_feedforward_enabled", enabled);
    }
    return enabled;
  }

  void apply_output_calibration(
    const SideState & state, std::vector<double> & positions)
  {
    const auto & side = state.side;
    const auto & names = state.names;
    const std::string prefix = side + "_";
    for (std::size_t i = 0; i < names.size() && i < positions.size(); ++i) {
      std::string suffix = names[i];
      if (suffix.rfind(prefix, 0) == 0) {
        suffix = suffix.substr(prefix.size());
      }
      const double scale = double_param("physical_" + side + "_" + suffix + "_scale", 1.0);
      const double offset = deg_to_rad(double_param("physical_" + side + "_" + suffix + "_offset_deg", 0.0));
      const double calibrated = positions[i] * scale + offset;
      if (!std::isfinite(calibrated)) { throw std::runtime_error("Non-finite output calibration: " + names[i]); }
      positions[i] = std::clamp(calibrated, state.lower[i], state.upper[i]);
    }
  }

  std::vector<double> mit_gains_for_joints(
    const std::vector<std::string> & names,
    const std::string & field,
    double default_value)
  {
    std::vector<double> values;
    values.reserve(names.size());
    for (const auto & name : names) {
      const double value = double_param("mit_" + name + "_" + field, -1.0);
      values.push_back(std::isfinite(value) && value >= 0.0 ? value : default_value);
    }
    return values;
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
  bool mit_velocity_feedforward_enabled_{true};
  double mit_default_kp_{0.4};
  double mit_default_kd_{0.05};
  std::shared_ptr<SideState> left_;
  std::shared_ptr<SideState> right_;
  rclcpp::Subscription<HandKinematics>::SharedPtr left_input_;
  rclcpp::Subscription<HandKinematics>::SharedPtr right_input_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace manus_revo3_retarget

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  options.enable_rosout(false);
  options.start_parameter_services(true);
  options.start_parameter_event_publisher(false);
  auto node = std::make_shared<manus_revo3_retarget::RetargetNodeCpp>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
