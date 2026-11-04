#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>
#include <revo3_mit_controller_msgs/msg/revo3_mit_command.hpp>

#include <algorithm>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace revo3_mit_controller
{

class Revo3MITController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    try
    {
      auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
      auto_declare<double>("default_kp", 5.0);
      auto_declare<double>("default_kd", 0.5);
      auto_declare<double>("default_effort", 0.0);
      auto_declare<double>("command_timeout_sec", 0.25);
      auto_declare<bool>("hold_position_on_timeout", true);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint_name : joint_names_)
    {
      config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
      config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
      config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
      config.names.push_back(joint_name + "/kp");
      config.names.push_back(joint_name + "/kd");
    }
    return config;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint_name : joint_names_)
    {
      config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
  }

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    default_kp_ = get_node()->get_parameter("default_kp").as_double();
    default_kd_ = get_node()->get_parameter("default_kd").as_double();
    default_effort_ = get_node()->get_parameter("default_effort").as_double();
    command_timeout_sec_ = get_node()->get_parameter("command_timeout_sec").as_double();
    hold_position_on_timeout_ = get_node()->get_parameter("hold_position_on_timeout").as_bool();

    if (joint_names_.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' must not be empty");
      return CallbackReturn::ERROR;
    }

    joint_index_.clear();
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      joint_index_[joint_names_[i]] = i;
    }

    command_position_.assign(joint_names_.size(), 0.0);
    command_velocity_.assign(joint_names_.size(), 0.0);
    command_effort_.assign(joint_names_.size(), default_effort_);
    command_kp_.assign(joint_names_.size(), default_kp_);
    command_kd_.assign(joint_names_.size(), default_kd_);
    has_command_ = false;

    RCLCPP_INFO(
      get_node()->get_logger(), "Revo3MITController configured with %zu joints", joint_names_.size());
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (command_interfaces_.size() != joint_names_.size() * 5 ||
      state_interfaces_.size() != joint_names_.size())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Interface count mismatch: cmd=%zu state=%zu expected cmd=%zu state=%zu",
        command_interfaces_.size(), state_interfaces_.size(), joint_names_.size() * 5,
        joint_names_.size());
      return CallbackReturn::ERROR;
    }

    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      for (std::size_t i = 0; i < joint_names_.size(); ++i)
      {
        command_position_[i] = get_state_value(i);
        command_velocity_[i] = 0.0;
        command_effort_[i] = default_effort_;
        command_kp_[i] = default_kp_;
        command_kd_[i] = default_kd_;
      }
      last_command_time_ = get_node()->now();
      has_command_ = false;
    }

    write_current_command();

    const std::string command_topic = std::string(get_node()->get_name()) + "/commands";
    command_sub_ = get_node()->create_subscription<revo3_mit_controller_msgs::msg::Revo3MITCommand>(
      command_topic, rclcpp::SystemDefaultsQoS(),
      [this](const revo3_mit_controller_msgs::msg::Revo3MITCommand::SharedPtr msg)
      {
        handle_command(msg);
      });

    active_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Revo3MITController active on '%s'", command_topic.c_str());
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    command_sub_.reset();
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      std::fill(command_velocity_.begin(), command_velocity_.end(), 0.0);
      std::fill(command_effort_.begin(), command_effort_.end(), default_effort_);
      // Keep default stiffness so position-only controllers can still drive MIT hardware.
      std::fill(command_kp_.begin(), command_kp_.end(), default_kp_);
      std::fill(command_kd_.begin(), command_kd_.end(), default_kd_);
    }
    write_current_command();
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration &) override
  {
    if (!active_)
    {
      return controller_interface::return_type::OK;
    }

    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      const bool timed_out = has_command_ && command_timeout_sec_ > 0.0 &&
        (time - last_command_time_).seconds() > command_timeout_sec_;
      if (timed_out)
      {
        std::fill(command_velocity_.begin(), command_velocity_.end(), 0.0);
        std::fill(command_effort_.begin(), command_effort_.end(), 0.0);
        if (hold_position_on_timeout_)
        {
          for (std::size_t i = 0; i < joint_names_.size(); ++i)
          {
            command_position_[i] = get_state_value(i);
          }
        }
        has_command_ = false;
      }
    }

    write_current_command();
    return controller_interface::return_type::OK;
  }

private:
  static bool valid_array_size(
    const std::vector<double> & values, const std::size_t expected_size, const char * field_name,
    const rclcpp::Logger & logger)
  {
    if (values.empty() || values.size() == expected_size)
    {
      return true;
    }
    RCLCPP_WARN(logger, "Ignoring command: field '%s' has size %zu, expected 0 or %zu",
      field_name, values.size(), expected_size);
    return false;
  }

  void handle_command(const revo3_mit_controller_msgs::msg::Revo3MITCommand::SharedPtr msg)
  {
    const bool named_command = !msg->joint_names.empty();
    const std::size_t command_size = named_command ? msg->joint_names.size() : joint_names_.size();
    const auto logger = get_node()->get_logger();

    if (!valid_array_size(msg->position, command_size, "position", logger) ||
      !valid_array_size(msg->velocity, command_size, "velocity", logger) ||
      !valid_array_size(msg->effort, command_size, "effort", logger) ||
      !valid_array_size(msg->kp, command_size, "kp", logger) ||
      !valid_array_size(msg->kd, command_size, "kd", logger))
    {
      return;
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    for (std::size_t source_index = 0; source_index < command_size; ++source_index)
    {
      std::size_t target_index = source_index;
      if (named_command)
      {
        const auto it = joint_index_.find(msg->joint_names[source_index]);
        if (it == joint_index_.end())
        {
          RCLCPP_WARN(logger, "Ignoring unknown joint '%s'", msg->joint_names[source_index].c_str());
          continue;
        }
        target_index = it->second;
      }

      if (!msg->position.empty()) command_position_[target_index] = msg->position[source_index];
      if (!msg->velocity.empty()) command_velocity_[target_index] = msg->velocity[source_index];
      if (!msg->effort.empty()) command_effort_[target_index] = msg->effort[source_index];
      if (!msg->kp.empty()) command_kp_[target_index] = msg->kp[source_index];
      if (!msg->kd.empty()) command_kd_[target_index] = msg->kd[source_index];
    }

    last_command_time_ = get_node()->now();
    has_command_ = true;
  }

  double get_state_value(const std::size_t index) const
  {
#if RCLCPP_VERSION_MAJOR >= 17
    const auto value = state_interfaces_[index].get_optional();
    return value ? *value : 0.0;
#else
    return state_interfaces_[index].get_value();
#endif
  }

  void set_command_value(const std::size_t index, const double value)
  {
#if RCLCPP_VERSION_MAJOR >= 17
    (void)command_interfaces_[index].set_value(value);
#else
    command_interfaces_[index].set_value(value);
#endif
  }

  void write_current_command()
  {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> effort;
    std::vector<double> kp;
    std::vector<double> kd;

    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      position = command_position_;
      velocity = command_velocity_;
      effort = command_effort_;
      kp = command_kp_;
      kd = command_kd_;
    }

    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      const std::size_t base = i * 5;
      set_command_value(base + 0, position[i]);
      set_command_value(base + 1, velocity[i]);
      set_command_value(base + 2, effort[i]);
      set_command_value(base + 3, kp[i]);
      set_command_value(base + 4, kd[i]);
    }
  }

  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, std::size_t> joint_index_;
  std::vector<double> command_position_;
  std::vector<double> command_velocity_;
  std::vector<double> command_effort_;
  std::vector<double> command_kp_;
  std::vector<double> command_kd_;
  double default_kp_{5.0};
  double default_kd_{0.5};
  double default_effort_{0.0};
  double command_timeout_sec_{0.25};
  bool hold_position_on_timeout_{true};
  bool active_{false};
  bool has_command_{false};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  std::mutex command_mutex_;
  rclcpp::Subscription<revo3_mit_controller_msgs::msg::Revo3MITCommand>::SharedPtr command_sub_;
};

}  // namespace revo3_mit_controller

PLUGINLIB_EXPORT_CLASS(
  revo3_mit_controller::Revo3MITController, controller_interface::ControllerInterface)
