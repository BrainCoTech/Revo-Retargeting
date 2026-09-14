#pragma once

#include "manus_revo3_retarget/retarget_common.hpp"

#include <memory>
#include <optional>

#include <Eigen/Dense>

namespace manus_revo3_retarget
{

using HandLandmarks = std::unordered_map<std::string, Eigen::Vector3d>;

struct ThumbDiagnostics
{
  double tip_error_m{0.0};
  double posture_error_rad{0.0};
  double normalized_residual{0.0};
};

struct ThumbConfig
{
  double spread_sign{1.0};
  std::string cmr_joint_name{"thumb_mcp_spread"};
  double ik_position_scale{1.0};
  double position_sigma_m{0.01};
  double posture_sigma_rad{deg_to_rad(10.0)};
  double smooth_sigma_rad{deg_to_rad(10.0)};
  double tip_weight{0.0004};
  double pip_weight{0.000001};
  double dip_weight{0.000001};
  std::array<double, 5> posture_joint_weights{0.0, 0.0625, 0.81, 1.44, 1.0};
  double ik_posture_weight{0.00030461741978670857};
  double ik_smooth_weight{0.00030461741978670857};
  int ik_max_iterations{10};
  double ik_max_step_rad{deg_to_rad(3.0)};
  double ik_damping{0.02};
  double ik_step_size{0.30};
  double ik_tolerance{5e-4};
};

class ThumbRetarget
{
public:
  ~ThumbRetarget();
  bool initialize(const std::string & model_base, const std::string & side, std::string * error);
  void set_config(const ThumbConfig & config);
  void apply(const JointPositions & joints, const HandLandmarks & landmarks, JointArray & q);
  int last_iteration_count() const;
  ThumbDiagnostics diagnostics() const { return diagnostics_; }
  void joint_limits(const std::string & side, JointArray & lower, JointArray & upper) const;

private:
  struct Impl;

  void posture_target(const JointPositions & joints, Eigen::VectorXd & target, Eigen::VectorXd & weights) const;
  void solve_ik(
    const Eigen::Vector3d & tip_target,
    const std::optional<Eigen::Vector3d> & dip_target,
    const std::optional<Eigen::Vector3d> & pip_target,
    const JointPositions & joints);
  void copy_solution(JointArray & q) const;
  int joint_qpos_adr(const std::string & joint_name) const;
  int joint_dof_adr(const std::string & joint_name) const;
  double joint_low(int adr) const;
  double joint_high(int adr) const;

  ThumbConfig config_;
  std::unique_ptr<Impl> impl_;
  std::vector<int> thumb_qpos_adrs_;
  std::vector<int> thumb_dof_adrs_;
  std::vector<double> jlow_;
  std::vector<double> jhigh_;
  Eigen::VectorXd current_q_;
  int last_iteration_count_{0};
  ThumbDiagnostics diagnostics_;
};

extern "C" {
void * manus_revo3_thumb_create();
void manus_revo3_thumb_destroy(void * handle);
bool manus_revo3_thumb_initialize(void * handle, const char * model_base, const char * side, std::string * error);
void manus_revo3_thumb_set_config(void * handle, const ThumbConfig * config);
void manus_revo3_thumb_apply(
  void * handle,
  const JointPositions * joints,
  const HandLandmarks * landmarks,
  JointArray * q);
int manus_revo3_thumb_last_iteration_count(void * handle);
void manus_revo3_thumb_diagnostics(void * handle, ThumbDiagnostics * out);
void manus_revo3_thumb_joint_limits(void * handle, const char * side, JointArray * lower, JointArray * upper);
}

}  // namespace manus_revo3_retarget
