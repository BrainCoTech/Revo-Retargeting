#pragma once

#include "manus_revo3_retarget_cpp/retarget_common.hpp"

#include <memory>
#include <optional>

#include <Eigen/Dense>

namespace manus_revo3_retarget_cpp
{

constexpr std::size_t kManusKeypointCount = 25;
using ManusKeypoints = std::array<std::optional<Eigen::Vector3d>, kManusKeypointCount>;

struct ThumbConfig
{
  double joint_offset_deg{0.0};
  double cmp_offset_deg{0.0};
  double cmp_scale{1.0};
  double cmr_offset_deg{0.0};
  double mcp_offset_deg{0.0};
  double mcp_scale{1.0};
  double pip_scale{1.0};
  double dip_scale{1.0};
  double spread_sign{1.0};
  double manus_out_y_sign{-1.0};
  double manus_z_rotation_rad{M_PI / 2.0};
  double manus_scale_xz{1.0};
  double reach_scale{1.0};
  double ik_position_scale{1.0};
  double pip_ik_scale{1.0};
  double dip_ik_scale{1.0};
  double ema_prev{0.9};
  double ema_cur{0.1};
  double ik_posture_weight{0.1};
  double ik_smooth_weight{0.1};
  int ik_max_iterations{10};
  double ik_max_step_rad{deg_to_rad(3.0)};
  double ik_max_frame_delta_rad{deg_to_rad(6.0)};
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
  void apply(const Ergonomics & ergonomics, const ManusKeypoints & keypoints, JointArray & q);
  int last_iteration_count() const;

private:
  struct Impl;

  Eigen::Vector3d transform_manus_xyz(const Eigen::Vector3d & xyz) const;
  Eigen::Vector3d apply_reach_scale(const Eigen::Vector3d & thumb, const Eigen::Vector3d & center) const;
  void posture_target(const Ergonomics & ergonomics, Eigen::VectorXd & target, Eigen::VectorXd & weights) const;
  void solve_ik(
    const Eigen::Vector3d & tip_target,
    const std::optional<Eigen::Vector3d> & dip_target,
    const std::optional<Eigen::Vector3d> & pip_target,
    const Ergonomics & ergonomics);
  void apply_output_calibration(JointArray & q) const;
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
  int thumb_site_id_{-1};
  int thumb_dip_body_id_{-1};
  int thumb_pip_body_id_{-1};
  Eigen::VectorXd current_q_;
  std::optional<Eigen::Vector3d> filtered_thumb_target_;
  int last_iteration_count_{0};
};

extern "C" {
void * manus_revo3_thumb_create();
void manus_revo3_thumb_destroy(void * handle);
bool manus_revo3_thumb_initialize(void * handle, const char * model_base, const char * side, std::string * error);
void manus_revo3_thumb_set_config(void * handle, const ThumbConfig * config);
void manus_revo3_thumb_apply(
  void * handle,
  const Ergonomics * ergonomics,
  const ManusKeypoints * keypoints,
  JointArray * q);
int manus_revo3_thumb_last_iteration_count(void * handle);
}

}  // namespace manus_revo3_retarget_cpp
