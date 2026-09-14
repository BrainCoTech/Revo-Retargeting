#include "manus_revo3_retarget/thumb_retarget.hpp"
#include "manus_revo3_retarget/ik_residual.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <tuple>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace manus_revo3_retarget
{

namespace
{

std::optional<Eigen::Vector3d> landmark(const HandLandmarks & landmarks, const std::string & name)
{
  const auto it = landmarks.find(name);
  return it == landmarks.end() ? std::nullopt : std::optional<Eigen::Vector3d>(it->second);
}

double clamp(double value, double low, double high)
{
  return std::min(high, std::max(low, value));
}

std::string urdf_path_for(const std::string & description_base, const std::string & side)
{
  const auto file = side == "left" ? "revo3_left_may3.urdf" : "revo3_right_may3.urdf";
  return (std::filesystem::path(description_base) / "urdf" / file).string();
}
}  // namespace

struct ThumbRetarget::Impl
{
  pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> data;
  pinocchio::FrameIndex thumb_tip_frame{0};
  pinocchio::FrameIndex thumb_dip_frame{0};
  pinocchio::FrameIndex thumb_pip_frame{0};
};

ThumbRetarget::~ThumbRetarget() = default;

bool ThumbRetarget::initialize(const std::string & model_base, const std::string & side, std::string * error)
{
  impl_ = std::make_unique<Impl>();
  const std::string path = urdf_path_for(model_base, side);
  try {
    pinocchio::urdf::buildModel(path, impl_->model);
  } catch (const std::exception & exc) {
    if (error) {
      *error = std::string("Pinocchio buildModel failed for ") + path + ": " + exc.what();
    }
    impl_.reset();
    return false;
  }
  impl_->data = std::make_unique<pinocchio::Data>(impl_->model);

  const std::string prefix = side + "_";
  const std::array<std::string, 5> thumb_joints = {
    prefix + "thumb_CMP_joint",
    prefix + "thumb_CMR_joint",
    prefix + "thumb_MCP_joint",
    prefix + "thumb_PIP_joint",
    prefix + "thumb_DIP_joint",
  };

  thumb_qpos_adrs_.clear();
  thumb_dof_adrs_.clear();
  for (const auto & joint_name : thumb_joints) {
    const int qpos = joint_qpos_adr(joint_name);
    const int dof = joint_dof_adr(joint_name);
    if (qpos >= 0 && dof >= 0) {
      thumb_qpos_adrs_.push_back(qpos);
      thumb_dof_adrs_.push_back(dof);
    }
  }

  const auto tip_name = prefix + "thumb_tip_Link";
  const auto dip_name = prefix + "thumb_DIP_Link";
  const auto pip_name = prefix + "thumb_PIP_Link";
  if (!impl_->model.existFrame(tip_name) || !impl_->model.existFrame(dip_name) || !impl_->model.existFrame(pip_name)) {
    if (error) {
      *error = "thumb Pinocchio frames were not found in " + path;
    }
    impl_.reset();
    return false;
  }
  impl_->thumb_tip_frame = impl_->model.getFrameId(tip_name);
  impl_->thumb_dip_frame = impl_->model.getFrameId(dip_name);
  impl_->thumb_pip_frame = impl_->model.getFrameId(pip_name);

  if (thumb_qpos_adrs_.size() != 5 || thumb_qpos_adrs_.size() != thumb_dof_adrs_.size()) {
    if (error) {
      *error = "thumb Pinocchio joint names were not found in " + path;
    }
    impl_.reset();
    return false;
  }

  jlow_.assign(static_cast<std::size_t>(impl_->model.nq), -M_PI);
  jhigh_.assign(static_cast<std::size_t>(impl_->model.nq), M_PI);
  for (int i = 0; i < impl_->model.nq; ++i) {
    const double lo = impl_->model.lowerPositionLimit[i];
    const double hi = impl_->model.upperPositionLimit[i];
    if (std::isfinite(lo) && std::isfinite(hi) && lo < hi) {
      jlow_[static_cast<std::size_t>(i)] = lo;
      jhigh_[static_cast<std::size_t>(i)] = hi;
    }
  }

  current_q_ = Eigen::VectorXd::Zero(impl_->model.nq);
  for (int i = 0; i < impl_->model.nq; ++i) {
    current_q_[i] = (joint_low(i) <= 0.0 && 0.0 <= joint_high(i)) ? 0.0 : joint_low(i);
  }
  return true;
}

void ThumbRetarget::set_config(const ThumbConfig & config)
{
  residual_row_scale(config.tip_weight, config.position_sigma_m);
  residual_row_scale(config.pip_weight, config.position_sigma_m);
  residual_row_scale(config.dip_weight, config.position_sigma_m);
  residual_row_scale(config.ik_posture_weight, config.posture_sigma_rad);
  residual_row_scale(config.ik_smooth_weight, config.smooth_sigma_rad);
  for (double weight : config.posture_joint_weights) { residual_row_scale(weight, 1.0); }
  config_ = config;
}

int ThumbRetarget::last_iteration_count() const
{
  return last_iteration_count_;
}

void ThumbRetarget::apply(const JointPositions & joints, const HandLandmarks & landmarks, JointArray & q)
{
  if (!impl_ || !impl_->data) {
    return;
  }
  const auto thumb_raw = landmark(landmarks, "thumb_tip");
  if (!thumb_raw) {
    copy_solution(q);
    return;
  }

  const auto dip_target = landmark(landmarks, "thumb_dip");
  const auto pip_target = landmark(landmarks, "thumb_pip");
  solve_ik(
    *thumb_raw * config_.ik_position_scale,
    dip_target ? std::optional<Eigen::Vector3d>(*dip_target * config_.ik_position_scale) : std::nullopt,
    pip_target ? std::optional<Eigen::Vector3d>(*pip_target * config_.ik_position_scale) : std::nullopt,
    joints);
  copy_solution(q);
}

void ThumbRetarget::posture_target(
  const JointPositions & joints,
  Eigen::VectorXd & target,
  Eigen::VectorXd & weights) const
{
  const int n = static_cast<int>(thumb_qpos_adrs_.size());
  target = Eigen::VectorXd::Constant(n, std::numeric_limits<double>::quiet_NaN());
  weights = Eigen::VectorXd::Zero(n);
  const std::array<std::tuple<int, const char *, bool>, 4> sources = {
    std::make_tuple(1, config_.cmr_joint_name.c_str(), false),
    std::make_tuple(2, "thumb_mcp", true),
    std::make_tuple(3, "thumb_pip", true),
    std::make_tuple(4, "thumb_dip", true),
  };
  for (const auto & [index, name, clamp_positive] : sources) {
    if (index >= n) {
      continue;
    }
    double value_rad = joint_value(joints, name, std::numeric_limits<double>::quiet_NaN());
    if (!std::isfinite(value_rad)) {
      continue;
    }
    if (clamp_positive) {
      value_rad = std::max(0.0, value_rad);
    } else {
      value_rad *= config_.spread_sign;
    }
    const int adr = thumb_qpos_adrs_[static_cast<std::size_t>(index)];
    target[index] = clamp(value_rad, joint_low(adr), joint_high(adr));
    weights[index] = config_.posture_joint_weights[static_cast<std::size_t>(index)];
  }
}

void ThumbRetarget::solve_ik(
  const Eigen::Vector3d & tip_target,
  const std::optional<Eigen::Vector3d> & dip_target,
  const std::optional<Eigen::Vector3d> & pip_target,
  const JointPositions & joints)
{
  const int n = static_cast<int>(std::min(thumb_qpos_adrs_.size(), thumb_dof_adrs_.size()));
  if (!impl_ || !impl_->data || n <= 0) {
    return;
  }

  Eigen::VectorXd q_prev = current_q_;
  Eigen::VectorXd q = current_q_;
  Eigen::VectorXd posture;
  Eigen::VectorXd posture_weights;
  posture_target(joints, posture, posture_weights);
  last_iteration_count_ = 0;

  for (int iter = 0; iter < std::max(0, config_.ik_max_iterations); ++iter) {
    last_iteration_count_ = iter + 1;
    pinocchio::forwardKinematics(impl_->model, *impl_->data, q);
    pinocchio::computeJointJacobians(impl_->model, *impl_->data, q);
    pinocchio::updateFramePlacements(impl_->model, *impl_->data);

    std::vector<Eigen::VectorXd> residual_blocks;
    std::vector<Eigen::MatrixXd> jac_blocks;

    auto add_position_task = [&](pinocchio::FrameIndex frame, const Eigen::Vector3d & target, double weight) {
      weight = residual_row_scale(weight, config_.position_sigma_m);
      const Eigen::Vector3d current = impl_->data->oMf[frame].translation();
      residual_blocks.push_back(weight * (target - current));
      Eigen::Matrix<double, 6, Eigen::Dynamic> full_jac(6, impl_->model.nv);
      full_jac.setZero();
      pinocchio::getFrameJacobian(
        impl_->model, *impl_->data, frame, pinocchio::LOCAL_WORLD_ALIGNED, full_jac);
      Eigen::MatrixXd j(3, n);
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < n; ++c) {
          j(r, c) = weight * full_jac(r, thumb_dof_adrs_[static_cast<std::size_t>(c)]);
        }
      }
      jac_blocks.push_back(j);
    };

    add_position_task(impl_->thumb_tip_frame, tip_target, config_.tip_weight);
    if (dip_target) {
      add_position_task(impl_->thumb_dip_frame, *dip_target, config_.dip_weight);
    }
    if (pip_target) {
      add_position_task(impl_->thumb_pip_frame, *pip_target, config_.pip_weight);
    }

    if (config_.ik_posture_weight > 0.0) {
      for (int i = 0; i < n; ++i) {
        if (!std::isfinite(posture[i]) || posture_weights[i] <= 0.0) {
          continue;
        }
        const double row_weight = residual_row_scale(config_.ik_posture_weight * posture_weights[i], config_.posture_sigma_rad);
        Eigen::VectorXd residual(1);
        residual[0] = row_weight * (posture[i] - q[thumb_qpos_adrs_[static_cast<std::size_t>(i)]]);
        Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(1, n);
        jac(0, i) = row_weight;
        residual_blocks.push_back(residual);
        jac_blocks.push_back(jac);
      }
    }

    if (config_.ik_smooth_weight > 0.0) {
      const double smooth_scale = residual_row_scale(config_.ik_smooth_weight, config_.smooth_sigma_rad);
      Eigen::VectorXd residual(n);
      Eigen::MatrixXd jac = Eigen::MatrixXd::Identity(n, n) * smooth_scale;
      for (int i = 0; i < n; ++i) {
        const int adr = thumb_qpos_adrs_[static_cast<std::size_t>(i)];
        residual[i] = smooth_scale * (q_prev[adr] - q[adr]);
      }
      residual_blocks.push_back(residual);
      jac_blocks.push_back(jac);
    }

    int rows = 0;
    for (const auto & block : residual_blocks) {
      rows += static_cast<int>(block.rows());
    }
    Eigen::VectorXd residual(rows);
    Eigen::MatrixXd jacobian(rows, n);
    int cursor = 0;
    for (std::size_t i = 0; i < residual_blocks.size(); ++i) {
      const int block_rows = static_cast<int>(residual_blocks[i].rows());
      residual.segment(cursor, block_rows) = residual_blocks[i];
      jacobian.block(cursor, 0, block_rows, n) = jac_blocks[i];
      cursor += block_rows;
    }

    if (residual.norm() < config_.ik_tolerance || !residual.allFinite() || !jacobian.allFinite()) {
      break;
    }

    const Eigen::MatrixXd lhs = jacobian * jacobian.transpose() +
      (config_.ik_damping * config_.ik_damping) * Eigen::MatrixXd::Identity(rows, rows);
    Eigen::VectorXd dq = jacobian.transpose() * lhs.ldlt().solve(residual);
    if (!dq.allFinite()) {
      break;
    }
    dq *= config_.ik_step_size;
    if (config_.ik_max_step_rad > 0.0) {
      for (int i = 0; i < dq.rows(); ++i) {
        dq[i] = clamp(dq[i], -config_.ik_max_step_rad, config_.ik_max_step_rad);
      }
    }
    if (dq.norm() < 1e-5) {
      break;
    }
    for (int i = 0; i < n; ++i) {
      const int adr = thumb_qpos_adrs_[static_cast<std::size_t>(i)];
      q[adr] = clamp(q[adr] + dq[i], joint_low(adr), joint_high(adr));
    }
  }

  current_q_ = q;
  pinocchio::forwardKinematics(impl_->model, *impl_->data, q);
  pinocchio::updateFramePlacements(impl_->model, *impl_->data);
  diagnostics_.tip_error_m = (impl_->data->oMf[impl_->thumb_tip_frame].translation() - tip_target).norm();
  double energy = config_.tip_weight * std::pow(diagnostics_.tip_error_m / config_.position_sigma_m, 2);
  if (dip_target) { energy += config_.dip_weight * (impl_->data->oMf[impl_->thumb_dip_frame].translation() - *dip_target).squaredNorm() / std::pow(config_.position_sigma_m, 2); }
  if (pip_target) { energy += config_.pip_weight * (impl_->data->oMf[impl_->thumb_pip_frame].translation() - *pip_target).squaredNorm() / std::pow(config_.position_sigma_m, 2); }
  double posture_squared = 0.0;
  int posture_count = 0;
  for (int i = 0; i < n; ++i) {
    const int adr = thumb_qpos_adrs_[static_cast<std::size_t>(i)];
    if (std::isfinite(posture[i]) && posture_weights[i] > 0.0) {
      const double delta = posture[i] - q[adr];
      posture_squared += delta * delta;
      ++posture_count;
      energy += config_.ik_posture_weight * posture_weights[i] * std::pow(delta / config_.posture_sigma_rad, 2);
    }
    energy += config_.ik_smooth_weight * std::pow((q_prev[adr] - q[adr]) / config_.smooth_sigma_rad, 2);
  }
  diagnostics_.posture_error_rad = posture_count ? std::sqrt(posture_squared / posture_count) : 0.0;
  diagnostics_.normalized_residual = std::sqrt(energy);
}

void ThumbRetarget::copy_solution(JointArray & q) const
{
  if (thumb_qpos_adrs_.size() < 5 || current_q_.size() == 0) {
    return;
  }
  const std::array<JointIndex, 5> indices = {ThumbCMP, ThumbCMR, ThumbMCP, ThumbPIP, ThumbDIP};
  for (std::size_t i = 0; i < indices.size(); ++i) {
    q[indices[i]] = current_q_[thumb_qpos_adrs_[i]];
  }
}

void ThumbRetarget::joint_limits(const std::string & side, JointArray & lower, JointArray & upper) const
{
  const auto names = joint_names(side);
  for (std::size_t i = 0; i < names.size(); ++i) {
    const int adr = joint_qpos_adr(names[i]);
    if (adr < 0) { throw std::runtime_error("Missing robot joint limit: " + names[i]); }
    lower[i] = joint_low(adr);
    upper[i] = joint_high(adr);
  }
}

int ThumbRetarget::joint_qpos_adr(const std::string & joint_name) const
{
  if (!impl_ || !impl_->model.existJointName(joint_name)) {
    return -1;
  }
  const pinocchio::JointIndex jid = impl_->model.getJointId(joint_name);
  return impl_->model.idx_qs[jid];
}

int ThumbRetarget::joint_dof_adr(const std::string & joint_name) const
{
  if (!impl_ || !impl_->model.existJointName(joint_name)) {
    return -1;
  }
  const pinocchio::JointIndex jid = impl_->model.getJointId(joint_name);
  return impl_->model.idx_vs[jid];
}

double ThumbRetarget::joint_low(int adr) const
{
  return (adr >= 0 && static_cast<std::size_t>(adr) < jlow_.size()) ? jlow_[static_cast<std::size_t>(adr)] : -M_PI;
}

double ThumbRetarget::joint_high(int adr) const
{
  return (adr >= 0 && static_cast<std::size_t>(adr) < jhigh_.size()) ? jhigh_[static_cast<std::size_t>(adr)] : M_PI;
}

extern "C" void * manus_revo3_thumb_create()
{
  return new ThumbRetarget();
}

extern "C" void manus_revo3_thumb_destroy(void * handle)
{
  delete static_cast<ThumbRetarget *>(handle);
}

extern "C" bool manus_revo3_thumb_initialize(
  void * handle,
  const char * model_base,
  const char * side,
  std::string * error)
{
  if (handle == nullptr || model_base == nullptr || side == nullptr) {
    if (error != nullptr) {
      *error = "invalid thumb plugin handle or arguments";
    }
    return false;
  }
  return static_cast<ThumbRetarget *>(handle)->initialize(model_base, side, error);
}

extern "C" void manus_revo3_thumb_set_config(void * handle, const ThumbConfig * config)
{
  if (handle == nullptr || config == nullptr) {
    return;
  }
  static_cast<ThumbRetarget *>(handle)->set_config(*config);
}

extern "C" void manus_revo3_thumb_apply(
  void * handle,
  const JointPositions * joints,
  const HandLandmarks * landmarks,
  JointArray * q)
{
  if (handle == nullptr || joints == nullptr || landmarks == nullptr || q == nullptr) {
    return;
  }
  static_cast<ThumbRetarget *>(handle)->apply(*joints, *landmarks, *q);
}

extern "C" void manus_revo3_thumb_diagnostics(void * handle, ThumbDiagnostics * out)
{
  *out = static_cast<ThumbRetarget *>(handle)->diagnostics();
}

extern "C" void manus_revo3_thumb_joint_limits(void * handle, const char * side, JointArray * lower, JointArray * upper)
{
  static_cast<ThumbRetarget *>(handle)->joint_limits(side, *lower, *upper);
}

extern "C" int manus_revo3_thumb_last_iteration_count(void * handle)
{
  if (handle == nullptr) {
    return 0;
  }
  return static_cast<ThumbRetarget *>(handle)->last_iteration_count();
}

}  // namespace manus_revo3_retarget
