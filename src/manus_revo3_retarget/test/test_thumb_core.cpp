#include <cmath>
#include <iostream>
#include <stdexcept>
#include "manus_revo3_retarget/thumb_retarget.hpp"
#include "manus_revo3_retarget/hand_kinematics_input.hpp"
#include "manus_revo3_retarget/ik_residual.hpp"
using namespace manus_revo3_retarget;
void check(bool valid, const char * text) { if (!valid) { throw std::runtime_error(text); } }
int main(int argc, char ** argv) {
  check(argc == 2, "description path required");
  check(std::abs(residual_row_scale(.0004,.01)-2.) < 1e-12, "legacy tip row equivalence");
  check(std::abs(residual_row_scale(.0004,10.)*10.-residual_row_scale(.0004,.01)*.01) < 1e-12, "position unit invariance");
  bool rejected=false;
  try { residual_row_scale(1.,0.); } catch(const std::invalid_argument &) { rejected=true; }
  check(rejected,"invalid sigma rejected");
  hand_teleop_msgs::msg::HandKinematics msg;
  msg.side=msg.RIGHT; msg.header.stamp.sec=1;msg.header.frame_id="hand_retarget_right";
  for (const auto * f : {"index","middle","ring","little"}) {
    for (const auto * j : {"mcp","pip","dip","spread"}) {msg.joint_names.push_back(std::string(f)+"_"+j);msg.joint_positions_rad.push_back(0.);}
  }
  msg.landmark_names={"thumb_tip"};msg.landmarks_m.resize(1);std::string error;
  check(parse_hand_kinematics(msg,"right","hand_retarget_right","spread",error).has_value(),"four tips not required");
  msg.landmark_names.clear();msg.landmarks_m.clear();
  check(!parse_hand_kinematics(msg,"right","hand_retarget_right","spread",error),"thumb tip required");
  for (const auto * side : {"left","right"}) {
    void * h=manus_revo3_thumb_create();
    check(manus_revo3_thumb_initialize(h,argv[1],side,&error), error.c_str());
    ThumbConfig cfg;manus_revo3_thumb_set_config(h,&cfg);
    JointArray low,high,q{};manus_revo3_thumb_joint_limits(h,side,&low,&high);
    JointPositions joints{{"thumb_mcp",.2},{"thumb_pip",.3},{"thumb_dip",.2}};
    HandLandmarks points{{"thumb_tip",Eigen::Vector3d(.02,.02,.08)}};
    for(int n=0;n<10;++n) {manus_revo3_thumb_apply(h,&joints,&points,&q);}
    for(std::size_t i=0;i<q.size();++i) {check(std::isfinite(q[i]),"finite solution"); check(low[i]<high[i],"valid joint limits");}
    ThumbDiagnostics d;manus_revo3_thumb_diagnostics(h,&d);
    check(std::isfinite(d.tip_error_m) && std::isfinite(d.normalized_residual),"finite diagnostics");
    manus_revo3_thumb_destroy(h);
  }
  std::cout << "thumb core checks passed\n";
}
