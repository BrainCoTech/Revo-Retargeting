#pragma once

#include <array>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

namespace manus_revo3_retarget
{

constexpr std::size_t kJointCount = 21;

inline double deg_to_rad(double value)
{
  return value * M_PI / 180.0;
}

inline double finite_or(double value, double fallback)
{
  return std::isfinite(value) ? value : fallback;
}

inline std::vector<std::string> joint_names(const std::string & side)
{
  const std::string p = side + "_";
  return {
    p + "little_MPR_joint", p + "little_MCP_joint", p + "little_PIP_joint", p + "little_DIP_joint",
    p + "ring_MPR_joint", p + "ring_MCP_joint", p + "ring_PIP_joint", p + "ring_DIP_joint",
    p + "middle_MPR_joint", p + "middle_MCP_joint", p + "middle_PIP_joint", p + "middle_DIP_joint",
    p + "index_MPR_joint", p + "index_MCP_joint", p + "index_PIP_joint", p + "index_DIP_joint",
    p + "thumb_MCP_joint", p + "thumb_PIP_joint", p + "thumb_DIP_joint", p + "thumb_CMP_joint",
    p + "thumb_CMR_joint",
  };
}

enum JointIndex : std::size_t
{
  LittleMPR = 0,
  LittleMCP = 1,
  LittlePIP = 2,
  LittleDIP = 3,
  RingMPR = 4,
  RingMCP = 5,
  RingPIP = 6,
  RingDIP = 7,
  MiddleMPR = 8,
  MiddleMCP = 9,
  MiddlePIP = 10,
  MiddleDIP = 11,
  IndexMPR = 12,
  IndexMCP = 13,
  IndexPIP = 14,
  IndexDIP = 15,
  ThumbMCP = 16,
  ThumbPIP = 17,
  ThumbDIP = 18,
  ThumbCMP = 19,
  ThumbCMR = 20,
};

using Ergonomics = std::unordered_map<std::string, double>;
using JointArray = std::array<double, kJointCount>;

inline double ergonomic_value(const Ergonomics & ergonomics, const std::string & name, double fallback = 0.0)
{
  const auto it = ergonomics.find(name);
  if (it == ergonomics.end()) {
    return fallback;
  }
  return finite_or(it->second, fallback);
}

}  // namespace manus_revo3_retarget
