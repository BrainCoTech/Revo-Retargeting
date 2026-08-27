#ifndef MANUS_ROS2_CALIBRATION_FILES_HPP_
#define MANUS_ROS2_CALIBRATION_FILES_HPP_

#include "ManusSDKTypes.h"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace manus_ros2
{

std::string SideToSlug(Side side);
std::string SideToDisplayName(Side side);
std::optional<Side> ParseSideSlug(const std::string& value);

std::string DeviceFamilyToSlug(DeviceFamilyType family);
std::string DeviceFamilyToDisplayName(DeviceFamilyType family);
std::optional<DeviceFamilyType> ParseDeviceFamilySlug(const std::string& value);

std::filesystem::path ExpandUserPath(const std::string& path);

std::optional<std::filesystem::path> SourceCalibrationDirectoryFromPackageShare(
    const std::filesystem::path& package_share_directory,
    const std::string& package_name);

std::filesystem::path DefaultCalibrationPath(
    const std::filesystem::path& calibration_directory,
    DeviceFamilyType family,
    Side side);

std::vector<std::filesystem::path> CollectCalibrationFiles(
    const std::filesystem::path& calibration_directory);

std::optional<std::filesystem::path> FindCalibrationFile(
    const std::filesystem::path& calibration_directory,
    DeviceFamilyType family,
    Side side);

}  // namespace manus_ros2

#endif  // MANUS_ROS2_CALIBRATION_FILES_HPP_
