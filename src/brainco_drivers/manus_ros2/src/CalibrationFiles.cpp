#include "CalibrationFiles.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>

namespace manus_ros2
{
namespace
{

std::string NormalizeCompact(const std::string& value)
{
    std::string out;
    out.reserve(value.size());
    for (unsigned char ch : value)
    {
        if (std::isalnum(ch))
        {
            out.push_back(static_cast<char>(std::tolower(ch)));
        }
    }
    return out;
}

}  // namespace

std::string SideToSlug(Side side)
{
    switch (side)
    {
    case Side_Left:
        return "left";
    case Side_Right:
        return "right";
    default:
        return "invalid";
    }
}

std::string SideToDisplayName(Side side)
{
    switch (side)
    {
    case Side_Left:
        return "Left";
    case Side_Right:
        return "Right";
    default:
        return "Invalid";
    }
}

std::optional<Side> ParseSideSlug(const std::string& value)
{
    const std::string normalized = NormalizeCompact(value);
    if (normalized == "left" || normalized == "l")
    {
        return Side_Left;
    }
    if (normalized == "right" || normalized == "r")
    {
        return Side_Right;
    }
    return std::nullopt;
}

std::string DeviceFamilyToSlug(DeviceFamilyType family)
{
    switch (family)
    {
    case DeviceFamilyType_Prime1:
        return "prime1";
    case DeviceFamilyType_Prime2:
        return "prime2";
    case DeviceFamilyType_PrimeX:
        return "primex";
    case DeviceFamilyType_Metaglove:
        return "metaglove";
    case DeviceFamilyType_Prime3:
        return "prime3";
    case DeviceFamilyType_Virtual:
        return "virtual";
    case DeviceFamilyType_MetaglovePro:
        return "metaglovepro";
    case DeviceFamilyType_MetagloveProPrecision:
        return "metagloveproprecision";
    case DeviceFamilyType_MetagloveProHaptics:
        return "metagloveprohaptics";
    case DeviceFamilyType_MetagloveProPrecisionHaptics:
        return "metagloveproprecisionhaptics";
    default:
        return "unknown";
    }
}

std::string DeviceFamilyToDisplayName(DeviceFamilyType family)
{
    switch (family)
    {
    case DeviceFamilyType_Prime1:
        return "Prime 1";
    case DeviceFamilyType_Prime2:
        return "Prime 2";
    case DeviceFamilyType_PrimeX:
        return "Prime X";
    case DeviceFamilyType_Metaglove:
        return "Metaglove";
    case DeviceFamilyType_Prime3:
        return "Prime 3";
    case DeviceFamilyType_Virtual:
        return "Virtual";
    case DeviceFamilyType_MetaglovePro:
        return "Metaglove Pro";
    case DeviceFamilyType_MetagloveProPrecision:
        return "Metaglove Pro Precision";
    case DeviceFamilyType_MetagloveProHaptics:
        return "Metaglove Pro Haptics";
    case DeviceFamilyType_MetagloveProPrecisionHaptics:
        return "Metaglove Pro Precision Haptics";
    default:
        return "Unknown";
    }
}

std::optional<DeviceFamilyType> ParseDeviceFamilySlug(const std::string& value)
{
    const std::string normalized = NormalizeCompact(value);
    const DeviceFamilyType families[] = {
        DeviceFamilyType_Unknown,
        DeviceFamilyType_Prime1,
        DeviceFamilyType_Prime2,
        DeviceFamilyType_PrimeX,
        DeviceFamilyType_Metaglove,
        DeviceFamilyType_Prime3,
        DeviceFamilyType_Virtual,
        DeviceFamilyType_MetaglovePro,
        DeviceFamilyType_MetagloveProPrecision,
        DeviceFamilyType_MetagloveProHaptics,
        DeviceFamilyType_MetagloveProPrecisionHaptics,
    };

    for (DeviceFamilyType family : families)
    {
        if (normalized == DeviceFamilyToSlug(family))
        {
            return family;
        }
    }
    return std::nullopt;
}

std::filesystem::path ExpandUserPath(const std::string& path)
{
    if (!path.empty() && path[0] == '~')
    {
        const char* home = std::getenv("HOME");
        if (home != nullptr)
        {
            if (path.size() == 1)
            {
                return std::filesystem::path(home);
            }
            if (path[1] == '/')
            {
                return std::filesystem::path(home) / path.substr(2);
            }
        }
    }
    return std::filesystem::path(path);
}

std::optional<std::filesystem::path> SourceCalibrationDirectoryFromPackageShare(
    const std::filesystem::path& package_share_directory,
    const std::string& package_name)
{
    std::filesystem::path cursor = package_share_directory;
    std::error_code ec;
    cursor = std::filesystem::weakly_canonical(cursor, ec);
    if (ec)
    {
        cursor = package_share_directory;
    }

    while (!cursor.empty())
    {
        if (cursor.filename() == "install")
        {
            const std::filesystem::path src_root = cursor.parent_path() / "src";
            const std::filesystem::path candidate =
                src_root / package_name / "calibrations";
            if (std::filesystem::exists(candidate, ec) &&
                std::filesystem::is_directory(candidate, ec))
            {
                return candidate;
            }

            if (std::filesystem::exists(src_root, ec) &&
                std::filesystem::is_directory(src_root, ec))
            {
                for (const auto& entry : std::filesystem::recursive_directory_iterator(
                         src_root, std::filesystem::directory_options::skip_permission_denied, ec))
                {
                    if (ec)
                    {
                        break;
                    }
                    if (!entry.is_directory(ec) || entry.path().filename() != package_name)
                    {
                        continue;
                    }
                    const std::filesystem::path nested_candidate =
                        entry.path() / "calibrations";
                    if (std::filesystem::exists(nested_candidate, ec) &&
                        std::filesystem::is_directory(nested_candidate, ec))
                    {
                        return nested_candidate;
                    }
                }
            }
        }

        const std::filesystem::path parent = cursor.parent_path();
        if (parent == cursor)
        {
            break;
        }
        cursor = parent;
    }

    return std::nullopt;
}

std::filesystem::path DefaultCalibrationPath(
    const std::filesystem::path& calibration_directory,
    DeviceFamilyType family,
    Side side)
{
    return calibration_directory / DeviceFamilyToSlug(family) /
        ("Calibration_" + SideToSlug(side) + ".mcal");
}

std::vector<std::filesystem::path> CollectCalibrationFiles(
    const std::filesystem::path& calibration_directory)
{
    std::vector<std::filesystem::path> files;
    std::error_code ec;
    if (!std::filesystem::exists(calibration_directory, ec) ||
        !std::filesystem::is_directory(calibration_directory, ec))
    {
        return files;
    }

    for (const auto& entry : std::filesystem::recursive_directory_iterator(
             calibration_directory, std::filesystem::directory_options::skip_permission_denied, ec))
    {
        if (ec)
        {
            break;
        }
        if (entry.is_regular_file(ec) && entry.path().extension() == ".mcal")
        {
            files.push_back(entry.path());
        }
    }

    std::sort(files.begin(), files.end());
    return files;
}

std::optional<std::filesystem::path> FindCalibrationFile(
    const std::filesystem::path& calibration_directory,
    DeviceFamilyType family,
    Side side)
{
    const std::filesystem::path expected_file =
        DefaultCalibrationPath(calibration_directory, family, side);
    std::error_code ec;
    if (std::filesystem::exists(expected_file, ec) &&
        std::filesystem::is_regular_file(expected_file, ec))
    {
        return expected_file;
    }

    return std::nullopt;
}

}  // namespace manus_ros2
