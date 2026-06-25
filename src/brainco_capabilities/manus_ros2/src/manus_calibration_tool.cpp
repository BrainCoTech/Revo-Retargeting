#include "CalibrationFiles.hpp"
#include "ManusSDK.h"
#include "ManusSDKTypeInitializers.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

struct Options
{
    std::filesystem::path calibration_directory;
    std::optional<uint32_t> glove_id;
    std::optional<Side> side;
    std::optional<DeviceFamilyType> family;
    std::string ip;
    bool use_core_connection = false;
    bool list_only = false;
    bool export_current = false;
    bool overwrite = false;
    bool verbose_sdk_logs = false;
    int wait_seconds = 20;
};

std::mutex g_landscape_mutex;
std::condition_variable g_landscape_cv;
std::optional<Landscape> g_landscape;
bool g_verbose_sdk_logs = false;

void OnSdkLog(LogSeverity severity, const char* const log, uint32_t length)
{
    if (log == nullptr)
    {
        return;
    }

    if (!g_verbose_sdk_logs && severity != LogSeverity_Error)
    {
        return;
    }

    std::cerr << "[MANUS SDK] "
              << std::string(log, log + length);
    if (length == 0 || log[length - 1] != '\n')
    {
        std::cerr << "\n";
    }
}

void OnLandscapeCallback(const Landscape* const landscape)
{
    if (landscape == nullptr)
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(g_landscape_mutex);
        g_landscape = *landscape;
    }
    g_landscape_cv.notify_all();
}

std::filesystem::path DefaultCalibrationDirectory()
{
    try
    {
        const std::filesystem::path package_share =
            ament_index_cpp::get_package_share_directory("manus_ros2");
        const auto source_directory =
            manus_ros2::SourceCalibrationDirectoryFromPackageShare(package_share, "manus_ros2");
        if (source_directory)
        {
            return *source_directory;
        }
        return package_share / "calibrations";
    }
    catch (const std::exception&)
    {
        return manus_ros2::ExpandUserPath("~/Documents/manus-calibrations");
    }
}

void PrintUsage()
{
    std::cout
        << "Usage: ros2 run manus_ros2 manus_calibration_tool [options]\n"
        << "\n"
        << "Options:\n"
        << "  --list                         List connected gloves and exit\n"
        << "  --side left|right              Select glove side\n"
        << "  --family FAMILY                Select glove family slug, e.g. metagloveproprecision\n"
        << "  --glove-id ID                  Select exact MANUS glove id\n"
        << "  --calibration-directory PATH   Directory for .mcal files\n"
        << "  --export-current               Save current calibration without running a new calibration\n"
        << "  --overwrite                    Replace an existing .mcal without asking\n"
        << "  --core                         Use Core connection initialization instead of integrated\n"
        << "  --ip ADDRESS                   Connect to a specific MANUS Core host IP\n"
        << "  --verbose-sdk-logs             Show all MANUS SDK logs\n"
        << "  --wait-seconds N               Seconds to wait for glove landscape, default 20\n"
        << "  --help                         Show this help\n";
}

uint32_t ParseUint32(const std::string& value, const std::string& option_name)
{
    size_t consumed = 0;
    const unsigned long parsed = std::stoul(value, &consumed, 10);
    if (consumed != value.size() || parsed > UINT32_MAX)
    {
        throw std::runtime_error("Invalid value for " + option_name + ": " + value);
    }
    return static_cast<uint32_t>(parsed);
}

Options ParseArgs(int argc, char* argv[])
{
    Options options;
    options.calibration_directory = DefaultCalibrationDirectory();

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        auto require_value = [&](const std::string& option) -> std::string {
            if (i + 1 >= argc)
            {
                throw std::runtime_error("Missing value for " + option);
            }
            return argv[++i];
        };

        if (arg == "--help" || arg == "-h")
        {
            PrintUsage();
            std::exit(0);
        }
        else if (arg == "--list")
        {
            options.list_only = true;
        }
        else if (arg == "--side")
        {
            const std::string value = require_value(arg);
            auto side = manus_ros2::ParseSideSlug(value);
            if (!side)
            {
                throw std::runtime_error("Unknown side: " + value);
            }
            options.side = *side;
        }
        else if (arg == "--family")
        {
            const std::string value = require_value(arg);
            auto family = manus_ros2::ParseDeviceFamilySlug(value);
            if (!family)
            {
                throw std::runtime_error("Unknown glove family: " + value);
            }
            options.family = *family;
        }
        else if (arg == "--glove-id")
        {
            options.glove_id = ParseUint32(require_value(arg), arg);
        }
        else if (arg == "--calibration-directory")
        {
            options.calibration_directory = manus_ros2::ExpandUserPath(require_value(arg));
        }
        else if (arg == "--export-current")
        {
            options.export_current = true;
        }
        else if (arg == "--overwrite")
        {
            options.overwrite = true;
        }
        else if (arg == "--verbose-sdk-logs")
        {
            options.verbose_sdk_logs = true;
        }
        else if (arg == "--core")
        {
            options.use_core_connection = true;
        }
        else if (arg == "--ip")
        {
            options.ip = require_value(arg);
        }
        else if (arg == "--wait-seconds")
        {
            options.wait_seconds = static_cast<int>(ParseUint32(require_value(arg), arg));
        }
        else
        {
            throw std::runtime_error("Unknown option: " + arg);
        }
    }

    return options;
}

bool SdkOk(SDKReturnCode code, const std::string& action)
{
    if (code == SDKReturnCode::SDKReturnCode_Success)
    {
        return true;
    }

    std::cerr << action << " failed with SDK return code " << static_cast<int32_t>(code) << "\n";
    return false;
}

class SdkSession
{
public:
    explicit SdkSession(const Options& options)
    {
        g_verbose_sdk_logs = options.verbose_sdk_logs;

        const SDKReturnCode init_result = options.use_core_connection
            ? CoreSdk_InitializeCore()
            : CoreSdk_InitializeIntegrated();
        if (!SdkOk(init_result, "SDK initialization"))
        {
            throw std::runtime_error("Failed to initialize MANUS SDK");
        }
        initialized_ = true;

        CoreSdk_RegisterCallbackForOnLog(OnSdkLog);

        if (!SdkOk(CoreSdk_RegisterCallbackForLandscapeStream(OnLandscapeCallback),
                   "Register landscape callback"))
        {
            throw std::runtime_error("Failed to register landscape callback");
        }

        CoordinateSystemVUH coordinate_system;
        CoordinateSystemVUH_Init(&coordinate_system);
        if (!SdkOk(CoreSdk_InitializeCoordinateSystemWithVUH(coordinate_system, true),
                   "Initialize coordinate system"))
        {
            throw std::runtime_error("Failed to initialize coordinate system");
        }

        Connect(options.ip);
    }

    ~SdkSession()
    {
        if (initialized_)
        {
            CoreSdk_ShutDown();
        }
    }

    SdkSession(const SdkSession&) = delete;
    SdkSession& operator=(const SdkSession&) = delete;

private:
    void Connect(const std::string& ip)
    {
        if (!SdkOk(CoreSdk_LookForHosts(5, false), "Look for MANUS Core hosts"))
        {
            throw std::runtime_error("Failed to look for MANUS Core hosts");
        }

        uint32_t host_count = 0;
        if (!SdkOk(CoreSdk_GetNumberOfAvailableHostsFound(&host_count), "Get host count"))
        {
            throw std::runtime_error("Failed to get MANUS Core host count");
        }

        if (host_count == 0)
        {
            throw std::runtime_error("No MANUS Core hosts found");
        }

        std::vector<ManusHost> hosts(host_count);
        if (!SdkOk(CoreSdk_GetAvailableHostsFound(hosts.data(), host_count), "Get host list"))
        {
            throw std::runtime_error("Failed to get MANUS Core host list");
        }

        size_t selected_host = 0;
        if (!ip.empty())
        {
            bool found = false;
            for (size_t i = 0; i < hosts.size(); ++i)
            {
                const std::string address = hosts[i].ipAddress;
                const std::string host_ip = address.substr(0, address.find(':'));
                if (host_ip == ip)
                {
                    selected_host = i;
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                throw std::runtime_error("Could not find MANUS Core host with IP " + ip);
            }
        }

        if (!SdkOk(CoreSdk_ConnectToHost(hosts[selected_host]), "Connect to MANUS Core"))
        {
            throw std::runtime_error("Failed to connect to MANUS Core");
        }
    }

    bool initialized_ = false;
};

Landscape WaitForLandscape(int wait_seconds)
{
    std::unique_lock<std::mutex> lock(g_landscape_mutex);
    const bool ready = g_landscape_cv.wait_for(
        lock,
        std::chrono::seconds(wait_seconds),
        [] {
            return g_landscape.has_value() &&
                g_landscape->gloveDevices.gloveCount > 0;
        });

    if (!ready)
    {
        throw std::runtime_error("Timed out waiting for connected MANUS gloves");
    }

    return *g_landscape;
}

std::vector<GloveLandscapeData> FilterGloves(const Landscape& landscape, const Options& options)
{
    std::vector<GloveLandscapeData> gloves;
    for (uint32_t i = 0; i < landscape.gloveDevices.gloveCount; ++i)
    {
        const GloveLandscapeData& glove = landscape.gloveDevices.gloves[i];
        if (options.glove_id && glove.id != *options.glove_id)
        {
            continue;
        }
        if (options.side && glove.side != *options.side)
        {
            continue;
        }
        if (options.family && glove.familyType != *options.family)
        {
            continue;
        }
        gloves.push_back(glove);
    }
    return gloves;
}

void PrintGloves(const std::vector<GloveLandscapeData>& gloves)
{
    if (gloves.empty())
    {
        std::cout << "No matching gloves found.\n";
        return;
    }

    std::cout << "Connected gloves:\n";
    for (size_t i = 0; i < gloves.size(); ++i)
    {
        const GloveLandscapeData& glove = gloves[i];
        std::cout << "  [" << i << "] id=" << glove.id
                  << " side=" << manus_ros2::SideToSlug(glove.side)
                  << " family=" << manus_ros2::DeviceFamilyToSlug(glove.familyType)
                  << " (" << manus_ros2::DeviceFamilyToDisplayName(glove.familyType) << ")"
                  << " haptics=" << (glove.isHaptics ? "yes" : "no")
                  << "\n";
    }
}

GloveLandscapeData SelectGlove(const std::vector<GloveLandscapeData>& gloves)
{
    if (gloves.empty())
    {
        throw std::runtime_error("No matching gloves found");
    }

    if (gloves.size() == 1)
    {
        return gloves.front();
    }

    PrintGloves(gloves);
    std::cout << "Select glove index: ";
    std::string line;
    if (!std::getline(std::cin, line))
    {
        throw std::runtime_error("No glove selected");
    }

    const uint32_t selected = ParseUint32(line, "glove index");
    if (selected >= gloves.size())
    {
        throw std::runtime_error("Glove index out of range");
    }
    return gloves[selected];
}

void WaitForEnter(const std::string& prompt)
{
    std::cout << prompt;
    std::string line;
    if (!std::getline(std::cin, line))
    {
        throw std::runtime_error("Input ended");
    }
}

bool AskYesNo(const std::string& prompt)
{
    while (true)
    {
        std::cout << prompt << " [y/N]: ";
        std::string line;
        if (!std::getline(std::cin, line))
        {
            return false;
        }
        if (line == "y" || line == "Y" || line == "yes" || line == "YES")
        {
            return true;
        }
        if (line.empty() || line == "n" || line == "N" || line == "no" || line == "NO")
        {
            return false;
        }
    }
}

void RunCalibration(uint32_t glove_id)
{
    GloveCalibrationArgs args{};
    args.gloveId = glove_id;

    bool sdk_result = false;
    if (!SdkOk(CoreSdk_GloveCalibrationStart(args, &sdk_result), "Start glove calibration") ||
        !sdk_result)
    {
        throw std::runtime_error("MANUS Core rejected calibration start");
    }

    uint32_t step_count = 0;
    if (!SdkOk(CoreSdk_GloveCalibrationGetNumberOfSteps(args, &step_count),
               "Get calibration step count"))
    {
        CoreSdk_GloveCalibrationStop(args, &sdk_result);
        throw std::runtime_error("Could not get calibration steps");
    }

    std::cout << "Calibration has " << step_count << " step(s).\n";
    for (uint32_t step = 0; step < step_count; ++step)
    {
        GloveCalibrationStepArgs step_args{};
        step_args.gloveId = glove_id;
        step_args.stepIndex = step;

        GloveCalibrationStepData step_data{};
        if (!SdkOk(CoreSdk_GloveCalibrationGetStepData(step_args, &step_data),
                   "Get calibration step data"))
        {
            CoreSdk_GloveCalibrationStop(args, &sdk_result);
            throw std::runtime_error("Could not get calibration step data");
        }

        std::cout << "\nStep " << (step + 1) << "/" << step_count << "\n";
        if (step_data.title[0] != '\0')
        {
            std::cout << step_data.title << "\n";
        }
        if (step_data.description[0] != '\0')
        {
            std::cout << step_data.description << "\n";
        }
        const double step_seconds = step < 2 ? 1.0 : 10.0;
        std::cout << "Duration: " << step_seconds << " seconds\n";

        WaitForEnter("Press Enter when ready to start this step...");

        sdk_result = false;
        if (!SdkOk(CoreSdk_GloveCalibrationStartStep(step_args, &sdk_result),
                   "Start calibration step") ||
            !sdk_result)
        {
            CoreSdk_GloveCalibrationStop(args, &sdk_result);
            throw std::runtime_error("MANUS Core rejected calibration step");
        }

        const auto wait_time = std::chrono::milliseconds(
            static_cast<int>(step_seconds * 1000.0) + 250);
        std::this_thread::sleep_for(wait_time);
        std::cout << "Step capture window elapsed.\n";
    }

    sdk_result = false;
    if (!SdkOk(CoreSdk_GloveCalibrationFinish(args, &sdk_result), "Finish glove calibration") ||
        !sdk_result)
    {
        CoreSdk_GloveCalibrationStop(args, &sdk_result);
        throw std::runtime_error("MANUS Core rejected calibration finish");
    }
}

void SaveCalibration(const GloveLandscapeData& glove, const Options& options)
{
    const std::filesystem::path output_path = manus_ros2::DefaultCalibrationPath(
        options.calibration_directory, glove.familyType, glove.side);

    std::error_code ec;
    if (std::filesystem::exists(output_path, ec) && !options.overwrite)
    {
        if (!AskYesNo("Calibration file exists: " + output_path.string() + ". Overwrite?"))
        {
            throw std::runtime_error("Not overwriting existing calibration file");
        }
    }

    uint32_t calibration_size = 0;
    if (!SdkOk(CoreSdk_GetGloveCalibrationSize(glove.id, &calibration_size),
               "Get glove calibration size"))
    {
        throw std::runtime_error("Could not get calibration size");
    }

    if (calibration_size == 0)
    {
        throw std::runtime_error("SDK returned an empty calibration");
    }

    std::vector<unsigned char> calibration_bytes(calibration_size);
    if (!SdkOk(CoreSdk_GetGloveCalibration(calibration_bytes.data(), calibration_size),
               "Get glove calibration bytes"))
    {
        throw std::runtime_error("Could not get calibration bytes");
    }

    std::filesystem::create_directories(output_path.parent_path(), ec);
    if (ec)
    {
        throw std::runtime_error("Could not create directory " + output_path.parent_path().string());
    }

    std::ofstream output(output_path, std::ios::binary);
    if (!output.is_open())
    {
        throw std::runtime_error("Could not open " + output_path.string() + " for writing");
    }

    output.write(
        reinterpret_cast<const char*>(calibration_bytes.data()),
        static_cast<std::streamsize>(calibration_bytes.size()));
    if (!output.good())
    {
        throw std::runtime_error("Failed while writing " + output_path.string());
    }

    std::cout << "Saved calibration: " << output_path << "\n";
}

int Run(int argc, char* argv[])
{
    const Options options = ParseArgs(argc, argv);

    SdkSession session(options);
    const Landscape landscape = WaitForLandscape(options.wait_seconds);
    const std::vector<GloveLandscapeData> gloves = FilterGloves(landscape, options);

    if (options.list_only)
    {
        PrintGloves(gloves);
        return 0;
    }

    const GloveLandscapeData glove = SelectGlove(gloves);
    std::cout << "Selected glove id=" << glove.id
              << " side=" << manus_ros2::SideToSlug(glove.side)
              << " family=" << manus_ros2::DeviceFamilyToSlug(glove.familyType)
              << "\n";

    if (!options.export_current)
    {
        RunCalibration(glove.id);
    }

    SaveCalibration(glove, options);
    return 0;
}

}  // namespace

int main(int argc, char* argv[])
{
    try
    {
        return Run(argc, argv);
    }
    catch (const std::exception& exc)
    {
        std::cerr << "manus_calibration_tool: " << exc.what() << "\n";
        return 1;
    }
}
