#include "config.h"
#include <windows.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

namespace
{

constexpr const char* CONFIG_FILE_NAME = "ecodrive.cfg";

std::string trim(const std::string& value)
{
    std::size_t first = 0;
    while (first < value.size() && std::isspace(static_cast<unsigned char>(value[first]))) ++first;
    std::size_t last = value.size();
    while (last > first && std::isspace(static_cast<unsigned char>(value[last - 1]))) --last;
    return value.substr(first, last - first);
}

bool parse_float(const std::string& text, float& output)
{
    try { std::size_t position = 0; const float value = std::stof(text, &position); if (position != text.size() || !std::isfinite(value)) return false; output = value; return true; }
    catch (...) { return false; }
}

bool parse_int(const std::string& text, int& output)
{
    try { std::size_t position = 0; const int value = std::stoi(text, &position); if (position != text.size()) return false; output = value; return true; }
    catch (...) { return false; }
}

bool parse_unsigned(const std::string& text, unsigned& output)
{
    try { std::size_t position = 0; const unsigned long value = std::stoul(text, &position); if (position != text.size() || value > std::numeric_limits<unsigned>::max()) return false; output = static_cast<unsigned>(value); return true; }
    catch (...) { return false; }
}

bool parse_bool(const std::string& text, bool& output)
{
    std::string value = text;
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (value == "true" || value == "1" || value == "yes" || value == "on") { output = true; return true; }
    if (value == "false" || value == "0" || value == "no" || value == "off") { output = false; return true; }
    return false;
}

std::string get_dll_directory()
{
    HMODULE module = nullptr;
    if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, reinterpret_cast<LPCSTR>(&get_dll_directory), &module)) return {};
    char path[MAX_PATH]{};
    const DWORD length = GetModuleFileNameA(module, path, MAX_PATH);
    if (length == 0 || length >= MAX_PATH) return {};
    std::string result(path, length);
    const std::size_t slash = result.find_last_of("\\/");
    if (slash == std::string::npos) return {};
    result.resize(slash);
    return result;
}

std::string get_config_path()
{
    const std::string directory = get_dll_directory();
    if (directory.empty()) return {};
    return directory + "\\" + CONFIG_FILE_NAME;
}

}

namespace ecodrive
{

Config::Config() { set_defaults(); }

void Config::set_defaults()
{
    upshift_rpm = 1350.0f;
    downshift_rpm = 1050.0f;
    min_upshift_throttle = 0.12f;
    automatic_shift_cooldown_ms = 350;

    max_forward_gear = 16;
    takeoff_gear = 1;
    multi_upshift_enabled = true;
    multi_upshift_max_gear = 5;

    neutral_coasting_enabled = true;
    coast_zero_throttle_delay_updates = 60;
    restore_throttle = 0.15f;
    restore_wait_updates = 4;

    hillclimb_downshift_enabled = true;
    hillclimb_throttle_threshold = 0.70f;
    hillclimb_rpm_threshold = 1150.0f;
    load_downshift_rpm = 950.0f;

    cruise_economy_enabled = true;
    cruise_economy_min_rpm = 1250.0f;
    adaptive_learning_enabled = true;

    brake_downshift_enabled = true;
    brake_downshift_threshold = 0.15f;

    grade_detection_enabled = true;
    retarder_downshift_enabled = true;
    load_adaptive_shifting_enabled = true;
    speed_limit_awareness_enabled = true;

    telemetry_logging = false;
    shift_logging = true;
    trip_summary_interval_minutes = 5;
}

void Config::write_log(scs_log_t log, const char* message) const
{
    if (log && message) log(SCS_LOG_TYPE_message, message);
}

bool Config::write_default_file(const char* path, scs_log_t log)
{
    if (!path || !*path) return false;
    std::ofstream file(path);
    if (!file.is_open()) return false;

    file
        << "# ==============================================================================\n"
        << "# EcoDrive Configuration File\n"
        << "# ==============================================================================\n\n"
        << "upshift_rpm=1350\n"
        << "downshift_rpm=1050\n"
        << "min_upshift_throttle=0.12\n"
        << "automatic_shift_cooldown_ms=350\n\n"
        << "max_forward_gear=16\n"
        << "takeoff_gear=1\n"
        << "multi_upshift_enabled=true\n"
        << "multi_upshift_max_gear=5\n\n"
        << "# Neutral coasting and drivetrain restoration\n"
        << "neutral_coasting_enabled=true\n"
        << "coast_zero_throttle_delay_updates=60\n"
        << "restore_throttle=0.15\n"
        << "restore_wait_updates=4\n\n"
        << "hillclimb_downshift_enabled=true\n"
        << "hillclimb_throttle_threshold=0.70\n"
        << "hillclimb_rpm_threshold=1150\n"
        << "load_downshift_rpm=950\n\n"
        << "cruise_economy_enabled=true\n"
        << "cruise_economy_min_rpm=1250\n"
        << "adaptive_learning_enabled=true\n\n"
        << "brake_downshift_enabled=true\n"
        << "brake_downshift_threshold=0.15\n\n"
        << "grade_detection_enabled=true\n"
        << "retarder_downshift_enabled=true\n"
        << "load_adaptive_shifting_enabled=true\n"
        << "speed_limit_awareness_enabled=true\n\n"
        << "logging_level=shifts\n"
        << "trip_summary_interval_minutes=5\n";

    file.close();
    write_log(log, "EcoDrive: created default ecodrive.cfg");
    return true;
}

bool Config::check_and_reload_if_modified(scs_log_t log)
{
    if (++frame_counter_ < FILE_CHECK_INTERVAL_FRAMES) return false;
    frame_counter_ = 0;

    const std::string path = get_config_path();
    if (path.empty()) return false;

    WIN32_FILE_ATTRIBUTE_DATA file_attr{};
    if (!GetFileAttributesExA(path.c_str(), GetFileExInfoStandard, &file_attr)) return false;

    const uint64_t current_time = (static_cast<uint64_t>(file_attr.ftLastWriteTime.dwHighDateTime) << 32) | static_cast<uint64_t>(file_attr.ftLastWriteTime.dwLowDateTime);
    if (last_file_write_time_ != 0 && current_time != last_file_write_time_)
    {
        write_log(log, "EcoDrive: detected change in ecodrive.cfg -> reloading settings live!");
        return load(log);
    }
    return false;
}

bool Config::load(scs_log_t log)
{
    set_defaults();

    const std::string path = get_config_path();
    if (path.empty())
    {
        write_log(log, "EcoDrive: could not determine DLL directory; using defaults");
        return false;
    }

    std::ifstream file(path);
    if (!file.is_open())
    {
        if (!write_default_file(path.c_str(), log))
        {
            write_log(log, "EcoDrive: could not create ecodrive.cfg; using defaults");
            return false;
        }

        WIN32_FILE_ATTRIBUTE_DATA file_attr{};
        if (GetFileAttributesExA(path.c_str(), GetFileExInfoStandard, &file_attr))
            last_file_write_time_ = (static_cast<uint64_t>(file_attr.ftLastWriteTime.dwHighDateTime) << 32) | static_cast<uint64_t>(file_attr.ftLastWriteTime.dwLowDateTime);
        return true;
    }

    WIN32_FILE_ATTRIBUTE_DATA file_attr{};
    if (GetFileAttributesExA(path.c_str(), GetFileExInfoStandard, &file_attr))
        last_file_write_time_ = (static_cast<uint64_t>(file_attr.ftLastWriteTime.dwHighDateTime) << 32) | static_cast<uint64_t>(file_attr.ftLastWriteTime.dwLowDateTime);

    std::string line;
    unsigned line_number = 0;
    bool logging_level_set = false;

    while (std::getline(file, line))
    {
        ++line_number;
        const std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#' || trimmed[0] == ';' || trimmed[0] == '[') continue;

        const std::size_t equals = trimmed.find('=');
        if (equals == std::string::npos)
        {
            char message[256]{};
            std::snprintf(message, sizeof(message), "EcoDrive: invalid config line %u", line_number);
            write_log(log, message);
            continue;
        }

        const std::string key = trim(trimmed.substr(0, equals));
        const std::string value = trim(trimmed.substr(equals + 1));

        if (key == "upshift_rpm") parse_float(value, upshift_rpm);
        else if (key == "downshift_rpm") parse_float(value, downshift_rpm);
        else if (key == "min_upshift_throttle") parse_float(value, min_upshift_throttle);
        else if (key == "automatic_shift_cooldown_ms") parse_unsigned(value, automatic_shift_cooldown_ms);
        else if (key == "max_forward_gear") parse_int(value, max_forward_gear);
        else if (key == "takeoff_gear") parse_int(value, takeoff_gear);
        else if (key == "multi_upshift_enabled") parse_bool(value, multi_upshift_enabled);
        else if (key == "multi_upshift_max_gear") parse_int(value, multi_upshift_max_gear);
        else if (key == "neutral_coasting_enabled") parse_bool(value, neutral_coasting_enabled);
        else if (key == "coast_zero_throttle_delay_updates") parse_unsigned(value, coast_zero_throttle_delay_updates);
        else if (key == "restore_throttle") parse_float(value, restore_throttle);
        else if (key == "restore_wait_updates") parse_unsigned(value, restore_wait_updates);
        else if (key == "hillclimb_downshift_enabled") parse_bool(value, hillclimb_downshift_enabled);
        else if (key == "hillclimb_throttle_threshold") parse_float(value, hillclimb_throttle_threshold);
        else if (key == "hillclimb_rpm_threshold") parse_float(value, hillclimb_rpm_threshold);
        else if (key == "load_downshift_rpm") parse_float(value, load_downshift_rpm);
        else if (key == "cruise_economy_enabled") parse_bool(value, cruise_economy_enabled);
        else if (key == "cruise_economy_min_rpm") parse_float(value, cruise_economy_min_rpm);
        else if (key == "adaptive_learning_enabled") parse_bool(value, adaptive_learning_enabled);
        else if (key == "brake_downshift_enabled") parse_bool(value, brake_downshift_enabled);
        else if (key == "brake_downshift_threshold") parse_float(value, brake_downshift_threshold);
        else if (key == "grade_detection_enabled") parse_bool(value, grade_detection_enabled);
        else if (key == "retarder_downshift_enabled") parse_bool(value, retarder_downshift_enabled);
        else if (key == "load_adaptive_shifting_enabled") parse_bool(value, load_adaptive_shifting_enabled);
        else if (key == "speed_limit_awareness_enabled") parse_bool(value, speed_limit_awareness_enabled);
        else if (key == "trip_summary_interval_minutes") parse_unsigned(value, trip_summary_interval_minutes);
        else if (key == "logging_level")
        {
            std::string level = value;
            std::transform(level.begin(), level.end(), level.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (level == "off") { telemetry_logging = false; shift_logging = false; logging_level_set = true; }
            else if (level == "shifts") { telemetry_logging = false; shift_logging = true; logging_level_set = true; }
            else if (level == "verbose") { telemetry_logging = true; shift_logging = true; logging_level_set = true; }
            else { char message[256]{}; std::snprintf(message, sizeof(message), "EcoDrive: invalid logging_level on line %u", line_number); write_log(log, message); }
        }
        else if (key == "telemetry_logging" && !logging_level_set) parse_bool(value, telemetry_logging);
        else if (key == "shift_logging" && !logging_level_set) parse_bool(value, shift_logging);
    }

    file.close();

    upshift_rpm = std::clamp(upshift_rpm, 800.0f, 2500.0f);
    downshift_rpm = std::clamp(downshift_rpm, 500.0f, 2000.0f);
    if (downshift_rpm >= upshift_rpm - 100.0f)
    {
        downshift_rpm = std::max(500.0f, upshift_rpm - 150.0f);
        write_log(log, "EcoDrive warning: downshift_rpm was >= upshift_rpm - 100; auto-adjusted for hysteresis.");
    }

    min_upshift_throttle = std::clamp(min_upshift_throttle, 0.01f, 1.0f);
    restore_throttle = std::clamp(restore_throttle, 0.01f, 1.0f);
    max_forward_gear = std::clamp(max_forward_gear, 1, 32);
    takeoff_gear = std::clamp(takeoff_gear, 1, 8);
    multi_upshift_max_gear = std::clamp(multi_upshift_max_gear, 1, 32);
    restore_wait_updates = std::clamp(restore_wait_updates, 1u, 100u);
    coast_zero_throttle_delay_updates = std::clamp(coast_zero_throttle_delay_updates, 5u, 300u);
    automatic_shift_cooldown_ms = std::clamp(automatic_shift_cooldown_ms, 50u, 5000u);
    hillclimb_throttle_threshold = std::clamp(hillclimb_throttle_threshold, 0.10f, 1.0f);
    hillclimb_rpm_threshold = std::clamp(hillclimb_rpm_threshold, 600.0f, 2200.0f);
    load_downshift_rpm = std::clamp(load_downshift_rpm, 500.0f, 2000.0f);
    cruise_economy_min_rpm = std::clamp(cruise_economy_min_rpm, 600.0f, 2200.0f);
    brake_downshift_threshold = std::clamp(brake_downshift_threshold, 0.05f, 1.0f);
    trip_summary_interval_minutes = std::clamp(trip_summary_interval_minutes, 0u, 120u);

    char message[512]{};
    std::snprintf(message, sizeof(message), "EcoDrive: config loaded | upshift %.0f | downshift %.0f | coast: %s | multi-up: %s | grade: %s | retarder: %s | load-adapt: %s", upshift_rpm, downshift_rpm, neutral_coasting_enabled ? "ON" : "OFF", multi_upshift_enabled ? "ON" : "OFF", grade_detection_enabled ? "ON" : "OFF", retarder_downshift_enabled ? "ON" : "OFF", load_adaptive_shifting_enabled ? "ON" : "OFF");
    write_log(log, message);
    return true;
}

}