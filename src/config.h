#pragma once
#include "scssdk.h"
#include <cstdint>

namespace ecodrive
{

class Config
{
public:
    Config();

    bool load(scs_log_t log);
    bool check_and_reload_if_modified(scs_log_t log);

    // Core RPM & Automatic Shifting
    float upshift_rpm;
    float downshift_rpm;
    float min_upshift_throttle;
    unsigned automatic_shift_cooldown_ms;

    // Gear Limits & Multi-Upshift
    int max_forward_gear;
    int takeoff_gear;
    bool multi_upshift_enabled;
    int multi_upshift_max_gear;

    // Neutral Coasting & Restoration
    bool neutral_coasting_enabled;
    unsigned coast_zero_throttle_delay_updates;
    float restore_throttle;
    unsigned restore_wait_updates;

    // Hill Climb & Heavy Load
    bool hillclimb_downshift_enabled;
    float hillclimb_throttle_threshold;
    float hillclimb_rpm_threshold;
    float load_downshift_rpm;

    // Cruise Control & Adaptive Learning
    bool cruise_economy_enabled;
    float cruise_economy_min_rpm;
    bool adaptive_learning_enabled;

    // Braking Assistance
    bool brake_downshift_enabled;
    float brake_downshift_threshold;

    // Advanced Game Engine Powertrain Features
    bool grade_detection_enabled;
    bool retarder_downshift_enabled;
    bool load_adaptive_shifting_enabled;
    bool speed_limit_awareness_enabled;

    // Logging & Diagnostics
    bool telemetry_logging;
    bool shift_logging;
    unsigned trip_summary_interval_minutes;

private:
    void set_defaults();

    bool write_default_file(
        const char* path,
        scs_log_t log);

    void write_log(
        scs_log_t log,
        const char* message) const;

    uint64_t last_file_write_time_ = 0;

    // FIX: throttle config file checks to avoid Windows API call every frame.
    unsigned frame_counter_ = 0;
    static constexpr unsigned FILE_CHECK_INTERVAL_FRAMES = 60;
};

}