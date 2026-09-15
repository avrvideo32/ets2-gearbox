#pragma once
#include "scssdk_telemetry.h"
#include "config.h"
#include "trip_tracker.h"
#include "adaptive_learner.h"
#include "coasting_controller.h"
#include "shifter.h"
#include <chrono>
#include <atomic>
#include <fstream>
#include <mutex>
#include <string>

namespace ecodrive
{

class InputDevice;

class Telemetry
{
public:
    explicit Telemetry(InputDevice& input_device);

    scs_result_t initialize(const scs_telemetry_init_params_v100_t& params);
    void shutdown();

private:
    static SCSAPI_VOID telemetry_channel_callback(
        const scs_string_t name,
        const scs_u32_t index,
        const scs_value_t* const value,
        const scs_context_t context);

    static SCSAPI_VOID telemetry_configuration_callback(
        const scs_event_t event,
        const void* const event_info,
        const scs_context_t context);

    static SCSAPI_VOID telemetry_frame_end_callback(
        const scs_event_t event,
        const void* const event_info,
        const scs_context_t context);

    void handle_channel(
        const scs_string_t name,
        const scs_u32_t index,
        const scs_value_t* const value);

    void handle_configuration(const scs_telemetry_configuration_t* config);

    void on_frame_end();

    bool cruise_active() const;
    bool driver_wants_drive() const;
    bool manual_driver_wants_drive() const;

    int effective_max_gear() const;
    int calculate_dynamic_takeoff_gear() const;

    void log(const char* message);
    void open_file_log();
    void close_file_log();
    void write_file_log(const char* message);

    InputDevice& input_device_;

    Config config_;
    PowertrainContext powertrain_context_{};

    TripTracker trip_tracker_;
    AdaptiveLearner adaptive_learner_;
    CoastingController coasting_;
    Shifter shifter_;

    scs_telemetry_unregister_from_channel_t unregister_from_channel_ = nullptr;
    scs_telemetry_unregister_from_event_t unregister_from_event_ = nullptr;

    scs_log_t log_ = nullptr;

    std::ofstream file_log_;
    std::mutex file_log_mutex_;

    std::string file_log_path_;
    unsigned file_log_flush_counter_ = 0;

    bool initialized_ = false;

    mutable std::mutex state_mutex_;

    int detected_forward_gears_ = 0;
    float detected_rpm_limit_ = 0.0f;

    std::string truck_brand_;
    std::string truck_name_;
    std::string truck_id_;

    std::atomic<float> rpm_{0.0f};
    std::atomic<float> speed_{0.0f};
    std::atomic<float> throttle_{0.0f};
    std::atomic<float> input_throttle_{0.0f};
    std::atomic<float> input_brake_{0.0f};
    std::atomic<float> clutch_{0.0f};
    std::atomic<int> gear_{0};
    std::atomic<float> cruise_control_{0.0f};
    std::atomic<float> fuel_{0.0f};

    int minimum_gear_ = 0;

    bool engine_running_ = false;
    bool minimum_gear_locked_ = false;

    int previous_gear_ = 0;

    // FIX: now stores previous absolute speed.
    float previous_speed_ = 0.0f;
    float speed_delta_ = 0.0f;

    float previous_cruise_ = 0.0f;

    std::chrono::steady_clock::time_point last_frame_time_{};

    bool throttle_active_ = false;
    unsigned throttle_active_updates_ = 0;
    unsigned throttle_inactive_updates_ = 0;

    unsigned telemetry_log_counter_ = 0;
};

}