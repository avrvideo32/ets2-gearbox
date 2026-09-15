#pragma once
#include "shift_types.h"
#include "config.h"
#include "input_device.h"
#include <array>
#include <chrono>
#include <functional>

namespace ecodrive
{

class CoastingController;
class AdaptiveLearner;
class TripTracker;

class Shifter
{
public:
    Shifter() = default;

    void on_rpm_tick();

    void on_speed_sample(
        float new_speed,
        float delta,
        float throttle,
        float current_cruise,
        int current_gear,
        float current_rpm,
        const Config& config,
        AdaptiveLearner& learner,
        const std::function<void(const char*)>& logger);

    void on_brake_sample(float brake, const Config& config);

    void on_cruise_changed(
        float old_cruise,
        float new_cruise,
        const Config& config,
        const std::function<void(const char*)>& logger);

    void update_automatic_shifter(
        InputDevice& input,
        int g,
        float r,
        float s,
        float speed_delta,
        float t,
        float clutch,
        bool cruise,
        float cruise_target,
        bool throttle_active,
        int minimum_gear,
        bool minimum_gear_locked,
        int effective_max_gear,
        const Config& config,
        CoastingController& coasting,
        AdaptiveLearner& learner,
        const std::function<void(const char*)>& logger,
        const PowertrainContext& context = {});

    void on_gear_update(
        InputDevice& input,
        int g,
        int previous_gear,
        int effective_max_gear,
        const Config& config,
        CoastingController& coasting,
        AdaptiveLearner& learner,
        TripTracker& tracker,
        const std::function<void(const char*)>& logger);

    bool is_shift_in_progress() const
    {
        return shift_in_progress_;
    }

    void set_shift_in_progress(bool v)
    {
        shift_in_progress_ = v;
    }

    int requested_gear() const
    {
        return requested_gear_;
    }

    void set_requested_gear(int g)
    {
        requested_gear_ = g;
    }

    ShiftPurpose shift_purpose() const
    {
        return shift_purpose_;
    }

    void set_shift_purpose(ShiftPurpose p)
    {
        shift_purpose_ = p;
    }

    unsigned shift_wait_updates() const
    {
        return shift_wait_updates_;
    }

    void set_shift_wait_updates(unsigned u)
    {
        shift_wait_updates_ = u;
    }

    unsigned post_shift_drive_grace_updates() const
    {
        return post_shift_drive_grace_updates_;
    }

    void set_post_shift_drive_grace_updates(unsigned u)
    {
        post_shift_drive_grace_updates_ = u;
    }

    bool is_brake_active() const;

    void reset();

private:
    static constexpr std::size_t GEAR_COUNT = 33;

    bool shift_in_progress_ = false;
    int requested_gear_ = 0;
    ShiftPurpose shift_purpose_ = ShiftPurpose::none;
    unsigned shift_wait_updates_ = 0;

    std::chrono::steady_clock::time_point last_automatic_shift_time_{};

    int multi_upshift_target_gear_ = 0;
    unsigned multi_upshift_wait_updates_ = 0;

    int multi_downshift_target_gear_ = 0;
    unsigned multi_downshift_wait_updates_ = 0;

    unsigned load_downshift_cooldown_updates_ = 0;
    unsigned load_downshift_upshift_block_updates_ = 0;
    float load_downshift_start_speed_ = 0.0f;

    std::array<unsigned, GEAR_COUNT> gear_upshift_block_updates_{};

    unsigned brake_active_updates_ = 0;
    bool brake_downshift_triggered_ = false;

    // FIX: store actual brake value for correct logging.
    float last_brake_value_ = 0.0f;

    unsigned load_downshift_stable_updates_ = 0;
    unsigned hillclimb_downshift_stable_updates_ = 0;
    unsigned normal_downshift_stable_updates_ = 0;

    float hillclimb_reference_speed_ = 0.0f;

    unsigned cruise_economy_cooldown_updates_ = 0;
    int cruise_blocked_upshift_gear_ = 0;
    float cruise_blocked_target_speed_ = 0.0f;
    unsigned cruise_block_updates_ = 0;

    float cruise_shift_start_speed_ = 0.0f;
    int cruise_shift_from_gear_ = 0;
    float cruise_shift_throttle_ = 0.0f;
    float cruise_shift_speed_delta_ = 0.0f;

    unsigned post_shift_drive_grace_updates_ = 0;
};

}