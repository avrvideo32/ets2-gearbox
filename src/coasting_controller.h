#pragma once
#include "shift_types.h"
#include "config.h"
#include "input_device.h"
#include <functional>

namespace ecodrive
{

class CoastingController
{
public:
    CoastingController() = default;

    void on_throttle_sample(
        float sim_throttle,
        float driver_throttle,
        int current_gear,
        bool shift_in_progress,
        const Config& config);

    // FIX: added speed sample for low-speed debounce.
    void on_speed_sample(float speed);

    bool should_start_coasting(
        bool manual_driver_wants_drive,
        unsigned grace_updates,
        bool brake_active,
        float sim_throttle,
        int current_gear,
        float current_speed,
        const Config& config,
        const PowertrainContext& context = {}) const;

    void start_coasting(
        int current_gear,
        float current_speed,
        float current_cruise,
        const Config& config,
        const std::function<void(const char*)>& logger);

    // FIX: added optional force_restore for safety restoration from neutral.
    void start_restore_if_needed(
        InputDevice& input,
        int current_gear,
        float speed,
        float driver_throttle,
        bool cruise_active,
        bool driver_wants_drive,
        bool shift_in_progress,
        int minimum_gear,
        int effective_max_gear,
        const Config& config,
        const std::function<void(const char*)>& logger,
        const PowertrainContext& context = {},
        bool force_restore = false);

    void update_neutral_logic(
        InputDevice& input,
        int current_gear,
        float speed,
        float speed_delta,
        float driver_throttle,
        bool cruise_active,
        bool driver_wants_drive,
        int minimum_gear,
        int effective_max_gear,
        const Config& config,
        const std::function<void(const char*)>& logger,
        bool& shift_in_progress,
        int& requested_gear,
        ShiftPurpose& shift_purpose,
        unsigned& shift_wait_updates,
        const PowertrainContext& context = {});

    void update_restore_logic(
        InputDevice& input,
        int current_gear,
        bool driver_wants_drive,
        int effective_max_gear,
        const Config& config,
        const std::function<void(const char*)>& logger,
        bool& shift_in_progress,
        int& requested_gear,
        ShiftPurpose& shift_purpose,
        unsigned& shift_wait_updates);

    void on_shift_confirmed(
        InputDevice& input,
        ShiftPurpose purpose,
        int confirmed_gear,
        int effective_max_gear,
        const Config& config,
        const std::function<void(const char*)>& logger);

    void on_manual_override(
        InputDevice& input,
        int confirmed_gear,
        int effective_max_gear,
        const Config& config,
        const std::function<void(const char*)>& logger);

    bool is_neutral_in_progress() const
    {
        return neutral_in_progress_;
    }

    void set_neutral_in_progress(bool v)
    {
        neutral_in_progress_ = v;
    }

    bool is_restore_in_progress() const
    {
        return restore_in_progress_;
    }

    void set_restore_in_progress(bool v)
    {
        restore_in_progress_ = v;
    }

    bool is_post_neutral_recovery() const
    {
        return post_neutral_recovery_;
    }

    void set_post_neutral_recovery(bool v)
    {
        post_neutral_recovery_ = v;
    }

    int remembered_gear() const
    {
        return remembered_gear_;
    }

    void set_remembered_gear(int g)
    {
        remembered_gear_ = g;
    }

    float remembered_cruise_speed() const
    {
        return remembered_cruise_speed_;
    }

    void set_remembered_cruise_speed(float s)
    {
        remembered_cruise_speed_ = s;
    }

    int restore_target_gear() const
    {
        return restore_target_gear_;
    }

    int calculate_speed_matched_gear(
        float speed,
        int takeoff_gear,
        int remembered,
        int effective_max_gear,
        const PowertrainContext& context = {}) const;

    void reset();

private:
    bool neutral_in_progress_ = false;
    bool restore_in_progress_ = false;
    bool post_neutral_recovery_ = false;

    int remembered_gear_ = 0;
    int restore_target_gear_ = 0;

    float remembered_cruise_speed_ = 0.0f;
    float coasting_start_speed_ = 0.0f;

    unsigned restore_wait_updates_ = 0;
    unsigned restore_throttle_updates_ = 0;
    unsigned effective_throttle_zero_updates_ = 0;

    // FIX: debounce low-speed neutral entry.
    unsigned low_speed_updates_ = 0;
};

}