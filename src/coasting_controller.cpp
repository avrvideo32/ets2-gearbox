#include "coasting_controller.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace ecodrive
{
namespace
{
constexpr float THROTTLE_RELEASE_THRESHOLD = 0.02f;
constexpr float COAST_EXIT_THROTTLE = 0.35f;
constexpr float STANDSTILL_SPEED_THRESHOLD = 0.50f;
constexpr float COASTING_MIN_SPEED = 4.5f;
constexpr float COASTING_RESTORE_SPEED = 2.5f;
constexpr unsigned RESTORE_THROTTLE_DEBOUNCE_UPDATES = 4;
constexpr unsigned MAX_RECOVERY_BURST = 16;
constexpr int COASTING_GEAR_DROP = 2;

int rolling_recovery_min_gear(float speed, int takeoff_gear, int effective_max_gear)
{
    const float abs_speed = std::fabs(speed);
    if (abs_speed <= STANDSTILL_SPEED_THRESHOLD)
        return takeoff_gear;

    int minimum = takeoff_gear;
    if (abs_speed >= 4.5f) minimum = std::max(minimum, 2);
    if (abs_speed >= 8.0f) minimum = std::max(minimum, 3);
    if (abs_speed >= 12.0f) minimum = std::max(minimum, 4);
    if (abs_speed >= 17.0f) minimum = std::max(minimum, 5);

    return std::clamp(minimum, takeoff_gear, effective_max_gear);
}
}

void CoastingController::on_throttle_sample(float sim_throttle, float driver_throttle, int current_gear, bool shift_in_progress, const Config& config)
{
    if (sim_throttle > THROTTLE_RELEASE_THRESHOLD)
        effective_throttle_zero_updates_ = 0;

    if (driver_throttle >= COAST_EXIT_THROTTLE && !shift_in_progress)
        ++restore_throttle_updates_;
    else if (driver_throttle < COAST_EXIT_THROTTLE)
        restore_throttle_updates_ = 0;

    if (sim_throttle <= THROTTLE_RELEASE_THRESHOLD && driver_throttle <= THROTTLE_RELEASE_THRESHOLD && !shift_in_progress && current_gear >= 1)
        ++effective_throttle_zero_updates_;
    else
        effective_throttle_zero_updates_ = 0;

    (void)config;
}

void CoastingController::on_speed_sample(float speed)
{
    (void)speed;
}

bool CoastingController::should_start_coasting(bool manual_driver_wants_drive, unsigned grace_updates, bool brake_active, float sim_throttle, int current_gear, float current_speed, const Config& config, const PowertrainContext& context) const
{
    if (!config.neutral_coasting_enabled)
        return false;

    // A coast session is already active. Do not start a new session after
    // reaching its final two-gears-down target; doing so would create a
    // 12 -> 10 -> 8 -> 6 cascade while the driver remains off throttle.
    if (neutral_in_progress_)
        return false;

    const float abs_speed = std::fabs(current_speed);
    if (abs_speed < COASTING_MIN_SPEED)
        return false;

    if (current_gear < COASTING_GEAR_DROP + 1)
        return false;

    if (config.grade_detection_enabled && context.grade_percent > 2.5f)
        return false;
    if (manual_driver_wants_drive || grace_updates > 0 || brake_active)
        return false;
    if (sim_throttle > THROTTLE_RELEASE_THRESHOLD || current_gear <= 0)
        return false;

    return effective_throttle_zero_updates_ >= config.coast_zero_throttle_delay_updates;
}

void CoastingController::start_coasting(int current_gear, float current_speed, float current_cruise, const Config& config, const std::function<void(const char*)>& logger)
{
    if (current_gear < COASTING_GEAR_DROP + 1)
    {
        neutral_in_progress_ = false;
        return;
    }

    neutral_in_progress_ = true;
    restore_in_progress_ = false;
    remembered_gear_ = current_gear;
    coasting_start_speed_ = std::fabs(current_speed);
    remembered_cruise_speed_ = (current_cruise > 0.10f) ? current_cruise : 0.0f;
    restore_target_gear_ = 0;
    restore_wait_updates_ = 0;
    post_neutral_recovery_ = false;
    restore_throttle_updates_ = 0;

    if (config.shift_logging && logger)
    {
        char b[240];
        const int target = current_gear - COASTING_GEAR_DROP;
        if (remembered_cruise_speed() > 0.10f)
            std::snprintf(b, sizeof(b), "EcoDrive: COAST %d -> %d (2 gears down, cruise %.2f m/s)", current_gear, target, remembered_cruise_speed_);
        else
            std::snprintf(b, sizeof(b), "EcoDrive: COAST %d -> %d (2 gears down)", current_gear, target);
        logger(b);
    }
}

int CoastingController::calculate_speed_matched_gear(float speed, int takeoff_gear, int remembered, int effective_max_gear, const PowertrainContext& context) const
{
    const float abs_speed = std::fabs(speed);
    if (abs_speed <= STANDSTILL_SPEED_THRESHOLD)
        return takeoff_gear;

    const int rolling_minimum = rolling_recovery_min_gear(abs_speed, takeoff_gear, effective_max_gear);

    if (context.has_exact_ratios())
    {
        int best_gear = rolling_minimum;
        float best_diff = 99999.0f;
        constexpr float TARGET_REENGAGE_RPM = 1250.0f;

        for (int g = effective_max_gear; g >= rolling_minimum; --g)
        {
            const float landing_rpm = context.calculate_engine_rpm(g, abs_speed);
            if (landing_rpm >= 1050.0f && landing_rpm <= 1700.0f)
            {
                const float diff = std::fabs(landing_rpm - TARGET_REENGAGE_RPM);
                if (diff < best_diff)
                {
                    best_diff = diff;
                    best_gear = g;
                }
            }
        }

        if (best_diff < 90000.0f)
            return std::clamp(best_gear, rolling_minimum, effective_max_gear);
    }

    const float max_speed_reference = 25.0f;
    const float speed_per_gear = max_speed_reference / static_cast<float>(std::max(1, effective_max_gear));
    int speed_gear = static_cast<int>(std::round(abs_speed / speed_per_gear));
    speed_gear = std::clamp(speed_gear, rolling_minimum, effective_max_gear);

    if (remembered >= takeoff_gear && remembered <= effective_max_gear && coasting_start_speed_ > STANDSTILL_SPEED_THRESHOLD)
    {
        const float speed_ratio = std::clamp(abs_speed / coasting_start_speed_, 0.20f, 1.05f);
        const int scaled_gear = std::clamp(static_cast<int>(std::round(remembered * speed_ratio)), rolling_minimum, remembered);
        speed_gear = std::max(speed_gear, scaled_gear);
    }

    return std::clamp(speed_gear, rolling_minimum, effective_max_gear);
}

void CoastingController::start_restore_if_needed(InputDevice& input, int current_gear, float speed, float driver_throttle, bool cruise_active, bool driver_wants_drive, bool shift_in_progress, int minimum_gear, int effective_max_gear, const Config& config, const std::function<void(const char*)>& logger, const PowertrainContext& context, bool force_restore)
{
    if (shift_in_progress || restore_in_progress_ || current_gear != 0)
        return;

    const float abs_speed = std::fabs(speed);
    const bool has_cruise_memory = remembered_cruise_speed() > 0.10f;
    const bool cruise_restore_needed = has_cruise_memory && abs_speed <= remembered_cruise_speed() - 0.05f;
    const bool standstill_takeoff_request = abs_speed <= STANDSTILL_SPEED_THRESHOLD && (driver_throttle > THROTTLE_RELEASE_THRESHOLD || cruise_active);
    const bool low_speed_safety_restore = abs_speed <= COASTING_RESTORE_SPEED;
    const bool driver_restore_needed = driver_wants_drive && (driver_throttle >= config.restore_throttle || restore_throttle_updates_ >= RESTORE_THROTTLE_DEBOUNCE_UPDATES);

    if (!force_restore && !cruise_restore_needed && !driver_restore_needed && !standstill_takeoff_request && !low_speed_safety_restore)
        return;

    const int safe_max = std::max(1, effective_max_gear);
    const int takeoff = std::clamp(config.takeoff_gear, 1, safe_max);
    const int safe_minimum = (minimum_gear >= 1 && minimum_gear <= safe_max) ? minimum_gear : 1;
    const int remembered = (remembered_gear_ >= 1 && remembered_gear_ <= safe_max) ? remembered_gear_ : takeoff;

    if (has_cruise_memory)
        restore_target_gear_ = takeoff;
    else
        restore_target_gear_ = (abs_speed <= STANDSTILL_SPEED_THRESHOLD) ? takeoff : calculate_speed_matched_gear(speed, takeoff, remembered, safe_max, context);

    restore_target_gear_ = std::clamp(std::max(safe_minimum, restore_target_gear_), takeoff, safe_max);
    post_neutral_recovery_ = false;
    restore_in_progress_ = true;
    neutral_in_progress_ = false;
    restore_wait_updates_ = 0;
    restore_throttle_updates_ = 0;

    if (config.shift_logging && logger)
    {
        char b[280];
        std::snprintf(b, sizeof(b), "EcoDrive: legacy neutral -> FAST restoring gear %d (remembered %d, speed %.2f m/s%s%s)", restore_target_gear_, remembered, abs_speed, cruise_restore_needed ? ", cruise speed threshold reached" : "", force_restore ? ", forced safety restore" : "");
        logger(b);
    }
}

void CoastingController::update_neutral_logic(InputDevice& input, int current_gear, float speed, float speed_delta, float driver_throttle, bool cruise_active, bool driver_wants_drive, int minimum_gear, int effective_max_gear, const Config& config, const std::function<void(const char*)>& logger, bool& shift_in_progress, int& requested_gear, ShiftPurpose& shift_purpose, unsigned& shift_wait_updates, const PowertrainContext& context)
{
    (void)speed_delta;
    (void)minimum_gear;
    (void)context;

    if (neutral_in_progress_ && current_gear >= 1)
    {
        const bool driver_accelerates = driver_throttle >= COAST_EXIT_THROTTLE || restore_throttle_updates_ >= RESTORE_THROTTLE_DEBOUNCE_UPDATES;
        const bool cruise_needs_acceleration = cruise_active && remembered_cruise_speed() > 0.10f && speed < remembered_cruise_speed() - 0.05f;

        if (driver_accelerates || cruise_needs_acceleration || std::fabs(speed) <= COASTING_RESTORE_SPEED)
        {
            const bool active_restore = driver_accelerates || cruise_needs_acceleration;
            const int safe_max = std::max(1, effective_max_gear);
            const int remembered_target = std::clamp(remembered_gear_, 1, safe_max);

            neutral_in_progress_ = false;
            restore_throttle_updates_ = 0;
            effective_throttle_zero_updates_ = 0;

            if (active_restore && remembered_target > current_gear)
            {
                // Normal coasting is exactly two gears down, so exiting coast
                // should return to the gear that was active when coasting began.
                // Keep this separate from legacy neutral restoration: the truck
                // is still in a forward gear here, so the generic neutral-only
                // restore path cannot handle this transition.
                restore_target_gear_ = remembered_target;
                restore_in_progress_ = true;
                restore_wait_updates_ = 0;
                post_neutral_recovery_ = false;

                if (config.shift_logging && logger)
                {
                    char b[220];
                    std::snprintf(b, sizeof(b), "EcoDrive: COAST exit -> restoring remembered gear %d -> %d", current_gear, remembered_target);
                    logger(b);
                }
            }
            else
            {
                post_neutral_recovery_ = true;

                if (config.shift_logging && logger)
                    logger(driver_accelerates || cruise_needs_acceleration ? "EcoDrive: COAST exit -> normal automatic shifting." : "EcoDrive: COAST exit at low speed -> normal automatic shifting.");
            }
            return;
        }

        const int target_gear = std::max(1, remembered_gear_ - COASTING_GEAR_DROP);
        if (!shift_in_progress && current_gear > target_gear)
        {
            const int next_gear = current_gear - 1;
            if (input.request_command(InputDevice::Command::gear_down))
            {
                shift_in_progress = true;
                requested_gear = next_gear;
                shift_purpose = ShiftPurpose::neutralize;
                shift_wait_updates = 0;

                if (config.shift_logging && logger)
                {
                    char b[180];
                    std::snprintf(b, sizeof(b), "EcoDrive: COAST step %d -> %d | final target %d", current_gear, next_gear, target_gear);
                    logger(b);
                }
            }
        }
        return;
    }

    if (current_gear <= 0)
    {
        neutral_in_progress_ = false;
        restore_throttle_updates_ = 0;
        return;
    }

    if (shift_in_progress)
        return;
    if (driver_wants_drive && !cruise_active)
        return;
    if (current_gear < COASTING_GEAR_DROP + 1)
        return;

    const int target_gear = current_gear - COASTING_GEAR_DROP;

    if (input.request_command(InputDevice::Command::gear_down))
    {
        shift_in_progress = true;
        requested_gear = current_gear - 1;
        shift_purpose = ShiftPurpose::neutralize;
        shift_wait_updates = 0;

        if (config.shift_logging && logger)
        {
            char b[180];
            std::snprintf(b, sizeof(b), "EcoDrive: COAST step %d -> %d | final target %d", current_gear, requested_gear, target_gear);
            logger(b);
        }
    }
}

void CoastingController::update_restore_logic(InputDevice& input, int current_gear, bool driver_wants_drive, int effective_max_gear, const Config& config, const std::function<void(const char*)>& logger, bool& shift_in_progress, int& requested_gear, ShiftPurpose& shift_purpose, unsigned& shift_wait_updates)
{
    if (!restore_in_progress_)
        return;

    if (current_gear >= 1 && (restore_target_gear_ <= 0 || current_gear >= restore_target_gear_))
    {
        restore_in_progress_ = false;
        restore_target_gear_ = 0;
        restore_wait_updates_ = 0;
        remembered_gear_ = current_gear;
        post_neutral_recovery_ = true;

        if (remembered_cruise_speed() > 0.10f)
        {
            input.request_command(InputDevice::Command::cruise_resume);
            if (config.shift_logging && logger) logger("EcoDrive: gear restoration complete -> queued cruise resume.");
        }
        else if (config.shift_logging && logger)
            logger("EcoDrive: gear restoration complete.");
        return;
    }

    (void)driver_wants_drive;

    if (restore_target_gear_ < 1 || restore_target_gear_ > effective_max_gear)
    {
        restore_in_progress_ = false;
        restore_target_gear_ = 0;
        restore_wait_updates_ = 0;
        return;
    }

    if (shift_in_progress)
        return;

    if (restore_wait_updates_ < config.restore_wait_updates)
    {
        ++restore_wait_updates_;
        return;
    }

    if (current_gear >= effective_max_gear)
        return;

    const unsigned remaining = static_cast<unsigned>(restore_target_gear_ - std::max(0, current_gear));
    const unsigned burst = std::min<unsigned>(remaining, MAX_RECOVERY_BURST);

    if (burst > 0)
    {
        input.request_gear_up_burst(burst);
        shift_in_progress = true;
        requested_gear = current_gear + static_cast<int>(burst);
        shift_purpose = ShiftPurpose::restore;
        shift_wait_updates = 0;
        restore_wait_updates_ = config.restore_wait_updates;

        if (config.shift_logging && logger)
        {
            char b[220];
            std::snprintf(b, sizeof(b), "EcoDrive: FAST RESTORE burst %d -> %d | target %d | %u gear-up pulses", current_gear, requested_gear, restore_target_gear_, burst);
            logger(b);
        }
    }
}

void CoastingController::on_shift_confirmed(InputDevice& input, ShiftPurpose purpose, int confirmed_gear, int effective_max_gear, const Config& config, const std::function<void(const char*)>& logger)
{
    if (purpose == ShiftPurpose::neutralize)
    {
        restore_in_progress_ = false;
        restore_target_gear_ = 0;

        if (confirmed_gear >= 1 && confirmed_gear <= effective_max_gear)
        {
            neutral_in_progress_ = true;
            post_neutral_recovery_ = false;
            input.cancel_burst();
            input.set_clutch_hold(false);

            if (config.shift_logging && logger)
            {
                char b[180];
                std::snprintf(b, sizeof(b), "EcoDrive: COAST confirmed at gear %d (start gear %d, final target %d)", confirmed_gear, remembered_gear_, std::max(1, remembered_gear_ - COASTING_GEAR_DROP));
                logger(b);
            }
        }
        else if (confirmed_gear == 0)
        {
            neutral_in_progress_ = true;
            if (config.shift_logging && logger) logger("EcoDrive: external neutral confirmed while coasting.");
        }
        else
        {
            neutral_in_progress_ = false;
        }
    }
    else if (purpose == ShiftPurpose::restore && confirmed_gear >= 1 && confirmed_gear <= effective_max_gear)
    {
        remembered_gear_ = confirmed_gear;
        if (confirmed_gear >= restore_target_gear_)
        {
            restore_in_progress_ = false;
            restore_target_gear_ = 0;
            restore_wait_updates_ = 0;
            post_neutral_recovery_ = true;

            if (remembered_cruise_speed() > 0.10f)
            {
                input.request_command(InputDevice::Command::cruise_resume);
                if (config.shift_logging && logger) logger("EcoDrive: gear restoration complete -> queued cruise resume.");
            }
            else if (config.shift_logging && logger)
                logger("EcoDrive: gear restoration complete.");
        }
    }
}

void CoastingController::on_manual_override(InputDevice& input, int confirmed_gear, int effective_max_gear, const Config& config, const std::function<void(const char*)>& logger)
{
    restore_in_progress_ = false;
    restore_target_gear_ = 0;
    restore_wait_updates_ = 0;
    neutral_in_progress_ = false;
    post_neutral_recovery_ = false;
    remembered_cruise_speed_ = 0.0f;
    input.cancel_burst();
    input.set_clutch_hold(false);

    if (confirmed_gear >= 1 && confirmed_gear <= effective_max_gear)
    {
        remembered_gear_ = confirmed_gear;
        if (config.shift_logging && logger)
        {
            char b[180];
            std::snprintf(b, sizeof(b), "EcoDrive: manual override remembered gear %d", remembered_gear_);
            logger(b);
        }
    }
}

void CoastingController::reset()
{
    effective_throttle_zero_updates_ = 0;
    restore_throttle_updates_ = 0;
    restore_wait_updates_ = 0;
    restore_target_gear_ = 0;
    remembered_gear_ = 0;
    coasting_start_speed_ = 0.0f;
    remembered_cruise_speed_ = 0.0f;
    neutral_in_progress_ = false;
    restore_in_progress_ = false;
    post_neutral_recovery_ = false;
    low_speed_updates_ = 0;
}
}