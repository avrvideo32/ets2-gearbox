#include "shifter.h"
#include "coasting_controller.h"
#include "adaptive_learner.h"
#include "trip_tracker.h"
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace ecodrive
{
namespace
{
constexpr unsigned MAX_SHIFT_WAIT_GEAR_UPDATES = 12;
constexpr float THROTTLE_RELEASE_THRESHOLD = 0.02f;
constexpr unsigned POST_SHIFT_DRIVE_GRACE_UPDATES = 25;
constexpr unsigned MULTI_UPSHIFT_WAIT_UPDATES = 6;
constexpr unsigned MULTI_DOWNSHIFT_WAIT_UPDATES = 6;
constexpr float MULTI_UPSHIFT_MIN_THROTTLE = 0.35f;
constexpr float MULTI_UPSHIFT_STRONG_THROTTLE = 0.75f;
constexpr float MULTI_UPSHIFT_MIN_SPEED_GAIN = 0.01f;
constexpr unsigned LOAD_DOWNSHIFT_COOLDOWN_UPDATES = 30;
constexpr unsigned LOAD_DOWNSHIFT_UPSHIFT_BLOCK_UPDATES = 90;
constexpr float MANUAL_BRAKE_MIN_SPEED = 4.0f;
constexpr unsigned MANUAL_BRAKE_STABLE_UPDATES_REQUIRED = 3;
constexpr unsigned MANUAL_BRAKE_DOWNSHIFT_COOLDOWN_UPDATES = 25;
constexpr unsigned CRUISE_ECONOMY_COOLDOWN_UPDATES = 25;
constexpr float CRUISE_SPEED_TOLERANCE = 0.015f;
constexpr float CRUISE_TARGET_SPEED_TOLERANCE = 0.35f;
constexpr float CRUISE_SPEED_LOSS_TO_REJECT = 0.15f;
constexpr float CRUISE_ECONOMY_RECOVERY_RPM = 1300.0f;
constexpr float NEUTRAL_RECOVERY_UPSHIFT_RPM = 1800.0f;
constexpr float NEUTRAL_RECOVERY_EXIT_RPM = 1600.0f;
constexpr float NEUTRAL_RECOVERY_MIN_LANDING_RPM = 950.0f;
constexpr float NEUTRAL_RECOVERY_MAX_LANDING_RPM = 1350.0f;
constexpr float LOAD_DOWNSHIFT_THROTTLE_THRESHOLD = 0.35f;
constexpr unsigned LOAD_DOWNSHIFT_STABLE_UPDATES_REQUIRED = 12;
constexpr float LOAD_DOWNSHIFT_SPEED_CHANGE_TOLERANCE = 0.02f;
constexpr float HILLCLIMB_MIN_SPEED_LOSS = 0.18f;
constexpr float HILLCLIMB_CLEAR_SPEED_GAIN = 0.06f;
constexpr unsigned HILLCLIMB_STABLE_UPDATES_REQUIRED = 6;
constexpr unsigned ADAPTIVE_FAILED_GEAR_BLOCK_UPDATES = 180;
constexpr unsigned CRUISE_FAILED_GEAR_BLOCK_UPDATES = 240;
constexpr unsigned CRUISE_ECONOMY_FAILED_GEAR_BLOCK_UPDATES = 1200;
constexpr float ADAPTIVE_RETRY_SPEED_GAIN = 0.10f;
constexpr float CRUISE_ACTIVE_SPEED_THRESHOLD = 0.10f;
}

bool Shifter::is_brake_active() const
{
    return brake_active_updates_ >= MANUAL_BRAKE_STABLE_UPDATES_REQUIRED;
}

void Shifter::on_rpm_tick()
{
    if (post_shift_drive_grace_updates_ > 0) --post_shift_drive_grace_updates_;
    if (load_downshift_cooldown_updates_ > 0) --load_downshift_cooldown_updates_;
    if (load_downshift_upshift_block_updates_ > 0) --load_downshift_upshift_block_updates_;
    for (std::size_t i = 0; i < gear_upshift_block_updates_.size(); ++i)
        if (gear_upshift_block_updates_[i] > 0) --gear_upshift_block_updates_[i];
    if (cruise_block_updates_ > 0)
    {
        --cruise_block_updates_;
        if (cruise_block_updates_ == 0)
        {
            cruise_blocked_upshift_gear_ = 0;
            cruise_blocked_target_speed_ = 0.0f;
        }
    }
}

void Shifter::on_speed_sample(float new_speed, float delta, float throttle, float current_cruise, int current_gear, float current_rpm, const Config& config, AdaptiveLearner& learner, const std::function<void(const char*)>& logger)
{
    if (!shift_in_progress_ && throttle >= LOAD_DOWNSHIFT_THROTTLE_THRESHOLD && delta <= LOAD_DOWNSHIFT_SPEED_CHANGE_TOLERANCE) ++load_downshift_stable_updates_;
    else load_downshift_stable_updates_ = 0;

    if (!shift_in_progress_ && config.hillclimb_downshift_enabled && throttle >= config.hillclimb_throttle_threshold)
    {
        if (hillclimb_downshift_stable_updates_ == 0) hillclimb_reference_speed_ = new_speed;
        if (delta < 0.0f) ++hillclimb_downshift_stable_updates_;
        else if (delta > HILLCLIMB_CLEAR_SPEED_GAIN) { hillclimb_downshift_stable_updates_ = 0; hillclimb_reference_speed_ = new_speed; }
    }
    else { hillclimb_downshift_stable_updates_ = 0; hillclimb_reference_speed_ = new_speed; }

    if (cruise_economy_cooldown_updates_ > 0)
    {
        --cruise_economy_cooldown_updates_;
        if (cruise_shift_start_speed_ > 0.0f && cruise_shift_from_gear_ > 0 && current_gear == cruise_shift_from_gear_ + 1 && new_speed < cruise_shift_start_speed_ - CRUISE_SPEED_LOSS_TO_REJECT)
        {
            cruise_blocked_upshift_gear_ = cruise_shift_from_gear_ + 1;
            cruise_blocked_target_speed_ = current_cruise;
            cruise_block_updates_ = CRUISE_FAILED_GEAR_BLOCK_UPDATES;
            cruise_shift_start_speed_ = 0.0f;
            if (config.adaptive_learning_enabled && learner.is_observation_pending() && learner.observation_target_gear() == current_gear)
                learner.finish_observation(new_speed, current_rpm, true, config.shift_logging, logger);
            if (config.shift_logging && logger)
            {
                char b[220];
                std::snprintf(b, sizeof(b), "EcoDrive: cruise economy rejected gear %d -> speed fell to %.2f", cruise_blocked_upshift_gear_, new_speed);
                logger(b);
            }
        }
    }

    if (config.adaptive_learning_enabled)
        learner.on_speed_update(new_speed, current_rpm, current_gear, config.shift_logging, logger);

    if (cruise_blocked_upshift_gear_ > 0 && current_cruise > CRUISE_ACTIVE_SPEED_THRESHOLD && cruise_blocked_target_speed_ > CRUISE_ACTIVE_SPEED_THRESHOLD && std::fabs(current_cruise - cruise_blocked_target_speed_) > 0.50f)
    {
        if (config.shift_logging && logger)
        {
            char b[220];
            std::snprintf(b, sizeof(b), "EcoDrive: cruise economy block cleared after target change %.2f -> %.2f", cruise_blocked_target_speed_, current_cruise);
            logger(b);
        }
        cruise_blocked_upshift_gear_ = 0;
        cruise_blocked_target_speed_ = 0.0f;
        cruise_block_updates_ = 0;
    }
}

void Shifter::on_brake_sample(float brake, const Config& config)
{
    last_brake_value_ = brake;
    if (brake >= config.brake_downshift_threshold)
    {
        if (brake_active_updates_ < MANUAL_BRAKE_STABLE_UPDATES_REQUIRED) ++brake_active_updates_;
    }
    else { brake_active_updates_ = 0; brake_downshift_triggered_ = false; }
}

void Shifter::on_cruise_changed(float old_cruise, float new_cruise, const Config& config, const std::function<void(const char*)>& logger)
{
    const bool was_active = old_cruise > CRUISE_ACTIVE_SPEED_THRESHOLD;
    const bool now_active = new_cruise > CRUISE_ACTIVE_SPEED_THRESHOLD;
    if (was_active != now_active)
    {
        cruise_economy_cooldown_updates_ = 0;
        if (now_active)
        {
            if (cruise_blocked_upshift_gear_ > 0 && cruise_blocked_target_speed_ > CRUISE_ACTIVE_SPEED_THRESHOLD && std::fabs(new_cruise - cruise_blocked_target_speed_) > 0.50f)
            {
                cruise_blocked_upshift_gear_ = 0;
                cruise_blocked_target_speed_ = 0.0f;
                cruise_block_updates_ = 0;
            }
            if (config.shift_logging && logger) logger("EcoDrive: cruise control active.");
        }
    }
}

void Shifter::update_automatic_shifter(InputDevice& input, int g, float r, float s, float speed_delta, float t, float clutch, bool cruise, float cruise_target, bool throttle_active, int minimum_gear, bool minimum_gear_locked, int effective_max_gear, const Config& config, CoastingController& coasting, AdaptiveLearner& learner, const std::function<void(const char*)>& logger, const PowertrainContext& context)
{
    const bool effective_power_request = t > THROTTLE_RELEASE_THRESHOLD;
    const bool manual_brake_active = is_brake_active();

    if (g == 0 && effective_power_request)
    {
        if (!shift_in_progress_ && !coasting.is_neutral_in_progress() && !coasting.is_restore_in_progress())
            coasting.start_restore_if_needed(input, g, s, t, cruise, true, shift_in_progress_, minimum_gear, effective_max_gear, config, logger, context);
        return;
    }

    const bool is_coasting = !throttle_active && !effective_power_request && !manual_brake_active;
    if (g < 1 || g > effective_max_gear || shift_in_progress_ || coasting.is_neutral_in_progress()) return;

    if (minimum_gear_locked && minimum_gear >= 1 && minimum_gear <= effective_max_gear && g < minimum_gear && !is_coasting)
    {
        if (input.request_command(InputDevice::Command::gear_up))
        {
            shift_in_progress_ = true; requested_gear_ = g + 1; shift_purpose_ = ShiftPurpose::automatic; shift_wait_updates_ = 0;
            if (config.shift_logging && logger)
            {
                char b[180];
                std::snprintf(b, sizeof(b), "EcoDrive: MINIMUM request %d -> %d | minimum %d", g, g + 1, minimum_gear);
                logger(b);
            }
        }
        return;
    }

    if (clutch > 0.10f) return;

    if (multi_upshift_target_gear_ > g && g >= 1 && g < effective_max_gear && effective_power_request && t >= MULTI_UPSHIFT_MIN_THROTTLE && speed_delta >= MULTI_UPSHIFT_MIN_SPEED_GAIN && post_shift_drive_grace_updates_ == 0 && !manual_brake_active)
    {
        if (multi_upshift_wait_updates_ > 0) { --multi_upshift_wait_updates_; return; }
        if (input.request_command(InputDevice::Command::gear_up))
        {
            shift_in_progress_ = true; requested_gear_ = g + 1; shift_purpose_ = ShiftPurpose::automatic; shift_wait_updates_ = 0;
            if (config.shift_logging && logger)
            {
                char b[220];
                std::snprintf(b, sizeof(b), "EcoDrive: MULTI-UP step %d -> %d | target %d | RPM %.0f | throttle %.2f | dSpeed %.3f", g, g + 1, multi_upshift_target_gear_, r, t, speed_delta);
                logger(b);
            }
        }
        return;
    }

    if (multi_upshift_target_gear_ > g && (!effective_power_request || t < MULTI_UPSHIFT_MIN_THROTTLE || speed_delta < -CRUISE_SPEED_TOLERANCE || manual_brake_active))
    {
        multi_upshift_target_gear_ = 0;
        multi_upshift_wait_updates_ = 0;
    }

    if (multi_downshift_target_gear_ > 0 && s < 1.0f) { multi_downshift_target_gear_ = 0; multi_downshift_wait_updates_ = 0; }
    if (multi_downshift_target_gear_ > 0 && g > multi_downshift_target_gear_ && g > 1 && post_shift_drive_grace_updates_ == 0 && !effective_power_request)
    {
        if (multi_downshift_wait_updates_ > 0) { --multi_downshift_wait_updates_; return; }
        if (input.request_command(InputDevice::Command::gear_down))
        {
            shift_in_progress_ = true; requested_gear_ = g - 1; shift_purpose_ = ShiftPurpose::automatic; shift_wait_updates_ = 0;
            if (config.shift_logging && logger)
            {
                char b[220];
                std::snprintf(b, sizeof(b), "EcoDrive: BLOCK-DOWN step %d -> %d | target %d | RPM %.0f | Speed %.2f", g, g - 1, multi_downshift_target_gear_, r, s);
                logger(b);
            }
        }
        return;
    }
    if (multi_downshift_target_gear_ > 0 && (g <= multi_downshift_target_gear_ || effective_power_request)) { multi_downshift_target_gear_ = 0; multi_downshift_wait_updates_ = 0; }

    // Neutral recovery is a special transition. Gear 1 can have a very high
    // engine RPM at road speed, so never use the current RPM as the sole reason
    // to climb 1->2->3. Calculate the landing RPM in each candidate gear and
    // select the highest gear that lands inside the economy/recovery band.
    if (coasting.is_post_neutral_recovery())
    {
        if (post_shift_drive_grace_updates_ > 0) return;

        const bool cruise_recovery = cruise && coasting.remembered_cruise_speed() > CRUISE_ACTIVE_SPEED_THRESHOLD;
        if (cruise_recovery && context.has_exact_ratios() && g < effective_max_gear && !manual_brake_active)
        {
            int target_gear = g;
            float target_rpm = r;
            for (int candidate = effective_max_gear; candidate > g; --candidate)
            {
                const float landing = context.calculate_engine_rpm(candidate, s);
                if (landing >= NEUTRAL_RECOVERY_MIN_LANDING_RPM && landing <= NEUTRAL_RECOVERY_MAX_LANDING_RPM)
                {
                    target_gear = candidate;
                    target_rpm = landing;
                    break;
                }
            }

            if (target_gear > g)
            {
                const unsigned burst = static_cast<unsigned>(target_gear - g);
                if (input.request_gear_up_burst(std::min<unsigned>(burst, 16u)))
                {
                    shift_in_progress_ = true;
                    requested_gear_ = target_gear;
                    shift_purpose_ = ShiftPurpose::automatic;
                    shift_wait_updates_ = 0;
                    multi_upshift_target_gear_ = 0;
                    multi_upshift_wait_updates_ = 0;
                    if (config.shift_logging && logger)
                    {
                        char b[300];
                        std::snprintf(b, sizeof(b), "EcoDrive: CRUISE RECOVERY speed-match %d -> %d | speed %.2f | current RPM %.0f | landing RPM %.0f | target band %.0f-%.0f", g, target_gear, s, r, target_rpm, NEUTRAL_RECOVERY_MIN_LANDING_RPM, NEUTRAL_RECOVERY_MAX_LANDING_RPM);
                        logger(b);
                    }
                }
                return;
            }
        }

        if (r <= NEUTRAL_RECOVERY_EXIT_RPM)
        {
            coasting.set_post_neutral_recovery(false);
            if (config.shift_logging && logger) logger("EcoDrive: neutral recovery complete -> normal shift logic.");
        }
        else if (g < effective_max_gear && r >= NEUTRAL_RECOVERY_UPSHIFT_RPM && !manual_brake_active)
        {
            if (input.request_command(InputDevice::Command::gear_up))
            {
                shift_in_progress_ = true; requested_gear_ = g + 1; shift_purpose_ = ShiftPurpose::automatic; shift_wait_updates_ = 0;
                if (config.multi_upshift_enabled && g <= 4 && t >= MULTI_UPSHIFT_MIN_THROTTLE)
                {
                    int skip = (g == 1 && t >= MULTI_UPSHIFT_STRONG_THROTTLE) ? 3 : 2;
                    multi_upshift_target_gear_ = std::min(g + skip, std::min(config.multi_upshift_max_gear, effective_max_gear));
                    multi_upshift_wait_updates_ = MULTI_UPSHIFT_WAIT_UPDATES;
                }
                if (config.shift_logging && logger)
                {
                    char b[260];
                    std::snprintf(b, sizeof(b), "EcoDrive: NEUTRAL RECOVERY UP request %d -> %d | RPM %.0f | threshold %.0f%s", g, g + 1, r, NEUTRAL_RECOVERY_UPSHIFT_RPM, (multi_upshift_target_gear_ > g) ? " | multi-up target" : "");
                    logger(b);
                }
            }
            return;
        }
    }

    const bool manual_braking = config.brake_downshift_enabled && g > 2 && s >= MANUAL_BRAKE_MIN_SPEED && manual_brake_active && !brake_downshift_triggered_;
    if (manual_braking)
    {
        if (input.request_command(InputDevice::Command::gear_down))
        {
            shift_in_progress_ = true; requested_gear_ = g - 1;
            multi_upshift_target_gear_ = 0; multi_upshift_wait_updates_ = 0;
            shift_purpose_ = ShiftPurpose::automatic; shift_wait_updates_ = 0; brake_downshift_triggered_ = true;
            load_downshift_stable_updates_ = 0; hillclimb_downshift_stable_updates_ = 0; load_downshift_cooldown_updates_ = MANUAL_BRAKE_DOWNSHIFT_COOLDOWN_UPDATES; load_downshift_upshift_block_updates_ = LOAD_DOWNSHIFT_UPSHIFT_BLOCK_UPDATES;
            if (config.shift_logging && logger)
            {
                char b[220];
                std::snprintf(b, sizeof(b), "EcoDrive: BRAKE DOWN request %d -> %d | RPM %.0f | Speed %.2f | Brake %.2f | dSpeed %.3f", g, g - 1, r, s, last_brake_value_, speed_delta);
                logger(b);
            }
        }
        return;
    }

    const bool auxiliary_braking_active = config.retarder_downshift_enabled && (context.motor_brake_active || context.retarder_level > 0);
    if (auxiliary_braking_active && g > 2 && s >= 6.0f && !effective_power_request && r < 1550.0f && load_downshift_cooldown_updates_ == 0 && post_shift_drive_grace_updates_ == 0)
    {
        float target_landing_rpm = 0.0f; bool safe_to_downshift = true;
        if (context.has_exact_ratios()) { target_landing_rpm = context.calculate_engine_rpm(g - 1, s); if (target_landing_rpm > 2150.0f || target_landing_rpm <= r) safe_to_downshift = false; }
        else if (r > 1400.0f) safe_to_downshift = false;
        if (safe_to_downshift && input.request_command(InputDevice::Command::gear_down))
        {
            shift_in_progress_ = true; requested_gear_ = g - 1; multi_upshift_target_gear_ = 0; multi_upshift_wait_updates_ = 0; shift_purpose_ = ShiftPurpose::retarder_downshift; shift_wait_updates_ = 0; load_downshift_stable_updates_ = 0; hillclimb_downshift_stable_updates_ = 0; load_downshift_cooldown_updates_ = MANUAL_BRAKE_DOWNSHIFT_COOLDOWN_UPDATES; load_downshift_upshift_block_updates_ = LOAD_DOWNSHIFT_UPSHIFT_BLOCK_UPDATES;
            if (config.shift_logging && logger)
            {
                char b[260];
                std::snprintf(b, sizeof(b), "EcoDrive: RETARDER DOWN request %d -> %d | RPM %.0f | Speed %.2f | landing %.0f", g, g - 1, r, s, target_landing_rpm);
                logger(b);
            }
        }
        return;
    }

    const float grade_rpm_bias = std::clamp(std::fabs(context.grade_percent) * 8.0f, 0.0f, 220.0f);
    const float cargo_rpm_bias = std::clamp(context.cargo_mass_kg / 1000.0f * 2.0f, 0.0f, 140.0f);
    const bool hillclimb_downshift = config.hillclimb_downshift_enabled && t >= config.hillclimb_throttle_threshold && speed_delta < -HILLCLIMB_MIN_SPEED_LOSS && hillclimb_downshift_stable_updates_ >= HILLCLIMB_STABLE_UPDATES_REQUIRED && g > 1;
    const bool load_downshift = config.load_adaptive_shifting_enabled && g > 1 && t >= LOAD_DOWNSHIFT_THROTTLE_THRESHOLD && load_downshift_stable_updates_ >= LOAD_DOWNSHIFT_STABLE_UPDATES_REQUIRED && speed_delta < -LOAD_DOWNSHIFT_SPEED_CHANGE_TOLERANCE && r <= config.load_downshift_rpm;
    if (hillclimb_downshift || load_downshift)
    {
        if (input.request_command(InputDevice::Command::gear_down))
        {
            shift_in_progress_ = true; requested_gear_ = g - 1; shift_purpose_ = hillclimb_downshift ? ShiftPurpose::hillclimb_downshift : ShiftPurpose::load_downshift; shift_wait_updates_ = 0; multi_upshift_target_gear_ = 0; multi_upshift_wait_updates_ = 0; load_downshift_cooldown_updates_ = LOAD_DOWNSHIFT_COOLDOWN_UPDATES; load_downshift_upshift_block_updates_ = LOAD_DOWNSHIFT_UPSHIFT_BLOCK_UPDATES;
            if (config.shift_logging && logger)
            {
                char b[240];
                std::snprintf(b, sizeof(b), "EcoDrive: %s request %d -> %d | RPM %.0f | Speed %.2f | dSpeed %.3f | throttle %.2f | grade %.1f%%", hillclimb_downshift ? "HILL DOWN" : "LOAD DOWN", g, g - 1, r, s, speed_delta, t, context.grade_percent);
                logger(b);
            }
        }
        return;
    }

    const float normal_upshift_rpm = std::clamp(config.upshift_rpm + (grade_rpm_bias * 0.35f) + cargo_rpm_bias, 950.0f, 1550.0f);
    const bool speed_limit_hold = context.speed_limit_mps > 0.1f && s >= context.speed_limit_mps - 0.20f && speed_delta <= 0.002f;
    const bool exact_landing_ok = !context.has_exact_ratios() || (g >= effective_max_gear ? true : context.calculate_engine_rpm(g + 1, s) >= 850.0f);
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed_shift_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_automatic_shift_time_).count();
    const bool normal_upshift_allowed = (!cruise || !config.cruise_economy_enabled) && !is_coasting;
    if (normal_upshift_allowed && g < effective_max_gear && load_downshift_upshift_block_updates_ == 0 && elapsed_shift_ms >= static_cast<long long>(config.automatic_shift_cooldown_ms) && r >= normal_upshift_rpm && t >= config.min_upshift_throttle && !manual_brake_active && !speed_limit_hold && exact_landing_ok && speed_delta >= -CRUISE_SPEED_TOLERANCE)
    {
        if (input.request_command(InputDevice::Command::gear_up))
        {
            last_automatic_shift_time_ = now; shift_in_progress_ = true; requested_gear_ = g + 1; shift_purpose_ = ShiftPurpose::automatic; shift_wait_updates_ = 0; load_downshift_stable_updates_ = 0; hillclimb_downshift_stable_updates_ = 0; load_downshift_cooldown_updates_ = LOAD_DOWNSHIFT_COOLDOWN_UPDATES;
            if (config.multi_upshift_enabled && g <= 4 && t >= MULTI_UPSHIFT_MIN_THROTTLE && speed_delta >= MULTI_UPSHIFT_MIN_SPEED_GAIN && (!config.load_adaptive_shifting_enabled || context.cargo_mass_kg <= 25000.0f))
            {
                int skip = 2;
                if (g == 1 && t >= MULTI_UPSHIFT_STRONG_THROTTLE && (!config.load_adaptive_shifting_enabled || context.cargo_mass_kg < 10000.0f)) skip = 3;
                multi_upshift_target_gear_ = std::min(g + skip, std::min(config.multi_upshift_max_gear, effective_max_gear));
                multi_upshift_wait_updates_ = MULTI_UPSHIFT_WAIT_UPDATES;
                if (context.has_exact_ratios() && multi_upshift_target_gear_ > g + 1)
                    while (multi_upshift_target_gear_ > g + 1 && context.calculate_engine_rpm(multi_upshift_target_gear_, s) < 850.0f) --multi_upshift_target_gear_;
            }
            else { multi_upshift_target_gear_ = 0; multi_upshift_wait_updates_ = 0; }
            if (config.shift_logging && logger)
            {
                char b[220];
                std::snprintf(b, sizeof(b), "EcoDrive: UP request %d -> %d | RPM %.0f | cap %.0f%s", g, g + 1, r, normal_upshift_rpm, (multi_upshift_target_gear_ > g) ? " | multi-up target" : "");
                logger(b);
            }
        }
        return;
    }

    const float max_downshift_rpm = normal_upshift_rpm * 0.72f;
    const float effective_downshift_rpm = std::min(config.downshift_rpm + (grade_rpm_bias * 0.5f) + cargo_rpm_bias, max_downshift_rpm);
    const bool can_downshift_timing = (post_shift_drive_grace_updates_ == 0) && (elapsed_shift_ms >= static_cast<long long>(config.automatic_shift_cooldown_ms));
    const bool is_lugging = r <= 820.0f;
    const bool speed_loss_or_idle = speed_delta < 0.001f || t < config.min_upshift_throttle;
    const bool can_downshift = (g > 1 && can_downshift_timing) && (r <= effective_downshift_rpm || is_lugging) && (speed_loss_or_idle || is_lugging || manual_brake_active) && !is_coasting;
    if (can_downshift)
    {
        ++normal_downshift_stable_updates_;
        if (normal_downshift_stable_updates_ >= 4)
        {
            normal_downshift_stable_updates_ = 0;
            if (input.request_command(InputDevice::Command::gear_down))
            {
                last_automatic_shift_time_ = now; shift_in_progress_ = true; requested_gear_ = g - 1; multi_upshift_target_gear_ = 0; multi_upshift_wait_updates_ = 0; multi_downshift_target_gear_ = 0; multi_downshift_wait_updates_ = 0; shift_purpose_ = ShiftPurpose::automatic; shift_wait_updates_ = 0; load_downshift_upshift_block_updates_ = LOAD_DOWNSHIFT_UPSHIFT_BLOCK_UPDATES / 2;
                if (config.shift_logging && logger)
                {
                    char b[240];
                    std::snprintf(b, sizeof(b), "EcoDrive: DOWN request %d -> %d | RPM %.0f | Speed %.2f%s", g, g - 1, r, s, cruise ? " | cruise" : "");
                    logger(b);
                }
            }
            return;
        }
    }
    else normal_downshift_stable_updates_ = 0;

    float adaptive_cruise_rpm_floor = config.cruise_economy_min_rpm;
    if (config.adaptive_learning_enabled) adaptive_cruise_rpm_floor = learner.calculate_cruise_rpm_floor(g, speed_delta, t, config.cruise_economy_min_rpm);
    const unsigned higher_gear_block = (g + 1 < static_cast<int>(gear_upshift_block_updates_.size())) ? gear_upshift_block_updates_[static_cast<std::size_t>(g + 1)] : 0;
    const bool recently_load_downshifted = load_downshift_upshift_block_updates_ > 0 || higher_gear_block > 0;
    const float required_upshift_rpm = recently_load_downshifted ? std::max(CRUISE_ECONOMY_RECOVERY_RPM, adaptive_cruise_rpm_floor) : adaptive_cruise_rpm_floor;
    const bool recovered_from_load_downshift = !recently_load_downshifted && (load_downshift_start_speed_ <= 0.0f || s >= load_downshift_start_speed_ + ADAPTIVE_RETRY_SPEED_GAIN);
    if (load_downshift_start_speed_ > 0.0f && !recently_load_downshifted && s >= load_downshift_start_speed_ + ADAPTIVE_RETRY_SPEED_GAIN) load_downshift_start_speed_ = 0.0f;
    const bool speed_is_not_falling = speed_delta >= -CRUISE_SPEED_TOLERANCE;
    const bool cruise_target_reached = s >= cruise_target - CRUISE_TARGET_SPEED_TOLERANCE;
    const bool cruise_acceleration_upshift = cruise && effective_power_request && speed_delta >= 0.0f && r >= required_upshift_rpm;
    const bool cruise_settled_upshift = cruise && cruise_target_reached && speed_is_not_falling && r >= required_upshift_rpm;
    const bool higher_gear_ok = cruise_blocked_upshift_gear_ != g + 1 && higher_gear_block == 0;
    const bool economy_shift_allowed = config.cruise_economy_enabled && cruise && g < effective_max_gear && !is_coasting && !manual_brake_active && !hillclimb_downshift && !speed_limit_hold && exact_landing_ok && (cruise_acceleration_upshift || cruise_settled_upshift) && cruise_economy_cooldown_updates_ == 0 && higher_gear_ok && recovered_from_load_downshift;
    if (economy_shift_allowed)
    {
        const float speed_now = s;
        if (input.request_command(InputDevice::Command::gear_up))
        {
            last_automatic_shift_time_ = now; shift_in_progress_ = true; requested_gear_ = g + 1; shift_purpose_ = ShiftPurpose::cruise_economy; shift_wait_updates_ = 0; load_downshift_stable_updates_ = 0; load_downshift_cooldown_updates_ = LOAD_DOWNSHIFT_COOLDOWN_UPDATES; cruise_shift_start_speed_ = speed_now; cruise_shift_from_gear_ = g; cruise_shift_throttle_ = t; cruise_shift_speed_delta_ = speed_delta;
            if (config.adaptive_learning_enabled && !learner.is_observation_pending()) learner.start_observation(g, g + 1, speed_now, t, speed_delta);
            if (config.shift_logging && logger)
            {
                char b[240];
                std::snprintf(b, sizeof(b), "EcoDrive: CRUISE ECONOMY UP request %d -> %d | RPM %.0f | floor %.0f | Speed %.2f | Cruise %.2f | throttle %.2f%s", g, g + 1, r, adaptive_cruise_rpm_floor, speed_now, cruise_target, t, cruise_acceleration_upshift ? " | accelerating" : " | settled");
                logger(b);
            }
        }
        return;
    }
}

void Shifter::on_gear_update(InputDevice& input, int g, int previous_gear, int effective_max_gear, const Config& config, CoastingController& coasting, AdaptiveLearner& learner, const std::function<void(const char*)>& logger)
{
    (void)input; (void)previous_gear; (void)effective_max_gear; (void)config; (void)coasting; (void)learner; (void)logger;
}

}