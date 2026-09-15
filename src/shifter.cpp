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

bool Shifter::is_brake_active() const { return brake_active_updates_ >= MANUAL_BRAKE_STABLE_UPDATES_REQUIRED; }

void Shifter::on_rpm_tick()
{
    if (post_shift_drive_grace_updates_ > 0) --post_shift_drive_grace_updates_;
    if (load_downshift_cooldown_updates_ > 0) --load_downshift_cooldown_updates_;
    if (load_downshift_upshift_block_updates_ > 0) --load_downshift_upshift_block_updates_;
    for (std::size_t i = 0; i < gear_upshift_block_updates_.size(); ++i) if (gear_upshift_block_updates_[i] > 0) --gear_upshift_block_updates_[i];
    if (cruise_block_updates_ > 0) { --cruise_block_updates_; if (cruise_block_updates_ == 0) { cruise_blocked_upshift_gear_ = 0; cruise_blocked_target_speed_ = 0.0f; } }
}

void Shifter::on_speed_sample(float new_speed, float delta, float throttle, float current_cruise, int current_gear, float current_rpm, const Config& config, AdaptiveLearner& learner, const std::function<void(const char*)>& logger)
{
    if (!shift_in_progress_ && throttle >= LOAD_DOWNSHIFT_THROTTLE_THRESHOLD && delta <= LOAD_DOWNSHIFT_SPEED_CHANGE_TOLERANCE) ++load_downshift_stable_updates_; else load_downshift_stable_updates_ = 0;
    if (!shift_in_progress_ && config.hillclimb_downshift_enabled && throttle >= config.hillclimb_throttle_threshold) { if (hillclimb_downshift_stable_updates_ == 0) hillclimb_reference_speed_ = new_speed; if (delta < 0.0f) ++hillclimb_downshift_stable_updates_; else if (delta > HILLCLIMB_CLEAR_SPEED_GAIN) { hillclimb_downshift_stable_updates_ = 0; hillclimb_reference_speed_ = new_speed; } }
    else { hillclimb_downshift_stable_updates_ = 0; hillclimb_reference_speed_ = new_speed; }
    if (cruise_economy_cooldown_updates_ > 0) { --cruise_economy_cooldown_updates_; if (cruise_shift_start_speed_ > 0.0f && cruise_shift_from_gear_ > 0 && current_gear == cruise_shift_from_gear_ + 1 && new_speed < cruise_shift_start_speed_ - CRUISE_SPEED_LOSS_TO_REJECT) { cruise_blocked_upshift_gear_ = cruise_shift_from_gear_ + 1; cruise_blocked_target_speed_ = current_cruise; cruise_block_updates_ = CRUISE_FAILED_GEAR_BLOCK_UPDATES; cruise_shift_start_speed_ = 0.0f; if (config.adaptive_learning_enabled && learner.is_observation_pending() && learner.observation_target_gear() == current_gear) learner.finish_observation(new_speed, current_rpm, true, config.shift_logging, logger); if (config.shift_logging && logger) { char b[220]; std::snprintf(b, sizeof(b), "EcoDrive: cruise economy rejected gear %d -> speed fell to %.2f", cruise_blocked_upshift_gear_, new_speed); logger(b); } } }
    if (config.adaptive_learning_enabled) learner.on_speed_update(new_speed, current_rpm, current_gear, config.shift_logging, logger);
    if (cruise_blocked_upshift_gear_ > 0 && current_cruise > CRUISE_ACTIVE_SPEED_THRESHOLD && cruise_blocked_target_speed_ > CRUISE_ACTIVE_SPEED_THRESHOLD && std::fabs(current_cruise - cruise_blocked_target_speed_) > 0.50f) { cruise_blocked_upshift_gear_ = 0; cruise_blocked_target_speed_ = 0.0f; cruise_block_updates_ = 0; if (config.shift_logging && logger) logger("EcoDrive: cruise economy block cleared after target change."); }
}

void Shifter::on_brake_sample(float brake, const Config& config)
{
    last_brake_value_ = brake;
    if (brake >= config.brake_downshift_threshold) { if (brake_active_updates_ < MANUAL_BRAKE_STABLE_UPDATES_REQUIRED) ++brake_active_updates_; } else { brake_active_updates_ = 0; brake_downshift_triggered_ = false; }
}

void Shifter::on_cruise_changed(float old_cruise, float new_cruise, const Config& config, const std::function<void(const char*)>& logger)
{
    const bool was_active = old_cruise > CRUISE_ACTIVE_SPEED_THRESHOLD;
    const bool now_active = new_cruise > CRUISE_ACTIVE_SPEED_THRESHOLD;
    if (was_active != now_active) { cruise_economy_cooldown_updates_ = 0; if (now_active && config.shift_logging && logger) logger("EcoDrive: cruise control active."); }
}

void Shifter::update_automatic_shifter(InputDevice& input, int g, float r, float s, float speed_delta, float t, float clutch, bool cruise, float cruise_target, bool throttle_active, int minimum_gear, bool minimum_gear_locked, int effective_max_gear, const Config& config, CoastingController& coasting, AdaptiveLearner& learner, TripTracker& tracker, const std::function<void(const char*)>& logger, const PowertrainContext& context)
{
    const bool effective_power_request = t > THROTTLE_RELEASE_THRESHOLD;
    const bool manual_brake_active = is_brake_active();
    if (g == 0 && effective_power_request) { if (!shift_in_progress_ && !coasting.is_neutral_in_progress() && !coasting.is_restore_in_progress()) coasting.start_restore_if_needed(input, g, s, t, cruise, true, shift_in_progress_, minimum_gear, effective_max_gear, config, logger, context); return; }
    const bool is_coasting = !throttle_active && !effective_power_request && !manual_brake_active;
    if (g < 1 || g > effective_max_gear || shift_in_progress_ || coasting.is_neutral_in_progress()) return;
    if (minimum_gear_locked && minimum_gear >= 1 && minimum_gear <= effective_max_gear && g < minimum_gear && !is_coasting) { if (input.request_command(InputDevice::Command::gear_up)) { shift_in_progress_ = true; requested_gear_ = g + 1; shift_purpose_ = ShiftPurpose::automatic; shift_wait_updates_ = 0; } return; }
    if (clutch > 0.10f) return;
    if (multi_upshift_target_gear_ > g && effective_power_request && t >= MULTI_UPSHIFT_MIN_THROTTLE && speed_delta >= MULTI_UPSHIFT_MIN_SPEED_GAIN && post_shift_drive_grace_updates_ == 0 && !manual_brake_active) { if (multi_upshift_wait_updates_ > 0) { --multi_upshift_wait_updates_; return; } if (input.request_command(InputDevice::Command::gear_up)) { shift_in_progress_ = true; requested_gear_ = g + 1; shift_purpose_ = ShiftPurpose::automatic; shift_wait_updates_ = 0; } return; }
    if (multi_upshift_target_gear_ > g && (!effective_power_request || t < MULTI_UPSHIFT_MIN_THROTTLE || speed_delta < -CRUISE_SPEED_TOLERANCE || manual_brake_active)) { multi_upshift_target_gear_ = 0; multi_upshift_wait_updates_ = 0; }
    if (coasting.is_post_neutral_recovery())
    {
        if (post_shift_drive_grace_updates_ > 0) return;
        if (cruise && context.has_exact_ratios() && g < effective_max_gear && !manual_brake_active)
        {
            int target = g;
            float landing_target = 0.0f;
            for (int candidate = effective_max_gear; candidate > g; --candidate)
            {
                const float landing = context.calculate_engine_rpm(candidate, s);
                if (landing >= 950.0f && landing <= 1350.0f) { target = candidate; landing_target = landing; break; }
            }
            if (target > g)
            {
                const unsigned burst = static_cast<unsigned>(target - g);
                if (input.request_gear_up_burst(std::min<unsigned>(burst, 16u)))
                {
                    shift_in_progress_ = true; requested_gear_ = target; shift_purpose_ = ShiftPurpose::automatic; shift_wait_updates_ = 0; multi_upshift_target_gear_ = 0; multi_upshift_wait_updates_ = 0;
                    if (config.shift_logging && logger) { char b[260]; std::snprintf(b, sizeof(b), "EcoDrive: CRUISE RECOVERY speed-match %d -> %d | speed %.2f | current RPM %.0f | landing RPM %.0f", g, target, s, r, landing_target); logger(b); }
                }
                return;
            }
        }
        if (r <= NEUTRAL_RECOVERY_EXIT_RPM) { coasting.set_post_neutral_recovery(false); }
        else if (g < effective_max_gear && r >= NEUTRAL_RECOVERY_UPSHIFT_RPM && !manual_brake_active) { if (input.request_command(InputDevice::Command::gear_up)) { shift_in_progress_ = true; requested_gear_ = g + 1; shift_purpose_ = ShiftPurpose::automatic; shift_wait_updates_ = 0; } return; }
    }

    float grade_rpm_bias = 0.0f;
    float grade_throttle_bias = 0.0f;
    if (config.grade_detection_enabled) { if (context.grade_percent > 1.5f) { grade_rpm_bias = std::min(context.grade_percent * 35.0f, 300.0f); grade_throttle_bias = std::min(context.grade_percent * 0.05f, 0.25f); } else if (context.grade_percent < -2.0f) grade_rpm_bias = -80.0f; }
    float cargo_rpm_bias = 0.0f;
    if (config.load_adaptive_shifting_enabled && context.cargo_mass_kg > 0.0f) { if (context.cargo_mass_kg > 25000.0f) cargo_rpm_bias = 120.0f; else if (context.cargo_mass_kg < 5000.0f) cargo_rpm_bias = -60.0f; }
    const float effective_hillclimb_throttle = std::max(0.40f, config.hillclimb_throttle_threshold - grade_throttle_bias);
    const float effective_hillclimb_rpm = config.hillclimb_rpm_threshold + grade_rpm_bias + cargo_rpm_bias;
    const float effective_load_downshift_rpm = config.load_downshift_rpm + grade_rpm_bias * 0.5f + cargo_rpm_bias;
    const bool hillclimb_downshift = config.hillclimb_downshift_enabled && g > 2 && t >= effective_hillclimb_throttle && r <= effective_hillclimb_rpm && hillclimb_downshift_stable_updates_ >= HILLCLIMB_STABLE_UPDATES_REQUIRED && hillclimb_reference_speed_ > 0.0f && hillclimb_reference_speed_ - s >= HILLCLIMB_MIN_SPEED_LOSS;
    const bool low_rpm_load_downshift = g > 2 && t >= LOAD_DOWNSHIFT_THROTTLE_THRESHOLD && r <= effective_load_downshift_rpm && load_downshift_stable_updates_ >= LOAD_DOWNSHIFT_STABLE_UPDATES_REQUIRED && speed_delta <= 0.0f;
    if ((hillclimb_downshift || low_rpm_load_downshift) && load_downshift_cooldown_updates_ == 0) { if (input.request_command(InputDevice::Command::gear_down)) { shift_in_progress_ = true; requested_gear_ = g - 1; multi_upshift_target_gear_ = 0; multi_upshift_wait_updates_ = 0; shift_purpose_ = ShiftPurpose::load_downshift; shift_wait_updates_ = 0; load_downshift_upshift_block_updates_ = LOAD_DOWNSHIFT_UPSHIFT_BLOCK_UPDATES; load_downshift_start_speed_ = s; } return; }
    const float normal_upshift_rpm = std::clamp(config.upshift_rpm + grade_rpm_bias * 0.35f + cargo_rpm_bias, 950.0f, 1550.0f);
    const bool speed_limit_hold = context.speed_limit_mps > 0.1f && s >= context.speed_limit_mps - 0.20f && speed_delta <= 0.002f;
    const bool exact_landing_ok = !context.has_exact_ratios() || g >= effective_max_gear || context.calculate_engine_rpm(g + 1, s) >= 850.0f;
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed_shift_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_automatic_shift_time_).count();
    const bool normal_upshift_allowed = (!cruise || !config.cruise_economy_enabled) && !is_coasting;
    if (normal_upshift_allowed && g < effective_max_gear && load_downshift_upshift_block_updates_ == 0 && elapsed_shift_ms >= static_cast<long long>(config.automatic_shift_cooldown_ms) && r >= normal_upshift_rpm && t >= config.min_upshift_throttle && !manual_brake_active && !speed_limit_hold && exact_landing_ok && speed_delta >= -CRUISE_SPEED_TOLERANCE) { if (input.request_command(InputDevice::Command::gear_up)) { last_automatic_shift_time_ = now; shift_in_progress_ = true; requested_gear_ = g + 1; shift_purpose_ = ShiftPurpose::automatic; shift_wait_updates_ = 0; } return; }
    float adaptive_floor = config.cruise_economy_min_rpm;
    if (config.adaptive_learning_enabled) adaptive_floor = learner.calculate_cruise_rpm_floor(g, speed_delta, t, config.cruise_economy_min_rpm);
    const unsigned higher_block = (g + 1 < static_cast<int>(gear_upshift_block_updates_.size())) ? gear_upshift_block_updates_[static_cast<std::size_t>(g + 1)] : 0;
    const bool economy_allowed = config.cruise_economy_enabled && cruise && g < effective_max_gear && !is_coasting && !manual_brake_active && !speed_limit_hold && exact_landing_ok && r >= adaptive_floor && cruise_economy_cooldown_updates_ == 0 && higher_block == 0;
    if (economy_allowed && input.request_command(InputDevice::Command::gear_up)) { last_automatic_shift_time_ = now; shift_in_progress_ = true; requested_gear_ = g + 1; shift_purpose_ = ShiftPurpose::cruise_economy; shift_wait_updates_ = 0; cruise_shift_start_speed_ = s; cruise_shift_from_gear_ = g; cruise_shift_throttle_ = t; cruise_shift_speed_delta_ = speed_delta; if (config.adaptive_learning_enabled && !learner.is_observation_pending()) learner.start_observation(g, g + 1, s, t, speed_delta); return; }
}

void Shifter::on_gear_update(InputDevice& input, int g, int previous_gear, int effective_max_gear, const Config& config, CoastingController& coasting, AdaptiveLearner& learner, TripTracker& tracker, const std::function<void(const char*)>& logger)
{
    const bool is_restoring = coasting.is_restore_in_progress() || shift_purpose_ == ShiftPurpose::restore;
    if (is_restoring && g > 0 && shift_in_progress_) { requested_gear_ = g; shift_purpose_ = ShiftPurpose::restore; }
    const bool plugin_shift = shift_in_progress_;
    if (plugin_shift && g != requested_gear_ && g != previous_gear)
    {
        if (config.shift_logging && logger) { char b[180]; std::snprintf(b, sizeof(b), "EcoDrive: manual override -> gear %d (requested %d cancelled)", g, requested_gear_); logger(b); }
        shift_in_progress_ = false; requested_gear_ = 0; multi_upshift_target_gear_ = 0; multi_upshift_wait_updates_ = 0; multi_downshift_target_gear_ = 0; multi_downshift_wait_updates_ = 0; shift_purpose_ = ShiftPurpose::none; shift_wait_updates_ = 0; coasting.on_manual_override(input, g, effective_max_gear, config, logger); learner.cancel_observation(); return;
    }
    if (plugin_shift && g == requested_gear_)
    {
        const ShiftPurpose purpose = shift_purpose_;
        shift_in_progress_ = false; requested_gear_ = 0; shift_purpose_ = ShiftPurpose::none; shift_wait_updates_ = 0;
        if (multi_upshift_target_gear_ > 0 && g >= multi_upshift_target_gear_) { multi_upshift_target_gear_ = 0; multi_upshift_wait_updates_ = 0; } else if (multi_upshift_target_gear_ > 0) multi_upshift_wait_updates_ = MULTI_UPSHIFT_WAIT_UPDATES;
        if (multi_downshift_target_gear_ > 0 && g <= multi_downshift_target_gear_) { multi_downshift_target_gear_ = 0; multi_downshift_wait_updates_ = 0; } else if (multi_downshift_target_gear_ > 0) multi_downshift_wait_updates_ = MULTI_DOWNSHIFT_WAIT_UPDATES;
        if (config.shift_logging && logger) { char b[160]; std::snprintf(b, sizeof(b), "EcoDrive: shift confirmed -> gear %d", g); logger(b); }
        post_shift_drive_grace_updates_ = POST_SHIFT_DRIVE_GRACE_UPDATES;
        tracker.on_shift_confirmed(purpose, previous_gear, g);
        if (purpose == ShiftPurpose::cruise_economy) cruise_economy_cooldown_updates_ = CRUISE_ECONOMY_COOLDOWN_UPDATES;
        coasting.on_shift_confirmed(input, purpose, g, effective_max_gear, config, logger);
    }
    else if (plugin_shift && ++shift_wait_updates_ >= MAX_SHIFT_WAIT_GEAR_UPDATES)
    {
        if (config.shift_logging && logger) { char b[180]; std::snprintf(b, sizeof(b), "EcoDrive: shift timeout -> requested %d, actual %d", requested_gear_, g); logger(b); }
        learner.cancel_observation(); shift_in_progress_ = false; requested_gear_ = 0; shift_purpose_ = ShiftPurpose::none; shift_wait_updates_ = 0;
    }
    if (!plugin_shift && g >= 1 && g <= effective_max_gear && g != previous_gear) coasting.set_remembered_gear(g);
}

void Shifter::reset()
{
    shift_in_progress_ = false; requested_gear_ = 0; shift_purpose_ = ShiftPurpose::none; shift_wait_updates_ = 0; last_automatic_shift_time_ = {};
    multi_upshift_target_gear_ = 0; multi_upshift_wait_updates_ = 0; multi_downshift_target_gear_ = 0; multi_downshift_wait_updates_ = 0;
    load_downshift_cooldown_updates_ = 0; load_downshift_upshift_block_updates_ = 0; load_downshift_start_speed_ = 0.0f; gear_upshift_block_updates_.fill(0);
    brake_active_updates_ = 0; brake_downshift_triggered_ = false; last_brake_value_ = 0.0f; load_downshift_stable_updates_ = 0; hillclimb_downshift_stable_updates_ = 0; normal_downshift_stable_updates_ = 0; hillclimb_reference_speed_ = 0.0f;
    cruise_economy_cooldown_updates_ = 0; cruise_blocked_upshift_gear_ = 0; cruise_blocked_target_speed_ = 0.0f; cruise_block_updates_ = 0; cruise_shift_start_speed_ = 0.0f; cruise_shift_from_gear_ = 0; cruise_shift_throttle_ = 0.0f; cruise_shift_speed_delta_ = 0.0f; post_shift_drive_grace_updates_ = 0;
}
}
