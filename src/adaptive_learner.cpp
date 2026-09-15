#include "adaptive_learner.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace ecodrive
{

namespace
{

constexpr unsigned ADAPTIVE_MIN_OBSERVATIONS = 3;
constexpr unsigned ADAPTIVE_OBSERVATION_UPDATES = 120;

constexpr float ADAPTIVE_ACCEPTABLE_SPEED_LOSS = 0.15f;
constexpr float ADAPTIVE_SPEED_LOSS_MARGIN = 0.05f;
constexpr float ADAPTIVE_RPM_STEP = 100.0f;
constexpr float CRUISE_ECONOMY_RECOVERY_RPM = 1300.0f;

constexpr float LEARNING_LOAD_1 = 0.10f;
constexpr float LEARNING_LOAD_2 = 0.35f;
constexpr float LEARNING_LOAD_3 = 0.70f;

constexpr float LEARNING_SPEED_TREND_THRESHOLD = 0.015f;

}

std::size_t AdaptiveLearner::learning_bucket_index(
    int from_gear,
    int to_gear,
    float throttle,
    float speed_delta) const
{
    const int from =
        std::clamp(from_gear, 0, static_cast<int>(LEARNING_GEAR_COUNT - 1));

    const int to =
        std::clamp(to_gear, 0, static_cast<int>(LEARNING_GEAR_COUNT - 1));

    std::size_t load_band = 0;

    if (throttle >= LEARNING_LOAD_3)
        load_band = 3;
    else if (throttle >= LEARNING_LOAD_2)
        load_band = 2;
    else if (throttle >= LEARNING_LOAD_1)
        load_band = 1;

    std::size_t trend_band = 1;

    if (speed_delta > LEARNING_SPEED_TREND_THRESHOLD)
        trend_band = 2;
    else if (speed_delta < -LEARNING_SPEED_TREND_THRESHOLD)
        trend_band = 0;

    const std::size_t pair =
        static_cast<std::size_t>(from) * LEARNING_GEAR_COUNT +
        static_cast<std::size_t>(to);

    const std::size_t index =
        ((pair * LEARNING_LOAD_BANDS) + load_band) *
            LEARNING_SPEED_TREND_BANDS +
        trend_band;

    return std::min(index, LEARNING_BUCKET_COUNT - 1);
}

void AdaptiveLearner::start_observation(
    int from_gear,
    int to_gear,
    float speed,
    float throttle,
    float speed_delta)
{
    learning_observation_pending_ = true;
    learning_observation_updates_ = 0;

    learning_shift_from_gear_ = from_gear;
    learning_shift_to_gear_ = to_gear;

    learning_shift_start_speed_ = speed;
    learning_shift_throttle_ = throttle;
    learning_shift_speed_delta_ = speed_delta;
}

void AdaptiveLearner::cancel_observation()
{
    learning_observation_pending_ = false;
    learning_observation_updates_ = 0;

    learning_shift_from_gear_ = 0;
    learning_shift_to_gear_ = 0;

    learning_shift_start_speed_ = 0.0f;
    learning_shift_throttle_ = 0.0f;
    learning_shift_speed_delta_ = 0.0f;
}

void AdaptiveLearner::on_speed_update(
    float new_speed,
    float current_rpm,
    int current_gear,
    bool logging_enabled,
    const std::function<void(const char*)>& logger)
{
    if (learning_observation_pending_ &&
        current_gear == learning_shift_to_gear_)
    {
        ++learning_observation_updates_;

        if (learning_observation_updates_ >= ADAPTIVE_OBSERVATION_UPDATES)
        {
            finish_observation(
                new_speed,
                current_rpm,
                false,
                logging_enabled,
                logger);
        }
    }
}

void AdaptiveLearner::finish_observation(
    float observed_speed,
    float observed_rpm,
    bool left_gear_early,
    bool logging_enabled,
    const std::function<void(const char*)>& logger)
{
    if (!learning_observation_pending_ ||
        learning_shift_from_gear_ < 1 ||
        learning_shift_to_gear_ < 1 ||
        learning_shift_to_gear_ >= static_cast<int>(LEARNING_GEAR_COUNT))
    {
        cancel_observation();
        return;
    }

    const float speed_loss =
        std::max(0.0f, learning_shift_start_speed_ - observed_speed);

    const std::size_t bucket = learning_bucket_index(
        learning_shift_from_gear_,
        learning_shift_to_gear_,
        learning_shift_throttle_,
        learning_shift_speed_delta_);

    ShiftLearning& learning = shift_learning_[bucket];

    ++learning.attempts;

    learning.average_speed_loss +=
        (speed_loss - learning.average_speed_loss) /
        static_cast<float>(learning.attempts);

    learning.average_rpm_after +=
        (observed_rpm - learning.average_rpm_after) /
        static_cast<float>(learning.attempts);

    const bool good =
        !left_gear_early &&
        speed_loss <= ADAPTIVE_ACCEPTABLE_SPEED_LOSS;

    if (good)
        ++learning.successful;

    if (logging_enabled && logger)
    {
        char b[360];

        std::snprintf(
            b,
            sizeof(b),
            "EcoDrive adaptive: %d -> %d | load %.2f | dSpeed %.3f | speed loss %.2f | RPM after %.0f | result %s | samples %u | good %u",
            learning_shift_from_gear_,
            learning_shift_to_gear_,
            learning_shift_throttle_,
            learning_shift_speed_delta_,
            speed_loss,
            observed_rpm,
            good ? "GOOD" : "BAD",
            learning.attempts,
            learning.successful);

        logger(b);
    }

    cancel_observation();
}

float AdaptiveLearner::calculate_cruise_rpm_floor(
    int current_gear,
    float speed_delta,
    float throttle,
    float base_floor_rpm) const
{
    float floor = base_floor_rpm;

    if (current_gear + 1 < static_cast<int>(LEARNING_GEAR_COUNT))
    {
        const std::size_t bucket = learning_bucket_index(
            current_gear,
            current_gear + 1,
            throttle,
            speed_delta);

        const ShiftLearning& learning = shift_learning_[bucket];

        if (learning.attempts >= ADAPTIVE_MIN_OBSERVATIONS)
        {
            const float success_rate =
                static_cast<float>(learning.successful) /
                static_cast<float>(learning.attempts);

            if (success_rate < 0.34f ||
                learning.average_speed_loss >
                    ADAPTIVE_ACCEPTABLE_SPEED_LOSS +
                        ADAPTIVE_SPEED_LOSS_MARGIN)
            {
                floor = std::min(
                    CRUISE_ECONOMY_RECOVERY_RPM,
                    base_floor_rpm + ADAPTIVE_RPM_STEP);
            }
        }
    }

    return floor;
}

void AdaptiveLearner::reset()
{
    shift_learning_.fill(ShiftLearning{});
    cancel_observation();
}

}