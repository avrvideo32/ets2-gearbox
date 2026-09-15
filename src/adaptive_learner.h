#pragma once
#include <array>
#include <cstddef>
#include <functional>

namespace ecodrive
{

class AdaptiveLearner
{
public:
    struct ShiftLearning
    {
        unsigned attempts = 0;
        unsigned successful = 0;
        float average_speed_loss = 0.0f;
        float average_rpm_after = 0.0f;
    };

    static constexpr std::size_t LEARNING_GEAR_COUNT = 33;
    static constexpr std::size_t LEARNING_LOAD_BANDS = 4;
    static constexpr std::size_t LEARNING_SPEED_TREND_BANDS = 3;

    static constexpr std::size_t LEARNING_BUCKET_COUNT =
        LEARNING_GEAR_COUNT *
        LEARNING_GEAR_COUNT *
        LEARNING_LOAD_BANDS *
        LEARNING_SPEED_TREND_BANDS;

    AdaptiveLearner() = default;

    void start_observation(
        int from_gear,
        int to_gear,
        float speed,
        float throttle,
        float speed_delta);

    void on_speed_update(
        float new_speed,
        float current_rpm,
        int current_gear,
        bool logging_enabled,
        const std::function<void(const char*)>& logger);

    void finish_observation(
        float observed_speed,
        float observed_rpm,
        bool left_gear_early,
        bool logging_enabled,
        const std::function<void(const char*)>& logger);

    float calculate_cruise_rpm_floor(
        int current_gear,
        float speed_delta,
        float throttle,
        float base_floor_rpm) const;

    bool is_observation_pending() const
    {
        return learning_observation_pending_;
    }

    int observation_target_gear() const
    {
        return learning_shift_to_gear_;
    }

    // FIX: allow cancellation on manual override / shift timeout / engine stop.
    void cancel_observation();

    void reset();

private:
    std::size_t learning_bucket_index(
        int from_gear,
        int to_gear,
        float throttle,
        float speed_delta) const;

    std::array<ShiftLearning, LEARNING_BUCKET_COUNT> shift_learning_{};

    bool learning_observation_pending_ = false;
    unsigned learning_observation_updates_ = 0;

    int learning_shift_from_gear_ = 0;
    int learning_shift_to_gear_ = 0;

    float learning_shift_start_speed_ = 0.0f;
    float learning_shift_throttle_ = 0.0f;
    float learning_shift_speed_delta_ = 0.0f;
};

}