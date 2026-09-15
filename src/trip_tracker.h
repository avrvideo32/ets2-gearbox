#pragma once

#include "shift_types.h"
#include <chrono>
#include <functional>
#include <string>

namespace ecodrive
{
    class TripTracker
    {
    public:
        struct Metrics
        {
            float distance_meters = 0.0f;
            float drive_time_seconds = 0.0f;
            float coast_time_seconds = 0.0f;
            unsigned total_shifts = 0;
            unsigned automatic_upshifts = 0;
            unsigned automatic_downshifts = 0;
            unsigned cruise_economy_shifts = 0;
            unsigned neutral_coasts = 0;
            unsigned restore_shifts = 0;
            double rpm_sample_sum = 0.0;
            unsigned long long rpm_sample_count = 0;
            float initial_fuel = -1.0f;
            float current_fuel = -1.0f;
            float fuel_consumed = 0.0f;
        };

        TripTracker() = default;

        void on_rpm_sample(float rpm, float speed, int gear, float dt);
        void on_fuel_sample(float fuel_amount);
        void on_shift_confirmed(ShiftPurpose purpose, int previous_gear, int current_gear);

        bool check_periodic_report(std::chrono::steady_clock::time_point current_time, unsigned interval_minutes);
        void log_report(const std::string& brand, const std::string& name, const std::string& id, int max_gears, const std::function<void(const char*)>& logger);

        void reset();
        const Metrics& metrics() const { return metrics_; }

    private:
        Metrics metrics_{};
        std::chrono::steady_clock::time_point last_periodic_summary_time_{};
    };
}
