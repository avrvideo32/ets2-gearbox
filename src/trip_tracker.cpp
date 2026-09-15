#include "trip_tracker.h"
#include <cstdio>
#include <algorithm>
#include <cmath>

namespace ecodrive
{

namespace
{

constexpr float STANDSTILL_SPEED_THRESHOLD = 0.50f;

// FIX: reject fuel noise and impossible single-sample fuel drops.
constexpr float FUEL_NOISE_THRESHOLD = 0.005f;
constexpr float FUEL_MAX_DROP_PER_SAMPLE = 0.20f;

}

void TripTracker::on_rpm_sample(float rpm, float speed, int gear, float dt)
{
    if (dt <= 0.0f || dt >= 1.0f)
        return;

    // FIX: use absolute speed so reverse distance is not lost.
    const float abs_speed = std::fabs(speed);

    if (abs_speed > 0.1f)
    {
        metrics_.distance_meters += abs_speed * dt;
    }

    if (gear == 0 && abs_speed > STANDSTILL_SPEED_THRESHOLD)
    {
        metrics_.coast_time_seconds += dt;
    }
    else if (gear > 0 && abs_speed > STANDSTILL_SPEED_THRESHOLD)
    {
        metrics_.drive_time_seconds += dt;
        metrics_.rpm_sample_sum += rpm;
        ++metrics_.rpm_sample_count;
    }
}

void TripTracker::on_fuel_sample(float fuel_amount)
{
    if (metrics_.initial_fuel < 0.0f)
    {
        metrics_.initial_fuel = fuel_amount;
        metrics_.current_fuel = fuel_amount;
        return;
    }

    const float delta = metrics_.current_fuel - fuel_amount;

    if (delta > FUEL_MAX_DROP_PER_SAMPLE)
    {
        // Likely telemetry glitch/refuel snap. Resync but do not consume.
    }
    else if (delta > FUEL_NOISE_THRESHOLD)
    {
        metrics_.fuel_consumed += delta;
    }

    metrics_.current_fuel = fuel_amount;
}

void TripTracker::on_shift_confirmed(
    ShiftPurpose purpose,
    int previous_gear,
    int current_gear)
{
    ++metrics_.total_shifts;

    if (purpose == ShiftPurpose::automatic)
    {
        if (current_gear > previous_gear)
            ++metrics_.automatic_upshifts;
        else
            ++metrics_.automatic_downshifts;
    }
    else if (purpose == ShiftPurpose::cruise_economy)
    {
        ++metrics_.cruise_economy_shifts;
    }
    else if (purpose == ShiftPurpose::neutralize)
    {
        ++metrics_.neutral_coasts;
    }
    else if (purpose == ShiftPurpose::restore)
    {
        ++metrics_.restore_shifts;
    }
    else if (purpose == ShiftPurpose::load_downshift ||
             purpose == ShiftPurpose::retarder_downshift)
    {
        ++metrics_.automatic_downshifts;
    }
}

bool TripTracker::check_periodic_report(
    std::chrono::steady_clock::time_point current_time,
    unsigned interval_minutes)
{
    if (interval_minutes == 0)
        return false;

    if (last_periodic_summary_time_.time_since_epoch().count() == 0)
    {
        last_periodic_summary_time_ = current_time;
        return false;
    }

    if (std::chrono::duration_cast<std::chrono::minutes>(
            current_time - last_periodic_summary_time_)
            .count() >= static_cast<long long>(interval_minutes))
    {
        last_periodic_summary_time_ = current_time;
        return true;
    }

    return false;
}

void TripTracker::log_report(
    const std::string& brand,
    const std::string& name,
    const std::string& id,
    int max_gears,
    const std::function<void(const char*)>& logger)
{
    if (!logger)
        return;

    const float total_time =
        metrics_.drive_time_seconds + metrics_.coast_time_seconds;

    if (total_time < 5.0f && metrics_.distance_meters < 50.0f)
        return;

    const float coast_pct =
        total_time > 0.0f
            ? (metrics_.coast_time_seconds / total_time * 100.0f)
            : 0.0f;

    const float distance_km = metrics_.distance_meters / 1000.0f;

    const float avg_rpm =
        metrics_.rpm_sample_count > 0
            ? static_cast<float>(
                  metrics_.rpm_sample_sum /
                  static_cast<double>(metrics_.rpm_sample_count))
            : 0.0f;

    const float l_per_100km =
        (distance_km > 1.0f && metrics_.fuel_consumed > 0.0f)
            ? (metrics_.fuel_consumed / distance_km * 100.0f)
            : 0.0f;

    const unsigned drive_m =
        static_cast<unsigned>(metrics_.drive_time_seconds) / 60;

    const unsigned drive_s =
        static_cast<unsigned>(metrics_.drive_time_seconds) % 60;

    const unsigned coast_m =
        static_cast<unsigned>(metrics_.coast_time_seconds) / 60;

    const unsigned coast_s =
        static_cast<unsigned>(metrics_.coast_time_seconds) % 60;

    char b1[320];
    char b2[320];
    char b3[320];

    logger(
        "================== [EcoDrive Trip Efficiency Report] ==================");

    std::snprintf(
        b1,
        sizeof(b1),
        "EcoDrive: Vehicle: %s %s (%d gears) | Distance: %.1f km | Drive: %um %02us | Coast: %um %02us (%.1f%% coasting)",
        brand.empty() ? "Truck" : brand.c_str(),
        name.empty() ? id.c_str() : name.c_str(),
        max_gears,
        distance_km,
        drive_m,
        drive_s,
        coast_m,
        coast_s,
        coast_pct);

    logger(b1);

    std::snprintf(
        b2,
        sizeof(b2),
        "EcoDrive: Shifts: %u total (%u up, %u down, %u cruise-eco, %u coasts) | Avg RPM: %.0f",
        metrics_.total_shifts,
        metrics_.automatic_upshifts,
        metrics_.automatic_downshifts,
        metrics_.cruise_economy_shifts,
        metrics_.neutral_coasts,
        avg_rpm);

    logger(b2);

    if (metrics_.fuel_consumed > 0.0f)
    {
        std::snprintf(
            b3,
            sizeof(b3),
            "EcoDrive: Fuel Consumed: %.2f L (avg %.1f L/100km)",
            metrics_.fuel_consumed,
            l_per_100km);

        logger(b3);
    }

    logger(
        "=======================================================================");
}

void TripTracker::reset()
{
    metrics_ = {};
    last_periodic_summary_time_ = {};
}

}