#pragma once
#include <vector>
#include <cmath>

namespace ecodrive
{

enum class ShiftPurpose
{
    none,
    automatic,
    load_downshift,
    retarder_downshift,
    cruise_economy,
    neutralize,
    restore
};

struct PowertrainContext
{
    // Transmission & Axle Ratios
    float differential_ratio{0.0f};
    std::vector<float> forward_ratios{};

    // Dynamic / calibrated tire radius.
    float wheel_radius{0.51f};
    bool wheel_radius_calibrated{false};

    // 3D Road Gradient
    float grade_percent{0.0f};

    // Cargo & Loading
    float cargo_mass_kg{0.0f};
    bool has_job{false};

    // Engine Brake & Retarder
    bool motor_brake_active{false};
    int retarder_level{0};

    // Navigation Speed Limit
    float speed_limit_mps{0.0f};

    // Fuel Consumption & RPM Limit
    float avg_fuel_consumption{0.0f};
    float rpm_limit{2300.0f};

    bool has_exact_ratios() const
    {
        return differential_ratio > 0.01f && !forward_ratios.empty();
    }

    float get_ratio(int gear) const
    {
        if (gear >= 1 && gear <= static_cast<int>(forward_ratios.size()))
        {
            return forward_ratios[gear - 1];
        }

        return 0.0f;
    }

    // FIX: use absolute speed so reverse / negative speed cannot break calculations.
    float calculate_engine_rpm(int gear, float speed_mps) const
    {
        const float abs_speed = std::fabs(speed_mps);

        if (abs_speed <= 0.05f || gear < 1)
            return 0.0f;

        const float ratio = get_ratio(gear);

        if (ratio > 0.001f &&
            differential_ratio > 0.001f &&
            wheel_radius > 0.1f)
        {
            constexpr float TWO_PI = 6.2831853f;

            const float wheel_rpm =
                (abs_speed / wheel_radius) * (60.0f / TWO_PI);

            return wheel_rpm * ratio * differential_ratio;
        }

        return 0.0f;
    }

    // FIX: use absolute speed for wheel-radius calibration.
    void calibrate_wheel_radius(int gear, float speed_mps, float current_rpm)
    {
        const float abs_speed = std::fabs(speed_mps);

        if (gear < 1 || abs_speed < 5.0f || current_rpm < 800.0f)
            return;

        const float ratio = get_ratio(gear);
        if (ratio <= 0.001f || differential_ratio <= 0.001f)
            return;

        constexpr float TWO_PI = 6.2831853f;

        const float calculated_r =
            (abs_speed * 60.0f * ratio * differential_ratio) /
            (TWO_PI * current_rpm);

        if (calculated_r >= 0.35f && calculated_r <= 0.70f)
        {
            if (!wheel_radius_calibrated)
            {
                wheel_radius = calculated_r;
                wheel_radius_calibrated = true;
            }
            else
            {
                wheel_radius = wheel_radius * 0.98f + calculated_r * 0.02f;
            }
        }
    }
};

}