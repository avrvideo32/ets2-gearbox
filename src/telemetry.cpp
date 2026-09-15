#include "telemetry.h"
#include "input_device.h"
#include "common/scssdk_telemetry_common_configs.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <windows.h>

namespace ecodrive
{

namespace
{

Telemetry* g_telemetry = nullptr;

constexpr const char* CHANNEL_RPM = "truck.engine.rpm";
constexpr const char* CHANNEL_SPEED = "truck.speed";
constexpr const char* CHANNEL_THROTTLE = "truck.effective.throttle";
constexpr const char* CHANNEL_INPUT_THROTTLE = "truck.input.throttle";
constexpr const char* CHANNEL_INPUT_BRAKE = "truck.input.brake";
constexpr const char* CHANNEL_GEAR = "truck.engine.gear";
constexpr const char* CHANNEL_CLUTCH = "truck.effective.clutch";
constexpr const char* CHANNEL_CRUISE = "truck.cruise_control";
constexpr const char* CHANNEL_FUEL = "truck.fuel.amount";
constexpr const char* CHANNEL_LINEAR_ACCEL = "truck.local.acceleration.linear";
constexpr const char* CHANNEL_WORLD_PLACEMENT = "truck.world.placement";
constexpr const char* CHANNEL_MOTOR_BRAKE = "truck.brake.motor";
constexpr const char* CHANNEL_RETARDER_LEVEL = "truck.brake.retarder";
constexpr const char* CHANNEL_SPEED_LIMIT = "truck.navigation.speed.limit";
constexpr const char* CHANNEL_AVG_CONSUMPTION = "truck.fuel.consumption.average";

constexpr float THROTTLE_ACTIVE_THRESHOLD = 0.05f;
constexpr float THROTTLE_RELEASE_THRESHOLD = 0.02f;

constexpr unsigned THROTTLE_DEBOUNCE_UPDATES = 3;
constexpr unsigned THROTTLE_DEBOUNCE_DOWN_UPDATES = 3;

constexpr float ENGINE_RUNNING_RPM_THRESHOLD = 300.0f;
constexpr float CRUISE_ACTIVE_SPEED_THRESHOLD = 0.10f;
constexpr float STANDSTILL_SPEED_THRESHOLD = 0.50f;

constexpr unsigned FILE_LOG_FLUSH_INTERVAL = 64;

// FIX: throttle verbose telemetry snapshots.
constexpr unsigned TELEMETRY_SNAPSHOT_LOG_INTERVAL = 60;

// FIX: clamp unreasonable per-frame speed deltas.
constexpr float MAX_SPEED_DELTA_PER_FRAME = 0.5f;

}

Telemetry::Telemetry(InputDevice& input_device)
    : input_device_(input_device)
{
    g_telemetry = this;
}

scs_result_t Telemetry::initialize(
    const scs_telemetry_init_params_v100_t& params)
{
    if (initialized_)
        return SCS_RESULT_already_registered;

    if (!params.register_for_channel || !params.register_for_event)
        return SCS_RESULT_invalid_parameter;

    log_ = params.common.log;
    unregister_from_channel_ = params.unregister_from_channel;
    unregister_from_event_ = params.unregister_from_event;

    config_.load(log_);
    open_file_log();

    struct ChannelReg
    {
        const char* name;
        scs_value_type_t type;
    };

    const ChannelReg channels[] =
    {
        { CHANNEL_RPM, SCS_VALUE_TYPE_float },
        { CHANNEL_SPEED, SCS_VALUE_TYPE_float },
        { CHANNEL_THROTTLE, SCS_VALUE_TYPE_float },
        { CHANNEL_INPUT_THROTTLE, SCS_VALUE_TYPE_float },
        { CHANNEL_INPUT_BRAKE, SCS_VALUE_TYPE_float },
        { CHANNEL_GEAR, SCS_VALUE_TYPE_s32 },
        { CHANNEL_CLUTCH, SCS_VALUE_TYPE_float },
        { CHANNEL_CRUISE, SCS_VALUE_TYPE_float },
        { CHANNEL_FUEL, SCS_VALUE_TYPE_float },
        { CHANNEL_LINEAR_ACCEL, SCS_VALUE_TYPE_fvector },
        { CHANNEL_WORLD_PLACEMENT, SCS_VALUE_TYPE_dplacement },
        { CHANNEL_MOTOR_BRAKE, SCS_VALUE_TYPE_bool },
        { CHANNEL_RETARDER_LEVEL, SCS_VALUE_TYPE_s32 },
        { CHANNEL_SPEED_LIMIT, SCS_VALUE_TYPE_float },
        { CHANNEL_AVG_CONSUMPTION, SCS_VALUE_TYPE_float }
    };

    for (const auto& ch : channels)
    {
        params.register_for_channel(
            ch.name,
            SCS_U32_NIL,
            ch.type,
            SCS_TELEMETRY_CHANNEL_FLAG_none,
            telemetry_channel_callback,
            this);
    }

    params.register_for_event(
        SCS_TELEMETRY_EVENT_configuration,
        telemetry_configuration_callback,
        this);

    params.register_for_event(
        SCS_TELEMETRY_EVENT_frame_end,
        telemetry_frame_end_callback,
        this);

    initialized_ = true;
    minimum_gear_ = calculate_dynamic_takeoff_gear();

    log("EcoDrive: Telemetry subsystem successfully initialized.");

    return SCS_RESULT_ok;
}

void Telemetry::shutdown()
{
    if (!initialized_)
        return;

    if (unregister_from_channel_)
    {
        const char* const channel_names[] =
        {
            CHANNEL_RPM,
            CHANNEL_SPEED,
            CHANNEL_THROTTLE,
            CHANNEL_INPUT_THROTTLE,
            CHANNEL_INPUT_BRAKE,
            CHANNEL_GEAR,
            CHANNEL_CLUTCH,
            CHANNEL_CRUISE,
            CHANNEL_FUEL,
            CHANNEL_LINEAR_ACCEL,
            CHANNEL_WORLD_PLACEMENT,
            CHANNEL_MOTOR_BRAKE,
            CHANNEL_RETARDER_LEVEL,
            CHANNEL_SPEED_LIMIT,
            CHANNEL_AVG_CONSUMPTION
        };

        for (const char* name : channel_names)
        {
            unregister_from_channel_(
                name,
                SCS_U32_NIL,
                SCS_VALUE_TYPE_INVALID);
        }
    }

    if (unregister_from_event_)
    {
        unregister_from_event_(SCS_TELEMETRY_EVENT_configuration);
        unregister_from_event_(SCS_TELEMETRY_EVENT_frame_end);
    }

    close_file_log();
    initialized_ = false;
}

SCSAPI_VOID Telemetry::telemetry_channel_callback(
    const scs_string_t name,
    const scs_u32_t index,
    const scs_value_t* const value,
    const scs_context_t context)
{
    auto* self = static_cast<Telemetry*>(context);
    if (self)
    {
        self->handle_channel(name, index, value);
    }
}

SCSAPI_VOID Telemetry::telemetry_configuration_callback(
    const scs_event_t event,
    const void* const event_info,
    const scs_context_t context)
{
    auto* self = static_cast<Telemetry*>(context);

    if (self && event_info)
    {
        self->handle_configuration(
            static_cast<const scs_telemetry_configuration_t*>(event_info));
    }
}

SCSAPI_VOID Telemetry::telemetry_frame_end_callback(
    const scs_event_t event,
    const void* const event_info,
    const scs_context_t context)
{
    auto* self = static_cast<Telemetry*>(context);
    if (self)
    {
        self->on_frame_end();
    }
}

void Telemetry::handle_channel(
    const scs_string_t name,
    const scs_u32_t index,
    const scs_value_t* const value)
{
    if (!name || !value)
        return;

    if (std::strcmp(name, CHANNEL_RPM) == 0 &&
        value->type == SCS_VALUE_TYPE_float)
    {
        rpm_.store(
            std::max(0.0f, value->value_float.value),
            std::memory_order_relaxed);
    }
    else if (std::strcmp(name, CHANNEL_SPEED) == 0 &&
             value->type == SCS_VALUE_TYPE_float)
    {
        speed_.store(
            value->value_float.value,
            std::memory_order_relaxed);
    }
    else if (std::strcmp(name, CHANNEL_THROTTLE) == 0 &&
             value->type == SCS_VALUE_TYPE_float)
    {
        throttle_.store(
            std::clamp(value->value_float.value, 0.0f, 1.0f),
            std::memory_order_relaxed);
    }
    else if (std::strcmp(name, CHANNEL_INPUT_THROTTLE) == 0 &&
             value->type == SCS_VALUE_TYPE_float)
    {
        input_throttle_.store(
            std::clamp(value->value_float.value, 0.0f, 1.0f),
            std::memory_order_relaxed);
    }
    else if (std::strcmp(name, CHANNEL_INPUT_BRAKE) == 0 &&
             value->type == SCS_VALUE_TYPE_float)
    {
        input_brake_.store(
            std::clamp(value->value_float.value, 0.0f, 1.0f),
            std::memory_order_relaxed);
    }
    else if (std::strcmp(name, CHANNEL_GEAR) == 0 &&
             value->type == SCS_VALUE_TYPE_s32)
    {
        gear_.store(
            value->value_s32.value,
            std::memory_order_relaxed);
    }
    else if (std::strcmp(name, CHANNEL_CLUTCH) == 0 &&
             value->type == SCS_VALUE_TYPE_float)
    {
        clutch_.store(
            std::clamp(value->value_float.value, 0.0f, 1.0f),
            std::memory_order_relaxed);
    }
    else if (std::strcmp(name, CHANNEL_CRUISE) == 0 &&
             value->type == SCS_VALUE_TYPE_float)
    {
        cruise_control_.store(
            std::max(0.0f, value->value_float.value),
            std::memory_order_relaxed);
    }
    else if (std::strcmp(name, CHANNEL_FUEL) == 0 &&
             value->type == SCS_VALUE_TYPE_float)
    {
        fuel_.store(
            std::max(0.0f, value->value_float.value),
            std::memory_order_relaxed);
    }
    else if (std::strcmp(name, CHANNEL_LINEAR_ACCEL) == 0 &&
             value->type == SCS_VALUE_TYPE_fvector)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        const float az = value->value_fvector.z;

        if (std::isfinite(az))
        {
            const float grade = -(az / 9.81f) * 100.0f;

            powertrain_context_.grade_percent =
                powertrain_context_.grade_percent * 0.90f +
                grade * 0.10f;
        }
    }
    else if (std::strcmp(name, CHANNEL_MOTOR_BRAKE) == 0 &&
             value->type == SCS_VALUE_TYPE_bool)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        powertrain_context_.motor_brake_active =
            (value->value_bool.value != 0);
    }
    else if (std::strcmp(name, CHANNEL_RETARDER_LEVEL) == 0 &&
             value->type == SCS_VALUE_TYPE_s32)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        powertrain_context_.retarder_level = value->value_s32.value;
    }
    else if (std::strcmp(name, CHANNEL_SPEED_LIMIT) == 0 &&
             value->type == SCS_VALUE_TYPE_float)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        powertrain_context_.speed_limit_mps =
            std::max(0.0f, value->value_float.value);
    }
    else if (std::strcmp(name, CHANNEL_AVG_CONSUMPTION) == 0 &&
             value->type == SCS_VALUE_TYPE_float)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        powertrain_context_.avg_fuel_consumption =
            std::max(0.0f, value->value_float.value);
    }
}

void Telemetry::handle_configuration(
    const scs_telemetry_configuration_t* config)
{
    if (!config || !config->id || !config->attributes)
        return;

    std::lock_guard<std::mutex> lock(state_mutex_);

    if (std::strcmp(config->id, SCS_TELEMETRY_CONFIG_truck) == 0)
    {
        powertrain_context_.forward_ratios.clear();

        for (const scs_named_value_t* current = config->attributes;
             current->name != nullptr;
             ++current)
        {
            if (std::strcmp(current->name,
                            SCS_TELEMETRY_CONFIG_ATTRIBUTE_brand) == 0 &&
                current->value.type == SCS_VALUE_TYPE_string)
            {
                truck_brand_ = current->value.value_string.value;
            }
            else if (std::strcmp(current->name,
                                 SCS_TELEMETRY_CONFIG_ATTRIBUTE_name) == 0 &&
                     current->value.type == SCS_VALUE_TYPE_string)
            {
                truck_name_ = current->value.value_string.value;
            }
            else if (std::strcmp(current->name,
                                 SCS_TELEMETRY_CONFIG_ATTRIBUTE_id) == 0 &&
                     current->value.type == SCS_VALUE_TYPE_string)
            {
                truck_id_ = current->value.value_string.value;
            }
            else if (std::strcmp(current->name,
                                 SCS_TELEMETRY_CONFIG_ATTRIBUTE_forward_gear_count) == 0 &&
                     current->value.type == SCS_VALUE_TYPE_u32)
            {
                detected_forward_gears_ =
                    static_cast<int>(current->value.value_u32.value);
            }
            else if (std::strcmp(current->name,
                                 SCS_TELEMETRY_CONFIG_ATTRIBUTE_rpm_limit) == 0 &&
                     current->value.type == SCS_VALUE_TYPE_float)
            {
                detected_rpm_limit_ = current->value.value_float.value;
                powertrain_context_.rpm_limit = detected_rpm_limit_;
            }
            else if (std::strcmp(current->name,
                                 SCS_TELEMETRY_CONFIG_ATTRIBUTE_differential_ratio) == 0 &&
                     current->value.type == SCS_VALUE_TYPE_float)
            {
                powertrain_context_.differential_ratio =
                    current->value.value_float.value;
            }
            else if (std::strcmp(current->name,
                                 SCS_TELEMETRY_CONFIG_ATTRIBUTE_forward_ratio) == 0 &&
                     current->value.type == SCS_VALUE_TYPE_float)
            {
                if (current->index != SCS_U32_NIL)
                {
                    if (powertrain_context_.forward_ratios.size() <=
                        current->index)
                    {
                        powertrain_context_.forward_ratios.resize(
                            current->index + 1,
                            0.0f);
                    }

                    powertrain_context_.forward_ratios[current->index] =
                        current->value.value_float.value;
                }
            }
        }

        char msg[256];

        std::snprintf(
            msg,
            sizeof(msg),
            "EcoDrive: Truck detected: %s %s (ID: %s, Forward Gears: %d, Diff: %.2f)",
            truck_brand_.c_str(),
            truck_name_.c_str(),
            truck_id_.c_str(),
            detected_forward_gears_,
            powertrain_context_.differential_ratio);

        log(msg);
    }
    else if (std::strcmp(config->id, SCS_TELEMETRY_CONFIG_job) == 0)
    {
        powertrain_context_.has_job = false;
        powertrain_context_.cargo_mass_kg = 0.0f;

        for (const scs_named_value_t* current = config->attributes;
             current->name != nullptr;
             ++current)
        {
            if (std::strcmp(current->name,
                            SCS_TELEMETRY_CONFIG_ATTRIBUTE_cargo_mass) == 0 &&
                current->value.type == SCS_VALUE_TYPE_float)
            {
                powertrain_context_.cargo_mass_kg =
                    current->value.value_float.value;

                powertrain_context_.has_job =
                    (powertrain_context_.cargo_mass_kg > 0.1f);
            }
        }

        if (powertrain_context_.has_job)
        {
            char msg[128];

            std::snprintf(
                msg,
                sizeof(msg),
                "EcoDrive: Active job cargo mass: %.0f kg",
                powertrain_context_.cargo_mass_kg);

            log(msg);
        }
    }
}

void Telemetry::on_frame_end()
{
    const auto now = std::chrono::steady_clock::now();

    float dt = 0.0f;

    if (last_frame_time_.time_since_epoch().count() > 0)
    {
        dt = std::chrono::duration<float>(now - last_frame_time_).count();
    }

    last_frame_time_ = now;

    config_.check_and_reload_if_modified(log_);

    const float rpm = rpm_.load(std::memory_order_relaxed);
    const float raw_speed = speed_.load(std::memory_order_relaxed);
    const float sim_throttle = throttle_.load(std::memory_order_relaxed);
    const float driver_throttle = input_throttle_.load(std::memory_order_relaxed);
    const float brake = input_brake_.load(std::memory_order_relaxed);
    const float clutch = clutch_.load(std::memory_order_relaxed);
    const float cruise = cruise_control_.load(std::memory_order_relaxed);
    const float fuel = fuel_.load(std::memory_order_relaxed);
    const int current_gear = gear_.load(std::memory_order_relaxed);

    // FIX: use absolute speed everywhere.
    const float speed_abs = std::fabs(raw_speed);

    // FIX: clamp impossible per-frame speed deltas.
    const float raw_delta = speed_abs - previous_speed_;
    speed_delta_ = std::clamp(
        raw_delta,
        -MAX_SPEED_DELTA_PER_FRAME,
        MAX_SPEED_DELTA_PER_FRAME);

    previous_speed_ = speed_abs;

    const bool engine_was_running = engine_running_;
    engine_running_ = (rpm >= ENGINE_RUNNING_RPM_THRESHOLD);

    auto logger = [this](const char* msg)
    {
        this->log(msg);
    };

    if (driver_throttle >= THROTTLE_ACTIVE_THRESHOLD)
    {
        ++throttle_active_updates_;
        throttle_inactive_updates_ = 0;

        if (throttle_active_updates_ >= THROTTLE_DEBOUNCE_UPDATES)
        {
            throttle_active_ = true;
        }
    }
    else if (driver_throttle <= THROTTLE_RELEASE_THRESHOLD)
    {
        ++throttle_inactive_updates_;
        throttle_active_updates_ = 0;

        if (throttle_inactive_updates_ >= THROTTLE_DEBOUNCE_DOWN_UPDATES)
        {
            throttle_active_ = false;
        }
    }

    if (!engine_running_)
    {
        if (engine_was_running)
        {
            coasting_.reset();
            shifter_.reset();

            // FIX: cancel stale adaptive observations.
            adaptive_learner_.cancel_observation();

            minimum_gear_locked_ = false;
            minimum_gear_ = calculate_dynamic_takeoff_gear();
        }

        previous_gear_ = current_gear;
        previous_cruise_ = cruise;
        previous_speed_ = speed_abs;
        speed_delta_ = 0.0f;

        return;
    }

    const int max_gear = effective_max_gear();

    if (current_gear > 0 && current_gear >= minimum_gear_)
    {
        minimum_gear_locked_ = true;
    }

    if (speed_abs <= STANDSTILL_SPEED_THRESHOLD && !throttle_active_)
    {
        minimum_gear_ = calculate_dynamic_takeoff_gear();
        minimum_gear_locked_ = false;
    }

    // FIX: take a locked snapshot of powertrain context for this frame.
    PowertrainContext context;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        context = powertrain_context_;
    }

    shifter_.on_rpm_tick();

    if (cruise != previous_cruise_)
    {
        shifter_.on_cruise_changed(previous_cruise_, cruise, config_, logger);
        previous_cruise_ = cruise;
    }

    shifter_.on_brake_sample(brake, config_);

    shifter_.on_speed_sample(
        speed_abs,
        speed_delta_,
        sim_throttle,
        cruise,
        current_gear,
        rpm,
        config_,
        adaptive_learner_,
        logger);

    adaptive_learner_.on_speed_update(
        speed_abs,
        rpm,
        current_gear,
        config_.shift_logging,
        logger);

    context.calibrate_wheel_radius(current_gear, speed_abs, rpm);

    trip_tracker_.on_rpm_sample(rpm, speed_abs, current_gear, dt);
    trip_tracker_.on_fuel_sample(fuel);

    coasting_.on_speed_sample(speed_abs);

    coasting_.on_throttle_sample(
        sim_throttle,
        driver_throttle,
        current_gear,
        shifter_.is_shift_in_progress(),
        config_);

    if (coasting_.should_start_coasting(
            manual_driver_wants_drive(),
            shifter_.post_shift_drive_grace_updates(),
            shifter_.is_brake_active(),
            sim_throttle,
            current_gear,
            speed_abs,
            config_,
            context))
    {
        coasting_.start_coasting(
            current_gear,
            speed_abs,
            cruise,
            config_,
            logger);
    }

    if (current_gear != previous_gear_)
    {
        shifter_.on_gear_update(
            input_device_,
            current_gear,
            previous_gear_,
            max_gear,
            config_,
            coasting_,
            adaptive_learner_,
            trip_tracker_,
            logger);

        previous_gear_ = current_gear;
    }

    bool shift_prog = shifter_.is_shift_in_progress();
    int req_gear = shifter_.requested_gear();
    ShiftPurpose purpose = shifter_.shift_purpose();
    unsigned wait_updates = shifter_.shift_wait_updates();

    if (coasting_.is_neutral_in_progress())
    {
        coasting_.update_neutral_logic(
            input_device_,
            current_gear,
            speed_abs,
            speed_delta_,
            driver_throttle,
            cruise_active(),
            driver_wants_drive(),
            minimum_gear_,
            max_gear,
            config_,
            logger,
            shift_prog,
            req_gear,
            purpose,
            wait_updates,
            context);

        shifter_.set_shift_in_progress(shift_prog);
        shifter_.set_requested_gear(req_gear);
        shifter_.set_shift_purpose(purpose);
        shifter_.set_shift_wait_updates(wait_updates);
    }
    else if (coasting_.is_restore_in_progress() || current_gear == 0)
    {
        coasting_.update_restore_logic(
            input_device_,
            current_gear,
            driver_wants_drive(),
            max_gear,
            config_,
            logger,
            shift_prog,
            req_gear,
            purpose,
            wait_updates);

        shifter_.set_shift_in_progress(shift_prog);
        shifter_.set_requested_gear(req_gear);
        shifter_.set_shift_purpose(purpose);
        shifter_.set_shift_wait_updates(wait_updates);
    }
    else
    {
        shifter_.update_automatic_shifter(
            input_device_,
            current_gear,
            rpm,
            speed_abs,
            speed_delta_,
            sim_throttle,
            clutch,
            cruise_active(),
            cruise,
            throttle_active_,
            minimum_gear_,
            minimum_gear_locked_,
            max_gear,
            config_,
            coasting_,
            adaptive_learner_,
            logger,
            context);
    }

    // FIX: write back only calibrated wheel-radius fields.
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        powertrain_context_.wheel_radius = context.wheel_radius;
        powertrain_context_.wheel_radius_calibrated =
            context.wheel_radius_calibrated;
    }

    if (trip_tracker_.check_periodic_report(
            now,
            config_.trip_summary_interval_minutes))
    {
        trip_tracker_.log_report(
            truck_brand_,
            truck_name_,
            truck_id_,
            max_gear,
            logger);
    }

    // FIX: throttled verbose telemetry snapshot.
    if (config_.telemetry_logging)
    {
        if (++telemetry_log_counter_ >= TELEMETRY_SNAPSHOT_LOG_INTERVAL)
        {
            telemetry_log_counter_ = 0;

            const bool effective_power_request =
                sim_throttle > THROTTLE_RELEASE_THRESHOLD;

            const bool coast_state =
                !throttle_active_ &&
                !effective_power_request &&
                !shifter_.is_brake_active();

            const char* mode = "DRIVE";

            if (shifter_.is_brake_active())
                mode = "BRAKE";
            else if (cruise_active())
                mode = "CRUISE_ECO";
            else if (coast_state)
                mode = "COAST";

            char b[512];

            std::snprintf(
                b,
                sizeof(b),
                "EcoDrive telemetry: Mode %s | RPM %.0f | Speed %.2f m/s | dSpeed %.3f | Throttle %.2f | Brake %.2f | Gear %d | Grade %.1f%% | Retarder %d | Cargo %.1ft | Limit %.0f km/h",
                mode,
                rpm,
                speed_abs,
                speed_delta_,
                sim_throttle,
                brake,
                current_gear,
                context.grade_percent,
                context.retarder_level,
                context.cargo_mass_kg / 1000.0f,
                context.speed_limit_mps * 3.6f);

            log(b);
        }
    }
}

bool Telemetry::cruise_active() const
{
    return cruise_control_.load(std::memory_order_relaxed) >
           CRUISE_ACTIVE_SPEED_THRESHOLD;
}

bool Telemetry::driver_wants_drive() const
{
    return throttle_active_ || cruise_active();
}

bool Telemetry::manual_driver_wants_drive() const
{
    return throttle_active_;
}

int Telemetry::effective_max_gear() const
{
    if (detected_forward_gears_ > 0)
    {
        return std::min(config_.max_forward_gear, detected_forward_gears_);
    }

    return config_.max_forward_gear;
}

int Telemetry::calculate_dynamic_takeoff_gear() const
{
    int base_takeoff = config_.takeoff_gear;
    const int max_gear = effective_max_gear();

    if (base_takeoff < 1)
        base_takeoff = 1;

    if (base_takeoff > max_gear)
        base_takeoff = max_gear;

    if (config_.load_adaptive_shifting_enabled &&
        powertrain_context_.has_job)
    {
        if (powertrain_context_.cargo_mass_kg > 25000.0f &&
            base_takeoff > 1)
        {
            base_takeoff = 1;
        }
        else if (powertrain_context_.cargo_mass_kg > 15000.0f &&
                 base_takeoff > 2)
        {
            base_takeoff = 2;
        }
    }

    return base_takeoff;
}

void Telemetry::log(const char* message)
{
    if (log_)
    {
        log_(SCS_LOG_TYPE_message, message);
    }

    write_file_log(message);
}

void Telemetry::open_file_log()
{
    if (!config_.telemetry_logging && !config_.shift_logging)
        return;

    std::lock_guard<std::mutex> lock(file_log_mutex_);

    if (!file_log_.is_open())
    {
        char buffer[MAX_PATH];
        DWORD length = GetModuleFileNameA(NULL, buffer, MAX_PATH);

        std::string base_dir;

        if (length > 0)
        {
            std::string path(buffer, length);
            std::size_t pos = path.find_last_of("\\/");

            if (pos != std::string::npos)
                base_dir = path.substr(0, pos + 1);
        }

        file_log_path_ = base_dir + "ecodrive.log";

        file_log_.open(file_log_path_, std::ios::out | std::ios::app);

        if (file_log_.is_open())
        {
            file_log_ << "=== EcoDrive Telemetry Log Initialized ===\n";
            file_log_.flush();
        }
    }
}

void Telemetry::close_file_log()
{
    std::lock_guard<std::mutex> lock(file_log_mutex_);

    if (file_log_.is_open())
    {
        file_log_ << "=== EcoDrive Telemetry Log Closed ===\n";
        file_log_.close();
    }
}

void Telemetry::write_file_log(const char* message)
{
    if (!message)
        return;

    std::lock_guard<std::mutex> lock(file_log_mutex_);

    if (file_log_.is_open())
    {
        file_log_ << message << "\n";

        if (++file_log_flush_counter_ >= FILE_LOG_FLUSH_INTERVAL)
        {
            file_log_.flush();
            file_log_flush_counter_ = 0;
        }
    }
}

}