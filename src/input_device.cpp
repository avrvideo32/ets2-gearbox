#include "input_device.h"

#include <algorithm>

namespace ecodrive
{
    namespace
    {
        InputDevice* g_device = nullptr;
    }

    InputDevice::InputDevice()
    {
        /*
         * These names correspond to ETS2's semantic
         * sequential gearbox mixes:
         *
         * semantical.gearup
         * semantical.geardown
         * semantical.gear0
         */
        inputs_[INPUT_GEAR_UP].name = "gearup";
        inputs_[INPUT_GEAR_UP].display_name = "EcoDrive Gear Up";
        inputs_[INPUT_GEAR_UP].value_type = SCS_VALUE_TYPE_bool;

        inputs_[INPUT_GEAR_DOWN].name = "geardown";
        inputs_[INPUT_GEAR_DOWN].display_name = "EcoDrive Gear Down";
        inputs_[INPUT_GEAR_DOWN].value_type = SCS_VALUE_TYPE_bool;

        inputs_[INPUT_NEUTRAL].name = "gear0";
        inputs_[INPUT_NEUTRAL].display_name = "EcoDrive Neutral";
        inputs_[INPUT_NEUTRAL].value_type = SCS_VALUE_TYPE_bool;

        // ETS2 uses this exact semantic command name for cruise resume.
        inputs_[INPUT_CRUISE_RESUME].name = "cruiectrlres";
        inputs_[INPUT_CRUISE_RESUME].display_name = "EcoDrive Cruise Resume";
        inputs_[INPUT_CRUISE_RESUME].value_type = SCS_VALUE_TYPE_bool;

        inputs_[INPUT_CLUTCH].name = "dclutch";
        inputs_[INPUT_CLUTCH].display_name = "EcoDrive Clutch";
        inputs_[INPUT_CLUTCH].value_type = SCS_VALUE_TYPE_bool;

        inputs_[INPUT_GEAR_2].name = "gear2";
        inputs_[INPUT_GEAR_2].display_name = "EcoDrive Direct Gear 2";
        inputs_[INPUT_GEAR_2].value_type = SCS_VALUE_TYPE_bool;

        inputs_[INPUT_GEAR_4].name = "gear4";
        inputs_[INPUT_GEAR_4].display_name = "EcoDrive Direct Gear 4";
        inputs_[INPUT_GEAR_4].value_type = SCS_VALUE_TYPE_bool;

        inputs_[INPUT_GEAR_6].name = "gear6";
        inputs_[INPUT_GEAR_6].display_name = "EcoDrive Direct Gear 6";
        inputs_[INPUT_GEAR_6].value_type = SCS_VALUE_TYPE_bool;

        inputs_[INPUT_GEAR_8].name = "gear8";
        inputs_[INPUT_GEAR_8].display_name = "EcoDrive Direct Gear 8";
        inputs_[INPUT_GEAR_8].value_type = SCS_VALUE_TYPE_bool;

        inputs_[INPUT_GEAR_10].name = "gear10";
        inputs_[INPUT_GEAR_10].display_name = "EcoDrive Direct Gear 10";
        inputs_[INPUT_GEAR_10].value_type = SCS_VALUE_TYPE_bool;

        inputs_[INPUT_GEAR_12].name = "gear12";
        inputs_[INPUT_GEAR_12].display_name = "EcoDrive Direct Gear 12";
        inputs_[INPUT_GEAR_12].value_type = SCS_VALUE_TYPE_bool;

        device_.name = "ecodrive";
        device_.display_name = "EcoDrive";
        device_.type = SCS_INPUT_DEVICE_TYPE_semantical;
        device_.input_count = INPUT_COUNT;
        device_.inputs = inputs_;
        device_.callback_context = this;
        device_.input_active_callback = nullptr;
        device_.input_event_callback = input_event_callback;

        g_device = this;
    }

    scs_result_t InputDevice::register_device(
        scs_input_register_device_t register_device)
    {
        if (registered_)
            return SCS_RESULT_already_registered;

        if (!register_device)
            return SCS_RESULT_invalid_parameter;

        const scs_result_t result =
            register_device(&device_);

        if (result == SCS_RESULT_ok)
            registered_ = true;

        return result;
    }

    bool InputDevice::request_command(Command command)
    {
        if (command == Command::none)
            return false;

        /*
         * Never overwrite an existing command.
         *
         * This is important because telemetry can be updated
         * several times per frame and we do not want to flood
         * ETS2 with gear commands.
         */
        int expected =
            static_cast<int>(Command::none);

        return pending_command_.compare_exchange_strong(
            expected,
            static_cast<int>(command),
            std::memory_order_acq_rel);
    }

    void InputDevice::request_gear_up_burst(unsigned count)
    {
        gear_up_burst_count_.store(count, std::memory_order_release);
    }

    void InputDevice::request_gear_down_burst(unsigned count)
    {
        gear_down_burst_count_.store(count, std::memory_order_release);
    }

    void InputDevice::cancel_burst()
    {
        gear_up_burst_count_.store(0, std::memory_order_release);
        gear_down_burst_count_.store(0, std::memory_order_release);
    }

    unsigned InputDevice::pending_gear_up_burst() const
    {
        return gear_up_burst_count_.load(std::memory_order_acquire);
    }

    void InputDevice::set_current_gear(int gear)
    {
        current_gear_.store(gear, std::memory_order_relaxed);
    }

    void InputDevice::set_max_forward_gear(int gear)
    {
        max_forward_gear_.store(std::max(0, gear), std::memory_order_relaxed);
    }

    void InputDevice::set_clutch_hold(bool hold)
    {
        clutch_hold_.store(hold, std::memory_order_release);
    }

    bool InputDevice::is_clutch_held() const
    {
        return clutch_hold_.load(std::memory_order_acquire);
    }

    scs_result_t InputDevice::handle_event(
        scs_input_event_t* event_info,
        scs_u32_t /*flags*/)
    {
        if (!event_info)
            return SCS_RESULT_invalid_parameter;

        /*
         * If the previous callback generated the button press,
         * generate the matching release now.
         */
        if (release_pending_)
        {
            event_info->input_index =
                static_cast<scs_u32_t>(release_input_index_);

            event_info->value_bool.value = 0;

            release_pending_ = false;

            return SCS_RESULT_ok;
        }

        /*
         * Handle digital clutch hold transitions.
         * When clutch_hold is active, the clutch is kept disengaged during
         * rapid sequential gear shifts so the engine does not over-rev.
         */
        const bool desired_clutch = clutch_hold_.load(std::memory_order_relaxed);
        if (desired_clutch != clutch_applied_)
        {
            clutch_applied_ = desired_clutch;
            event_info->input_index = static_cast<scs_u32_t>(INPUT_CLUTCH);
            event_info->value_bool.value = desired_clutch ? 1 : 0;
            return SCS_RESULT_ok;
        }

        /*
         * Check for rapid burst pulses (e.g. rapid restoration from neutral).
         * Fires successive press/release pairs on each input frame without waiting
         * for full transmission round-trip animations.
         */
        unsigned burst_up = gear_up_burst_count_.load(std::memory_order_relaxed);
        if (burst_up > 0)
        {
            gear_up_burst_count_.store(burst_up - 1, std::memory_order_relaxed);
            event_info->input_index = static_cast<scs_u32_t>(INPUT_GEAR_UP);
            event_info->value_bool.value = 1;
            release_input_index_ = INPUT_GEAR_UP;
            release_pending_ = true;
            return SCS_RESULT_ok;
        }

        unsigned burst_down = gear_down_burst_count_.load(std::memory_order_relaxed);
        if (burst_down > 0)
        {
            gear_down_burst_count_.store(burst_down - 1, std::memory_order_relaxed);
            event_info->input_index = static_cast<scs_u32_t>(INPUT_GEAR_DOWN);
            event_info->value_bool.value = 1;
            release_input_index_ = INPUT_GEAR_DOWN;
            release_pending_ = true;
            return SCS_RESULT_ok;
        }

        /*
         * Take exactly one pending command.
         */
        const int command_value =
            pending_command_.exchange(
                static_cast<int>(Command::none),
                std::memory_order_acq_rel);

        const Command command =
            static_cast<Command>(command_value);

        unsigned input_index = INPUT_GEAR_UP;

        switch (command)
        {
        case Command::gear_up:
            input_index = INPUT_GEAR_UP;
            break;

        case Command::gear_down:
            input_index = INPUT_GEAR_DOWN;
            break;

        case Command::neutral:
            input_index = INPUT_NEUTRAL;
            break;

        case Command::cruise_resume:
            input_index = INPUT_CRUISE_RESUME;
            break;

        case Command::direct_gear_2:
        case Command::direct_gear_4:
        case Command::direct_gear_6:
        case Command::direct_gear_8:
        case Command::direct_gear_10:
        case Command::direct_gear_12:
        {
            int target_gear = 2;
            switch (command)
            {
            case Command::direct_gear_2:  target_gear = 2;  break;
            case Command::direct_gear_4:  target_gear = 4;  break;
            case Command::direct_gear_6:  target_gear = 6;  break;
            case Command::direct_gear_8:  target_gear = 8;  break;
            case Command::direct_gear_10: target_gear = 10; break;
            case Command::direct_gear_12: target_gear = 12; break;
            default: break;
            }

            const int max_gear = max_forward_gear_.load(std::memory_order_relaxed);
            if (max_gear > 0)
                target_gear = std::min(target_gear, max_gear);

            const int current_gear = current_gear_.load(std::memory_order_relaxed);
            const int delta = target_gear - current_gear;

            if (delta > 0)
            {
                gear_up_burst_count_.store(
                    static_cast<unsigned>(delta),
                    std::memory_order_release);
            }
            else if (delta < 0)
            {
                gear_down_burst_count_.store(
                    static_cast<unsigned>(-delta),
                    std::memory_order_release);
            }

            return SCS_RESULT_ok;
        }

        case Command::none:
        default:
            return SCS_RESULT_not_found;
        }

        /*
         * Generate the PRESS.
         *
         * The next callback will generate the RELEASE.
         */
        event_info->input_index =
            static_cast<scs_u32_t>(input_index);

        event_info->value_bool.value = 1;

        release_input_index_ = input_index;
        release_pending_ = true;

        return SCS_RESULT_ok;
    }

    SCSAPI_RESULT InputDevice::input_event_callback(
        scs_input_event_t* const event_info,
        const scs_u32_t flags,
        const scs_context_t context)
    {
        InputDevice* device =
            static_cast<InputDevice*>(context);

        if (!device)
            device = g_device;

        if (!device)
            return SCS_RESULT_not_found;

        return device->handle_event(
            event_info,
            flags);
    }
}
