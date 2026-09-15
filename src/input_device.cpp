#include "input_device.h"

#include <cstdio>

namespace ecodrive
{
    namespace
    {
        InputDevice* g_device = nullptr;

        constexpr unsigned CRUISE_RESUME_DELAY_UPDATES = 1;
        constexpr unsigned CRUISE_RESUME_MAX_ATTEMPTS = 12;
        constexpr unsigned CRUISE_RESUME_RETRY_COOLDOWN = 2;
    }

    InputDevice::InputDevice()
    {
        inputs_[INPUT_GEAR_UP].name = "gearup";
        inputs_[INPUT_GEAR_UP].display_name = "EcoDrive Gear Up";
        inputs_[INPUT_GEAR_UP].value_type = SCS_VALUE_TYPE_bool;

        inputs_[INPUT_GEAR_DOWN].name = "geardown";
        inputs_[INPUT_GEAR_DOWN].display_name = "EcoDrive Gear Down";
        inputs_[INPUT_GEAR_DOWN].value_type = SCS_VALUE_TYPE_bool;

        inputs_[INPUT_NEUTRAL].name = "gear0";
        inputs_[INPUT_NEUTRAL].display_name = "EcoDrive Neutral";
        inputs_[INPUT_NEUTRAL].value_type = SCS_VALUE_TYPE_bool;

        inputs_[INPUT_CRUISE_RESUME].name = "cruiectrlres";
        inputs_[INPUT_CRUISE_RESUME].display_name = "EcoDrive Cruise Resume";
        inputs_[INPUT_CRUISE_RESUME].value_type = SCS_VALUE_TYPE_bool;

        inputs_[INPUT_CLUTCH].name = "dclutch";
        inputs_[INPUT_CLUTCH].display_name = "EcoDrive Clutch";
        inputs_[INPUT_CLUTCH].value_type = SCS_VALUE_TYPE_bool;

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

        const scs_result_t result = register_device(&device_);

        if (result == SCS_RESULT_ok)
            registered_ = true;

        return result;
    }

    bool InputDevice::request_command(Command command)
    {
        if (command == Command::none)
            return false;

        // During cruise re-engagement, do not allow the normal gearbox logic
        // to issue another neutral or RPM-driven gear change. Gear 1 is only
        // a staging gear; ETS2 must receive cruise resume before we continue
        // climbing the gearbox. Recovery bursts remain available because they
        // are handled separately in handle_event().
        if (cruise_resume_pending_.load(std::memory_order_acquire) ||
            cruise_resume_attempts_left_ > 0)
        {
            if (command == Command::gear_up ||
                command == Command::gear_down ||
                command == Command::neutral)
            {
                return false;
            }
        }

        if (command == Command::cruise_resume)
        {
            cruise_resume_pending_.store(true, std::memory_order_release);
            return true;
        }

        int expected = static_cast<int>(Command::none);

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

        if (release_pending_)
        {
            event_info->input_index = static_cast<scs_u32_t>(release_input_index_);
            event_info->value_bool.value = 0;
            release_pending_ = false;

            if (release_input_index_ == INPUT_CRUISE_RESUME)
                std::printf("EcoDrive: INPUT cruiectrlres -> RELEASE\n");

            return SCS_RESULT_ok;
        }

        const bool desired_clutch = clutch_hold_.load(std::memory_order_relaxed);
        if (desired_clutch != clutch_applied_)
        {
            clutch_applied_ = desired_clutch;
            event_info->input_index = static_cast<scs_u32_t>(INPUT_CLUTCH);
            event_info->value_bool.value = desired_clutch ? 1 : 0;
            return SCS_RESULT_ok;
        }

        // Recovery bursts deliberately have priority over the normal command
        // slot. Once cruise is confirmed by the game, the shifter can use a
        // burst to leave staging gear 1 immediately.
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

        if (cruise_resume_pending_.exchange(false, std::memory_order_acq_rel))
        {
            cruise_resume_delay_updates_ = CRUISE_RESUME_DELAY_UPDATES;
            cruise_resume_attempts_left_ = CRUISE_RESUME_MAX_ATTEMPTS;
            cruise_resume_cooldown_updates_ = 0;
            std::printf("EcoDrive: cruise resume queued; protected recovery started.\n");
        }

        if (cruise_resume_attempts_left_ > 0)
        {
            if (cruise_resume_delay_updates_ > 0)
            {
                --cruise_resume_delay_updates_;
                return SCS_RESULT_not_found;
            }

            if (cruise_resume_cooldown_updates_ > 0)
            {
                --cruise_resume_cooldown_updates_;
                return SCS_RESULT_not_found;
            }

            --cruise_resume_attempts_left_;
            event_info->input_index = static_cast<scs_u32_t>(INPUT_CRUISE_RESUME);
            event_info->value_bool.value = 1;
            release_input_index_ = INPUT_CRUISE_RESUME;
            release_pending_ = true;
            cruise_resume_cooldown_updates_ = CRUISE_RESUME_RETRY_COOLDOWN;

            std::printf(
                "EcoDrive: INPUT cruiectrlres -> PRESS (attempt %u/%u)\n",
                CRUISE_RESUME_MAX_ATTEMPTS - cruise_resume_attempts_left_,
                CRUISE_RESUME_MAX_ATTEMPTS);

            return SCS_RESULT_ok;
        }

        const int command_value = pending_command_.exchange(
            static_cast<int>(Command::none),
            std::memory_order_acq_rel);

        const Command command = static_cast<Command>(command_value);
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
            event_info->input_index = static_cast<scs_u32_t>(INPUT_CRUISE_RESUME);
            event_info->value_bool.value = 1;
            release_input_index_ = INPUT_CRUISE_RESUME;
            release_pending_ = true;
            std::printf("EcoDrive: INPUT cruiectrlres -> PRESS (compat)\n");
            return SCS_RESULT_ok;

        case Command::none:
        default:
            return SCS_RESULT_not_found;
        }

        event_info->input_index = static_cast<scs_u32_t>(input_index);
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
        InputDevice* device = static_cast<InputDevice*>(context);

        if (!device)
            device = g_device;

        if (!device)
            return SCS_RESULT_not_found;

        return device->handle_event(event_info, flags);
    }
}
