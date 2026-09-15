#pragma once

#include "scssdk_input_device.h"

#include <atomic>

namespace ecodrive
{
    class InputDevice
    {
    public:
        enum class Command
        {
            none = 0,
            gear_up,
            gear_down,
            neutral,
            cruise_resume
        };

        InputDevice();

        scs_result_t register_device(
            scs_input_register_device_t register_device);

        bool request_command(Command command);
        void request_gear_up_burst(unsigned count);
        void request_gear_down_burst(unsigned count);
        void cancel_burst();
        unsigned pending_gear_up_burst() const;

        void set_clutch_hold(bool hold);
        bool is_clutch_held() const;

        scs_result_t handle_event(
            scs_input_event_t* event_info,
            scs_u32_t flags);

    private:
        static SCSAPI_RESULT input_event_callback(
            scs_input_event_t* const event_info,
            const scs_u32_t flags,
            const scs_context_t context);

        static constexpr unsigned INPUT_GEAR_UP = 0;
        static constexpr unsigned INPUT_GEAR_DOWN = 1;
        static constexpr unsigned INPUT_NEUTRAL = 2;
        static constexpr unsigned INPUT_CRUISE_RESUME = 3;
        static constexpr unsigned INPUT_CLUTCH = 4;
        static constexpr unsigned INPUT_COUNT = 5;

        scs_input_device_input_t inputs_[INPUT_COUNT]{};
        scs_input_device_t device_{};

        std::atomic<int> pending_command_{
            static_cast<int>(Command::none)
        };

        std::atomic<unsigned> gear_up_burst_count_{0};
        std::atomic<unsigned> gear_down_burst_count_{0};

        std::atomic<bool> clutch_hold_{false};
        bool clutch_applied_ = false;

        bool release_pending_ = false;
        unsigned release_input_index_ = INPUT_GEAR_UP;

        bool registered_ = false;
    };
}
