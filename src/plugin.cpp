#include "scssdk.h"
#include "scssdk_input.h"
#include "scssdk_telemetry.h"

#include "input_device.h"
#include "telemetry.h"

namespace
{
    ecodrive::InputDevice g_input_device;
    ecodrive::Telemetry g_telemetry(g_input_device);

    scs_log_t g_log = nullptr;


    void log_message(const char* message)
    {
        if (g_log)
        {
            g_log(
                SCS_LOG_TYPE_message,
                message);
        }
    }
}


extern "C"
{

    SCSAPI_RESULT scs_input_init(
        const scs_u32_t version,
        const scs_input_init_params_t* const params)
    {
        if (!params)
            return SCS_RESULT_invalid_parameter;


        if (version != SCS_INPUT_VERSION_1_00)
            return SCS_RESULT_unsupported;


        const auto* params_v100 =
            static_cast<const scs_input_init_params_v100_t*>(
                params);


        g_log =
            params_v100->common.log;


        log_message(
            "EcoDrive: initializing SCS Input API.");


        const scs_result_t result =
            g_input_device.register_device(
                params_v100->register_device);


        if (result != SCS_RESULT_ok)
        {
            log_message(
                "EcoDrive: failed to register input device.");

            return result;
        }


        log_message(
            "EcoDrive: semantic input device registered.");


        return SCS_RESULT_ok;
    }


    SCSAPI_VOID scs_input_shutdown()
    {
        log_message(
            "EcoDrive: SCS Input API shutdown.");
    }


    SCSAPI_RESULT scs_telemetry_init(
        const scs_u32_t version,
        const scs_telemetry_init_params_t* const params)
    {
        if (!params)
            return SCS_RESULT_invalid_parameter;


        if (version != SCS_TELEMETRY_VERSION_1_01 &&
            version != SCS_TELEMETRY_VERSION_1_00)
        {
            return SCS_RESULT_unsupported;
        }


        const auto* params_v100 =
            static_cast<const scs_telemetry_init_params_v100_t*>(
                params);


        if (!g_log)
            g_log =
            params_v100->common.log;


        log_message(
            "EcoDrive: initializing SCS Telemetry API.");


        const scs_result_t result =
            g_telemetry.initialize(
                *params_v100);


        if (result != SCS_RESULT_ok)
        {
            log_message(
                "EcoDrive: failed to initialize telemetry.");

            return result;
        }


        log_message(
            "EcoDrive: telemetry channels registered.");


        return SCS_RESULT_ok;
    }


    SCSAPI_VOID scs_telemetry_shutdown()
    {
        g_telemetry.shutdown();


        log_message(
            "EcoDrive: SCS Telemetry API shutdown.");
    }

}