cmake_minimum_required(VERSION 3.20)

project(EcoDrive LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(ecodrive SHARED
    src/plugin.cpp
    src/input_device.cpp
    src/telemetry.cpp
    src/config.cpp
    src/trip_tracker.cpp
    src/adaptive_learner.cpp
    src/coasting_controller.cpp
    src/shifter.cpp
)

target_include_directories(ecodrive PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/sdk
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(MSVC)
    target_compile_definitions(ecodrive PRIVATE
        _CRT_SECURE_NO_WARNINGS
        WIN32_LEAN_AND_MEAN
        NOMINMAX
    )

    target_link_options(ecodrive PRIVATE
        "/DEF:${CMAKE_CURRENT_SOURCE_DIR}/ecodrive.def"
    )
endif()

set_target_properties(ecodrive PROPERTIES
    OUTPUT_NAME "ecodrive"
    PREFIX ""
)
add_compile_definitions(
    WIN32 _WINDOWS _USRDLL UNICODE _UNICODE NOMINMAX
    SCS_PLUGIN_EXPORTS
    _CRT_SECURE_NO_WARNINGS
    _WIN32_WINNT=0x0601
)

if (MSVC)
    # Force the legacy preprocessor; the SCS SDK headers predate /Zc:preprocessor.
    add_compile_options(/Zc:preprocessor-)
    add_compile_options(/permissive)
endif()