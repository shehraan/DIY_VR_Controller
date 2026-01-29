cmake_minimum_required(VERSION 3.20)
project(vr_controller_driver LANGUAGES C CXX)   # C needed for hidapi windows/hid.c

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(DRIVER_NAME   "vr_controller")
set(DRIVER_BINARY "driver_${DRIVER_NAME}")

# ── OpenVR headers ─────────────────────────────────────────────────────────────
# Layout: third_party/openvr/openvr_driver.h
# Source files include it as "openvr/openvr_driver.h", so root must be third_party/.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/openvr/openvr_driver.h")
    set(OPENVR_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
else()
    message(FATAL_ERROR
        "OpenVR headers not found. "
        "Place openvr_driver.h at third_party/openvr/openvr_driver.h")
endif()

# ── hidapi: build Windows HID backend as a static lib (no hidapi.dll) ──────────
# Downloads hidapi source via FetchContent; compiles only windows/hid.c.
# Runtime deps: setupapi.dll + hid.dll – both are Windows system DLLs (always present).
include(FetchContent)
# Silence deprecation of the name-only FetchContent_Populate signature in CMake 3.30+.
if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
endif()
FetchContent_Declare(
    hidapi_src
    GIT_REPOSITORY https://github.com/libusb/hidapi.git
    GIT_TAG        hidapi-0.14.0
    GIT_SHALLOW    TRUE
)
FetchContent_GetProperties(hidapi_src)
if(NOT hidapi_src_POPULATED)
    FetchContent_Populate(hidapi_src)
endif()

add_library(hidapi_static STATIC
    "${hidapi_src_SOURCE_DIR}/windows/hid.c"
)
target_include_directories(hidapi_static
    # PUBLIC: our driver code uses #include "hidapi/hidapi.h"
    PUBLIC  "${hidapi_src_SOURCE_DIR}"
    # PRIVATE: windows/hid.c and its companion headers use #include "hidapi.h"
    PRIVATE "${hidapi_src_SOURCE_DIR}/hidapi"
)
target_link_libraries(hidapi_static PUBLIC setupapi hid)
# Suppress warnings from upstream source
if(MSVC)
    target_compile_options(hidapi_static PRIVATE /W0)
endif()

# ── Driver DLL ─────────────────────────────────────────────────────────────────
add_library(${DRIVER_BINARY} SHARED
    src/driver_main.cpp
    src/driver_log.cpp
    src/hid_transport.cpp
)

target_include_directories(${DRIVER_BINARY}
    PRIVATE
        include
        "${OPENVR_INCLUDE_DIR}"
)

target_compile_definitions(${DRIVER_BINARY}
    PRIVATE
        _CRT_SECURE_NO_WARNINGS
        NOMINMAX
)

target_link_libraries(${DRIVER_BINARY}
    PRIVATE
        hidapi_static   # static Windows HID backend; pulls in setupapi + hid
)

target_compile_options(${DRIVER_BINARY}
    PRIVATE
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

set_target_properties(${DRIVER_BINARY}
    PROPERTIES
        PREFIX ""
        OUTPUT_NAME "${DRIVER_BINARY}"
        RUNTIME_OUTPUT_DIRECTORY         "${CMAKE_CURRENT_SOURCE_DIR}/bin/win64"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CMAKE_CURRENT_SOURCE_DIR}/bin/win64"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/bin/win64"
        LIBRARY_OUTPUT_DIRECTORY         "${CMAKE_CURRENT_SOURCE_DIR}/bin/win64"
        ARCHIVE_OUTPUT_DIRECTORY         "${CMAKE_CURRENT_SOURCE_DIR}/bin/win64"
)

# ── Package: stage BuiltDrivers/vr_controller ──────────────────────────────────
set(BUILT_DRIVERS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/BuiltDrivers/${DRIVER_NAME}")
set(BUILT_DRIVERS_BIN  "${BUILT_DRIVERS_ROOT}/bin/win64")

# Locate the MSVC VC143 CRT DLLs that must ship next to the driver.
# VCToolsRedistDir is set by the VS developer environment (IDE build or x64 Native Tools prompt).
set(_vc_redist_dir "$ENV{VCToolsRedistDir}")
if(_vc_redist_dir)
    file(TO_CMAKE_PATH "${_vc_redist_dir}" _vc_redist_dir)
    string(REGEX REPLACE "[/\\]+$" "" _vc_redist_dir "${_vc_redist_dir}")
    file(GLOB MSVC_CRT_DLLS
        "${_vc_redist_dir}/x64/Microsoft.VC143.CRT/msvcp140.dll"
        "${_vc_redist_dir}/x64/Microsoft.VC143.CRT/msvcp140_1.dll"
        "${_vc_redist_dir}/x64/Microsoft.VC143.CRT/msvcp140_2.dll"
        "${_vc_redist_dir}/x64/Microsoft.VC143.CRT/vcruntime140.dll"
        "${_vc_redist_dir}/x64/Microsoft.VC143.CRT/vcruntime140_1.dll"
        "${_vc_redist_dir}/x64/Microsoft.VC143.CRT/concrt140.dll"
    )
endif()

# Hard-coded fallback: VS 2022 Community, toolset 14.42.34433 (update if VS is updated).
if(NOT MSVC_CRT_DLLS)
    set(_crt_dir
        "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Redist/MSVC/14.42.34433/x64/Microsoft.VC143.CRT"
    )
    file(GLOB MSVC_CRT_DLLS
        "${_crt_dir}/msvcp140.dll"
        "${_crt_dir}/msvcp140_1.dll"
        "${_crt_dir}/msvcp140_2.dll"
        "${_crt_dir}/vcruntime140.dll"
        "${_crt_dir}/vcruntime140_1.dll"
        "${_crt_dir}/concrt140.dll"
    )
endif()

if(MSVC_CRT_DLLS)
    message(STATUS "MSVC CRT DLLs staged: ${MSVC_CRT_DLLS}")
else()
    message(WARNING
        "MSVC CRT DLLs not found. "
        "BuiltDrivers/bin/win64 will be missing the MSVC runtime. "
        "Set VCToolsRedistDir or update the fallback path in CMakeLists.txt.")
endif()

# ── POST_BUILD: assemble the SteamVR-ready package ─────────────────────────────
add_custom_command(TARGET ${DRIVER_BINARY} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${BUILT_DRIVERS_BIN}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:${DRIVER_BINARY}>"
        "${BUILT_DRIVERS_BIN}/"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/driver.vrdrivermanifest"
        "${BUILT_DRIVERS_ROOT}/driver.vrdrivermanifest"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/resources"
        "${BUILT_DRIVERS_ROOT}/resources"
    VERBATIM
)

if(MSVC_CRT_DLLS)
    add_custom_command(TARGET ${DRIVER_BINARY} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${MSVC_CRT_DLLS}
            "${BUILT_DRIVERS_BIN}/"
        VERBATIM
        COMMAND_EXPAND_LISTS
    )
endif()
