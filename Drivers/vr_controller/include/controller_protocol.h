#pragma once

#include <cstddef>
#include <cstdint>

namespace vr_controller {

// Firmware BLE PnP (see lib/HIDTransport/HIDTransport.cpp hid->pnp). Windows hidapi may report
// byte-swapped values (e.g. 0x02E5 / 0x11A1); HidTransport accepts either order.
constexpr uint16_t kVendorId = 0xE502;
constexpr uint16_t kProductId = 0xA111;
constexpr const char* kProductName = "ESP32-Controller";

constexpr uint8_t kInputReportId = 0x01;
constexpr uint8_t kOutputReportId = 0x02;
constexpr size_t kInputPayloadBytes = 63;

constexpr uint8_t kButtonA = 1u << 0;
constexpr uint8_t kButtonB = 1u << 1;
constexpr uint8_t kButtonStickClick = 1u << 2;

struct ControllerReport {
    float quat_w = 1.0f;
    float quat_y = 0.0f;
    float quat_z = 0.0f;
    float quat_x = 0.0f;
    uint8_t buttons = 0;
    uint16_t joystick_x = 0;
    uint16_t joystick_y = 0;
    uint32_t send_time_us = 0;
    uint32_t packet_id = 0;
};

} // namespace vr_controller
