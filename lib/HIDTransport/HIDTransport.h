#pragma once

#include <Arduino.h>

namespace HIDTransport {

// Initializes HID transport.
// ESP32 uses BLE HID.
// On unsupported targets this becomes a no-op.
void begin();

// Returns true when a BLE central is connected and notifications are enabled.
bool ready();

// Sends quaternion orientation in Controller-compatible component order.
void sendQuaternion(float w, float x, float y, float z);

}
