#pragma once

#include <Arduino.h>

namespace HIDTransport {

// Initializes the USB HID descriptor used by the Relativty SteamVR driver.
void begin();

// Sends quaternion orientation in Relativty-compatible component order.
void sendQuaternion(float w, float x, float y, float z);

}
