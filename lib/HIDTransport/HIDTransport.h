#pragma once

#include <Arduino.h>

namespace HIDTransport {

// Initializes HID transport.
// ESP32 uses BLE HID.
// On unsupported targets this becomes a no-op.
void begin();

// Returns true when a BLE central is connected.
bool ready();

// Sends quaternion orientation and optional controller inputs.
// buttons bitmask: bit0=A, bit1=B, bit2=JoystickButton.
void sendQuaternion(
	float w,
	float x,
	float y,
	float z,
	uint8_t buttons = 0,
	uint16_t joystickX = 0,
	uint16_t joystickY = 0
);

}
