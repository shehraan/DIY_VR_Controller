#pragma once

#include <Arduino.h>

namespace HIDTransport {

struct RumbleCommand {
	uint8_t amplitude;
	uint16_t durationMs;
	uint8_t flags;
};

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

// Returns true when a new rumble command was received from BLE HID output.
// Packet format supported:
// 1) [amplitude][durationMs LSB][durationMs MSB]
// 2) [0x01][amplitude][durationMs LSB][durationMs MSB][flags]
bool pollRumbleCommand(RumbleCommand &outCommand);

}
