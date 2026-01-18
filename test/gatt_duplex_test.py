import asyncio
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "ESP32-Controller"
RUMBLE_WRITE_UUID = "12345678-1234-1234-1234-123456789abd"
BUTTON_READ_UUID = "12345678-1234-1234-1234-123456789abe"


async def find_device():
    print(f"Scanning for {DEVICE_NAME}...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15)
    if not device:
        raise RuntimeError(f"{DEVICE_NAME} not found")
    print(f"Found: {device.name} [{device.address}]")
    return device


async def main():
    device = await find_device()

    async with BleakClient(device) as client:
        print(f"Connected: {client.is_connected}")
        print("Sending rumble once per second and polling button state...")
        sequence = [(180, 120), (255, 250), (120, 80), (0, 0)]
        index = 0
        last_button = None
        next_rumble_at = 0.0

        while True:
            now = asyncio.get_running_loop().time()

            if now >= next_rumble_at:
                amp, dur_ms = sequence[index % len(sequence)]
                index += 1
                dur_tenths = max(0, min(255, dur_ms // 10))
                packet = bytes([amp, dur_tenths])
                await client.write_gatt_char(RUMBLE_WRITE_UUID, packet, response=False)
                print(f"TX rumble amp={amp} dur={dur_tenths * 10}ms")
                next_rumble_at = now + 1.0

            raw = await client.read_gatt_char(BUTTON_READ_UUID)
            if len(raw) >= 1:
                button = 1 if raw[0] else 0
                if button != last_button:
                    state = "PRESSED" if button == 1 else "RELEASED"
                    print(f"RX button={button} ({state})")
                    last_button = button

            await asyncio.sleep(0.05)


if __name__ == "__main__":
    asyncio.run(main())
