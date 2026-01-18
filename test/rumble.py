import asyncio
from bleak import BleakClient, BleakScanner

DEVICE_NAME  = "Rumble-ESP32"
RUMBLE_UUID  = "12345678-1234-1234-1234-123456789abd"

async def find_device():
    print("Scanning for Rumble-ESP32...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10)
    if not device:
        raise RuntimeError(f'"{DEVICE_NAME}" not found. Is it powered on?')
    print(f"Found: {device.name} [{device.address}]")
    return device

async def rumble(client: BleakClient, amplitude: int, duration_ms: int):
    """
    amplitude  : 0-255
    duration_ms: 0-2550 (rounded down to nearest 10ms)
    """
    amplitude   = max(0, min(255, amplitude))
    dur_tenths  = max(0, min(255, duration_ms // 10))
    packet      = bytes([amplitude, dur_tenths])
    await client.write_gatt_char(RUMBLE_UUID, packet, response=False)
    print(f"Sent rumble: amp={amplitude} dur={dur_tenths * 10}ms")

async def main():
    device = await find_device()

    async with BleakClient(device) as client:
        print(f"Connected: {client.is_connected}")

        # ── Test sequence ──────────────────────────────────────────────────
        print("\n--- Short click ---")
        await rumble(client, amplitude=255, duration_ms=80)
        await asyncio.sleep(0.5)

        print("--- Medium buzz ---")
        await rumble(client, amplitude=200, duration_ms=300)
        await asyncio.sleep(0.6)

        print("--- Long strong rumble ---")
        await rumble(client, amplitude=255, duration_ms=1000)
        await asyncio.sleep(1.5)

        print("--- Stop (explicit zero) ---")
        await rumble(client, amplitude=0, duration_ms=0)

        print("\nDone.")

asyncio.run(main())
