import hid
import time

READ_REPORT_SIZE = 64
PRODUCT_HINT = "ESP32-Controller"
VID_HINT = 0xE502
PID_HINT = 0xA111


def find_device_path(product_hint: str = PRODUCT_HINT, vid_hint: int = VID_HINT, pid_hint: int = PID_HINT):
    devices = hid.enumerate()
    if not devices:
        raise RuntimeError("hid.enumerate() returned no devices.")

    print(f"[HID] Enumerated {len(devices)} HID devices")

    # Pass 1: Prefer explicit product name match.
    for dev in devices:
        product = dev.get("product_string") or ""
        if product_hint.lower() in product.lower():
            print(
                f"[HID] Found by name: product='{product}' "
                f"vid=0x{dev.get('vendor_id', 0):04X} pid=0x{dev.get('product_id', 0):04X}"
            )
            return dev["path"]

    # Pass 2: Fallback to VID/PID from firmware pnp(0x02, 0xE502, 0xA111, ...).
    for dev in devices:
        vid = int(dev.get("vendor_id") or 0)
        pid = int(dev.get("product_id") or 0)
        if vid == vid_hint and pid == pid_hint:
            product = dev.get("product_string") or ""
            print(
                f"[HID] Found by VID/PID: product='{product}' "
                f"vid=0x{vid:04X} pid=0x{pid:04X}"
            )
            return dev["path"]

    print("[HID] Candidate devices (vid/pid/product/manufacturer):")
    for dev in devices:
        vid = int(dev.get("vendor_id") or 0)
        pid = int(dev.get("product_id") or 0)
        product = dev.get("product_string") or ""
        manufacturer = dev.get("manufacturer_string") or ""
        print(f"  - 0x{vid:04X}:0x{pid:04X} product='{product}' manufacturer='{manufacturer}'")

    raise RuntimeError(
        "No matching HID device found. Pair the controller in Windows Bluetooth as HID, "
        "then rerun this script."
    )


def parse_input_report(data):
    if not data:
        return None

    payload = data
    if payload[0] == 0x01:
        payload = payload[1:]

    # Firmware layout:
    #   bytes 0..15: quaternion floats
    #   byte 16: button state
    #   bytes 20..23: sendTimeUs
    #   bytes 24..27: packetId
    if len(payload) < 28:
        return None

    button = payload[16] & 0x01
    packet_id = int.from_bytes(bytes(payload[24:28]), byteorder="little", signed=False)
    return button, packet_id


def write_rumble(dev: hid.device, amp: int, dur_ms: int) -> bool:
    dur_lsb = dur_ms & 0xFF
    dur_msb = (dur_ms >> 8) & 0xFF

    packets = [
        bytes([0x02, amp & 0xFF, dur_lsb, dur_msb, 0x00]),
        bytes([0x02, amp & 0xFF, dur_lsb, dur_msb]),
        bytes([0x02, amp & 0xFF, max(0, min(255, dur_ms // 10))]),
    ]

    for index, payload in enumerate(packets, start=1):
        try:
            written = dev.write(payload)
            print(
                f"[TX] format#{index} amp={amp} dur={dur_ms}ms "
                f"wrote={written} payload={payload.hex(' ')}"
            )
            return True
        except Exception as ex:
            print(f"[WARN] format#{index} failed: {ex}")

    return False


def main():
    path = find_device_path()
    dev = hid.device()
    dev.open_path(path)
    dev.set_nonblocking(True)

    print("[HID] Opened BLE HID device.")
    print("[TEST] Simultaneous test: periodic rumble TX + continuous button RX (Ctrl+C to stop).")

    last_button = None
    last_packet_id = None
    last_rx_ts = time.monotonic()
    last_wait_log_ts = time.monotonic()
    last_rumble_ts = 0.0
    rumble_period_s = 0.25
    rumble_pattern = [
        (220, 120),
        (0, 0),
        (140, 80),
        (0, 0),
    ]
    rumble_index = 0

    try:
        while True:
            now = time.monotonic()

            if (now - last_rumble_ts) >= rumble_period_s:
                amp, dur = rumble_pattern[rumble_index % len(rumble_pattern)]
                rumble_index += 1
                if not write_rumble(dev, amp, dur):
                    print("[ERR] Failed to send periodic rumble packet.")
                last_rumble_ts = now

            raw = dev.read(READ_REPORT_SIZE)
            if raw:
                last_rx_ts = now
                parsed = parse_input_report(raw)
                if parsed is None:
                    print(f"[RX] short/unknown report len={len(raw)} data={bytes(raw).hex()}")
                    time.sleep(0.005)
                    continue

                button, packet_id = parsed
                if last_packet_id is None:
                    last_packet_id = packet_id

                if button != last_button:
                    state = "PRESSED" if button else "RELEASED"
                    print(f"[RX] button={button} ({state}) packet={packet_id}")

                    last_button = button

            if (now - last_rx_ts) > 2.0 and (now - last_wait_log_ts) > 2.0:
                print("[WAIT] Connected, but no input reports received yet...")
                last_wait_log_ts = now

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n[TEST] Stopped by user.")
    finally:
        dev.close()
        print("[HID] Device closed.")


if __name__ == "__main__":
    main()
