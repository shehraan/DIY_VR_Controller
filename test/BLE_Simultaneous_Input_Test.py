import hid
import struct
import time

READ_REPORT_SIZE = 64
PRODUCT_HINT = "ESP32-Controller"
VID_HINT = 0xE502
PID_HINT = 0xA111
LOG_INTERVAL_S = 1.0


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


def _decode_current_layout(payload):
    # Current firmware layout (HIDTransport.cpp + main.cpp):
    #   0..3   w (float)
    #   4..7   y (float)
    #   8..11  z (float)
    #   12..15 x (float)
    #   16     buttons
    #   17..18 joystickX
    #   19..20 joystickY
    #   21..24 sendTimeUs
    #   25..28 packetId
    if len(payload) < 29:
        return None

    w, y, z, x = struct.unpack("<ffff", bytes(payload[0:16]))
    buttons = payload[16]
    joy_x = int.from_bytes(bytes(payload[17:19]), byteorder="little", signed=False)
    joy_y = int.from_bytes(bytes(payload[19:21]), byteorder="little", signed=False)
    send_time_us = int.from_bytes(bytes(payload[21:25]), byteorder="little", signed=False)
    packet_id = int.from_bytes(bytes(payload[25:29]), byteorder="little", signed=False)
    return {
        "layout": "current",
        "w": w,
        "x": x,
        "y": y,
        "z": z,
        "buttons": buttons,
        "button_a": buttons & 0x01,
        "joy_x": joy_x,
        "joy_y": joy_y,
        "send_time_us": send_time_us,
        "packet_id": packet_id,
    }


def _decode_disabled_layout(payload):
    # Older test firmware layout (main.cpp.disabled):
    #   0..3   w (float)
    #   4..7   y (float)
    #   8..11  z (float)
    #   12..15 x (float)
    #   16     button
    #   20..23 sendTimeUs
    #   24..27 packetId
    if len(payload) < 28:
        return None

    w, y, z, x = struct.unpack("<ffff", bytes(payload[0:16]))
    button = payload[16] & 0x01
    send_time_us = int.from_bytes(bytes(payload[20:24]), byteorder="little", signed=False)
    packet_id = int.from_bytes(bytes(payload[24:28]), byteorder="little", signed=False)
    return {
        "layout": "disabled",
        "w": w,
        "x": x,
        "y": y,
        "z": z,
        "buttons": button,
        "button_a": button,
        "joy_x": 0,
        "joy_y": 0,
        "send_time_us": send_time_us,
        "packet_id": packet_id,
    }


def parse_input_report(data):
    if not data:
        return None

    payload = data
    if payload[0] == 0x01:
        payload = payload[1:]

    current = _decode_current_layout(payload)
    disabled = _decode_disabled_layout(payload)

    if current is None and disabled is None:
        return None
    if current is None:
        return disabled
    if disabled is None:
        return current

    # Prefer the current layout when joystick bytes look non-trivial.
    if current["joy_x"] != 0 or current["joy_y"] != 0:
        return current
    # Otherwise keep backward compatibility for older test firmware.
    return disabled


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
            _ = written
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
    print("[TEST] Simultaneous test: periodic rumble TX + continuous input RX (Ctrl+C to stop).")

    last_buttons = None
    last_joy_x = None
    last_joy_y = None
    last_packet_id = None
    last_rx_ts = time.monotonic()
    last_wait_log_ts = time.monotonic()
    last_status_log_ts = 0.0
    rx_count = 0
    last_rumble_ts = 0.0
    last_rumble_log_ts = 0.0
    latest_sample = None
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
                elif (now - last_rumble_log_ts) >= LOG_INTERVAL_S:
                    print(f"[TX] amp={amp} dur={dur}ms")
                    last_rumble_log_ts = now
                last_rumble_ts = now

            raw = dev.read(READ_REPORT_SIZE)
            if raw:
                last_rx_ts = now
                parsed = parse_input_report(raw)
                if parsed is None:
                    print(f"[RX] short/unknown report len={len(raw)} data={bytes(raw).hex()}")
                    time.sleep(0.005)
                    continue

                rx_count += 1
                packet_id = parsed["packet_id"]
                latest_sample = parsed
                if last_packet_id is None:
                    last_packet_id = packet_id
                last_packet_id = packet_id

                buttons = parsed["buttons"]
                if buttons != last_buttons:
                    last_buttons = buttons

                # Log joystick whenever it meaningfully changes (or first sample).
                joy_x = parsed["joy_x"]
                joy_y = parsed["joy_y"]
                joy_changed = (
                    last_joy_x is None
                    or last_joy_y is None
                    or abs(joy_x - last_joy_x) >= 8
                    or abs(joy_y - last_joy_y) >= 8
                )
                if joy_changed:
                    last_joy_x = joy_x
                    last_joy_y = joy_y

                if latest_sample is not None and (now - last_status_log_ts) >= LOG_INTERVAL_S:
                    s = latest_sample
                    a_state = "1" if (s["buttons"] & 0x01) else "0"
                    b_state = "1" if (s["buttons"] & 0x02) else "0"
                    j_state = "1" if (s["buttons"] & 0x04) else "0"
                    print(
                        f"[STATE] layout={s['layout']} packet={s['packet_id']} "
                        f"btn(A,B,J)=({a_state},{b_state},{j_state}) "
                        f"joy=({s['joy_x']},{s['joy_y']}) "
                        f"q=({s['w']:+.4f},{s['x']:+.4f},{s['y']:+.4f},{s['z']:+.4f})"
                    )
                    last_status_log_ts = now

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
