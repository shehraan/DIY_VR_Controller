import math
import struct
import time
from dataclasses import dataclass

import hid

READ_REPORT_SIZE = 64
PRODUCT_HINT = "ESP32-Controller"
VID_HINT = 0xE502
PID_HINT = 0xA111
POLL_SLEEP_S = 0.005

# Bit mapping from firmware
BUTTON_A_MASK = 0x01
BUTTON_B_MASK = 0x02
JOYSTICK_BUTTON_MASK = 0x04


@dataclass
class DecodedReport:
    layout: str
    w: float
    x: float
    y: float
    z: float
    buttons: int
    joy_x: int
    joy_y: int
    send_time_us: int
    packet_id: int


@dataclass
class TestResult:
    name: str
    passed: bool
    detail: str


def find_device_path(product_hint: str = PRODUCT_HINT, vid_hint: int = VID_HINT, pid_hint: int = PID_HINT):
    devices = hid.enumerate()
    if not devices:
        raise RuntimeError("hid.enumerate() returned no devices.")

    print(f"[HID] Enumerated {len(devices)} HID devices")

    for dev in devices:
        product = dev.get("product_string") or ""
        if product_hint.lower() in product.lower():
            print(
                f"[HID] Found by name: product='{product}' "
                f"vid=0x{dev.get('vendor_id', 0):04X} pid=0x{dev.get('product_id', 0):04X}"
            )
            return dev["path"]

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
        "No matching HID device found. Pair controller in Windows as BLE HID, then rerun."
    )


def _decode_current_layout(payload):
    # Current firmware layout:
    # 0..3 w, 4..7 y, 8..11 z, 12..15 x
    # 16 buttons, 17..18 joyX, 19..20 joyY, 21..24 sendTimeUs, 25..28 packetId
    if len(payload) < 29:
        return None

    w, y, z, x = struct.unpack("<ffff", bytes(payload[0:16]))
    buttons = payload[16]
    joy_x = int.from_bytes(bytes(payload[17:19]), byteorder="little", signed=False)
    joy_y = int.from_bytes(bytes(payload[19:21]), byteorder="little", signed=False)
    send_time_us = int.from_bytes(bytes(payload[21:25]), byteorder="little", signed=False)
    packet_id = int.from_bytes(bytes(payload[25:29]), byteorder="little", signed=False)

    return DecodedReport(
        layout="current",
        w=w,
        x=x,
        y=y,
        z=z,
        buttons=buttons,
        joy_x=joy_x,
        joy_y=joy_y,
        send_time_us=send_time_us,
        packet_id=packet_id,
    )


def _decode_disabled_layout(payload):
    # Backward-compatible layout:
    # 0..3 w, 4..7 y, 8..11 z, 12..15 x
    # 16 buttonA only, 20..23 sendTimeUs, 24..27 packetId
    if len(payload) < 28:
        return None

    w, y, z, x = struct.unpack("<ffff", bytes(payload[0:16]))
    button_a = payload[16] & 0x01
    send_time_us = int.from_bytes(bytes(payload[20:24]), byteorder="little", signed=False)
    packet_id = int.from_bytes(bytes(payload[24:28]), byteorder="little", signed=False)

    return DecodedReport(
        layout="disabled",
        w=w,
        x=x,
        y=y,
        z=z,
        buttons=button_a,
        joy_x=0,
        joy_y=0,
        send_time_us=send_time_us,
        packet_id=packet_id,
    )


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

    if current.joy_x != 0 or current.joy_y != 0:
        return current
    return disabled


def read_reports_for(dev: hid.device, seconds: float):
    samples = []
    start = time.monotonic()
    while (time.monotonic() - start) < seconds:
        raw = dev.read(READ_REPORT_SIZE)
        if raw:
            parsed = parse_input_report(raw)
            if parsed is not None:
                samples.append(parsed)
        time.sleep(POLL_SLEEP_S)
    return samples


def wait_for_button_transition(dev: hid.device, mask: int, desired_pressed: bool, timeout_s: float):
    end_at = time.monotonic() + timeout_s
    want = 1 if desired_pressed else 0

    while time.monotonic() < end_at:
        raw = dev.read(READ_REPORT_SIZE)
        if raw:
            parsed = parse_input_report(raw)
            if parsed is None:
                continue
            got = 1 if (parsed.buttons & mask) else 0
            if got == want:
                return True
        time.sleep(POLL_SLEEP_S)

    return False


def test_button(dev: hid.device, mask: int, label: str) -> TestResult:
    print()
    print(f"[ACTION] {label}: release now, then press and hold, then release.")

    released = wait_for_button_transition(dev, mask, desired_pressed=False, timeout_s=4.0)
    pressed = wait_for_button_transition(dev, mask, desired_pressed=True, timeout_s=6.0)
    released_again = wait_for_button_transition(dev, mask, desired_pressed=False, timeout_s=6.0)

    passed = released and pressed and released_again
    detail = f"released={released}, pressed={pressed}, released_again={released_again}"
    return TestResult(label, passed, detail)


def test_axis(dev: hid.device, axis_name: str, axis_getter, span_threshold: int = 1000) -> TestResult:
    print()
    print(f"[ACTION] Move {axis_name} fully both directions for 4 seconds...")

    samples = read_reports_for(dev, 4.0)
    if not samples:
        return TestResult(f"{axis_name} axis", False, "no reports received")

    vals = [axis_getter(s) for s in samples]
    min_v = min(vals)
    max_v = max(vals)
    span = max_v - min_v
    passed = span >= span_threshold
    detail = f"min={min_v}, max={max_v}, span={span}, need>={span_threshold}"
    return TestResult(f"{axis_name} axis", passed, detail)


def quat_distance(a: DecodedReport, b: DecodedReport) -> float:
    return math.sqrt((a.w - b.w) ** 2 + (a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def test_imu_motion(dev: hid.device) -> TestResult:
    print()
    print("[ACTION] Rotate/move controller for 4 seconds...")

    samples = read_reports_for(dev, 4.0)
    if len(samples) < 5:
        return TestResult("IMU motion", False, "not enough quaternion samples")

    base = samples[0]
    max_dist = max(quat_distance(base, s) for s in samples[1:])
    passed = max_dist >= 0.08
    detail = f"max_quat_delta={max_dist:.4f}, need>=0.0800"
    return TestResult("IMU motion", passed, detail)


def write_rumble(dev: hid.device, amp: int, dur_ms: int) -> bool:
    dur_lsb = dur_ms & 0xFF
    dur_msb = (dur_ms >> 8) & 0xFF

    packets = [
        bytes([0x02, amp & 0xFF, dur_lsb, dur_msb, 0x00]),
        bytes([0x02, amp & 0xFF, dur_lsb, dur_msb]),
        bytes([0x02, amp & 0xFF, max(0, min(255, dur_ms // 10))]),
    ]

    for payload in packets:
        try:
            dev.write(payload)
            return True
        except Exception:
            continue

    return False


def test_rumble(dev: hid.device) -> TestResult:
    print()
    print("[ACTION] Rumble test: sending short burst pattern...")

    pattern = [(220, 120), (0, 0), (180, 100), (0, 0)]
    all_sent = True
    for amp, dur in pattern:
        ok = write_rumble(dev, amp, dur)
        all_sent = all_sent and ok
        time.sleep(0.20)

    print("Did you feel rumble? Type y or n and press Enter: ", end="", flush=True)
    answer = input().strip().lower()
    user_felt = answer in ("y", "yes")

    passed = all_sent and user_felt
    detail = f"packets_sent={all_sent}, user_confirmed={user_felt}"
    return TestResult("Rumble", passed, detail)


def test_report_flow(dev: hid.device) -> TestResult:
    print("[CHECK] Waiting for BLE input reports...")
    samples = read_reports_for(dev, 2.5)
    if not samples:
        return TestResult("BLE input stream", False, "no input reports")

    first = samples[0]
    last = samples[-1]
    delta_packets = last.packet_id - first.packet_id
    rate_hz = len(samples) / 2.5
    detail = f"samples={len(samples)}, packet_delta={delta_packets}, approx_rate={rate_hz:.1f}Hz, layout={first.layout}"
    return TestResult("BLE input stream", True, detail)


def print_summary(results):
    print()
    print("============== BLE TEST SUMMARY ==============")
    passed_count = 0
    for r in results:
        status = "PASS" if r.passed else "FAIL"
        print(f"[{status}] {r.name}: {r.detail}")
        if r.passed:
            passed_count += 1

    print("----------------------------------------------")
    print(f"Overall: {passed_count}/{len(results)} tests passed")


def main():
    path = find_device_path()
    dev = hid.device()
    dev.open_path(path)
    dev.set_nonblocking(True)

    print("[HID] Opened BLE HID device.")
    print("[TEST] This checks: BLE stream, ButtonA, ButtonB, Joystick button, Joy X, Joy Y, IMU motion, rumble.")

    results = []
    try:
        results.append(test_report_flow(dev))
        results.append(test_button(dev, BUTTON_A_MASK, "Button A"))
        results.append(test_button(dev, BUTTON_B_MASK, "Button B"))
        results.append(test_button(dev, JOYSTICK_BUTTON_MASK, "Joystick Button"))
        results.append(test_axis(dev, "Joystick X", lambda s: s.joy_x))
        results.append(test_axis(dev, "Joystick Y", lambda s: s.joy_y))
        results.append(test_imu_motion(dev))
        results.append(test_rumble(dev))

        print_summary(results)
    except KeyboardInterrupt:
        print("\n[TEST] Stopped by user.")
    finally:
        dev.close()
        print("[HID] Device closed.")


if __name__ == "__main__":
    main()
