import struct
import time

import hid


INPUT_REPORT_ID = 0x01
OUTPUT_REPORT_ID = 0x02
HID_READ_LEN = 64
HID_READ_TIMEOUT_MS = 10

TEST_DURATION_SEC = 20.0
RUMBLE_PERIOD_SEC = 1.2
RUMBLE_ON_MS = 220
RUMBLE_AMPLITUDE = 220
NO_REPORT_WARNING_SEC = 3.0

# Optional hardcoded filter. Set to part of the HID path string to force one device.
# Example: DEVICE_PATH_CONTAINS = "BTHLEDevice"
DEVICE_PATH_CONTAINS = None

# Button activity checks (this firmware reports bit0 as the button state).
REQUIRE_BUTTON_ACTIVITY = True
MIN_BUTTON_TRANSITIONS = 1
MIN_BUTTON_PRESSED_REPORTS = 1
MIN_BUTTON_RELEASED_REPORTS = 1

PROBE_EACH_PATH_SEC = 1.0
PROBE_READ_TIMEOUT_MS = 20

# Pass/fail thresholds
MIN_VALID_REPORTS = 100
MAX_SHORT_REPORTS = 20
MAX_UNKNOWN_REPORTS = 20
MAX_INTER_REPORT_GAP_MS = 800.0
MIN_RUMBLE_STARTS = 8


def find_device_path():
    candidates = []
    for d in hid.enumerate():
        product = d.get("product_string") or ""
        manufacturer = d.get("manufacturer_string") or ""
        if any(key in product for key in ("Controller", "ESP32-Controller", "ESP32")):
            candidates.append(d)
        elif "Controller" in manufacturer:
            candidates.append(d)

    if not candidates:
        raise RuntimeError(
            "No matching HID device found. Ensure the ESP32 is paired/connected and using HIDTransport::begin()."
        )

    print("Candidate HID devices:")
    for i, c in enumerate(candidates, start=1):
        print(
            f"  [{i}] product={c.get('product_string')} manufacturer={c.get('manufacturer_string')} "
            f"path={c.get('path')}"
        )

    if DEVICE_PATH_CONTAINS:
        for c in candidates:
            path_s = str(c.get("path"))
            if DEVICE_PATH_CONTAINS in path_s:
                print(f"Using hardcoded path match: {DEVICE_PATH_CONTAINS}")
                chosen = c
                break
        else:
            raise RuntimeError(
                f"DEVICE_PATH_CONTAINS='{DEVICE_PATH_CONTAINS}' did not match any candidate path."
            )
    else:
        ranked = sorted(
            candidates,
            key=lambda c: (
                0 if c.get("usage_page") == 0xFF00 else 1,
                0 if c.get("usage") == 0x0001 else 1,
                0 if "BTHLEDevice" in str(c.get("path")) else 1,
                0 if "COL01" in str(c.get("path")) else 1,
            ),
        )
        chosen = ranked[0]

    print("Matched HID device:")
    print(f"  product      : {chosen.get('product_string')}")
    print(f"  manufacturer : {chosen.get('manufacturer_string')}")
    print(f"  path         : {chosen.get('path')}")
    return chosen["path"]


def probe_path_for_input(path):
    dev = hid.device()
    reads = 0
    parsed_ok = 0
    short = 0
    unknown = 0
    start = time.perf_counter()

    try:
        dev.open_path(path)
        dev.set_nonblocking(False)

        # Try to trigger traffic while probing.
        try:
            send_rumble(dev, amplitude=120, duration_ms=80, flags=0)
            send_rumble(dev, amplitude=0, duration_ms=0, flags=1)
        except Exception:
            pass

        while (time.perf_counter() - start) < PROBE_EACH_PATH_SEC:
            raw = dev.read(HID_READ_LEN, PROBE_READ_TIMEOUT_MS)
            if not raw:
                continue
            reads += 1
            parsed = parse_input_report(raw)
            if parsed == "short":
                short += 1
            elif isinstance(parsed, dict) and parsed.get("kind") == "ok":
                parsed_ok += 1
            else:
                unknown += 1
    finally:
        try:
            dev.close()
        except Exception:
            pass

    return {
        "reads": reads,
        "parsed_ok": parsed_ok,
        "short": short,
        "unknown": unknown,
    }


def select_best_path(candidates):
    best = None
    best_score = (-1, -1)

    print("\nProbing candidate paths for incoming input reports...")
    for c in candidates:
        path = c["path"]
        info = probe_path_for_input(path)
        score = (info["parsed_ok"], info["reads"])
        print(
            "[PROBE] "
            f"path={path} parsed_ok={info['parsed_ok']} reads={info['reads']} "
            f"short={info['short']} unknown={info['unknown']}"
        )
        if score > best_score:
            best_score = score
            best = c

    return best, best_score


def find_device_path_with_probe():
    candidates = []
    for d in hid.enumerate():
        product = d.get("product_string") or ""
        manufacturer = d.get("manufacturer_string") or ""
        if any(key in product for key in ("Controller", "ESP32-Controller", "ESP32")):
            candidates.append(d)
        elif "Controller" in manufacturer:
            candidates.append(d)

    if not candidates:
        raise RuntimeError(
            "No matching HID device found. Ensure the ESP32 is paired/connected and using HIDTransport::begin()."
        )

    print("Candidate HID devices:")
    for i, c in enumerate(candidates, start=1):
        print(
            f"  [{i}] product={c.get('product_string')} manufacturer={c.get('manufacturer_string')} "
            f"usage_page={c.get('usage_page')} usage={c.get('usage')} path={c.get('path')}"
        )

    if DEVICE_PATH_CONTAINS:
        for c in candidates:
            path_s = str(c.get("path"))
            if DEVICE_PATH_CONTAINS in path_s:
                print(f"Using hardcoded path match: {DEVICE_PATH_CONTAINS}")
                return c["path"]
        raise RuntimeError(
            f"DEVICE_PATH_CONTAINS='{DEVICE_PATH_CONTAINS}' did not match any candidate path."
        )

    ranked = sorted(
        candidates,
        key=lambda c: (
            0 if c.get("usage_page") == 0xFF00 else 1,
            0 if c.get("usage") == 0x0001 else 1,
            0 if "BTHLEDevice" in str(c.get("path")) else 1,
            0 if "COL01" in str(c.get("path")) else 1,
        ),
    )

    best, best_score = select_best_path(ranked)
    if best is None:
        raise RuntimeError("No HID path could be selected.")

    if best_score[0] <= 0 and best_score[1] <= 0:
        print(
            "[WARN] Probe saw no incoming reports on any candidate path. "
            "Will use top-ranked path and continue with full test."
        )
        best = ranked[0]

    print("Matched HID device (selected):")
    print(f"  product      : {best.get('product_string')}")
    print(f"  manufacturer : {best.get('manufacturer_string')}")
    print(f"  usage_page   : {best.get('usage_page')}")
    print(f"  usage        : {best.get('usage')}")
    print(f"  path         : {best.get('path')}")
    return best["path"]


def parse_input_report(raw_report):
    def unpack_layout(buf, offset):
        w, y, z, x = struct.unpack_from("<ffff", buf, offset)
        buttons = buf[offset + 16]
        joystick_x = struct.unpack_from("<H", buf, offset + 17)[0]
        joystick_y = struct.unpack_from("<H", buf, offset + 19)[0]
        quat_mag = (w * w + x * x + y * y + z * z) ** 0.5
        return {
            "w": w,
            "x": x,
            "y": y,
            "z": z,
            "buttons": buttons,
            "joystick_x": joystick_x,
            "joystick_y": joystick_y,
            "quat_mag": quat_mag,
        }

    payload = bytes(raw_report)
    if not payload:
        return None

    had_report_id = False
    if payload[0] == INPUT_REPORT_ID:
        had_report_id = True
        payload = payload[1:]

    # Some host stacks may prepend an extra 0x00 slot before the report ID payload.
    if len(payload) >= 64 and payload[0] == 0x00:
        payload = payload[1:]

    # main.cpp layout via HIDTransport::sendQuaternion:
    # [0..15] = float w,y,z,x
    # [16]    = buttons
    # [17..18]= joystickX (LE)
    # [19..20]= joystickY (LE)
    if len(payload) < 21:
        return "short"

    decoded = unpack_layout(payload, 0)

    # If the main decode does not look sane, try one-byte realignment.
    if not (0.7 <= decoded["quat_mag"] <= 1.3) and len(payload) >= 22:
        alt = unpack_layout(payload, 1)
        if 0.7 <= alt["quat_mag"] <= 1.3:
            decoded = alt

    return {
        "kind": "ok",
        "report_id_present": had_report_id,
        "w": decoded["w"],
        "x": decoded["x"],
        "y": decoded["y"],
        "z": decoded["z"],
        "buttons": decoded["buttons"],
        "joystick_x": decoded["joystick_x"],
        "joystick_y": decoded["joystick_y"],
        "quat_mag": decoded["quat_mag"],
    }


def send_rumble(dev, amplitude=220, duration_ms=250, flags=0):
    amplitude = max(0, min(255, int(amplitude)))
    duration_ms = max(0, min(65535, int(duration_ms)))
    flags = max(0, min(255, int(flags)))

    duration_lsb = duration_ms & 0xFF
    duration_msb = (duration_ms >> 8) & 0xFF

    # Matches HIDTransport parser path: [report_id=0x02, amp, dur_lsb, dur_msb, flags]
    packet = bytes([OUTPUT_REPORT_ID, amplitude, duration_lsb, duration_msb, flags])
    written = dev.write(packet)

    print(
        f"[RUMBLE] written={written} id={OUTPUT_REPORT_ID:#04x} "
        f"amp={amplitude} dur={duration_ms}ms flags={flags:#04x}"
    )


def print_summary(stats):
    duration = max(0.001, stats["end_time"] - stats["start_time"])
    valid_rate = stats["valid_reports"] / duration
    avg_gap_ms = (stats["sum_gap_ms"] / stats["gap_count"]) if stats["gap_count"] else 0.0

    print("\n=== Stress Test Summary ===")
    print(f"Duration                 : {duration:.2f} s")
    print(f"Valid reports            : {stats['valid_reports']}")
    print(f"Short reports            : {stats['short_reports']}")
    print(f"Unknown reports          : {stats['unknown_reports']}")
    print(f"Report rate              : {valid_rate:.2f} reports/s")
    print(f"Average inter-report gap : {avg_gap_ms:.2f} ms")
    print(f"Max inter-report gap     : {stats['max_gap_ms']:.2f} ms")
    print(f"Rumble starts sent       : {stats['rumble_starts']}")
    print(f"Rumble stops sent        : {stats['rumble_stops']}")
    print(f"Raw reports seen         : {stats['raw_reports']}")
    print(f"Empty reads (timeouts)   : {stats['empty_reads']}")
    print(f"Button pressed reports   : {stats['button_pressed_reports']}")
    print(f"Button released reports  : {stats['button_released_reports']}")
    print(f"Button transitions       : {stats['button_transitions']}")

    checks = [
        ("valid_reports >= MIN_VALID_REPORTS", stats["valid_reports"] >= MIN_VALID_REPORTS),
        ("short_reports <= MAX_SHORT_REPORTS", stats["short_reports"] <= MAX_SHORT_REPORTS),
        ("unknown_reports <= MAX_UNKNOWN_REPORTS", stats["unknown_reports"] <= MAX_UNKNOWN_REPORTS),
        ("max_gap_ms <= MAX_INTER_REPORT_GAP_MS", stats["max_gap_ms"] <= MAX_INTER_REPORT_GAP_MS),
        ("rumble_starts >= MIN_RUMBLE_STARTS", stats["rumble_starts"] >= MIN_RUMBLE_STARTS),
    ]

    if REQUIRE_BUTTON_ACTIVITY:
        checks.extend(
            [
                (
                    "button_transitions >= MIN_BUTTON_TRANSITIONS",
                    stats["button_transitions"] >= MIN_BUTTON_TRANSITIONS,
                ),
                (
                    "button_pressed_reports >= MIN_BUTTON_PRESSED_REPORTS",
                    stats["button_pressed_reports"] >= MIN_BUTTON_PRESSED_REPORTS,
                ),
                (
                    "button_released_reports >= MIN_BUTTON_RELEASED_REPORTS",
                    stats["button_released_reports"] >= MIN_BUTTON_RELEASED_REPORTS,
                ),
            ]
        )

    print("\nThreshold checks:")
    all_pass = True
    for label, ok in checks:
        state = "PASS" if ok else "FAIL"
        print(f"  {state}  {label}")
        all_pass = all_pass and ok

    print(f"\nOverall result: {'PASS' if all_pass else 'FAIL'}")
    return all_pass


def main():
    path = find_device_path_with_probe()

    print("\nOpening HID device...")
    dev = hid.device()
    dev.open_path(path)
    dev.set_nonblocking(False)

    try:
        print("\nStarting continuous read + periodic rumble stress test...")
        print(
            f"Duration={TEST_DURATION_SEC:.1f}s, "
            f"rumble period={RUMBLE_PERIOD_SEC:.2f}s, "
            f"rumble on-time={RUMBLE_ON_MS}ms"
        )

        start_time = time.perf_counter()
        end_time = start_time + TEST_DURATION_SEC
        next_rumble_start = start_time
        rumble_active_until = 0.0
        last_valid_report_t = None
        last_print_t = start_time

        stats = {
            "start_time": start_time,
            "end_time": start_time,
            "valid_reports": 0,
            "short_reports": 0,
            "unknown_reports": 0,
            "max_gap_ms": 0.0,
            "sum_gap_ms": 0.0,
            "gap_count": 0,
            "rumble_starts": 0,
            "rumble_stops": 0,
            "raw_reports": 0,
            "empty_reads": 0,
            "button_pressed_reports": 0,
            "button_released_reports": 0,
            "button_transitions": 0,
        }

        last_button_down = None
        no_report_warning_printed = False

        while True:
            now = time.perf_counter()
            if now >= end_time:
                break

            # Periodic rumble start while continuously reading HID input.
            if now >= next_rumble_start:
                send_rumble(dev, amplitude=RUMBLE_AMPLITUDE, duration_ms=RUMBLE_ON_MS, flags=0)
                stats["rumble_starts"] += 1
                rumble_active_until = now + (RUMBLE_ON_MS / 1000.0)
                next_rumble_start += RUMBLE_PERIOD_SEC

            # Explicit stop shortly after requested duration to stress output path.
            if rumble_active_until > 0.0 and now >= rumble_active_until:
                send_rumble(dev, amplitude=0, duration_ms=0, flags=1)
                stats["rumble_stops"] += 1
                rumble_active_until = 0.0

            raw = dev.read(HID_READ_LEN, HID_READ_TIMEOUT_MS)
            if raw:
                stats["raw_reports"] += 1
                parsed = parse_input_report(raw)
                if parsed == "short":
                    stats["short_reports"] += 1
                elif isinstance(parsed, dict) and parsed.get("kind") == "ok":
                    stats["valid_reports"] += 1
                    if last_valid_report_t is not None:
                        gap_ms = (now - last_valid_report_t) * 1000.0
                        stats["sum_gap_ms"] += gap_ms
                        stats["gap_count"] += 1
                        if gap_ms > stats["max_gap_ms"]:
                            stats["max_gap_ms"] = gap_ms
                    last_valid_report_t = now

                    button_down = (parsed["buttons"] & 0x01) != 0
                    if button_down:
                        stats["button_pressed_reports"] += 1
                    else:
                        stats["button_released_reports"] += 1

                    if last_button_down is None:
                        last_button_down = button_down
                    elif button_down != last_button_down:
                        stats["button_transitions"] += 1
                        last_button_down = button_down
                        print(
                            f"[BUTTON] transition -> {'PRESSED' if button_down else 'released'} "
                            f"(transitions={stats['button_transitions']})"
                        )
                else:
                    stats["unknown_reports"] += 1
            else:
                stats["empty_reads"] += 1

            if (not no_report_warning_printed) and (now - start_time >= NO_REPORT_WARNING_SEC) and stats["raw_reports"] == 0:
                print(
                    "[WARN] No input reports received yet. "
                    "If rumble writes succeed but this stays zero, lock DEVICE_PATH_CONTAINS "
                    "to the exact controller HID path printed above."
                )
                no_report_warning_printed = True

            if now - last_print_t >= 2.0:
                print(
                    "[PROGRESS] "
                    f"valid={stats['valid_reports']} short={stats['short_reports']} "
                    f"unknown={stats['unknown_reports']} max_gap_ms={stats['max_gap_ms']:.2f} "
                    f"button_p={stats['button_pressed_reports']} button_r={stats['button_released_reports']} "
                    f"button_t={stats['button_transitions']} empty_reads={stats['empty_reads']} "
                    f"rumble_starts={stats['rumble_starts']} rumble_stops={stats['rumble_stops']}"
                )
                last_print_t = now

        # Ensure motor is left stopped.
        send_rumble(dev, amplitude=0, duration_ms=0, flags=1)
        stats["rumble_stops"] += 1
        stats["end_time"] = time.perf_counter()

        ok = print_summary(stats)
        if not ok:
            raise RuntimeError("Stress test result: FAIL")

    finally:
        dev.close()


if __name__ == "__main__":
    main()
