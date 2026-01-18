import hid
import struct
import time

ESP_TIMER_MAX = 2**32

def find_device_path():
    for d in hid.enumerate():
        if 'ESP32-Controller' in (d['product_string'] or ''):
            print(f"Found: {d['product_string']} at {d['path']}")
            return d['path']
    raise RuntimeError("ESP32-Controller not found. Is it paired and turned on?")

def main():
    path = find_device_path()
    print("\nOpening device...")
    dev = hid.device()
    dev.open_path(path)
    dev.set_nonblocking(False)

    print("Reading quaternion + latency reports (Ctrl+C to stop):\n")

    base_esp_us = None
    base_pc_ns = None

    try:
        while True:
            data = dev.read(64)
            if not data:
                continue

            recv_pc_ns = time.perf_counter_ns()

            payload = data
            if payload[0] == 0x01:
                payload = payload[1:]

            if len(payload) < 20:
                print(f"Short report: {bytes(data).hex()}")
                continue

            w, y, z, x = struct.unpack_from("<ffff", bytes(payload), 0)
            esp_send_us = struct.unpack_from("<I", bytes(payload), 16)[0]

            if base_esp_us is None:
                base_esp_us = esp_send_us
                base_pc_ns = recv_pc_ns
                print("Clock sync established from first packet.")
                continue

            esp_delta_us = (esp_send_us - base_esp_us) % ESP_TIMER_MAX
            expected_pc_ns = base_pc_ns + esp_delta_us * 1000
            one_way_latency_ms = (recv_pc_ns - expected_pc_ns) / 1_000_000.0

            mag = (w*w + x*x + y*y + z*z) ** 0.5

            print(
                f"w={w:+.4f}  x={x:+.4f}  y={y:+.4f}  z={z:+.4f}  "
                f"|q|={mag:.4f}  latency≈{one_way_latency_ms:.3f} ms"
            )

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        dev.close()

main()
