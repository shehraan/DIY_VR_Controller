#include "hid_transport.h"

#include "driver_log.h"

#include "hidapi/hidapi.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <cwchar>

namespace vr_controller {

namespace {
constexpr uint16_t kMaxJoystickValue = 4095;

// Firmware HID descriptor: Report ID(1) + Report Count(63) = 64 bytes total.
// hidapi returns the full buffer including the report ID byte.
constexpr size_t kReadBufferSize = 64;

// Blocking timeout per read attempt in the background thread.
// 16 ms gives the BLE GATT notification stack enough time to deliver data;
// at 30 Hz firmware send rate (~33 ms between packets) we will never miss one.
constexpr int kReadBlockMs = 16;

// Reconnect polling interval when no device is found.
constexpr int kReconnectIntervalMs = 1000;

// BLE PnP IDs: firmware uses 0xE502/0xA111; Windows hidapi may byte-swap to 0x02E5/0x11A1.
uint16_t SwapUsbIdEndian(uint16_t v) {
    return static_cast<uint16_t>((v >> 8) | (v << 8));
}

bool UsbIdsMatch(uint16_t configured, uint16_t reported) {
    return reported == configured || reported == SwapUsbIdEndian(configured);
}
} // namespace

HidTransport::HidTransport()
    : device_(nullptr),
      connected_(false),
      reconnect_interval_ms_(kReconnectIntervalMs),
      next_reconnect_attempt_(std::chrono::steady_clock::now()),
      last_receive_time_(std::chrono::steady_clock::time_point::min()),
      have_packet_id_(false),
      last_packet_id_(0),
      dropped_packets_(0),
      received_packets_(0),
      vendor_id_(kVendorId),
      product_id_(kProductId),
      product_name_(L"ESP32-Controller"),
      stop_thread_(false) {}

HidTransport::~HidTransport() { Stop(); }

bool HidTransport::Start() {
    if (hid_init() != 0) {
        DriverLog("[vr_controller] hid_init failed.");
        return false;
    }

    stop_thread_.store(false);
    read_thread_ = std::thread(&HidTransport::ReadThreadFunc, this);

    DriverLog("[vr_controller] HID read thread started.");
    return true;
}

void HidTransport::Stop() {
    stop_thread_.store(true);
    reconnect_cv_.notify_all();

    if (read_thread_.joinable()) {
        read_thread_.join();
    }

    CloseDevice();
    hid_exit();
}

void HidTransport::Configure(uint16_t vendor_id, uint16_t product_id, const std::string& product_name) {
    vendor_id_ = vendor_id;
    product_id_ = product_id;
    product_name_.assign(product_name.begin(), product_name.end());
}

bool HidTransport::IsConnected() const { return connected_.load(); }

// Poll() is only called from RunFrame() and drives reconnect bookkeeping.
// Actual HID reading happens in ReadThreadFunc().
void HidTransport::Poll() {
    // Nothing to do here; the background thread handles reads and reconnects.
    // Keeping this method in place so the RunFrame caller does not need to change.
}

bool HidTransport::SendRumble(uint8_t amplitude, uint16_t duration_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!connected_.load() || device_ == nullptr) {
        return false;
    }

    const uint8_t duration_lsb = static_cast<uint8_t>(duration_ms & 0xFF);
    const uint8_t duration_msb = static_cast<uint8_t>((duration_ms >> 8) & 0xFF);

    // Try both 5-byte and 4-byte forms (firmware accepts either).
    const uint8_t payload_a[5] = { kOutputReportId, amplitude, duration_lsb, duration_msb, 0 };
    const uint8_t payload_b[4] = { kOutputReportId, amplitude, duration_lsb, duration_msb };

    int written = hid_write(device_, payload_a, sizeof(payload_a));
    if (written > 0) return true;

    written = hid_write(device_, payload_b, sizeof(payload_b));
    return written > 0;
}

std::optional<ControllerReport> HidTransport::LatestReport() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_report_;
}

std::chrono::steady_clock::time_point HidTransport::LastReceiveTime() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_receive_time_;
}

uint64_t HidTransport::DroppedPacketCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_packets_;
}

uint64_t HidTransport::ReceivedPacketCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return received_packets_;
}

// ── Background read thread ────────────────────────────────────────────────────

void HidTransport::ReadThreadFunc() {
    uint8_t buffer[kReadBufferSize] = {};

    while (!stop_thread_.load()) {
        // ── Ensure device is open ────────────────────────────────────────────
        if (!connected_.load()) {
            const auto now = std::chrono::steady_clock::now();
            if (now < next_reconnect_attempt_) {
                // Wait until next reconnect attempt (or until Stop() wakes us).
                std::unique_lock<std::mutex> lk(reconnect_mutex_);
                reconnect_cv_.wait_until(lk, next_reconnect_attempt_,
                    [this] { return stop_thread_.load(); });
                continue;
            }

            if (OpenDevice()) {
                DriverLog("[vr_controller] HID device opened in read thread.");
            } else {
                next_reconnect_attempt_ =
                    std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(reconnect_interval_ms_);
            }
            continue;
        }

        // ── Blocking read with 16 ms timeout ────────────────────────────────
        // Using a non-zero timeout (rather than 0) lets the Windows BLE/GATT
        // stack deliver the async notification before we give up. At firmware
        // 30 Hz (~33 ms between packets) a 16 ms window is always sufficient.
        const int read_bytes = hid_read_timeout(device_, buffer, sizeof(buffer), kReadBlockMs);

        if (read_bytes == 0) {
            // Timeout expired with no data — perfectly normal, try again.
            continue;
        }

        if (read_bytes < 0) {
            DriverLog("[vr_controller] hid_read_timeout error — reconnecting.");
            CloseDevice();
            next_reconnect_attempt_ =
                std::chrono::steady_clock::now() +
                std::chrono::milliseconds(reconnect_interval_ms_);
            continue;
        }

        // ── Parse and store report ───────────────────────────────────────────
        ControllerReport parsed = {};
        if (!ParseReport(buffer, static_cast<size_t>(read_bytes), parsed)) {
            DriverLog(
                "[vr_controller] ParseReport failed: bytes=%d first=0x%02X",
                read_bytes,
                static_cast<int>(buffer[0])
            );
            continue;
        }

        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mutex_);
        latest_report_ = parsed;
        last_receive_time_ = now;
        ++received_packets_;

        if (have_packet_id_) {
            const uint32_t expected = last_packet_id_ + 1u;
            if (parsed.packet_id != expected) {
                const uint32_t gap = parsed.packet_id > expected
                    ? (parsed.packet_id - expected) : 0u;
                dropped_packets_ += static_cast<uint64_t>(gap);
                DriverLog(
                    "[vr_controller] packetId gap: expected=%u got=%u gap=%u",
                    expected, parsed.packet_id, gap
                );
            }
        }
        have_packet_id_ = true;
        last_packet_id_ = parsed.packet_id;
    }

    DriverLog("[vr_controller] HID read thread exiting.");
}

// ── Private helpers ───────────────────────────────────────────────────────────

bool HidTransport::OpenDevice() {
    hid_device_info* devices = hid_enumerate(0x0, 0x0);
    hid_device_info* current = devices;
    hid_device* found = nullptr;

    while (current != nullptr) {
        if (IsProductMatch(current)) {
            found = hid_open_path(current->path);
            if (found != nullptr) break;
        }
        current = current->next;
    }
    hid_free_enumeration(devices);

    if (found == nullptr) {
        return false;
    }

    // Leave the device in blocking mode (hid_set_nonblocking is NOT called).
    // The background thread uses hid_read_timeout with a finite timeout, which
    // is more reliable than non-blocking 0-ms polls on BLE HID under Windows.

    {
        std::lock_guard<std::mutex> lock(mutex_);
        device_ = found;
    }
    connected_.store(true);
    return true;
}

void HidTransport::CloseDevice() {
    hid_device* dev = nullptr;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        dev = device_;
        device_ = nullptr;
    }
    connected_.store(false);
    if (dev != nullptr) {
        hid_close(dev);
    }
}

bool HidTransport::ParseReport(const uint8_t* data, size_t len, ControllerReport& out_report) {
    if (data == nullptr || len == 0) return false;

    // hidapi on Windows returns the report ID as the first byte when the device
    // uses report IDs (our firmware uses ID 0x01). Strip it so the payload
    // offsets below match the firmware's memcpy layout.
    const uint8_t* payload = data;
    size_t payload_len = len;
    if (payload[0] == kInputReportId && payload_len > 1) {
        payload = data + 1;
        payload_len -= 1;
    }

    // Firmware wire layout (byte offsets within the 63-byte payload):
    //  0.. 3  w (float32 LE)
    //  4.. 7  y (float32 LE)
    //  8..11  z (float32 LE)
    // 12..15  x (float32 LE)
    // 16      buttons
    // 17..18  joystickX (uint16 LE)
    // 19..20  joystickY (uint16 LE)
    // 21..24  sendTimeUs (uint32 LE)
    // 25..28  packetId   (uint32 LE)
    if (payload_len < 29) return false;

    std::memcpy(&out_report.quat_w, payload + 0,  sizeof(float));
    std::memcpy(&out_report.quat_y, payload + 4,  sizeof(float));
    std::memcpy(&out_report.quat_z, payload + 8,  sizeof(float));
    std::memcpy(&out_report.quat_x, payload + 12, sizeof(float));

    out_report.buttons    = payload[16];
    out_report.joystick_x = static_cast<uint16_t>(payload[17] | (payload[18] << 8));
    out_report.joystick_y = static_cast<uint16_t>(payload[19] | (payload[20] << 8));
    out_report.send_time_us =
        static_cast<uint32_t>(payload[21]) |
        (static_cast<uint32_t>(payload[22]) << 8)  |
        (static_cast<uint32_t>(payload[23]) << 16) |
        (static_cast<uint32_t>(payload[24]) << 24);
    out_report.packet_id =
        static_cast<uint32_t>(payload[25]) |
        (static_cast<uint32_t>(payload[26]) << 8)  |
        (static_cast<uint32_t>(payload[27]) << 16) |
        (static_cast<uint32_t>(payload[28]) << 24);

    out_report.joystick_x = std::min<uint16_t>(out_report.joystick_x, kMaxJoystickValue);
    out_report.joystick_y = std::min<uint16_t>(out_report.joystick_y, kMaxJoystickValue);
    return true;
}

bool HidTransport::IsProductMatch(const hid_device_info* info) const {
    if (info == nullptr) return false;

    if (!UsbIdsMatch(vendor_id_, info->vendor_id) ||
        !UsbIdsMatch(product_id_, info->product_id)) {
        return false;
    }

    if (info->product_string == nullptr || product_name_.empty()) return true;
    return std::wcscmp(info->product_string, product_name_.c_str()) == 0;
}

} // namespace vr_controller
