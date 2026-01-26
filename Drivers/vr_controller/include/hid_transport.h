#pragma once

#include "controller_protocol.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

struct hid_device_;
struct hid_device_info;

namespace vr_controller {

// Manages BLE HID communication with the ESP32 controller.
//
// Reads are performed on a dedicated background thread using a blocking
// hid_read_timeout() call (16 ms window). This avoids the race between
// RunFrame() timing and async BLE GATT notifications that occurs when
// calling hid_read_timeout(0) directly from the SteamVR render thread.
class HidTransport {
public:
    HidTransport();
    ~HidTransport();

    bool Start();
    void Stop();

    // Poll() is called from RunFrame(). It no longer does the actual HID
    // read (the background thread does); it only drives reconnect logic.
    void Poll();

    void Configure(uint16_t vendor_id, uint16_t product_id, const std::string& product_name);

    bool IsConnected() const;

    bool SendRumble(uint8_t amplitude, uint16_t duration_ms);

    std::optional<ControllerReport> LatestReport() const;
    std::chrono::steady_clock::time_point LastReceiveTime() const;
    uint64_t DroppedPacketCount() const;
    uint64_t ReceivedPacketCount() const;

private:
    bool OpenDevice();
    void CloseDevice();
    bool ParseReport(const uint8_t* data, size_t len, ControllerReport& out_report);
    bool IsProductMatch(const hid_device_info* info) const;

    void ReadThreadFunc();

    hid_device_* device_;
    std::atomic<bool> connected_;
    uint32_t reconnect_interval_ms_;
    std::chrono::steady_clock::time_point next_reconnect_attempt_;
    std::chrono::steady_clock::time_point last_receive_time_;

    mutable std::mutex mutex_;
    std::optional<ControllerReport> latest_report_;

    bool have_packet_id_;
    uint32_t last_packet_id_;
    uint64_t dropped_packets_;
    uint64_t received_packets_;
    uint16_t vendor_id_;
    uint16_t product_id_;
    std::wstring product_name_;

    // Background read thread
    std::thread read_thread_;
    std::atomic<bool> stop_thread_;
    std::condition_variable reconnect_cv_;
    std::mutex reconnect_mutex_;
};

} // namespace vr_controller
