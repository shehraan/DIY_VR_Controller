#include "controller_protocol.h"
#include "driver_log.h"
#include "hid_transport.h"

#include "openvr/openvr_driver.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>

namespace vr_controller {

namespace {

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec(dllexport)
#else
#define HMD_DLL_EXPORT extern "C"
#endif

constexpr const char* kDriverId = "vr_controller";
constexpr const char* kControllerSerial = "vr_controller_001";
constexpr const char* kControllerModel = "VR Controller";
// {vr_controller} is the SteamVR driver-root token; it expands to the driver's
// install directory. SteamVR looks for:
//   <driver_root>/resources/rendermodels/steam_controller/steam_controller.obj
constexpr const char* kControllerRenderModel = "{vr_controller}/rendermodels/steam_controller";
constexpr const char* kInputProfilePath = "{vr_controller}/input/vr_controller_profile.json";
constexpr const char* kSettingsSection = "driver_vr_controller";

int32_t ReadSettingInt32(const char* key, int32_t fallback) {
    vr::EVRSettingsError error = vr::VRSettingsError_None;
    const int32_t value = vr::VRSettings()->GetInt32(kSettingsSection, key, &error);
    if (error != vr::VRSettingsError_None) {
        return fallback;
    }
    return value;
}

uint16_t ReadSettingU16(const char* key, uint16_t fallback) {
    const int32_t value = ReadSettingInt32(key, static_cast<int32_t>(fallback));
    return static_cast<uint16_t>(std::clamp(value, 0, 65535));
}

std::string ReadSettingString(const char* key, const char* fallback) {
    char buffer[128] = {};
    vr::EVRSettingsError error = vr::VRSettingsError_None;
    vr::VRSettings()->GetString(kSettingsSection, key, buffer, sizeof(buffer), &error);
    if (error != vr::VRSettingsError_None || buffer[0] == '\0') {
        return std::string(fallback);
    }
    return std::string(buffer);
}

float NormalizeJoystick(uint16_t value) {
    constexpr float kMax = 4095.0f;
    const float normalized = (static_cast<float>(value) / kMax) * 2.0f - 1.0f;
    return std::clamp(normalized, -1.0f, 1.0f);
}

vr::HmdQuaternion_t ToDriverQuat(const ControllerReport& report) {
    vr::HmdQuaternion_t q = {};
    // Firmware wire order is w,y,z,x. SteamVR expects x,y,z,w.
    q.w = report.quat_w;
    q.x = report.quat_x;
    q.y = report.quat_y;
    q.z = report.quat_z;
    return q;
}

class ControllerDevice final : public vr::ITrackedDeviceServerDriver {
public:
    ControllerDevice() = default;
    ~ControllerDevice() = default;

    vr::EVRInitError Activate(vr::TrackedDeviceIndex_t object_id) override {
        object_id_ = object_id;
        property_container_ = vr::VRProperties()->TrackedDeviceToPropertyContainer(object_id_);

        vr::VRProperties()->SetStringProperty(property_container_, vr::Prop_TrackingSystemName_String, kDriverId);
        vr::VRProperties()->SetStringProperty(property_container_, vr::Prop_ModelNumber_String, kControllerModel);
        vr::VRProperties()->SetStringProperty(property_container_, vr::Prop_RenderModelName_String, kControllerRenderModel);
        vr::VRProperties()->SetStringProperty(property_container_, vr::Prop_SerialNumber_String, kControllerSerial);
        vr::VRProperties()->SetStringProperty(property_container_, vr::Prop_RegisteredDeviceType_String, "vr_controller/right");
        vr::VRProperties()->SetStringProperty(property_container_, vr::Prop_InputProfilePath_String, kInputProfilePath);
        vr::VRProperties()->SetStringProperty(property_container_, vr::Prop_ManufacturerName_String, "Shehraan");
        vr::VRProperties()->SetStringProperty(property_container_, vr::Prop_ControllerType_String, "vr_controller");
        vr::VRProperties()->SetInt32Property(
            property_container_,
            vr::Prop_ControllerRoleHint_Int32,
            static_cast<int32_t>(vr::TrackedControllerRole_RightHand)
        );
        vr::VRProperties()->SetBoolProperty(property_container_, vr::Prop_DeviceIsWireless_Bool, true);
        vr::VRProperties()->SetBoolProperty(property_container_, vr::Prop_DeviceProvidesBatteryStatus_Bool, false);
        vr::VRProperties()->SetStringProperty(property_container_, vr::Prop_NamedIconPathDeviceOff_String, "{vr_controller}/icons/controller_status_off.png");
        vr::VRProperties()->SetStringProperty(property_container_, vr::Prop_NamedIconPathDeviceSearching_String, "{vr_controller}/icons/controller_status_searching.png");
        vr::VRProperties()->SetStringProperty(property_container_, vr::Prop_NamedIconPathDeviceReady_String, "{vr_controller}/icons/controller_status_ready.png");
        vr::VRProperties()->SetStringProperty(property_container_, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{vr_controller}/icons/controller_status_alert.png");

        vr::VRDriverInput()->CreateBooleanComponent(property_container_, "/input/a/click", &input_a_click_);
        vr::VRDriverInput()->CreateBooleanComponent(property_container_, "/input/b/click", &input_b_click_);
        vr::VRDriverInput()->CreateBooleanComponent(property_container_, "/input/joystick/click", &input_stick_click_);
        vr::VRDriverInput()->CreateScalarComponent(
            property_container_,
            "/input/joystick/x",
            &input_stick_x_,
            vr::VRScalarType_Absolute,
            vr::VRScalarUnits_NormalizedTwoSided
        );
        vr::VRDriverInput()->CreateScalarComponent(
            property_container_,
            "/input/joystick/y",
            &input_stick_y_,
            vr::VRScalarType_Absolute,
            vr::VRScalarUnits_NormalizedTwoSided
        );
        vr::VRDriverInput()->CreateHapticComponent(property_container_, "/output/haptic", &haptic_);

        pose_ = {};
        pose_.qWorldFromDriverRotation.w = 1.0;
        pose_.qDriverFromHeadRotation.w = 1.0;
        pose_.result = vr::TrackingResult_Running_OutOfRange;
        pose_.deviceIsConnected = false;
        pose_.poseIsValid = false;

        const uint16_t hid_vid = ReadSettingU16("hid_vendor_id", kVendorId);
        const uint16_t hid_pid = ReadSettingU16("hid_product_id", kProductId);
        const std::string hid_name = ReadSettingString("hid_product_name", kProductName);
        fresh_ms_ = static_cast<float>(ReadSettingInt32("fresh_threshold_ms", 50));
        stale_ms_ = static_cast<float>(ReadSettingInt32("timeout_threshold_ms", 100));
        log_stats_interval_ms_ = ReadSettingInt32("log_stats_interval_ms", 5000);
        if (stale_ms_ < fresh_ms_) {
            stale_ms_ = fresh_ms_;
        }
        if (log_stats_interval_ms_ < 250) {
            log_stats_interval_ms_ = 250;
        }

        transport_.Configure(hid_vid, hid_pid, hid_name);
        transport_.Start();
        DriverLog(
            "[vr_controller] Controller activated (VID=0x%04X PID=0x%04X product=%s fresh=%.0fms stale=%.0fms).",
            hid_vid,
            hid_pid,
            hid_name.c_str(),
            fresh_ms_,
            stale_ms_
        );
        return vr::VRInitError_None;
    }

    void Deactivate() override {
        transport_.Stop();
        object_id_ = vr::k_unTrackedDeviceIndexInvalid;
        property_container_ = vr::k_ulInvalidPropertyContainer;
    }

    void EnterStandby() override {}

    void* GetComponent(const char* pch_component_name_and_version) override {
        (void)pch_component_name_and_version;
        return nullptr;
    }

    void DebugRequest(const char* pch_request, char* pch_response_buffer, uint32_t un_response_buffer_size) override {
        (void)pch_request;
        if (un_response_buffer_size > 0) {
            pch_response_buffer[0] = '\0';
        }
    }

    vr::DriverPose_t GetPose() override { return pose_; }

    void RunFrame() {
        transport_.Poll();

        const auto maybe_report = transport_.LatestReport();
        if (maybe_report.has_value()) {
            const ControllerReport& report = maybe_report.value();
            pose_.qRotation = ToDriverQuat(report);
            pose_.poseTimeOffset = 0.0;
            last_report_ = report;

            vr::VRDriverInput()->UpdateBooleanComponent(input_a_click_, (report.buttons & kButtonA) != 0, 0.0);
            vr::VRDriverInput()->UpdateBooleanComponent(input_b_click_, (report.buttons & kButtonB) != 0, 0.0);
            vr::VRDriverInput()->UpdateBooleanComponent(input_stick_click_, (report.buttons & kButtonStickClick) != 0, 0.0);
            vr::VRDriverInput()->UpdateScalarComponent(input_stick_x_, NormalizeJoystick(report.joystick_x), 0.0);
            vr::VRDriverInput()->UpdateScalarComponent(input_stick_y_, NormalizeJoystick(report.joystick_y), 0.0);
        }

        const auto now = std::chrono::steady_clock::now();
        const auto last_rx = transport_.LastReceiveTime();
        const bool has_data = last_rx != std::chrono::steady_clock::time_point::min();

        float age_ms = 999999.0f;
        if (has_data) {
            age_ms = std::chrono::duration<float, std::milli>(now - last_rx).count();
        }

        pose_.deviceIsConnected = transport_.IsConnected();
        pose_.qWorldFromDriverRotation.w = 1.0;
        pose_.qDriverFromHeadRotation.w = 1.0;

        if (!has_data || age_ms > stale_ms_) {
            pose_.poseIsValid = false;
            pose_.result = vr::TrackingResult_Running_OutOfRange;
        } else {
            pose_.poseIsValid = true;
            pose_.result = vr::TrackingResult_Running_OK;
            if (age_ms >= fresh_ms_) {
                // Stale but not timed out: keep last pose and avoid extrapolation.
                pose_.poseTimeOffset = 0.0;
            }
        }

        if (object_id_ != vr::k_unTrackedDeviceIndexInvalid) {
            vr::VRServerDriverHost()->TrackedDevicePoseUpdated(object_id_, pose_, sizeof(vr::DriverPose_t));
        }

        const auto log_due = (now - last_stats_log_) > std::chrono::milliseconds(log_stats_interval_ms_);
        if (log_due) {
            last_stats_log_ = now;
            DriverLog(
                "[vr_controller] connected=%d received=%llu dropped=%llu age_ms=%.2f packetId=%u sendTimeUs=%u",
                transport_.IsConnected() ? 1 : 0,
                static_cast<unsigned long long>(transport_.ReceivedPacketCount()),
                static_cast<unsigned long long>(transport_.DroppedPacketCount()),
                age_ms,
                last_report_.packet_id,
                last_report_.send_time_us
            );
        }
    }

    void ProcessEvent(const vr::VREvent_t& event) {
        if (event.eventType != vr::VREvent_Input_HapticVibration) {
            return;
        }

        if (haptic_ != vr::k_ulInvalidInputComponentHandle &&
            event.data.hapticVibration.componentHandle != haptic_) {
            return;
        }

        const float clamped_amplitude = std::clamp(event.data.hapticVibration.fAmplitude, 0.0f, 1.0f);
        const float clamped_duration = std::max(event.data.hapticVibration.fDurationSeconds, 0.0f);
        const uint8_t amplitude_u8 = static_cast<uint8_t>(clamped_amplitude * 255.0f);
        const uint16_t duration_ms = static_cast<uint16_t>(
            std::clamp(clamped_duration * 1000.0f, 0.0f, 65535.0f)
        );

        transport_.SendRumble(amplitude_u8, duration_ms);
    }

private:
    vr::TrackedDeviceIndex_t object_id_ = vr::k_unTrackedDeviceIndexInvalid;
    vr::PropertyContainerHandle_t property_container_ = vr::k_ulInvalidPropertyContainer;
    vr::DriverPose_t pose_ = {};
    ControllerReport last_report_ = {};
    std::chrono::steady_clock::time_point last_stats_log_ = std::chrono::steady_clock::now();
    float fresh_ms_ = 50.0f;
    float stale_ms_ = 100.0f;
    int32_t log_stats_interval_ms_ = 5000;

    vr::VRInputComponentHandle_t input_a_click_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t input_b_click_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t input_stick_click_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t input_stick_x_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t input_stick_y_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t haptic_ = vr::k_ulInvalidInputComponentHandle;

    HidTransport transport_;
};

class ServerDriver final : public vr::IServerTrackedDeviceProvider {
public:
    vr::EVRInitError Init(vr::IVRDriverContext* driver_context) override {
        VR_INIT_SERVER_DRIVER_CONTEXT(driver_context);
        InitDriverLog();
        DriverLog("[vr_controller] Init");

        controller_ = std::make_unique<ControllerDevice>();
        if (!vr::VRServerDriverHost()->TrackedDeviceAdded(
                kControllerSerial,
                vr::TrackedDeviceClass_Controller,
                controller_.get())) {
            DriverLog("[vr_controller] TrackedDeviceAdded failed.");
            return vr::VRInitError_Driver_Failed;
        }
        return vr::VRInitError_None;
    }

    void Cleanup() override {
        DriverLog("[vr_controller] Cleanup");
        controller_.reset();
        CleanupDriverLog();
    }

    const char* const* GetInterfaceVersions() override { return vr::k_InterfaceVersions; }

    void RunFrame() override {
        if (controller_) {
            controller_->RunFrame();
        }

        vr::VREvent_t event = {};
        while (vr::VRServerDriverHost()->PollNextEvent(&event, sizeof(event))) {
            if (controller_) {
                controller_->ProcessEvent(event);
            }
        }
    }

    bool ShouldBlockStandbyMode() override { return false; }

    void EnterStandby() override {}

    void LeaveStandby() override {}

private:
    std::unique_ptr<ControllerDevice> controller_;
};

ServerDriver g_server_driver;

} // namespace

} // namespace vr_controller

HMD_DLL_EXPORT void* HmdDriverFactory(const char* interface_name, int* return_code) {
    if (0 == std::strcmp(interface_name, vr::IServerTrackedDeviceProvider_Version)) {
        return &vr_controller::g_server_driver;
    }

    if (return_code != nullptr) {
        *return_code = vr::VRInitError_Init_InterfaceNotFound;
    }
    return nullptr;
}
