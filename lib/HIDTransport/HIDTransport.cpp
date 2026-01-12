#include "HIDTransport.h"

#include <string.h>
#include <string>

#if defined(ARDUINO_ARCH_ESP32)
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEHIDDevice.h>
  #include <HIDTypes.h>
  #define HIDTRANSPORT_ESP32_BLE_HID 1
#else
  #define HIDTRANSPORT_UNAVAILABLE 1
#endif

namespace {

static const uint8_t kHidReportDescriptor[] PROGMEM = {
  0x06, 0x03, 0x00, // Usage Page (Vendor 0x0003)
  0x09, 0x00,       // Usage (0x00)
  0xA1, 0x01,       // Collection (Application)
  0x15, 0x00,       // Logical Min (0)
  0x26, 0xFF, 0x00, // Logical Max (255)
  0x85, 0x01,       // Report ID (1)
  0x75, 0x08,       // Report Size (8)
  0x95, 0x3F,       // Report Count (63)
  0x09, 0x00,       // Usage (0x00)
  0x81, 0x02,       // Input (Data,Var,Abs)
  0x85, 0x02,       // Report ID (2)
  0x75, 0x08,       // Report Size (8)
  0x95, 0x05,       // Report Count (5)
  0x09, 0x00,       // Usage (0x00)
  0x91, 0x02,       // Output (Data,Var,Abs)
  0xC0              // End Collection
};

static constexpr uint8_t kReportId = 0x01;
static constexpr uint8_t kOutputReportId = 0x02;
static constexpr size_t kReportLen = 63;
static constexpr uint16_t kBleAppearanceHidGeneric = 0x03C0;

#if defined(HIDTRANSPORT_ESP32_BLE_HID)
bool g_ready = false;
bool g_connected = false;
BLECharacteristic *g_inputReport = nullptr;
BLECharacteristic *g_outputReport = nullptr;
volatile bool g_hasPendingRumble = false;
volatile uint8_t g_pendingAmplitude = 0;
volatile uint16_t g_pendingDurationMs = 0;
volatile uint8_t g_pendingFlags = 0;

bool parseRumblePacket(const uint8_t *data, size_t len, uint8_t &amplitude, uint16_t &durationMs, uint8_t &flags) {
  if (data == nullptr || len == 0U) {
    return false;
  }

  amplitude = 0;
  durationMs = 0;
  flags = 0;

  if (len == 3U) {
    amplitude = data[0];
    durationMs = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
    return true;
  }

  if (len == 2U) {
    amplitude = data[0];
    durationMs = (uint16_t)data[1] * 10U;
    return true;
  }

  if (len >= 5U && data[0] == 0x01U) {
    amplitude = data[1];
    durationMs = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    flags = data[4];
    return true;
  }

  if (len >= 4U && data[0] == kOutputReportId) {
    amplitude = data[1];
    durationMs = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    if (len >= 5U) {
      flags = data[4];
    }
    return true;
  }

  return false;
}

class ControllerOutputReportCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    if (characteristic == nullptr) {
      return;
    }

    std::string payload = characteristic->getValue();
    uint8_t amplitude = 0;
    uint16_t durationMs = 0;
    uint8_t flags = 0;
    if (!parseRumblePacket((const uint8_t *)payload.data(), payload.size(), amplitude, durationMs, flags)) {
      return;
    }

    g_pendingAmplitude = amplitude;
    g_pendingDurationMs = durationMs;
    g_pendingFlags = flags;
    g_hasPendingRumble = true;
  }
};

void refreshReadyState() {
  g_ready = g_connected && (g_inputReport != nullptr);
}

class ControllerBleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    (void)pServer;
    g_connected = true;
    refreshReadyState();
  }

  void onDisconnect(BLEServer *pServer) override {
    g_connected = false;
    refreshReadyState();
    // Keep advertising active so reconnect is seamless.
    pServer->startAdvertising();
  }
};
#else
bool g_ready = false;
bool g_warned = false;
#endif

} // namespace

namespace HIDTransport {

void begin() {
#if defined(HIDTRANSPORT_ESP32_BLE_HID)
  BLEDevice::init("Controller-ESP32");

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ControllerBleServerCallbacks());

  BLEHIDDevice *hid = new BLEHIDDevice(server);
  g_inputReport = hid->inputReport(kReportId);
  g_outputReport = hid->outputReport(kOutputReportId);
  if (g_outputReport != nullptr) {
    g_outputReport->setCallbacks(new ControllerOutputReportCallbacks());
  }

  hid->manufacturer()->setValue("Controller");
  hid->pnp(0x02, 0xE502, 0xA111, 0x0110);
  hid->hidInfo(0x00, 0x01);
  hid->reportMap((uint8_t *)kHidReportDescriptor, sizeof(kHidReportDescriptor));
  hid->startServices();
  hid->setBatteryLevel(100);

  BLEAdvertising *advertising = server->getAdvertising();
  advertising->setAppearance(kBleAppearanceHidGeneric);
  advertising->addServiceUUID(hid->hidService()->getUUID());
  advertising->start();

  g_connected = false;
  g_hasPendingRumble = false;
  refreshReadyState();
#else
  g_ready = false;
  if (!g_warned) {
    g_warned = true;
    Serial.println(
      "HIDTransport disabled: target has no supported USB/BLE HID backend."
    );
  }
#endif
}

bool ready() {
#if defined(HIDTRANSPORT_ESP32_BLE_HID)
  refreshReadyState();
#endif
  return g_ready;
}

void sendQuaternion(
  float w,
  float x,
  float y,
  float z,
  uint8_t buttons,
  uint16_t joystickX,
  uint16_t joystickY
) {
#if defined(HIDTRANSPORT_ESP32_BLE_HID)
  refreshReadyState();
  if (!g_ready) {
    return;
  }

  float orderedQuat[4];
  orderedQuat[0] = w;
  orderedQuat[1] = y;
  orderedQuat[2] = z;
  orderedQuat[3] = x;

  uint8_t report[kReportLen];
  memset(report, 0, sizeof(report));
  memcpy(report, orderedQuat, sizeof(orderedQuat));
  report[16] = buttons;
  report[17] = (uint8_t)(joystickX & 0xFFU);
  report[18] = (uint8_t)((joystickX >> 8) & 0xFFU);
  report[19] = (uint8_t)(joystickY & 0xFFU);
  report[20] = (uint8_t)((joystickY >> 8) & 0xFFU);

  if (g_inputReport == nullptr) {
    return;
  }
  // Update characteristic value only; host can read it directly.
  g_inputReport->setValue(report, sizeof(report));
#else
  (void)w;
  (void)x;
  (void)y;
  (void)z;
  (void)buttons;
  (void)joystickX;
  (void)joystickY;
#endif
}

bool pollRumbleCommand(RumbleCommand &outCommand) {
#if defined(HIDTRANSPORT_ESP32_BLE_HID)
  if (!g_hasPendingRumble) {
    return false;
  }

  outCommand.amplitude = g_pendingAmplitude;
  outCommand.durationMs = g_pendingDurationMs;
  outCommand.flags = g_pendingFlags;
  g_hasPendingRumble = false;
  return true;
#else
  (void)outCommand;
  return false;
#endif
}

} // namespace
