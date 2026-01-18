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
static constexpr const char *kDeviceName = "ESP32-Controller";

#if defined(HIDTRANSPORT_ESP32_BLE_HID)
bool g_ready = false;
bool g_connected = false;
BLECharacteristic *g_inputReport = nullptr;
BLECharacteristic *g_outputReport = nullptr;
volatile bool g_hasPendingRumble = false;
volatile uint8_t g_pendingAmplitude = 0;
volatile uint16_t g_pendingDurationMs = 0;
volatile uint8_t g_pendingFlags = 0;

bool parseRumblePacket(const uint8_t *data, size_t len, uint8_t &amplitude, uint16_t &durationMs) {
  if (data == nullptr || len < 2U) {
    return false;
  }

  amplitude = data[0];
  durationMs = (uint16_t)data[1] | ((uint16_t)(len >= 3U ? data[2] : 0U) << 8);
  return true;
}

class ControllerOutputReportCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    if (characteristic == nullptr) {
      return;
    }

    std::string payload = characteristic->getValue();
    uint8_t amplitude = 0;
    uint16_t durationMs = 0;
    if (!parseRumblePacket((const uint8_t *)payload.data(), payload.size(), amplitude, durationMs)) {
      return;
    }

    g_pendingAmplitude = amplitude;
    g_pendingDurationMs = durationMs;
    g_pendingFlags = 0;
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
  BLEDevice::init(kDeviceName);

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ControllerBleServerCallbacks());

  BLEHIDDevice *hid = new BLEHIDDevice(server);
  g_inputReport = hid->inputReport(kReportId);
  g_outputReport = hid->outputReport(kOutputReportId);
  if (g_outputReport != nullptr) {
    g_outputReport->setCallbacks(new ControllerOutputReportCallbacks());
  }

  hid->manufacturer()->setValue("Shehraan");
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
  static uint32_t packetId = 0;
  static uint32_t lastSendUs = 0;

  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastSendUs) < 30000U) {
    return;
  }
  lastSendUs = nowUs;

  refreshReadyState();
  if (!g_ready) {
    return;
  }

  uint32_t sendTimeUs = micros();

  uint8_t report[kReportLen];
  memset(report, 0, sizeof(report));

  memcpy(report + 0, &w, 4);
  memcpy(report + 4, &y, 4);
  memcpy(report + 8, &z, 4);
  memcpy(report + 12, &x, 4);
  report[16] = buttons;
  memcpy(report + 17, &joystickX, 2);
  memcpy(report + 19, &joystickY, 2);
  memcpy(report + 21, &sendTimeUs, 4);
  memcpy(report + 25, &packetId, 4);

  if (g_inputReport == nullptr) {
    return;
  }
  g_inputReport->setValue(report, sizeof(report));
  g_inputReport->notify();
  packetId++;
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
  outCommand.flags = 0;
  g_hasPendingRumble = false;
  return true;
#else
  (void)outCommand;
  return false;
#endif
}

} // namespace
