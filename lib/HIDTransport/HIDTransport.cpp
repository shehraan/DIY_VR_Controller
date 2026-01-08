#include "HIDTransport.h"

#include <string.h>

#if defined(ARDUINO_ARCH_ESP32)
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEHIDDevice.h>
  #include <BLE2902.h>
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
  0xC0              // End Collection
};

static constexpr uint8_t kReportId = 0x01;
static constexpr size_t kReportLen = 63;
static constexpr uint16_t kBleAppearanceHidGeneric = 0x03C0;

#if defined(HIDTRANSPORT_ESP32_BLE_HID)
bool g_ready = false;
bool g_connected = false;
bool g_subscribed = false;
BLECharacteristic *g_inputReport = nullptr;
BLE2902 *g_inputReportCccd = nullptr;

bool notificationsEnabled() {
  if (g_inputReportCccd == nullptr) {
    return false;
  }

  uint8_t *cccdValue = g_inputReportCccd->getValue();
  const size_t len = g_inputReportCccd->getLength();

  if (cccdValue == nullptr || len < 2) {
    return false;
  }

  const uint16_t bits =
      static_cast<uint16_t>(cccdValue[0]) |
      (static_cast<uint16_t>(cccdValue[1]) << 8);

  return (bits & 0x0001U) != 0U;
}

void refreshReadyState() {
  g_subscribed = notificationsEnabled();
  g_ready = g_connected && g_subscribed && (g_inputReport != nullptr);
}

class ControllerBleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    (void)pServer;
    g_connected = true;
    g_subscribed = false;
    g_ready = false;
  }

  void onDisconnect(BLEServer *pServer) override {
    g_connected = false;
    g_subscribed = false;
    g_ready = false;
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
  g_inputReportCccd = new BLE2902();
  g_inputReportCccd->setNotifications(true);
  g_inputReport->addDescriptor(g_inputReportCccd);

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
  g_subscribed = false;
  g_ready = false;
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

void sendQuaternion(float w, float x, float y, float z) {
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

  if (g_inputReport == nullptr) {
    return;
  }
  g_inputReport->setValue(report, sizeof(report));
  g_inputReport->notify();
#else
  (void)w;
  (void)x;
  (void)y;
  (void)z;
#endif
}

} // namespace
