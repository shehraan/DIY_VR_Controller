#include "HIDTransport.h"

#include <string.h>

#if defined(ARDUINO_ARCH_RP2040)
  #define TinyUSB
  #include "Adafruit_TinyUSB.h"
#else
  #include "HID.h"
#endif

namespace {

static const uint8_t kHidReportDescriptor[] PROGMEM = {
  0x06, 0x03, 0x00,
  0x09, 0x00,
  0xa1, 0x01,
  0x15, 0x00,
  0x26, 0xff, 0x00,
  0x85, 0x01,
  0x75, 0x08,
  0x95, 0x3f,
  0x09, 0x00,
  0x81, 0x02,
  0xc0
};

#ifdef TinyUSB
Adafruit_USBD_HID g_hid(kHidReportDescriptor, sizeof(kHidReportDescriptor), HID_ITF_PROTOCOL_NONE, 2, false);
#endif

} // namespace

namespace HIDTransport {

void begin() {
#ifdef TinyUSB
  g_hid.begin();
  while (!TinyUSBDevice.mounted()) {
    delay(1);
  }
#else
  static HIDSubDescriptor node(kHidReportDescriptor, sizeof(kHidReportDescriptor));
  HID().AppendDescriptor(&node);
#endif
}

void sendQuaternion(float w, float x, float y, float z) {
  float orderedQuat[4];
  orderedQuat[0] = w;
  orderedQuat[1] = y;
  orderedQuat[2] = z;
  orderedQuat[3] = x;

  uint8_t report[63];
  memset(report, 0, sizeof(report));
  memcpy(report, orderedQuat, sizeof(orderedQuat));

#ifdef TinyUSB
  g_hid.sendReport(1, report, sizeof(report));
#else
  HID().SendReport(1, report, sizeof(report));
#endif
}

} // namespace
