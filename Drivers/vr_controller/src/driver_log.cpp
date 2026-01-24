#include "driver_log.h"

#include "openvr/openvr_driver.h"

#include <cstdarg>
#include <cstdio>
#include <mutex>

namespace {
std::mutex g_log_mutex;
}

void InitDriverLog() {}

void CleanupDriverLog() {}

void DriverLog(const char* fmt, ...) {
    if (fmt == nullptr || vr::VRDriverLog() == nullptr) {
        return;
    }

    std::lock_guard<std::mutex> lock(g_log_mutex);
    char buffer[1024] = {};

    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    vr::VRDriverLog()->Log(buffer);
}
