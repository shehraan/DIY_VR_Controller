# vr_controller — build guide

This folder builds a **Windows x64** SteamVR server driver: `driver_vr_controller.dll`. The runtime layout that SteamVR needs is assembled automatically under **`BuiltDrivers/vr_controller/`** after a successful Release build.

---

## Prerequisites

- **Windows 10 or 11**, **x64** only.
- **Visual Studio 2022**+ with the **Desktop development with C++** workload (MSVC v143, Windows SDK).
- **CMake 3.20+** (installer adds `cmake` to PATH, or use the one bundled with VS).
- **Git** (used by CMake **FetchContent** to download hidapi sources).

At **configure** time, CMake downloads [hidapi](https://github.com/libusb/hidapi) (`hidapi-0.14.0`) and compiles `windows/hid.c` into the driver. You need a working network for the **first** configure unless you already have a populated `build/_deps/hidapi_src-src/` cache.

## Compile (recommended: MSVC environment)

Use the **x64 Native Tools Command Prompt for VS 2022** (or run `vcvars64.bat`) so MSVC redistributable DLLs are found reliably when staging `BuiltDrivers`.

```bat
cd path\to\repo\Drivers\vr_controller
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```
---

## Outputs

| Location | What it is |
|----------|------------|
| `bin/win64/driver_vr_controller.dll` | Built DLL (and import `.lib` / `.exp` next to it in typical VS layouts) |
| `BuiltDrivers/vr_controller/` | **SteamVR-ready folder** — copy this entire tree into SteamVR’s `drivers` area |

What **`BuiltDrivers/vr_controller/`** contains after Release:

- `bin/win64/driver_vr_controller.dll`
- **MSVC runtime** DLLs copied next to the driver (dynamic CRT, `/MD` — same idea as shipping VC++ redistributables beside the binary)
- `driver.vrdrivermanifest`
- `resources/` (settings, input JSON, icons, render models, etc.)

**There is no `hidapi.dll`:** the Windows HID backend is linked statically into `driver_vr_controller.dll`. System DLLs such as `SETUPAPI.dll` / `HID.DLL` come from Windows.

---

## If CRT DLLs are missing from `BuiltDrivers`

CMake tries, in order:

1. `VCToolsRedistDir` (set when you build from a VS developer shell)
2. A **fallback** path in `CMakeLists.txt` for VS 2022 Community and a specific MSVC redist folder (e.g. `14.42.34433`)

If you install a **different** VS edition or a **newer** MSVC toolset, update the fallback path in **`CMakeLists.txt`** (search for `Microsoft.VC143.CRT`) or always build from **x64 Native Tools** so `VCToolsRedistDir` is set.

---

## Quick naming check before you ship to SteamVR

SteamVR expects a consistent name for the driver root, manifest, and DLL prefix:

- Folder name: **`vr_controller`**
- Manifest driver name + `driver_<name>.dll` → **`driver_vr_controller.dll`**

If those disagree, the driver may not load.

---

## After build — install

Copy **`BuiltDrivers/vr_controller`** (the folder itself) into:

`...\Steam\steamapps\common\SteamVR\drivers\vr_controller`

Then restart SteamVR. Details and checks: [VALIDATION.md](VALIDATION.md).

---

## Firmware / HID reference (short)

The driver talks to the same BLE HID layout as the ESP32 firmware in this repo: input report **ID `0x01`**, **63-byte** payload (quaternion `w,y,z,x` LE, buttons, joystick, `sendTimeUs`, `packetId`); output rumble **ID `0x02`**. On Windows, BLE PnP IDs may appear byte-swapped; settings in `resources/settings/default.vrsettings` tolerate that.

Tracking freshness uses **receive time on Windows** (`std::chrono::steady_clock`), not ESP32 `sendTimeUs`, per `default.vrsettings` thresholds.
