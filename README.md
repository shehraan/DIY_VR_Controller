# VR_CONTROLLER-PIO

PlatformIO firmware project for an ESP32-based VR controller with IMU tracking, joystick input, buttons, and haptic vibration.

## Firmware Target

- Board: `nodemcu-32s` (ESP32 DevKit V1)
- Framework: Arduino
- Monitor baud: 115200

Configured in `platformio.ini`.

## Exact Hardware Requirements

The parts below can also be viewed in `Hardware/Docs/VR Controller parts list.xlsx` and the attached controller schematic.

### Core Components

| Part | Component | Qty | Est. Price (CAD) |
|---|---|---:|---:|
| [ESP-32 Dev Kit](https://a.co/d/08W9CSqC) | CPU | 1 | 25.00 |
| [2000mAh 103450 3.7V LiPo Battery](https://www.aliexpress.com/item/1005007520210283.html?spm=a2g0o.order_list.order_list_main.5.19201802ylWPDs) | Battery | 1 | 20.00 |
| [TP4056 Charging Board](https://www.aliexpress.com/item/1005006043031985.html?spm=a2g0o.order_list.order_list_main.16.19201802ylWPDs) | USB-C Battery Charger | 1 | 5.00 |
| [PS2 Joycon KY-023](https://www.aliexpress.com/item/1005006195988088.html?spm=a2g0o.order_detail.order_detail_item.3.2d44f19cidb20h) | Joystick | 1 | 4.25 |
| [VC1030A002F Coin-sized DC Motor](https://www.aliexpress.com/item/1005009934713982.html?spm=a2g0o.order_list.order_list_main.16.1ab01802Qae0av) | Vibration Motor | 1 | 8.00 |
| [Gikfun 12x12x7.3mm Tactile Buttons](https://www.amazon.ca/Gikfun-12x12x7-3-Tactile-Momentary-Arduino/dp/B01E38OS7K) | Buttons (AE1027 style in schematic) | 2 | 16.00 |
| [MT3608 Boost Module](https://www.aliexpress.com/item/1005010404367609.html?spm=a2g0o.order_list.order_list_main.22.19201802ylWPDs) | Voltage Boost Module | 1 | 7.00 |
| [100nF Ceramic Capacitors](https://www.amazon.ca/Tiuimk-100-0-1uF-Ceramic-Capacitor/dp/B0CFTJR3L2) | Capacitor | 1 | 9.85 |
| [1N4001 Diodes](https://www.amazon.ca/Projects-General-Purpose-Silicon-Rectifiers/dp/B08JQP945V?source=ps-sl-shoppingads-lpcontext&ref_=fplfs&psc=1&smid=AODFMOUHD0RRM) | Diode | 1 | 10.53 |
| [MPU-6050 6DoF IMU](https://www.ebay.ca/itm/265931676678?chn=ps&google_free_listing_action=view_item) | IMU | 1 | 3.50 |

### Additional Required Discrete Parts

These appear in the attached schematic and are required in the build, and can be bought in multipacks:

- [2N2222 NPN transistor](https://www.amazon.ca/s?k=2N2222+transistor) x1 (motor driver switch)
- [1k ohm resistor](https://www.amazon.ca/s?k=1k+ohm+resistor+1%2F4w) x1 (base resistor for 2N2222)
- Hookup wire, solder, perfboard or PCB, and battery connector matching your LiPo pack

## Wiring Summary

### Power Path

1. USB-C input -> TP4056 charger input (`IN+`, `IN-`)
2. LiPo battery -> TP4056 battery pads (`B+`, `B-`)
3. TP4056 output (`OUT+`, `OUT-`) -> MT3608 input (`VIN+`, `VIN-`)
4. MT3608 output (`VOUT+`, `VOUT-`) -> ESP32 `VIN` and `GND`

### ESP32 Pin Map (firmware defaults)

- I2C SDA: GPIO 18 (MPU6050 SDA)
- I2C SCL: GPIO 4 (MPU6050 SCL)
- Button A: GPIO 12
- Button B: GPIO 13
- Joystick button (SW): GPIO 23
- Joystick VRX: GPIO 32
- Joystick VRY: GPIO 33
- Vibration motor PWM drive: GPIO 14

### Haptic Motor Driver Stage

- Motor low-side switched by 2N2222 transistor
- 1k resistor between ESP32 motor GPIO and transistor base
- Flyback diode (1N4001) across motor terminals
- 100nF capacitor for motor noise suppression (as shown in schematic)

## Software Setup

### 1) Open Project

Open this folder in VS Code:

- `VR_CONTROLLER-PIO`

### 2) Build

```powershell
pio run
```

### 3) Upload

```powershell
pio run --target upload
```

### 4) Serial Monitor

```powershell
pio device monitor
```

## Verification Checklist

- IMU responds over I2C at address `0x68`
- Joystick X/Y values change smoothly in telemetry
- Both buttons register consistently
- Haptic motor activates without brownouts or ESP32 resets

## Troubleshooting

- Upload fails: verify board (`nodemcu-32s`) and COM port in `platformio.ini`.
- Garbled serial output: ensure monitor speed is 115200.
- Random resets under vibration: check shared ground, boost converter current capability, and motor diode orientation.
- No IMU data: verify SDA/SCL wiring and MPU6050 power pins.