# DIY VR Controller


---

## Why this project exists

I built this controller for the same reason I built my [DIY VR headset](https://github.com/shehraan/DIY_VR_HEADSET): I wanted to understand how the system actually works instead of treating VR hardware like a black box.

For me, that meant building something real with real constraints:

1. **I wanted to understand controller tracking at a lower level** by working directly with IMU data, sensor fusion, button input, joystick input, haptics, and wireless communication.
2. **I wanted a controller that matched the spirit of the headset project**: not infinitely modular, not trying to be a consumer product, just one solid documented build that teaches a lot.
3. **I wanted to force myself through the annoying parts** like BLE quirks, noisy signals, power constraints, packet formats, mechanical fitment, and SteamVR-side integration.

---

## Build philosophy

This was my mindset:

- use real parts
- make tradeoffs consciously
- learn by shipping
- understand every layer

I care a lot about fast iteration, low-level debugging, and building things that force me to earn my understanding. This controller is a direct extension of that.

It is not meant to be the most polished controller possible.
It is meant to be a controller that teaches something useful at every stage of the build.

---

## Hardware stack

### Core electronics

- **Microcontroller:** ESP32 DevKit V1
- **IMU:** MPU-6050
- **Primary input:** PS2-style analog joystick module
- **Haptics:** 3V coin vibration motor
- **Motor driver components:** NPN transistor + flyback diode
- **Battery:** 3.7V Li-ion battery
- **Charging module:** TP4056
- **Controller shell:** custom 3D-printed enclosure

Depending on your exact revision of the build, you may also have:

- one or more tactile buttons
- status LED(s)
- additional support circuitry for cleaner power delivery

The exact goal of this repo is not to support every possible variation.
It is to document one concrete controller build around the **ESP32 + MPU-6050** stack.

---

## Mechanical layout

The controller is built around a custom 3D-printed shell that holds the electronics in a compact handheld form.

### Inside the enclosure

- ESP32 DevKit V1
- MPU-6050
- joystick module
- vibration motor and its driver components
- battery and charging hardware
- internal wiring and cable routing

This layout was chosen for a simple reason: I wanted something I could iterate on quickly, open easily, and modify without pretending I was manufacturing a polished product.

---

## System architecture

The controller works like this:

1. The **MPU-6050** measures motion data.
2. The **ESP32** reads that data over **I2C**.
3. The ESP32 also reads **joystick** and **button** inputs.
4. The firmware applies **sensor fusion** to produce more stable orientation output.
5. The controller sends tracking and input data to the PC.
6. The PC-side software or driver interprets that data for VR use.
7. When supported, the PC can send data back for **rumble / haptics**.

At a high level, this controller is doing two jobs at once:

- tracking orientation
- acting as an input device

That combination is what makes it fun, and also what makes it annoying to debug.

---

## Wiring

You will need to wire the components directly and keep the routing as clean as possible.
Shorter, cleaner wiring will make debugging easier and reduce the chance of random issues.

### ESP32 to MPU-6050

The IMU wiring is conceptually very similar to the headset build, but here the MCU is an **ESP32** instead of an **Arduino Pro Micro**.

**Typical connections:**

```text
ESP32        MPU-6050
3V3      ->  VCC
GND      ->  GND
GPIO21   ->  SDA
GPIO22   ->  SCL
```

Note: some ESP32 boards and firmware revisions may use different I2C pins, but **GPIO21/GPIO22** is the most common default.

### Haptic motor wiring

The vibration motor should **not** be driven directly from a GPIO pin.
Use a transistor driver stage and a diode for protection.

Typical setup:

- ESP32 GPIO -> transistor base (through resistor)
- transistor switches motor ground path
- diode across motor terminals for flyback protection

### Joystick wiring

Typical joystick module wiring:

- VCC
- GND
- VRx -> ESP32 analog input
- VRy -> ESP32 analog input
- SW -> ESP32 digital input (optional)

### Notes

- Double-check SDA/SCL before powering the IMU.
- Make sure your battery polarity is correct before connecting charging hardware.
- Do not drive the vibration motor directly from the ESP32.
- Keep power wiring and signal wiring organized so debugging stays sane.

---

## Software overview

This controller depends on two main software pieces:

- **PlatformIO firmware** running on the ESP32
- **PC-side integration** that reads controller tracking/input data for VR use

You will also need:

- **VSCode**
- **PlatformIO**
- whatever PC-side test utility / driver / SteamVR-side tooling your controller build uses

---

## Build guide

## 1. Prepare the hardware

You will need:

- ESP32 DevKit V1
- MPU-6050
- joystick module
- coin vibration motor
- transistor + diode for rumble circuit
- battery + charging hardware
- custom 3D-printed controller shell
- mounting hardware, wires, and connectors

Before final assembly, test the electronics outside the shell first.

That means:

- verify the ESP32 powers on and flashes correctly
- verify the MPU-6050 is detected over I2C
- verify joystick values are read correctly
- verify the rumble motor can be triggered safely

Do not fully assemble the controller until those pieces work separately.

---

## 2. Flash the firmware

Set up the controller firmware in **VSCode with PlatformIO**.

Then:

1. connect the ESP32 over USB
2. build the project
3. upload the firmware
4. open a serial monitor and confirm the controller is outputting sane data

At this stage, your goal is simple:

- the IMU should initialize properly
- joystick input should respond correctly
- the controller should not freeze or spam garbage data

---

## 3. Validate sensor fusion

Once the board is running, validate the tracking before worrying about enclosure fitment.

Check for:

- stable stationary readings
- sensible pitch / roll / yaw behavior
- acceptable drift
- consistent update timing

If the controller tracking is bad on the desk, it will be worse in your hand.
Fix firmware issues before mechanical assembly.

---

## 4. Validate input handling

Next, test the non-tracking inputs.

That includes:

- joystick axis range
- joystick center stability
- button presses
- any deadzone logic you apply

If you support haptics already, test that here too before closing the shell.

---

## 5. Assemble the controller

Once electronics and firmware are validated separately:

1. place the **ESP32** into the enclosure
2. mount the **MPU-6050** securely
3. install the **joystick module** in its intended opening
4. mount the **vibration motor** so it can be felt clearly
5. route and secure wiring carefully
6. install the battery and charging hardware safely
7. close the shell only after confirming nothing is under stress

The biggest mechanical issues usually come from:

- wire strain
- loose IMU mounting
- joystick misalignment
- battery placement
- enclosure tolerances that looked fine in CAD but are annoying in real life

---

## 6. Connect it to your PC-side stack

Once the controller hardware is stable, connect it to the software stack you are using on the PC side.

That may include:

- a serial test utility
- BLE HID testing tools
- a custom receiver app
- a custom SteamVR driver

At this stage, the controller should behave like a real system rather than a loose collection of parts.

---

## Functionality testing

A working controller build should be able to do the following consistently:

- boot reliably
- initialize the IMU without random failures
- report orientation data smoothly
- read joystick/button input correctly
- trigger rumble when commanded
- maintain a stable connection to the PC-side stack

If one of those is flaky, fix that before adding more features.

---

## Known constraints

This build has some intentional limitations:

- **orientation tracking is IMU-based**, so drift is part of the problem space
- the controller is built around a **specific ESP32 + MPU-6050 architecture**
- the enclosure assumes **my own mechanical design choices**
- PC-side integration depends on **your exact firmware / communication stack**

This repo is not trying to be universal.
It is documenting one real controller build.

---

## Troubleshooting

### The MPU-6050 is not detected

Check:

- SDA/SCL wiring
- power and ground
- I2C pin definitions in firmware
- whether the IMU is actually awake and responding

### Tracking is noisy or drifts badly

Check:

- IMU mounting rigidity
- calibration and gyro offsets
- sensor fusion tuning
- power noise
- loop timing consistency

### The joystick behaves strangely

Check:

- ADC pin mapping
- loose wires
- joystick center drift
- deadzone logic in firmware

### The vibration motor does not work

Check:

- transistor orientation
- GPIO assignment
- battery / motor voltage path
- diode placement
- whether the motor is being driven through a proper switching stage

### The controller works on the bench but breaks after assembly

Check:

- wire strain inside the shell
- components shorting against the enclosure
- loose IMU mounting
- battery shifting under movement
- a connector partially coming loose when the shell closes

---

## Final note

This controller came from the same mindset as the headset: build fast, get close to the metal, and make the project teach you something real.

You will probably fight drift, bad wiring, input weirdness, wireless issues, and mechanical annoyances.

That is part of the point.

If you build one too, I hope it teaches you something useful.
