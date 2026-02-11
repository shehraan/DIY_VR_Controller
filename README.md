# DIY VR Controller

A custom VR controller built using an ESP32, MPU-6050, joystick input, and onboard haptics to explore the full XR input stack from embedded sensing to SteamVR integration.

This project covers real-time sensor fusion, wireless/USB transport, input handling, haptic feedback, and hardware packaging inside a custom 3D-printed enclosure, built to understand VR controller systems end-to-end.

## What this project demonstrates

- Embedded firmware development on ESP32
- IMU integration and real-time motion tracking
- Sensor fusion for VR-oriented input pipelines
- Joystick, button subsystem integration
- Haptic feedback via vibration motor
- Wireless communication between device and PC
- Fast iteration through real-world debugging and integration

## Hardware stack

### Core Components

- **Microcontroller:** ESP32 DevKit V1
- **IMU:** MPU-6050
- **Primary input:** PS2-style analog joystick
- **Haptics:** 3V coin vibration motor
- **Motor driver stage:** transistor + protection diode
- **Power system:** rechargeable Li-ion battery + TP4056 charging
- **Enclosure:** custom 3D-printed shell

### Mechanical layout

<p align="center"> <img src="Hardware\Docs\Controller_Schematic.png"> </p>

The controller is built around a custom handheld enclosure that houses:
- ESP32
- MPU-6050
- joystick module
- rumble hardware
- battery/charging hardware
- internal wiring

This was all made using a form factor that let me iterate fast, debug easily, and keep full ownership over the mechanical and electrical tradeoffs.

## System architecture

At a high level, the controller works like this:

1. The **MPU-6050** captures motion data.
2. The **ESP32** reads that data over **I2C**.
3. Firmware performs **sensor fusion / filtering** to produce more stable orientation output.
4. The ESP32 also reads **joystick and button inputs**.
5. The controller sends tracking + input data to the PC.
6. The PC-side software interprets that data for testing and VR integration.
6.5. Simultaneously, When needed, the PC can also send output data back to the controller for **rumble / haptics**.

What made this interesting was getting sensing, input, communication, and physical packaging to work together as a single coherent device.

The full build guide is in `BUILD.md`.

## Key engineering areas

### 1. Embedded sensing and IMU integration

This controller uses the **MPU-6050** IMU, paired with an **ESP32**, instead of an older Arduino.
That meant reworking the same general IMU pipeline around a newer microcontroller and communication stack.

Typical I2C wiring:

```text
ESP32        MPU-6050
3V3      ->  VCC
GND      ->  GND
GPIO21   ->  SDA
GPIO22   ->  SCL
```

This part of the project involved:

- IMU bring-up and I2C debugging
- reading raw accelerometer/gyroscope data reliably
- managing drift and noise
- tuning filtering/sensor fusion for usable motion output
- making sure the mechanical mounting supported stable readings

### 2. Input device design

The controller combines motion sensing with traditional input hardware.
That meant designing firmware that could manage both continuous orientation updates and discrete / analog inputs at the same time.

This included:

- joystick axis reading
- input stability / deadzone handling
- button state handling
- packet design for sending multiple input types together

### 3. Haptics and output control

I also integrated rumble support through a vibration motor and switching stage.
That required handling a hardware output path correctly rather than treating the motor like a simple GPIO load.

This part involved:

- transistor-based motor switching
- flyback protection
- GPIO-side control from firmware
- thinking through how PC-side software could trigger device-side feedback

### 4. PC-side integration handling

Even though the controller starts as an embedded project, it only becomes useful once it fits into a larger software stack.
A big part of the value of this build was thinking through how the device should communicate with a PC-side receiver, test app, or SteamVR-side driver.

That meant working through questions like:

- what data should be sent
- how often it should be sent
- how to balance responsiveness vs reliability
- how haptics should be handled in the reverse direction
- what kinds of failures break immersion or usability fastest
- How the data would need to be converted, in order for SteamVR to be able to 

---

## Technical challenges

A project like this sounds simple from far away, but the real work shows up in the edge cases.
Some of the recurring engineering problems were:

- **sensor drift and noisy motion data**
- **timing consistency** in the control / update loop
- **power and wiring constraints** inside a compact enclosure
- **mechanical fitment issues** that affect both reliability and feel
- **communication quirks** between the device and the PC-side stack
- **keeping the system debuggable** while multiple subsystems are changing at once

That is part of why I value this project.
It forced me to deal with the kind of real integration problems that do not show up in isolated tutorials.

---

## Relevance to my broader work

This controller is part of a larger pattern in my projects.
I am especially interested in systems that sit between software and the physical world, particularly where latency, interaction design, embedded constraints, and real-time feedback all matter.

That is why a project like this is useful for me.
It combines:

- embedded firmware
- sensor processing
- low-level device integration
- mechanical iteration
- human-computer interaction
- XR system thinking

It is the kind of project that gives me a much better mental model for how real devices are built.

---

## Final note

I built this for the same reason I build most of my hardware projects: to get close to the metal, make the tradeoffs visible, and force myself to understand the system instead of just using it.