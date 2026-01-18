# DIY VR Controller

A custom VR controller built to explore the full input stack end-to-end: embedded sensing, wireless transport, haptics, input handling, and PC-side VR integration.

This project sits alongside my DIY VR headset and was built with the same goal: get past the black-box view of XR hardware by building a real system from the hardware layer up.

---

## Project summary

Most people interact with VR controllers as finished products.
I wanted to understand what it actually takes to make one.

So I built a controller around an **ESP32**, an **MPU-6050**, joystick/button input, and a custom 3D-printed enclosure, then worked through the practical problems that come with making those pieces behave like a singular coherent device.

That included:

- reading and filtering motion data from an IMU
- handling real-time input from joystick/buttons
- driving haptics from embedded firmware
- sending controller data to a PC over a custom communication path
- thinking through how a controller should plug into a broader SteamVR-style stack
- packaging the electronics into a usable handheld form factor

This is not meant to be a polished consumer controller.
It is a technical build focused on systems understanding, integration, and iteration speed.

---

## What this project demonstrates

This project is a good representation of how I like to work as an engineer:

- **I learn systems by building across layers** instead of staying inside one abstraction boundary.
- **I am comfortable close to the hardware**: wiring, power, microcontrollers, sensor interfaces, signal issues, and physical constraints.
- **I care about responsiveness and real-world behavior**, not just whether code compiles.
- **I iterate quickly**, then refine based on the actual failure modes of the system.
- **I use projects to force technical depth**, especially in areas like embedded systems, XR interfaces, and device integration.

---

## Hardware stack

### Core electronics

- **Microcontroller:** ESP32 DevKit V1
- **IMU:** MPU-6050
- **Primary input:** PS2-style analog joystick
- **Haptics:** 3V coin vibration motor
- **Motor driver stage:** transistor + protection diode
- **Power system:** rechargeable Li-ion battery + TP4056 charging
- **Enclosure:** custom 3D-printed shell

### Mechanical layout

The controller is built around a custom handheld enclosure that houses:

- ESP32
- MPU-6050
- joystick module
- rumble hardware
- battery/charging hardware
- internal wiring

The design priority was not industrial polish.
It was a form factor that let me iterate fast, debug easily, and keep full ownership over the mechanical and electrical tradeoffs.

---

## System architecture

At a high level, the controller works like this:

1. The **MPU-6050** captures motion data.
2. The **ESP32** reads that data over **I2C**.
3. Firmware performs **sensor fusion / filtering** to produce more stable orientation output.
4. The ESP32 also reads **joystick and button inputs**.
5. The controller sends tracking + input data to the PC.
6. The PC-side software interprets that data for testing and VR integration.
7. When supported, the PC can send feedback back to the controller for **rumble / haptics**.

What made this interesting was not any one subsystem in isolation.
It was getting sensing, input, communication, and physical packaging to work together as a single device.

---

## Key engineering areas

### 1. Embedded sensing and IMU integration

This controller uses the same **MPU-6050** family as my DIY headset, but here it is paired with an **ESP32** instead of an Arduino Pro Micro.
That meant reworking the same general IMU pipeline around a different microcontroller and communication stack.

Typical I2C wiring:

```text
ESP32        MPU-6050
3V3      ->  VCC
GND      ->  GND
GPIO21   ->  SDA
GPIO22   ->  SCL
```

From an engineering standpoint, this part of the project involved:

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

### 4. PC-side integration thinking

Even though the controller starts as an embedded project, it only becomes useful once it fits into a larger software stack.
A big part of the value of this build was thinking through how the device should communicate with a PC-side receiver, test app, or SteamVR-side driver.

That meant working through questions like:

- what data should be sent
- how often it should be sent
- how to balance responsiveness vs reliability
- how haptics should be handled in the reverse direction
- what kinds of failures break immersion or usability fastest

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

## Why I built it this way

I intentionally did not optimize this project for maximum modularity or broad consumer usability.

I optimized it for:

- learning the full system deeply
- shipping a real working prototype
- documenting clear technical decisions
- building intuition for embedded XR hardware

That tradeoff matters to me.
I would rather build one real controller that teaches me a lot than build a vague platform with too many moving targets.

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

## Current scope and constraints

This is a focused prototype built around a specific architecture:

- **ESP32-based**
- **MPU-6050-based** orientation sensing
- custom enclosure and wiring layout
- custom PC-side integration path

It is not presented as a universal controller platform.
It is a concrete engineering project with deliberate constraints.

---

## Final note

I built this for the same reason I build most of my hardware projects: to get close to the metal, make the tradeoffs visible, and force myself to understand the system instead of just using it.

For a recruiter, the important part is not that this is a DIY controller.
It is that I used a self-directed project like this to build skill in embedded systems, device integration, debugging, and XR hardware thinking through a real end-to-end implementation.
