# EVA-H: High-Degree-of-Freedom Bipedal Locomotion Platform

## 1. Abstract

The **EVA-H (Experimental V-type Articulated Humanoid)** is a research-grade bipedal platform engineered to investigate stable gait synthesis in low-cost, high-jitter servo environments. By utilizing a non-linear S-Curve velocity profile and a synchronous state-machine gait engine, EVA-H mitigates structural oscillations typical in hobby-scale robotics, achieving quasi-static stability during terrestrial locomotion.

---

## 2. System Architecture

The platform is built on an **ESP32-WROOM-32** microcontroller, facilitating high-frequency PWM signal generation and precise interpolation across 8 Degrees of Freedom (DoF).

### Technical Specifications

* **Controller:** ESP32 (Dual-core 240MHz, 520KB SRAM).
* **Actuation:** 8x High-torque digital servos.
* **Control Loop:** 50Hz asynchronous state-machine with hardware-accelerated PWM.
* **Kinematics:** Custom Inverse Kinematic (IK) coupling for ankle-roll stabilization.
* **Structural Design:** 3D-printed modular chassis with optimized center-of-mass (CoM) positioning.

---

## 3. Mathematical Foundations: The Gait Engine

The locomotion engine utilizes a **5-Phase Temporal Sequencer**. Unlike naive linear interpolation, the motion profile implements an S-Curve (Sigmoid) velocity function to minimize jerk, effectively dampening the mechanical impact of the `PLACE` phase.

### Jerk Derivative Function

$$j = \frac{da}{dt}$$

### Control Logic Formula

$$\theta(t) = \theta_{start} + (\theta_{end} - \theta_{start}) \cdot \sigma(t)$$

*Where $\sigma(t)$ represents the normalized sigmoid transition mapping.*

---

## 4. Hardware Integration & Pin Mapping

The platform requires an isolated power topology to prevent back-EMF noise from the servos from disrupting the ESP32 logic level.

| Actuator Function | GPIO Assignment |
| --- | --- |
| Left Hip Roll | GPIO 13 |
| Left Hip Pitch | GPIO 12 |
| Left Knee Pitch | GPIO 14 |
| Left Ankle Roll | GPIO 27 |
| Right Hip Roll | GPIO 26 |
| Right Hip Pitch | GPIO 25 |
| Right Knee Pitch | GPIO 33 |
| Right Ankle Roll | GPIO 32 |

---

## 5. Implementation Protocol

1. **Environment Setup:** Configure Arduino IDE with Espressif Systems ESP32 board manager (v3.0+).
2. **Power Topology:** Utilize a 6V/5A external regulator. **Mandatory:** Bridge the common ground (GND) between the servo power supply and the ESP32 to maintain signal integrity.
3. **Firmware Deployment:** Clone this repository and compile via the `src/` directory.

---

## 6. Project Roadmap & Contribution

This is an open-research platform. Contributions to the gait-stabilization algorithms and structural optimization are encouraged. Please consult the repository documentation for coding standards.

---

*Developed by YASH-SHARMA32. Researching the future of open-source bipedal robotics.*

---














## 📷 Complete Project Media Library

### 🛠️ Hardware & Assembly Gallery
<p align="center">
  <img src="WhatsApp Image 2026-05-22 at 6.25.29 PM.jpeg" width="24%" alt="EVA-H Initial Setup" />
  <img src="WhatsApp Image 2026-05-24 at 9.54.15 PM.jpeg" width="24%" alt="EVA-H Structural Build" />
  <img src="WhatsApp Image 2026-05-24 at 9.54.37 PM.jpeg" width="24%" alt="EVA-H Lower Kinematics" />
  <img src="WhatsApp Image 2026-05-24 at 9.54.59 PM.jpeg" width="24%" alt="EVA-H Leg Alignment" />
</p>

<p align="center">
  <img src="WhatsApp Image 2026-05-24 at 10.24.14 PM.jpeg" width="24%" alt="EVA-H Front View" />
  <img src="WhatsApp Image 2026-05-24 at 10.24.15 PM.jpeg" width="24%" alt="EVA-H Side Profile" />
  <img src="WhatsApp Image 2026-05-24 at 10.24.16 PM.jpeg" width="24%" alt="EVA-H Actuator Configuration" />
  <img src="WhatsApp Image 2026-05-24 at 10.24.17 PM.jpeg" width="24%" alt="EVA-H Joint Linkage" />
</p>

<p align="center">
  <img src="WhatsApp Image 2026-05-24 at 10.24.18 PM.jpeg" width="31%" alt="EVA-H Close-up" />
  <img src="WhatsApp Image 2026-05-24 at 10.24.29 PM.jpeg" width="31%" alt="EVA-H Electronic Wiring" />
</p>

---

### 🎥 Empirical Testing & Locomotion Videos

Click the links below to view the live capture videos of the bipedal locomotion tests:

* 🟢 **[Dynamic Walking Gait Integration Test](WhatsApp%20Video%202026-05-24%20at%2010.22.33%20PM.mp4)** — *Initial S-curve sweep validation on open platform.*
* 🟢 **[Active Closed-Loop Balance Test](WhatsApp%20Video%202026-05-24%20at%2010.22.37%20PM.mp4)** — *Real-time MPU6050 complementary filter feedback verification.*
* 🟢 **[System Stress & Power Stability Run](WhatsApp%20Video%202026-05-24%20at%209.57.39%20PM.mp4)** — *High-current multi-servo synchronous execution check.*
