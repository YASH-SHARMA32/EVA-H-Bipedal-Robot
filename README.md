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














## 📷 Hardware & Build Gallery

Here are the physical build and testing images for the EVA-H platform:

<p align="center">
  <img src="WhatsApp Image 2026-05-24 at 10.24.14 PM.jpeg" width="30%" alt="EVA-H Front View" />
  <img src="WhatsApp Image 2026-05-24 at 10.24.15 PM.jpeg" width="30%" alt="EVA-H Side Profile" />
  <img src="WhatsApp Image 2026-05-24 at 10.24.16 PM.jpeg" width="30%" alt="EVA-H Actuators" />
</p>
