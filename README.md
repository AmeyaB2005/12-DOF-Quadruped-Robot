# 12-DOF Quadruped Mobile Robot

A complete development, analysis, and implementation repository for a 12-Degree-of-Freedom (DOF) quadruped robot. This project was developed for **ME630: Introduction to Robotics** at the **Indian Institute of Technology (IIT) Goa**, focusing on real-time inverse kinematics, dynamic modeling, and wireless teleoperation.

<p align="center">
  <img src="https://github.com/user-attachments/assets/0905c24e-6de3-47c5-800b-1b224a62c530" width="600" alt="12-DOF Quadruped Robot"/>
</p>

---

# Table of Contents

- [Key Features](#key-features)
- [Demonstration](#demonstration)
- [Hardware Specifications](#hardware-specifications)
- [Prerequisites](#prerequisites)
- [Repository Structure](#repository-structure)
- [Quick Start Guide](#quick-start-guide)
- [Robotic Analysis (FK, IK, Dynamics)](#robotic-analysis)
- [Future Work](#future-work)
- [Team Members](#team-members)
- [Acknowledgements](#acknowledgements)
- [Citation](#citation)
- [License](#license)

---

# Key Features

* **12-DOF Architecture:** Symmetric configuration with 3 DOF per leg (Coxa, Femur, Tibia).

* **Analytical Inverse Kinematics:** Closed-form real-time IK solving for precise foot placement.

* **Stable Locomotion:** Implements a diagonal trot gait and smooth trajectory planning via cubic polynomial interpolation.

* **Wireless Teleoperation:** Bluetooth-enabled control for multidirectional movement and gesture execution.

* **Open Hardware:** Fully 3D-printed chassis and linkages using standard PLA.

---

# Demonstration

https://github.com/user-attachments/assets/99d48344-16b6-40cd-ba60-ab81547dad24

---

# Hardware Specifications

## Electronics

* **Controller:** Arduino Nano (with Expansion Shield)

* **Actuators:** 12x TowerPro SG90 Servo Motors

* **Communication:** HC-05 Bluetooth Module

* **Power Supply:** 2x 18650 Batteries, BMS Module, LM2596 Buck Converter

## Fabrication

* **Material:** PLA (0.2mm layer height, 30% infill)

* **Manufacturing Method:** Fused Deposition Modeling (FDM) via Ultimaker Cura

<p align="center">
<img width="332" height="307" alt="image" src="https://github.com/user-attachments/assets/277853ce-ea29-4a22-b4f3-c61840ac8f5c" />
</p>

---

# Prerequisites

To compile the firmware, modify the CAD, or run the simulations, ensure you have the following software installed:

## Firmware

* Arduino IDE (v1.8.x or newer) with the `Servo.h` and `FlexiTimer2` libraries installed.

## Simulations

* MATLAB R2023a (or later)

OR

* Python 3.8+ (with `numpy`, `matplotlib`, and `scipy`).

## Fabrication

* Ultimaker Cura v5.0+ (or your preferred slicing software).

---

# Repository Structure

All necessary files to replicate this build are organized within the following directories:

```text
├── CAD/                # Full CAD assembly (STEP) and print-ready files (STL)

├── firmware/           # Arduino code (IK solvers, gait control, servo mapping)

├── simulations/        # MATLAB/Python scripts for FK, IK, and dynamics analysis

├── docs/               # Detailed project report, diagrams, and Bill of Materials (BOM)

└── media/              # Images, GIFs, and videos of the prototype
```

---

# Quick Start Guide

## 1. Fabrication & Assembly

* Navigate to the `CAD/STL/` directory and 3D print all structural components.

* Assemble the chassis and mount the 12 SG90 servos.

* Wire the electronics according to the Wiring Diagram located in `docs/wiring_diagrams/`.

---

## 2. Firmware Architecture & Flashing

### Phase 1: Calibration (`firmware/servo_calibration/`)

* **Crucial:** Run this *before* attaching the plastic servo horns. It locks all 12 servos at exactly 90 degrees to ensure perfect mechanical alignment.

### Phase 2: Hardcoded Testing (`firmware/hardcoded_movements/`)

* Run this after full assembly. It executes a pre-programmed loop (Forward, Backward, Wave) to test the inverse kinematics and weight distribution without needing a wireless connection.

### Phase 3: Final Deployment (`firmware/bluetooth_teleoperation/`)

* The final code. It listens to the HC-05 module and translates single-character Bluetooth commands into real-time IK target coordinates.

---

## 3. Teleoperation & Android App

### Direct Download

Navigate to the `software/` folder in this repository and download the `Bluetooth_RC_Controller.apk` directly to your Android device.

### Connection Steps

1. Power on the robot.

2. Pair the HC-05 module with your Android device in your phone's Bluetooth settings (Default PIN is `0000` or `1234`).

3. Launch the app, click the gear icon, and connect to the HC-05 module. (The red LED on the HC-05 will stop flashing once connected).

4. Use the interface to drive: **F** (Forward), **B** (Backward), **L/R** (Turn), and the **Warning Triangle** (Wave Gesture).

<p align="center">
<img width="805" height="328" alt="image" src="https://github.com/user-attachments/assets/a145aec7-2ca8-41cf-9a17-1544a0025694" />
</p>

---

## 4. Running the Simulations

To verify the kinematics and dynamic models locally:

* Navigate to the `simulations/` directory.

* Run the Inverse Kinematics solver script to generate the operational workspace.

* Run the Trajectory Plotting script to visualize the joint position and velocity curves.

---

# Robotic Analysis

## Forward Kinematics (FK)

Each leg is modeled as a 3-DOF serial manipulator using Denavit-Hartenberg (DH) parameters. Forward kinematics is utilized to determine the Cartesian foot position relative to the joint angles.

### DH Parameter Table

| Link / Joint | $a$ (mm) | $\alpha$ (deg) | $d$ (mm) | $\theta$ |
|---|---|---|---|---|
| 1 (Coxa) | 55 | 90 | 0 | $\theta_1$ |
| 2 (Femur) | 77.5 | 0 | 0 | $\theta_2$ |
| 3 (Tibia) | 27.5 | 0 | 0 | $\theta_3$ |

### Homogeneous Transformation Matrices

Using the standard DH convention, the general transformation matrix from frame $i-1$ to frame $i$ is defined as:

$$
^{i-1}_{i}T = \begin{bmatrix} 
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\ 
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\ 
0 & \sin\alpha_i & \cos\alpha_i & d_i \\ 
0 & 0 & 0 & 1 
\end{bmatrix}
$$

Substituting the specific DH parameters for each link of the robot leg:

---

### 1. Base to Coxa Transformation (\(T_{01}\))

```math
^0_1T =
\begin{bmatrix}
\cos\theta_1 & 0 & \sin\theta_1 & 55\cos\theta_1 \\
\sin\theta_1 & 0 & -\cos\theta_1 & 55\sin\theta_1 \\
0 & 1 & 0 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
```

---

### 2. Coxa to Femur Transformation (\(T_{12}\))

```math
^1_2T =
\begin{bmatrix}
\cos\theta_2 & -\sin\theta_2 & 0 & 77.5\cos\theta_2 \\
\sin\theta_2 & \cos\theta_2 & 0 & 77.5\sin\theta_2 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
```

---

### 3. Femur to Tibia Transformation (\(T_{23}\))

```math
^2_3T =
\begin{bmatrix}
\cos\theta_3 & -\sin\theta_3 & 0 & 27.5\cos\theta_3 \\
\sin\theta_3 & \cos\theta_3 & 0 & 27.5\sin\theta_3 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
```

---

## Total Transformation Matrix

The cumulative homogeneous transformation matrix representing the final Cartesian position and orientation of the foot relative to the leg base is:

```math
^0_3T
=
^0_1T
\times
^1_2T
\times
^2_3T
```

### End Effector Equations

$$X(\theta_1,\theta_2,\theta_3) = \cos(\theta_1) \left[ 27.5\cos(\theta_2+\theta_3) + 77.5\cos(\theta_2) + 55 \right]$$

$$Y(\theta_1,\theta_2,\theta_3) = \sin(\theta_1) \left[ 27.5\cos(\theta_2+\theta_3) + 77.5\cos(\theta_2) + 55 \right]$$

$$Z(\theta_2,\theta_3) = 27.5\sin(\theta_2+\theta_3) + 77.5\sin(\theta_2)$$

### FK Verification Results

| Input Joint Angles | Computed Cartesian Position |
|---|---|
| $\theta_1$ = 30° | X = 116.67 mm |
| $\theta_2$ = 45° | Y = 67.36 mm |
| $\theta_3$ = -20° | Z = 66.42 mm |

---

## Inverse Kinematics (IK)

A closed-form analytical IK solver is implemented to compute necessary joint angles from a desired Cartesian foot position in real time.

### IK Equations

$$\theta_1 = \text{atan2}(Y,X)$$

$$\theta_2 = \text{atan2}(Z, r') - \text{atan2}(27.5\sin(\theta_3), 77.5 + 27.5\cos(\theta_3))$$

$$\theta_3 = \text{atan2}(\sqrt{1-D^2}, D)$$

where-

$$r = \sqrt{X^2 + Y^2}$$

$$D = \frac{r'^2 + Z^2 - 77.5^2 - 27.5^2}{2 \times 77.5 \times 27.5}$$

$$r' = r - 55$$

### IK Verification Results

| Target Cartesian Position | Computed Joint Angles |
|---|---|
| X = 62 mm | $\theta_1$ = 0° |
| Y = 0 mm | $\theta_2$ = -86.75° |
| Z = 50 mm | $\theta_3$ = 171.3° |

*Verification Error: 0.00 mm*

---

## Dynamic Modeling

The robot's dynamics were analyzed using the Lagrange-Euler formulation to estimate joint torques, inertial effects, Coriolis terms, and gravity loading during locomotion.

### Governing Equation

$$\tau = M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q)$$

### Where:

* $M(q)$ = Mass Matrix

* $C(q,\dot{q})$ = Coriolis and Centrifugal Terms

* $G(q)$ = Gravity Vector

* $\tau$ = Joint Torque Vector

This analysis confirmed that the chosen SG90 servos operate safely within acceptable torque limits for all planned gait trajectories.

<p align="center">
<img width="872" height="507" alt="image" src="https://github.com/user-attachments/assets/9d5fa83f-e79c-4305-b4ec-1fe0a96e21da" />
</p>

---

## Trajectory Planning

Smooth swing trajectories were generated utilizing cubic polynomial interpolation in joint space.

### Cubic Polynomial:

$$q(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3$$

### This trajectory planning ensures:

* Smooth foot lift-off and touchdown sequences.

* Zero-velocity boundary conditions to minimize mechanical shock.

* Stable diagonal trot gait locomotion.

### Trajectory Results:

<p align="center">
<img width="797" height="361" alt="image" src="https://github.com/user-attachments/assets/b3d7d439-f967-402e-b0b6-62be2108834d" />
</p>

---

# Acknowledgements

This project was made possible with the support, guidance, and inspiration of the following:

* **Dr. Keval Ramani:** For his continuous guidance, supervision, and support throughout the ME630 Introduction to Robotics course at IIT Goa.

* **Arijit1080:** For the open-source structural inspiration and the original Spidy The Spider Robot YouTube Demonstration, which served as the foundational mechanical concept for this project.(https://www.youtube.com/watch?v=fnMmnd9k6q8)

* **Indian Institute of Technology (IIT) Goa:** For providing the manufacturing facilities and academic resources required for development and testing.

---
