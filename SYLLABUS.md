# The Squid Drone Project
## From Soldering Iron to Swarm Intelligence

**Status:** Open Source Curriculum
**Level:** Graduate / Advanced Undergraduate
**Hardware Cost:** <$250 USD

---

### The Mission

Most drone tutorials focus on assembling off-the-shelf components and configuring pre-written software like ArduPilot or Betaflight. While valuable, this approach treats the drone as a "Black Box."

The Squid Project takes a different approach. We build an autonomous quadrotor from first principles. We write the low-level drivers, we derive the mathematical control laws on whiteboards, and we train the Neural Networks that guide flight.

The goal is not just to fly, but to understand the physics, mathematics, and software architecture that make flight possible. This curriculum mirrors the rigor of an Autonomous Mobile Robots course found at top engineering universities, bridging the gap between theory and real-world implementation.

---

### Core Competencies

By the end of this course, you will possess a "Full Stack" robotics skillset:

*   **Embedded Systems Engineering:** Understanding Real-time Linux constraints, writing I2C/UART drivers, and managing hardware interrupts.
*   **Control Systems Design:** Deriving PID controllers, analyzing stability margins, and performing physical System Identification.
*   **Signal Processing:** Implementing digital filters (Low-pass, Notch) and analyzing sensor noise using Allan Variance and FFTs.
*   **State Estimation:** fusing noisy sensor data (Lidar, IMU, Optical Flow) into a coherent state estimate using Extended Kalman Filters (EKF).
*   **Robotic Architecture:** Designing modular, scalable systems using ROS 2 (Robot Operating System) and Data Distribution Services (DDS).
*   **Artificial Intelligence:** Implementing Reinforcement Learning (PPO) pipelines, Sim-to-Real transfer strategies, and computer vision algorithms.

---

### Curriculum Roadmap

#### Phase I: The Mechanic (Hardware & Instincts)
*Building the machine and understanding its nervous system.*

*   **Module 0: The Build & Legal Compliance**
    Assembly of the custom Pavo20 frame. 
    **Prerequisites:** 
    1. [Hardware Foundations](docs/Hardware_Foundations.md) (Electronics, Soldering, Tools).
    2. [Module 0 Labs](docs/Module_0_Labs.md) (Bench testing, Linux, Wi-Fi mapping).
    3. [Deep Dive 0.1: The Mathematician](docs/deep_dives/Paper_0.1_The_Mathematician.md) (Understanding complex numbers and frequency).

*   **Module 1: The Bare Metal API**
    Writing raw Python drivers.

#### Phase II: The Test Pilot (Observability)
*   **Module 2: The Telemetry Stack**
*   **Module 3: FPV & Augmented Reality**

#### Phase III: The Engineer (Control & Math)
*   **Module 4: Signal Processing & Geometry**
    **Prerequisite:** [Theory 1: Coordinate Systems](docs/Theory_1_Coordinate_Systems.md).

*   **Module 5: Control Theory**
    Deriving PID. **Prerequisite:** [Theory 4: PID to LQR](docs/deep_dives/Theory_4_PID_to_LQR.md).

#### Phase IV: The Architect (Scale & Safety)
*   **Module 6: The ROS 2 Migration**
*   **Module 7: State Estimation (The Truth)**
    **Prerequisite:** [Theory 2: The EKF](docs/deep_dives/Theory_2_The_EKF.md).

#### Phase V: The Researcher (Autonomy)
*Building the brain that understands space.*

*   **Module 8: Perception & Mapping**
    **Prerequisite:** [Theory 5: SLAM & Factor Graphs](docs/deep_dives/Theory_5_SLAM_and_Factor_Graphs.md).

*   **Module 9: Trajectory Optimization**
    **Prerequisite:** [Theory 3: Splines & Optimization](docs/deep_dives/Theory_3_Splines_and_Optimization.md).

#### Phase VI: The Specialist (Tactics & AI)
*   **Module 10: Reinforcement Learning**
*   **Module 11: Aerial Combat & Guidance**
*   **Module 12: Outdoor Autonomy**
    Integrating GPS. **Prerequisite:** [Theory 7: Behavior Trees](docs/deep_dives/Theory_7_Behavior_Trees.md).

#### Phase VII: The Frontier (Experimental)
*PhD-level research using the foundational knowledge.*

*   **Module 13: Visual Inertial Odometry (VIO)**
    **Prerequisite:** [Theory 6: VIO & Preintegration](docs/deep_dives/Theory_6_VIO_and_Preintegration.md).

---

### **Support Tools**
*   [The Robotics Debugging Guide](docs/Robotics_Debugging_Guide.md)
*   [The Zero-to-PhD Cheat Sheet](docs/CHEAT_SHEET.md)

*   **Module 14: Swarm Theory**


---

### The Hardware Platform

The "Squid" is a custom-designed micro-drone optimized for indoor autonomy and educational accessibility.

*   **Frame:** BetaFPV Pavo20 (2-inch ducted CineWhoop)
*   **Compute:** Raspberry Pi Zero 2 W (Quad-core Linux, 512MB RAM)
*   **Flight Controller:** Betaflight F405 (STM32 microcontroller)
*   **Sensors:**
    *   **Mapping:** VL53L5CX (8x8 Zone Time-of-Flight Matrix)
    *   **Range:** 2x VL53L1X (1D Lidar for Altitude & Ceiling Lock)
    *   **Vision:** Arducam IMX219 (160-degree Fisheye Lens)
    *   **Optical Flow:** PMW3901 (Velocity estimation)
    *   **Position:** M10Q GPS + Compass (Outdoor navigation)

---

### Prerequisites

*   **Programming:** Intermediate Python experience. Familiarity with Object-Oriented Programming (Classes) is essential.
*   **Mathematics:** Basic Calculus (Derivatives/Integrals) and Linear Algebra (Matrix operations).
*   **Hardware:** Willingness to solder and debug electronic circuits.