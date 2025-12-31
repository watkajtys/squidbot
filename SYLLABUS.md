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
    3. [Theory 0.1: The Mathematician](docs/theory/Paper_0.1_The_Mathematician.md) (Complex numbers and frequency).
    4. [Theory 0.1: Network Survival Guide](docs/theory/Network_0.1_Talking_to_the_Drone.md) (SSH and Latency).
    5. [Theory 0.1: The Robot Class](docs/theory/Programming_0.1_The_Robot_Class.md) (Python OOP for hardware).
    6. [Theory 0.2: The Physics of Flight](docs/theory/Theory_0.2_The_Physics_of_Flight.md) (Torque, Throttle, and Yaw).
    7. [Theory 0.4: System Identification](docs/theory/Theory_0.4_System_Identification.md) (Measuring Mass and Thrust).
    8. [Theory 0.5: Euler-Lagrange Dynamics](docs/theory/Theory_0.5_Dynamics_Euler_Lagrange.md) (The Formal Math of Motion).
    9. [Theory 0.7: The Full State-Space Model](docs/theory/Theory_0.7_The_Full_State_Space_Model.md) (The 12-State Physics).
    10. [Theory 0.8: High-Speed Aerodynamics](docs/theory/Theory_0.8_High_Speed_Aerodynamics.md) (Drag & Blade Flapping).
    11. [ROS 2 Performance Tuning](docs/Environment_ROS2_DDS_Tuning.md) (The Pi Zero Profile).
    11. [Theory 1.5: Numerical Solvers & Sparsity](docs/theory/Theory_1.5_Numerical_Solvers_and_Sparsity.md) (Solving Ax=b).

*   **[Module 1: The Bare Metal API](docs/Module_1_The_Bare_Metal_API.md)**
    Writing raw Python drivers. **Prerequisite:** [Theory 0.6: The Motor Mixer Matrix](docs/theory/Theory_0.6_The_Motor_Mixer_Matrix.md).

#### Phase II: The Test Pilot (Observability)
*   **[Module 2: The Telemetry Stack](docs/Module_2_The_Telemetry_Stack.md)**
*   **[Module 3: FPV & HUD (Augmented Reality)](docs/Module_3_FPV_and_HUD.md)**

#### Phase III: The Engineer (Control & Math)
*   **[Module 4: Signal Processing & Geometry](docs/Module_4_Signal_Processing_and_Geometry.md)**
    **Prerequisite:** [Theory 1: Coordinate Systems](docs/theory/Theory_1_Coordinate_Systems.md).

*   **[Module 5: Control Theory](docs/Module_5_Control_Theory.md)**
    Deriving PID. **Prerequisite:** [Theory 4: PID to LQR](docs/theory/Theory_4_PID_to_LQR.md), [Theory 5.6: Nonlinear Control](docs/theory/Theory_5.6_Nonlinear_Control_and_LQR.md), [Theory 5.7: Time-Optimal Control](docs/theory/Theory_5.7_Time_Optimal_Control.md), [Theory 5.8: MPC](docs/theory/Theory_5_8_Model_Predictive_Control.md), and [Theory 5.9: Extreme Recovery](docs/theory/Theory_5.9_Recovery_from_Extreme_Attitudes.md).

#### Phase IV: The Architect (Scale & Safety)
*   **[Module 6: The ROS 2 Migration](docs/Module_6_ROS2_Migration.md)**
*   **[Module 7: State Estimation (The Truth)](docs/Module_7_State_Estimation.md)**
    **Prerequisite:** [Theory 2: The EKF](docs/theory/Theory_2_The_EKF.md), [Theory 7.1: Deriving Jacobians](docs/theory/Theory_7.1_Deriving_Jacobians.md), [Theory 7.5: Allan Variance](docs/theory/Theory_7.5_Allan_Variance.md), and [Theory 7.6: IMM Adversarial Tracking](docs/theory/Theory_7.6_IMM_Adversarial_Tracking.md).

#### Phase V: The Researcher (Autonomy)
*Building the brain that understands space.*

*   **[Module 8: Perception & Mapping](docs/Module_8_Perception_and_Mapping.md)**
    **Prerequisite:** [Theory 5: SLAM & Factor Graphs](docs/theory/Theory_5_SLAM_and_Factor_Graphs.md), [Theory 8.5: Modern SLAM](docs/theory/Theory_8.5_Factor_Graphs_and_Modern_SLAM.md), and [Theory 8.6: Semantic Mapping & Perching](docs/theory/Theory_8.6_Semantic_Mapping_and_Perching.md).

*   **[Module 9: Trajectory Optimization](docs/Module_9_Trajectory_Optimization.md)**
    **Prerequisite:** [Theory 3: Splines & Optimization](docs/theory/Theory_3_Splines_and_Optimization.md) and [Theory 9.5: Path Planning](docs/theory/Theory_9.5_Path_Planning_and_Search.md).

#### Phase VI: The Specialist (Tactics & AI)
*   **[Module 10: Reinforcement Learning](docs/Module_10_Reinforcement_Learning.md)**
    **Prerequisite:** [Theory 10.1: RL Foundations](docs/theory/Theory_10_1_RL_Foundations.md), [Theory 10.2: Reward Engineering](docs/theory/Theory_10_2_Reward_Engineering_for_Robotics.md), [Theory 10.5: MARL & Self-Play](docs/theory/Theory_10_5_MARL_and_Self_Play.md), and [Theory 10.6: Sim-to-Real Transfer](docs/theory/Theory_10_6_Sim_to_Real_Transfer.md).
*   **[Module 11: Aerial Combat & Guidance](docs/Module_11_Aerial_Combat_and_Guidance.md)**
    **Prerequisite:** [Theory 11.2: Pro-Nav & Closing Geometry](docs/theory/Theory_11.2_ProNav_and_Closing_Geometry.md), [Theory 11.3: Energy-Maneuverability](docs/theory/Theory_11.3_Energy_Maneuverability_Theory.md), and [Theory 11.5: Bayesian Search](docs/theory/Theory_11.5_Bayesian_Search_Theory.md).
*   **[Module 12: Outdoor Autonomy](docs/Module_12_Outdoor_Autonomy.md)**
    Integrating GPS. **Prerequisite:** [Theory 7: Behavior Trees](docs/theory/Theory_7_Behavior_Trees.md), [Theory 12.3: Pursuit-Evasion Games](docs/theory/Theory_12.3_Pursuit_Evasion_Games.md), and [Theory 12.6: Autonomous Docking](docs/theory/Theory_12.6_Autonomous_Docking_and_Recharging.md).

#### Phase VII: The Frontier (Experimental)
*PhD-level research using the foundational knowledge.*

*   **[Module 13: Visual Inertial Odometry (VIO)](docs/Module_13_Visual_Inertial_Odometry.md)**
    **Prerequisite:** [Theory 6: VIO & Preintegration](docs/theory/Theory_6_VIO_and_Preintegration.md) and [Lab: KLT Feature Tracking](src/labs/lab_13_klt_tracker.py).

*   **[Module 14: Swarm Theory](docs/Module_14_Swarm_Theory.md)**
    Algebraic Graph Theory & Consensus. **Prerequisite:** [Theory 14.3: Safety Barriers](docs/theory/Theory_14.3_Safety_Barriers.md), [Theory 14.4: Task Allocation](docs/theory/Theory_14.4_Distributed_Task_Allocation.md), [Theory 14.5: RVO Avoidance](docs/theory/Theory_14.5_Multi_Agent_Collision_Avoidance_RVO.md), and [Theory 14.6: Comm-Aware Control](docs/theory/Theory_14.6_Communication_Aware_Swarm_Control.md).

*   **Module 15: Deep Perception (Advanced)**
    CNNs & Metric Learning. **Prerequisite:** [Theory 15.5: Spatio-Temporal Change Detection](docs/theory/Theory_15_5_Spatio_Temporal_Change_Detection.md) and [Theory 15.6: Acoustic Localization](docs/theory/Theory_15_6_Acoustic_Localization.md).

---

### **Support Tools**
*   [The Squid Standard: Units & Conventions](docs/theory/The_Standard_Convention.md)
*   [The Robotics Debugging Guide](docs/Robotics_Debugging_Guide.md)
*   [The Zero-to-PhD Cheat Sheet](docs/CHEAT_SHEET.md)
*   [Environment Engineering (The Flight Room)](docs/Environment_Engineering.md)
*   [Hardware Reference & Pinout](docs/hardware_reference.md)
*   [Resources & Glossary](docs/RESOURCES_AND_GLOSSARY.md)

### **Numerical & Software Engineering**
*   [Python Robotics Patterns (The Rosetta Stone)](docs/theory/Python_Robotics_Patterns.md)
*   [Theory 0.6: The Motor Mixer Matrix](docs/theory/Theory_0.6_The_Motor_Mixer_Matrix.md)
*   [Theory 1.5: Numerical Solvers & Sparsity](docs/theory/Theory_1.5_Numerical_Solvers_and_Sparsity.md)
*   **Unit Testing Suite:** Run `pytest tests/` to verify lab logic.

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