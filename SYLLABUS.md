# The Squid Drone Syllabus
## From Soldering Iron to Swarm Intelligence

**Status:** v1.0 Production Ready
**Level:** Graduate / Advanced Undergraduate
**Hardware Cost:** <$250 USD
**Standard:** Matches ETH Zurich (Visual Computing) / UPenn (MEAM 620)
**Philosophy:** "First we prove it (Math), then we build it (Code), then we win it (Challenge)."

---

### The Mission
Most drone tutorials focus on assembling off-the-shelf components and configuring pre-written software like ArduPilot or Betaflight. While valuable, this approach treats the drone as a "Black Box."

The Squid Project takes a different approach. We build an autonomous quadrotor from first principles. We write the low-level drivers, we derive the mathematical control laws on whiteboards, and we train the Neural Networks that guide flight.

The goal is not just to fly, but to understand the physics, mathematics, and software architecture that make flight possible. This curriculum mirrors the rigor of an Autonomous Mobile Robots course found at top engineering universities, bridging the gap between theory and real-world implementation. The mission is to produce Systems Architects capable of working at the highest levels of aerospace and AI research.

---

### The Evolutionary Framework
We do not just learn the newest tools; we master the progression of technology. Every module follows this framework:
1.  **Foundations (Legacy Logic):** 1960s-1980s era math (PID, Eight-Point Algorithm, Linear Kalman Filters).
2.  **Industry Standard:** Current professional technology (MPC, EKF, IMU Pre-integration, PPO).
3.  **The Frontier:** 2025+ Research (Liquid Neural Networks, 3D Gaussian Splatting, Hamilton-Jacobi Reachability).

---

### Core Competencies
By the end of this course, you will possess a "Full Stack" robotics skillset:
*   **Embedded Systems Engineering:** Understanding Real-time Linux constraints, writing I2C/UART drivers, and managing hardware interrupts.
*   **Data Engineering & Observability:** Analyzing telemetry with industrial tools like Foxglove and PlotJuggler; implementing NIS/NEES consistency tests for state estimation.
*   **Sim-to-Real Transfer:** Building Software-in-the-Loop (SITL) simulations; mastering domain randomization to bridge the reality gap.
*   **Control Theory:** Evolution from PID to Model Predictive Control (MPC) and Hamilton-Jacobi Reachability for formal safety.
*   **Signal Processing:** Nyquist-Shannon sampling, digital filtering (Low-pass, Notch), and vibration analysis via FFTs.
*   **State Estimation:** Optimal estimation via Extended Kalman Filters (EKF), sensor fusion, and covariance analysis.
*   **Reliability Engineering:** Implementing failsafes, battery-aware flight logic, and graceful degradation under sensor loss.
*   **Robotic Architecture:** Scalable, distributed systems using ROS 2 (Robot Operating System) and Data Distribution Services (DDS).
*   **Artificial Intelligence:** Deep Reinforcement Learning (PPO), Curriculum Learning, and Adaptive Liquid Neural Networks (LNNs).
*   **Spatial Awareness:** Mapping via Occupancy Grids and 3D Gaussian Splatting (3DGS).

---

### Curriculum Roadmap

#### Phase I: The Mechanic (Hardware and Instincts)
*Building the machine and understanding its nervous system.*

*   **Module 0: The Build and Legal Compliance**
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
    10. [Theory 0.8: High-Speed Aerodynamics](docs/theory/Theory_0.8_High_Speed_Aerodynamics.md) (Drag and Blade Flapping).
    11. [ROS 2 Performance Tuning](docs/Environment_ROS2_DDS_Tuning.md) (The Pi Zero Profile).
    12. [Theory 1.5: Numerical Solvers and Sparsity](docs/theory/Theory_1.5_Numerical_Solvers_and_Sparsity.md) (Solving Ax=b).

*   **Module 1: The Bare Metal API**
    Writing raw Python drivers. **Prerequisite:** [Theory 0.6: The Motor Mixer Matrix](docs/theory/Theory_0.6_The_Motor_Mixer_Matrix.md).

#### Phase II: The Test Pilot (Observability)
*   **Module 2: The Telemetry Stack (Data Engineering)**
    Integrating **Foxglove Studio** and **PlotJuggler** for real-time diagnostics.
*   **Module 3: FPV and HUD (Augmented Reality)**
    **Prerequisite:** [Theory 0.3: Calibration and Extrinsics](docs/theory/Theory_0.3_Calibration_and_Extrinsics.md).

#### Phase III: The Engineer (Control and Math)
*   **Module 3.5: SITL Simulation**
    Building the **Software-in-the-Loop** environment in Gazebo/PyBullet.
*   **Module 4: Signal Processing and Geometry**
    **Prerequisite:** [Theory 1: Coordinate Systems](docs/theory/Theory_1_Coordinate_Systems.md).
*   **Module 5: Control Theory**
    Deriving PID. **Prerequisites:** 
    1. [Theory 4: PID to LQR](docs/theory/Theory_4_PID_to_LQR.md)
    2. [Theory 5.6: Nonlinear Control and LQR](docs/theory/Theory_5.6_Nonlinear_Control_and_LQR.md)
    3. [Theory 5.7: Time-Optimal Control](docs/theory/Theory_5.7_Time_Optimal_Control.md)
    4. [Theory 5.8: MPC](docs/theory/Theory_5.8_Model_Predictive_Control.md) ([Lab 5.8: MPC Lite](src/labs/lab_5_8_mpc_lite.py))
    5. [Theory 5.9: Extreme Recovery](docs/theory/Theory_5.9_Recovery_from_Extreme_Attitudes.md) ([Lab 5.9: Recovery](src/labs/lab_5_9_recovery.py))

#### Phase IV: The Architect (Scale and Safety)
*   **Module 6: The ROS 2 Migration**
*   **Module 6.5: Failsafes and Reliability**
    Implementing watchdogs, battery-critical logic, and sensor-loss fallbacks.
*   **Module 7: State Estimation (The Truth)**
    **Prerequisites:** 
    1. [Theory 2: The EKF](docs/theory/Theory_2_The_EKF.md)
    2. [Theory 7.1: Deriving Jacobians](docs/theory/Theory_7.1_Deriving_Jacobians.md)
    3. [Theory 7.2: Consistency Analysis (NIS/NEES)](docs/theory/Theory_7.2_Consistency_Tests.md)
    4. [Theory 7.5: Allan Variance](docs/theory/Theory_7.5_Allan_Variance.md) ([Lab 7.5: Allan Variance](src/labs/lab_7_5_allan_variance.py))
    5. [Theory 7.6: IMM Adversarial Tracking](docs/theory/Theory_7.6_IMM_Adversarial_Tracking.md)

#### Phase V: The Researcher (Autonomy)
*Building the brain that understands space.*

*   **Module 8: Perception and Mapping**
    **Prerequisites:** 
    1. [Theory 5: SLAM and Factor Graphs](docs/theory/Theory_5_SLAM_and_Factor_Graphs.md)
    2. [Theory 8.5: Modern SLAM](docs/theory/Theory_8.5_Factor_Graphs_and_Modern_SLAM.md)
    3. [Theory 8.6: Semantic Mapping and Perching](docs/theory/Theory_8.6_Semantic_Mapping_and_Perching.md)
*   **Module 9: Trajectory Optimization**
    **Prerequisites:** 
    1. [Theory 3: Splines and Optimization](docs/theory/Theory_3_Splines_and_Optimization.md)
    2. [Theory 9.5: Path Planning and Search](docs/theory/Theory_9.5_Path_Planning_and_Search.md)

#### Phase VI: The Specialist (Tactics and AI)
*   **Module 10: Reinforcement Learning**
    **Prerequisites:** 
    1. [Theory 10.1: RL Foundations](docs/theory/Theory_10.1_RL_Foundations.md)
    2. [Theory 10.2: Reward Engineering for Robotics](docs/theory/Theory_10.2_Reward_Engineering_for_Robotics.md)
    3. [Theory 10.5: MARL and Self-Play](docs/theory/Theory_10.5_MARL_and_Self_Play.md)
    4. [Theory 10.6: Sim-to-Real Transfer](docs/theory/Theory_10.6_Sim_to_Real_Transfer.md)
*   **Module 11: Aerial Combat and Guidance**
    **Prerequisites:** 
    1. [Theory 11.2: Pro-Nav and Closing Geometry](docs/theory/Theory_11.2_ProNav_and_Closing_Geometry.md)
    2. [Theory 11.3: Energy-Maneuverability Theory](docs/theory/Theory_11.3_Energy_Maneuverability_Theory.md)
    3. [Theory 11.5: Bayesian Search Theory](docs/theory/Theory_11.5_Bayesian_Search_Theory.md) ([Lab 11.5: Search Heatmap](src/labs/lab_11_5_search_heatmap.py))
*   **Module 12: Outdoor Autonomy**
    Integrating GPS. **Prerequisites:** 
    1. [Theory 7: Behavior Trees](docs/theory/Theory_7_Behavior_Trees.md)
    2. [Theory 12.3: Pursuit-Evasion Games](docs/theory/Theory_12.3_Pursuit_Evasion_Games.md)
    3. [Theory 12.6: Autonomous Docking and Recharging](docs/theory/Theory_12.6_Autonomous_Docking_and_Recharging.md)

#### Phase VII: The Frontier (Experimental)
*PhD-level research using foundational knowledge.*

*   **Module 13: Visual Inertial Odometry (VIO)**
    **Prerequisites:** 
    1. [Theory 6: VIO and Preintegration](docs/theory/Theory_6_VIO_and_Preintegration.md)
    2. [Lab: KLT Feature Tracking](src/labs/lab_13_klt_tracker.py)
*   **Module 14: Swarm Theory**
    Algebraic Graph Theory and Consensus. **Prerequisites:** 
    1. [Theory 14.3: Safety Barriers](docs/theory/Theory_14.3_Safety_Barriers.md)
    2. [Theory 14.4: Distributed Task Allocation](docs/theory/Theory_14.4_Distributed_Task_Allocation.md) ([Lab 14.4: Multi-Agent Trap](src/labs/lab_14_4_multi_agent_trap.py))
    3. [Theory 14.5: Multi-Agent Collision Avoidance (RVO)](docs/theory/Theory_14.5_Multi_Agent_Collision_Avoidance_RVO.md)
    4. [Theory 14.6: Communication-Aware Swarm Control](docs/theory/Theory_14.6_Communication_Aware_Swarm_Control.md)
*   **Module 15: Deep Perception (Advanced)**
    CNNs and Metric Learning. **Prerequisites:** 
    1. [Theory 15.5: Spatio-Temporal Change Detection](docs/theory/Theory_15.5_Spatio_Temporal_Change_Detection.md)
    2. [Theory 15.6: Acoustic Localization](docs/theory/Theory_15.6_Acoustic_Localization.md)

### **Support Tools**
*   [The Squid Standard: Units and Conventions](docs/theory/The_Standard_Convention.md)
*   [The Robotics Debugging Guide](docs/Robotics_Debugging_Guide.md)
*   [The Zero-to-PhD Cheat Sheet](docs/CHEAT_SHEET.md)
*   [Environment Engineering (The Flight Room)](docs/Environment_Engineering.md)
*   [Hardware Reference and Pinout](docs/hardware_reference.md)
*   [Resources and Glossary](docs/RESOURCES_AND_GLOSSARY.md)

### **Industrial Engineering Rigor**
*   **Continuous Integration:** Automated regression testing via GitHub Actions and `pytest`.
*   **SITL Environments:** Testing code in Gazebo before deploying to physical hardware.
*   **Black-Box Logging:** Implementation of high-frequency data logging for post-flight analysis.
*   **Hardware-in-the-Loop (HIL):** (Optional) Running the Pi Zero against a simulated physics engine.

*   [Python Robotics Patterns (The Rosetta Stone)](docs/theory/Python_Robotics_Patterns.md)
*   [Theory 0.6: The Motor Mixer Matrix](docs/theory/Theory_0.6_The_Motor_Mixer_Matrix.md)
*   [Theory 1.5: Numerical Solvers and Sparsity](docs/theory/Theory_1.5_Numerical_Solvers_and_Sparsity.md)
*   **Unit Testing Suite:** Run `pytest tests/` to verify lab logic before flight.

---

### The Hardware Platform
The Squid is a custom-designed micro-drone optimized for indoor autonomy and educational accessibility.
*   **Frame**: BetaFPV Pavo20 (2-inch ducted CineWhoop)
*   **Compute**: Raspberry Pi Zero 2 W (Quad-core Linux, 512MB RAM)
*   **Flight Controller**: Betaflight F405 (STM32 microcontroller)
*   **Sensors**:
    *   **Mapping**: VL53L5CX (8x8 Zone Time-of-Flight Matrix)
    *   **Range**: 2x VL53L1X (1D Lidar for Altitude and Ceiling Lock)
    *   **Vision**: Arducam IMX219 (160-degree Fisheye Lens)
    *   **Optical Flow**: PMW3901 (Velocity estimation)
    *   **Position**: M10Q GPS + Compass (Outdoor navigation)

---

### Prerequisites
*   **Programming**: Intermediate Python experience. Familiarity with Object-Oriented Programming (Classes) is essential.
*   **Mathematics**: Basic Calculus (Derivatives/Integrals) and Linear Algebra (Matrix operations).
*   **Hardware**: Willingness to solder and debug electronic circuits.