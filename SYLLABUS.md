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
    1. [Hardware Foundations](curriculum/Support/Hardware/Hardware_Foundations.md) (Electronics, Soldering, Tools).
    2. [Module 0 Labs](curriculum/Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Labs.md) (Bench testing, Linux, Wi-Fi mapping).
    3. [Theory 0.1: The Mathematician](curriculum/Library/00_Foundations/Paper_0.1_The_Mathematician.md).
    4. [Theory 0.1: Network Survival Guide](curriculum/Library/00_Foundations/Network_0.1_Talking_to_the_Drone.md).
    5. [Theory 0.1: The Robot Class](curriculum/Library/00_Foundations/Programming_0.1_The_Robot_Class.md).
    6. [Theory 0.2: The Physics of Flight](curriculum/Library/00_Foundations/Theory_0.2_The_Physics_of_Flight.md).
    7. [Theory 0.4: System Identification](curriculum/Library/02_Control_Dynamics/Theory_0.4_System_Identification.md).
    8. [Theory 0.5: Dynamics](curriculum/Library/00_Foundations/Theory_0.5_Dynamics_Euler_Lagrange.md).
    9. [Theory 0.7: Full State-Space Model](curriculum/Library/00_Foundations/Theory_0.7_The_Full_State_Space_Model.md).
    10. [Theory 0.8: High-Speed Aerodynamics](curriculum/Library/00_Foundations/Theory_0.8_High_Speed_Aerodynamics.md).
    11. [ROS 2 Performance Tuning](curriculum/Support/Setup/Environment_ROS2_DDS_Tuning.md).
    12. [Theory 1.5: Numerical Solvers](curriculum/Library/00_Foundations/Theory_1.5_Numerical_Solvers_and_Sparsity.md).
    **Labs:**
    *   [Lab 0.1: Jitter Watchdog](src/Labs/Phase_1_The_Mechanic/Module_00_The_Build/lab_0_1_realtime_jitter.py)
    *   [Lab 0.2: Morse Code Heartbeat](src/Labs/Phase_1_The_Mechanic/Module_00_The_Build/lab_0_2_morse_code.py)
    *   [Lab 0.3: Motor Safety Test](src/Labs/Phase_1_The_Mechanic/Module_00_The_Build/lab_0_3_motor_safety.py)

*   **Module 1: The Bare Metal API**
    Writing raw Python drivers. **Prerequisite:** [Theory 0.6: Motor Mixer Matrix](curriculum/Library/02_Control_Dynamics/Theory_0.6_The_Motor_Mixer_Matrix.md).
    **Lab:** [Lab 1.1: Motor Mixer Matrix](src/Labs/Phase_1_The_Mechanic/Module_01_Bare_Metal_API/lab_1_1_mixer_matrix.py)

#### Phase II: The Test Pilot (Observability)
*   **Module 2: The Telemetry Stack (Data Engineering)**
    Integrating **Foxglove Studio** and **PlotJuggler** for real-time diagnostics.
    **Lab:** [Lab 2.1: Battery Monitor](src/Labs/Phase_2_The_Test_Pilot/Module_02_Telemetry_Stack/lab_2_3_battery_monitor.py)

*   **Module 3: FPV and HUD (Augmented Reality)**
    **Prerequisite:** [Theory 0.3: Calibration and Extrinsics](curriculum/Library/01_Sensing_Estimation/Theory_0.3_Calibration_and_Extrinsics.md).
    **Labs:** [Lab 3.1: Camera Calibration](src/Labs/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/lab_2_1_calibration.py), [Lab 3.2: SITL Bridge](src/Labs/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/lab_2_2_sitl_bridge.py)

#### Phase III: The Engineer (Control and Math)
*   **Module 3.5: SITL Simulation**
    Building the **Software-in-the-Loop** environment in PyBullet.

*   **Module 4: Signal Processing and Geometry**
    **Prerequisite:** [Theory 1: Coordinate Systems](curriculum/Library/00_Foundations/Theory_1_Coordinate_Systems.md).
    **Labs:** [Lab 4.1: SysID](src/Labs/Phase_3_The_Engineer/Module_04_Signal_Processing/lab_3_1_sysid.py), [Lab 4.2: FFT Tuning](src/Labs/Phase_3_The_Engineer/Module_04_Signal_Processing/lab_3_2_fft_tuning.py)

*   **Module 5: Control Theory**
    Deriving PID. **Prerequisites:** 
    1. [Theory 4: PID to LQR](curriculum/Library/02_Control_Dynamics/Theory_4_PID_to_LQR.md)
    2. [Theory 5.6: Nonlinear Control and LQR](curriculum/Library/02_Control_Dynamics/Theory_5.6_Nonlinear_Control_and_LQR.md)
    3. [Theory 5.7: Time-Optimal Control](curriculum/Library/02_Control_Dynamics/Theory_5.7_Time_Optimal_Control.md)
    4. [Theory 5.8: MPC](curriculum/Library/02_Control_Dynamics/Theory_5.8_Model_Predictive_Control.md)
    5. [Theory 5.9: Extreme Recovery](curriculum/Library/02_Control_Dynamics/Theory_5.9_Recovery_from_Extreme_Attitudes.md)
    **Labs:**
    *   [Lab 5.1: PID](src/Labs/Phase_3_The_Engineer/Module_05_Control_Theory/lab_3_3_pid.py)
    *   [Lab 5.2: MPC Lite](src/Labs/Phase_3_The_Engineer/Module_05_Control_Theory/lab_3_4_mpc_lite.py)
    *   [Lab 5.3: Extreme Recovery](src/Labs/Phase_3_The_Engineer/Module_05_Control_Theory/lab_3_5_recovery.py)

#### Phase IV: The Architect (Scale and Safety)
*   **Module 6: The ROS 2 Migration**
*   **Module 6.5: Failsafes and Reliability**
    **Lab:** [Lab 6.1: IMU Pre-integration](src/Labs/Phase_4_The_Architect/Module_06_5_Reliability/lab_4_3_preintegration.py)

*   **Module 7: State Estimation (The Truth)**
    **Prerequisites:** 
    1. [Theory 2: The EKF](curriculum/Library/01_Sensing_Estimation/Theory_2_The_EKF.md)
    2. [Theory 7.1: Jacobians](curriculum/Library/01_Sensing_Estimation/Theory_7.1_Deriving_Jacobians.md)
    3. [Theory 7.2: Consistency Analysis](curriculum/Support/Reference/Real_World_Engineering.md)
    4. [Theory 7.5: Allan Variance](curriculum/Library/01_Sensing_Estimation/Theory_7.5_Allan_Variance.md)
    5. [Theory 7.6: IMM Tracking](curriculum/Library/01_Sensing_Estimation/Theory_7.6_IMM_Adversarial_Tracking.md)
    **Labs:** [Lab 7.1: Allan Variance](src/Labs/Phase_4_The_Architect/Module_07_State_Estimation/lab_4_1_allan_variance.py), [Lab 7.2: EKF](src/Labs/Phase_4_The_Architect/Module_07_State_Estimation/lab_4_2_ekf.py)

#### Phase V: The Researcher (Autonomy)
*Building the brain that understands space.*

*   **Module 8: Perception and Mapping**
    **Prerequisites:** 
    1. [Theory 5: SLAM and Factor Graphs](curriculum/Library/03_Mapping_Planning/Theory_5_SLAM_and_Factor_Graphs.md)
    2. [Theory 8.5: Modern SLAM](curriculum/Library/03_Mapping_Planning/Theory_8.5_Factor_Graphs_and_Modern_SLAM.md)
    3. [Theory 8.6: Semantic Mapping](curriculum/Library/03_Mapping_Planning/Theory_8.6_Semantic_Mapping_and_Perching.md)
    **Lab:** [Lab 8.1: Occupancy Mapping](src/Labs/Phase_5_The_Researcher/Module_08_Perception_and_Mapping/lab_5_1_mapping.py)

*   **Module 9: Trajectory Optimization**
    **Prerequisites:** 
    1. [Theory 3: Splines and Optimization](curriculum/Library/03_Mapping_Planning/Theory_3_Splines_and_Optimization.md)
    2. [Theory 9.5: Path Planning](curriculum/Library/03_Mapping_Planning/Theory_9.5_Path_Planning_and_Search.md)
    **Lab:** [Lab 9.1: Trajectory Planning](src/Labs/Phase_5_The_Researcher/Module_09_Trajectory_Optimization/lab_5_2_planning.py)

#### Phase VI: The Specialist (Tactics and AI)
*   **Module 10: Reinforcement Learning**
    **Prerequisites:** 
    1. [Theory 10.1: RL Foundations](curriculum/Library/04_AI_Learning/Theory_10.1_RL_Foundations.md)
    2. [Theory 10.2: Reward Engineering](curriculum/Library/04_AI_Learning/Theory_10.2_Reward_Engineering_for_Robotics.md)
    3. [Theory 10.5: MARL](curriculum/Library/04_AI_Learning/Theory_10.5_MARL_and_Self_Play.md)
    4. [Theory 10.6: Sim-to-Real](curriculum/Library/04_AI_Learning/Theory_10.6_Sim_to_Real_Transfer.md)
    **Labs:** [Lab 10.1: Sim-to-Real RL](src/Labs/Phase_6_The_Specialist/Module_10_Reinforcement_Learning/lab_6_1_sim_to_real.py), [Lab 10.2: LNN Adapter](src/Labs/Phase_6_The_Specialist/Module_10_Reinforcement_Learning/lab_6_2_liquid_nets.py)

*   **Module 11: Aerial Combat and Guidance**
    **Prerequisites:** 
    1. [Theory 11.2: Pro-Nav](curriculum/Library/05_Tactics_Swarms/Theory_11.2_ProNav_and_Closing_Geometry.md)
    2. [Theory 11.3: Energy-Maneuverability](curriculum/Library/05_Tactics_Swarms/Theory_11.3_Energy_Maneuverability_Theory.md)
    3. [Theory 11.5: Bayesian Search](curriculum/Library/05_Tactics_Swarms/Theory_11.5_Bayesian_Search_Theory.md)
    **Labs:** [Lab 11.1: Pro-Nav Guidance](src/Labs/Phase_6_The_Specialist/Module_11_Tactics_and_Guidance/lab_6_3_pro_nav.py), [Lab 11.2: Search Heatmap](src/Labs/Phase_6_The_Specialist/Module_11_Tactics_and_Guidance/lab_6_4_search_heatmap.py)

*   **Module 12: Outdoor Autonomy**
    Integrating GPS. **Prerequisites:** 
    1. [Theory 7: Behavior Trees](curriculum/Library/03_Mapping_Planning/Theory_7_Behavior_Trees.md)
    2. [Theory 12.3: Pursuit-Evasion Games](curriculum/Library/05_Tactics_Swarms/Theory_12.3_Pursuit_Evasion_Games.md)
    3. [Theory 12.6: Autonomous Docking](curriculum/Library/05_Tactics_Swarms/Theory_12.6_Autonomous_Docking_and_Recharging.md)
    **Labs:** [Lab 12.1: Reachability](src/Labs/Phase_6_The_Specialist/Module_12_Outdoor_Autonomy/lab_6_5_reachability.py), [Lab 12.2: Docking](src/Labs/Phase_6_The_Specialist/Module_12_Outdoor_Autonomy/lab_6_6_docking.py)

#### Phase VII: The Frontier (Experimental)
*   **Module 13: Visual Inertial Odometry (VIO)**
    **Prerequisites:** 
    1. [Theory 6: VIO and Preintegration](curriculum/Library/01_Sensing_Estimation/Theory_6_VIO_and_Preintegration.md)
    **Lab:** [Lab 13.1: KLT Tracker](src/Labs/Phase_7_The_Frontier/Module_13_VIO/lab_7_1_klt_tracker.py)

*   **Module 14: Swarm Theory**
    Algebraic Graph Theory and Consensus. **Prerequisites:** 
    1. [Theory 14.3: Safety Barriers](curriculum/Library/05_Tactics_Swarms/Theory_14.3_Safety_Barriers.md)
    2. [Theory 14.4: Distributed Task Allocation](curriculum/Library/05_Tactics_Swarms/Theory_14.4_Distributed_Task_Allocation.md)
    3. [Theory 14.5: RVO Collision](curriculum/Library/05_Tactics_Swarms/Theory_14.5_Multi_Agent_Collision_Avoidance_RVO.md)
    4. [Theory 14.6: Comm-Aware Control](curriculum/Library/05_Tactics_Swarms/Theory_14.6_Communication_Aware_Swarm_Control.md)
    **Labs:** [Lab 14.1: Safety Barriers](src/Labs/Phase_7_The_Frontier/Module_14_Swarm_Theory/lab_7_2_safety_barriers.py), [Lab 14.2: RVO Collision](src/Labs/Phase_7_The_Frontier/Module_14_Swarm_Theory/lab_7_3_rvo_collision.py), [Lab 14.3: Multi-Agent Trap](src/Labs/Phase_7_The_Frontier/Module_14_Swarm_Theory/lab_7_4_multi_agent_trap.py)

*   **Module 15: Deep Perception (Advanced)**
    CNNs and Metric Learning. **Prerequisites:** 
    1. [Theory 15.5: Change Detection](curriculum/Library/03_Mapping_Planning/Theory_15.5_Spatio_Temporal_Change_Detection.md)
    2. [Theory 15.6: Acoustic Localization](curriculum/Library/01_Sensing_Estimation/Theory_15.6_Acoustic_Localization.md)
    **Labs:**
    1. [Lab 15.1: Siamese Perception](src/Labs/Phase_7_The_Frontier/Module_15_Deep_Perception/lab_7_5_siamese_perception.py)
    2. [Lab 15.2: Quantization](src/Labs/Phase_7_The_Frontier/Module_15_5_Edge_AI/lab_7_6_quantization.py)
    3. [Lab 15.3: Change Detection](src/Labs/Phase_7_The_Frontier/Module_15_5_Edge_AI/lab_7_7_change_detection.py)
    4. [Lab 15.4: 3DGS Collision](src/Labs/Phase_7_The_Frontier/Module_15_5_Edge_AI/lab_7_8_3dgs_collision.py)
    5. [Lab 15.5: Acoustic Localization](src/Labs/Phase_7_The_Frontier/Module_15_5_Edge_AI/lab_7_9_acoustic_loc.py)

---

### **The Synthesis (Capstone Specials)**
*   **[Synthesis Projects](PROJECTS.md):** (1. Navigator, 2. Guardian, 3. Ghost, 4. Twin).
*   **[THE FINAL CHALLENGE](FINAL_CHALLENGE.md):** (Path A: Tactical, Path B: Swarm, Path C: AI Researcher).

### **Support Tools**
*   [The Squid Standard](curriculum/Library/00_Foundations/The_Standard_Convention.md)
*   [The Pi Zero Survival Guide](curriculum/Support/Setup/PI_ZERO_SURVIVAL_GUIDE.md)
*   [Hardware Component Deep Dive](curriculum/Support/Hardware/HARDWARE_DEEP_DIVE.md)
*   [The Robotics Debugging Guide](curriculum/Support/Reference/Robotics_Debugging_Guide.md)
*   [The Zero-to-PhD Cheat Sheet](curriculum/Support/Reference/CHEAT_SHEET.md)
*   [Environment Engineering](curriculum/Support/Setup/Environment_Engineering.md)
*   [Hardware Reference and Pinout](curriculum/Support/Hardware/hardware_reference.md)
*   [Resources and Glossary](curriculum/Support/Reference/RESOURCES_AND_GLOSSARY.md)

### **Industrial Engineering Rigor**
*   **Continuous Integration:** Automated regression testing via GitHub Actions and `pytest`.
*   **SITL Environments:** Testing code in PyBullet before deploying to physical hardware.
*   **Black-Box Logging:** Implementation of high-frequency data logging for post-flight analysis.
*   **Hardware-in-the-Loop (HIL):** (Optional) Running the Pi Zero against a simulated physics engine.

---

### **The Hardware Platform**
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

### **Prerequisites**
*   **Mathematical Maturity:** Review our **[Mathematical Readiness Diagnostic](curriculum/Support/Reference/MATHEMATICAL_READINESS.md)** before starting.
*   **Programming**: Intermediate Python experience. Familiarity with Object-Oriented Programming (Classes) is essential.
*   **Hardware**: Willingness to solder and debug electronic circuits.
