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
    1. [Hardware Foundations](curriculum/support_tools/Hardware_Foundations.md) (Electronics, Soldering, Tools).
    2. [Module 0 Labs](curriculum/phase_1_mechanic/Module_0_Labs.md) (Bench testing, Linux, Wi-Fi mapping).
    3. [Theory 0.1: The Mathematician](curriculum/theory_deep_dives/Paper_0.1_The_Mathematician.md) (Complex numbers and frequency).
    **Labs:**
    *   [Lab 0.1: Jitter Watchdog](src/labs/phase_1/lab_0_1_realtime_jitter.py)
    *   [Lab 1.0: Morse Code Heartbeat](src/labs/phase_1/lab_1_morse_code.py)

*   **Module 1: The Bare Metal API**
    Writing raw Python drivers. **Prerequisite:** [Theory 0.6: Motor Mixer Matrix](curriculum/theory_deep_dives/Theory_0.6_The_Motor_Mixer_Matrix.md).
    **Lab:** [Lab 0.6: Motor Mixer Matrix](src/labs/phase_1/lab_0_6_mixers.py)

#### Phase II: The Test Pilot (Observability)
*   **Module 2: The Telemetry Stack (Data Engineering)**
    Integrating **Foxglove Studio** and **PlotJuggler** for real-time diagnostics.
    **Lab:** [Lab 0.9: Battery SoC EKF](src/labs/phase_2/lab_0_9_battery_ekf.py)

*   **Module 3: FPV and HUD (Augmented Reality)**
    **Prerequisite:** [Theory 0.3: Calibration and Extrinsics](curriculum/theory_deep_dives/Theory_0.3_Calibration_and_Extrinsics.md).
    **Lab:** [Lab 3: Camera Calibration](src/labs/phase_2/lab_3_calibration.py)

#### Phase III: The Engineer (Control and Math)
*   **Module 3.5: SITL Simulation**
    Building the **Software-in-the-Loop** environment in PyBullet.
    **Lab:** [Lab 3.5: SITL Driver Bridge](src/labs/phase_2/lab_3_5_sitl_bridge.py)

*   **Module 4: Signal Processing and Geometry**
    **Prerequisite:** [Theory 1: Coordinate Systems](curriculum/theory_deep_dives/Theory_1_Coordinate_Systems.md).
    **Lab:** [Lab 4.5: FFT Vibration Tuning](src/labs/phase_3/lab_4_5_fft_tuning.py)

*   **Module 5: Control Theory**
    Deriving PID. **Prerequisites:** 
    1. [Theory 4: PID to LQR](curriculum/theory_deep_dives/Theory_4_PID_to_LQR.md)
    2. [Theory 5.8: MPC](curriculum/theory_deep_dives/Theory_5_8_Model_Predictive_Control.md)
    **Labs:**
    *   [Lab 5: PID](src/labs/phase_3/lab_5_pid.py)
    *   [Lab 5.8: MPC Lite](src/labs/phase_3/lab_5_8_mpc_lite.py)

#### Phase IV: The Architect (Scale and Safety)
*   **Module 6: The ROS 2 Migration**
*   **Module 6.5: Failsafes and Reliability**
    Implementing watchdogs and sensor-loss fallbacks.
    **Lab:** [Lab 6: IMU Pre-integration](src/labs/phase_4/lab_6_preintegration.py)

*   **Module 7: State Estimation (The Truth)**
    **Prerequisites:** 
    1. [Theory 2: The EKF](curriculum/theory_deep_dives/Theory_2_The_EKF.md)
    2. [Theory 7.1: Jacobians](curriculum/theory_deep_dives/Theory_7.1_Deriving_Jacobians.md)
    **Lab:** [Lab 7: Extended Kalman Filter](src/labs/phase_4/lab_7_ekf.py)

#### Phase V: The Researcher (Autonomy)
*   **Module 8: Perception and Mapping**
    **Prerequisites:** 
    1. [Theory 5: SLAM and Factor Graphs](curriculum/theory_deep_dives/Theory_5_SLAM_and_Factor_Graphs.md)
    **Lab:** [Lab 15.5: Spatio-Temporal Change Detection](src/labs/phase_5/lab_15_5_change_detection.py)

*   **Module 9: Trajectory Optimization**
    **Prerequisites:** 
    1. [Theory 3: Splines and Optimization](curriculum/theory_deep_dives/Theory_3_Splines_and_Optimization.md)
    **Lab:** [Lab 9: Trajectory Planning](src/labs/phase_5/lab_9_planning.py)

#### Phase VI: The Specialist (Tactics and AI)
*   **Module 10: Reinforcement Learning**
    **Prerequisites:** 
    1. [Theory 10.1: RL Foundations](curriculum/theory_deep_dives/Theory_10.1_RL_Foundations.md)
    2. [Theory 10.6: Sim-to-Real Transfer](curriculum/theory_deep_dives/Theory_10.6_Sim_to_Real_Transfer.md)
    **Lab:** [Lab 10.7: Liquid Neural Network Gain Adapter](src/labs/phase_6/lab_10_7_liquid_nets.py)

*   **Module 11: Aerial Combat and Guidance**
    **Prerequisites:** 
    1. [Theory 11.2: Pro-Nav](curriculum/theory_deep_dives/Theory_11.2_ProNav_and_Closing_Geometry.md)
    **Lab:** [Lab 11: Pro-Nav Guidance](src/labs/phase_6/lab_11_pro_nav.py)

*   **Module 12: Outdoor Autonomy**
    Integrating GPS. **Prerequisites:** 
    1. [Theory 12.3: Reachability](curriculum/theory_deep_dives/Theory_12.3_Pursuit_Evasion_Games.md)
    **Lab:** [Lab 12.3: Reachability](src/labs/phase_6/lab_12_3_reachability.py)

#### Phase VII: The Frontier (Experimental)
*   **Module 14: Swarm Theory**
    **Lab:** [Lab 14.5: RVO Collision Avoidance](src/labs/phase_7/lab_14_5_rvo_collision.py)

*   **Module 15: Deep Perception (Advanced)**
    **Labs:**
    1. [Lab 15.7: 3DGS Collision Check](src/labs/phase_7/lab_15_7_3dgs_collision.py)
    2. [Lab 15.2: Edge AI Quantization](src/labs/phase_7/lab_15_2_quantization.py)
    3. [Lab 15.6: Acoustic Localization](src/labs/phase_7/lab_15_6_acoustic_loc.py)

### **The Synthesis (Capstone Specials)**
*   **[Synthesis Projects](PROJECTS.md):** (1. Navigator, 2. Guardian, 3. Ghost, 4. Twin).
*   **[THE FINAL CHALLENGE](FINAL_CHALLENGE.md):** (Path A: Tactical, Path B: Swarm, Path C: AI Researcher).

### **Support Tools**
*   [The Squid Standard](curriculum/theory_deep_dives/The_Standard_Convention.md)
*   [The Pi Zero Survival Guide](curriculum/support_tools/PI_ZERO_SURVIVAL_GUIDE.md)
*   [Hardware Component Deep Dive](curriculum/support_tools/HARDWARE_DEEP_DIVE.md)
*   [The Robotics Debugging Guide](curriculum/support_tools/Robotics_Debugging_Guide.md)
*   [The Zero-to-PhD Cheat Sheet](curriculum/support_tools/CHEAT_SHEET.md)
*   [Environment Engineering](curriculum/support_tools/Environment_Engineering.md)
*   [Hardware Reference and Pinout](curriculum/support_tools/hardware_reference.md)
*   [Resources and Glossary](curriculum/support_tools/RESOURCES_AND_GLOSSARY.md)

### **Industrial Engineering Rigor**
*   **Continuous Integration:** Automated regression testing via GitHub Actions and `pytest`.
*   **SITL Environments:** Testing code in PyBullet before deploying to physical hardware.
*   **Black-Box Logging:** Implementation of high-frequency data logging for post-flight analysis.
*   **Hardware-in-the-Loop (HIL):** (Optional) Running the Pi Zero against a simulated physics engine.

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
