# The Squid Drone Integrated Course Guide
**Focus:** Hardware -> Software -> Control -> Estimation -> Intelligence.

This document links the theoretical Lectures to the practical Study Guides and the physical Squid Games challenges. Follow this step-by-step path to master the full robotics stack.

---

### Phase I: The Mechanic (Hardware Foundations)
1.  **Module 0: Systems Engineering**
    *   Learn: [Lecture 0](LECTURES.md#lecture-0-systems-engineering) and [Study Guide 0](study_guides/Module_0_Study_Guide.md).
    *   Do: [Module 0 Bench Labs](Module_0_Labs.md) (Power audit, Linux shell, Wi-Fi mapping).
    *   Win: **Morse Heartbeat Challenge** (Verify LED timing jitter).
2.  **Module 1: The Bare Metal API**
    *   Learn: [Lecture 1](LECTURES.md#lecture-1-embedded-communication--numerics) and [Study Guide 1](study_guides/Module_1_Study_Guide.md).
    *   Do: [Lab 1: Mixers](../src/labs/lab_0_6_mixers.py) (Writing the Motor Mixer Matrix).
    *   Win: **Three-Motor Limp** (Verify fault-tolerant mixing logic).

### Phase II: The Test Pilot (Observability)
3.  **Module 2: Telemetry Stack**
    *   Learn: [Lecture 2](LECTURES.md#lecture-2-stochastic-processes) and [Study Guide 2](study_guides/Module_2_Study_Guide.md).
    *   Do: [Dashboard Setup](Module_2_The_Telemetry_Stack.md) (Real-time plotting and CRC validation).
    *   Win: **Lag-Free HUD** (Verify sub-50ms data streaming latency).
4.  **Module 3: Sensing the World**
    *   Learn: [Lecture 3](LECTURES.md#lecture-3-computer-vision-fundamentals) and [Study Guide 3](study_guides/Module_3_Study_Guide.md).
    *   Do: [Lab 3: Calibration](../src/labs/lab_3_calibration.py) (Fisheye rectification).
    *   Win: **Virtual Horizon** (Verify IMU-to-Camera extrinsic alignment).

### Phase III: The Engineer (Dynamics and Control)
5.  **Module 4: Signal Processing**
    *   Learn: [Lecture 4](LECTURES.md#lecture-4-rotations--lie-groups-part-i) and [Study Guide 4](study_guides/Module_4_Study_Guide.md).
    *   Do: [Lab 4: SysID](../src/labs/lab_4_sysid.py) (Identifying Mass and Thrust constants).
    *   Win: **The Notch Filter** (Verify 15dB motor vibration reduction).
6.  **Module 5: Control Theory**
    *   Learn: [Lecture 5](LECTURES.md#lecture-5-control-theory) and [Study Guide 5](study_guides/Module_5_Study_Guide.md).
    *   Do: [Lab 5: PID](../src/labs/lab_5_pid.py) (Attitude and Altitude control).
    *   Win: **The Statue Hover Challenge** (Maintain 20cm RMS position error).

### Phase IV: The Architect (Scale and Safety)
7.  **Module 6: ROS 2 Migration**
    *   Learn: [Study Guide 6](study_guides/Module_6_Study_Guide.md) and [Migration Guide](Module_6_ROS2_Migration.md).
    *   Do: Build the scaffolded ROS 2 Workspace (`colcon build`).
    *   Win: **The Latency Hunt** (Verify <5ms transport delay between nodes).
8.  **Module 7: State Estimation**
    *   Learn: [Lecture 7](LECTURES.md#lecture-6--7-state-estimation-the-kalman-filter) and [Study Guide 7](study_guides/Module_7_Study_Guide.md).
    *   Do: [Lab 7: EKF](../src/labs/lab_7_ekf.py) (Fusing IMU, Lidar, and GPS).
    *   Win: **The Paper Trick** (Verify altitude outlier rejection).

### Phase V: The Researcher (Autonomy)
9.  **Module 8: Perception and Mapping**
    *   Learn: [Lecture 9](LECTURES.md#lecture-9-slam--geometry) and [Study Guide 8](study_guides/Module_8_Study_Guide.md).
    *   Do: [Lab 8: Mapping](../src/labs/lab_8_6_perch_finder.py) (Building Occupancy Grids).
    *   Win: **The Mirror Maze Challenge** (Verify reflection noise filtering).
10. **Module 9: Trajectory Optimization**
    *   Learn: [Lecture 9.5](LECTURES.md#lecture-95-path-planning--search) and [Study Guide 9](study_guides/Module_9_Study_Guide.md).
    *   Do: [Lab 9: Planning](../src/labs/lab_9_planning.py) (A* and Minimum Snap).
    *   Win: **The Maze Runner** (Verify planning optimality and smoothness).

### Phase VI: The Specialist (Tactics and AI)
11. **Module 10: Reinforcement Learning**
    *   Learn: [Study Guide 10](study_guides/Module_10_Study_Guide.md).
    *   Do: [Lab 10: RL](../src/labs/lab_10_6_sim_to_real.py) (Sim-to-Real Domain Randomization).
    *   Win: **Noise Resilience Challenge** (Verify policy survival at 50ms action latency).
12. **Module 11: Aerial Combat**
    *   Learn: [Lecture 12](LECTURES.md#lecture-11-reinforcement-learning) and [Study Guide 11](study_guides/Module_11_Study_Guide.md).
    *   Do: [Lab 11: Guidance](../src/labs/lab_11_pro_nav.py) (Proportional Navigation).
    *   Win: **Fox Two Lock-on** (Verify <20cm interception miss distance).
13. **Module 12: Outdoor Autonomy**
    *   Learn: [Lecture 12.6](LECTURES.md#lecture-123-pursuit-evasion-games) and [Study Guide 12](study_guides/Module_12_Study_Guide.md).
    *   Do: [Lab 12: Docking](../src/labs/lab_12_6_docking.py) (Behavior Tree navigation).
    *   Win: **Precision Dock Challenge** (Verify 5cm final approach accuracy).

### Phase VII: The Frontier (Experimental Research)
14. **Module 13: Visual Inertial Odometry (VIO)**
    *   Learn: [Lecture 13](LECTURES.md#lecture-13-visual-inertial-odometry-vio) and [Study Guide 13](study_guides/Module_13_Study_Guide.md).
    *   Do: [Lab 13: KLT](../src/labs/lab_13_klt_tracker.py) (Tightly-coupled fusion).
    *   Win: **The Shadow Hunt** (Verify feature tracking in low-light conditions).
15. **Module 14: Swarm Theory**
    *   Learn: [Lecture 14](LECTURES.md#lecture-14-swarm-theory) and [Study Guide 14](study_guides/Module_14_Study_Guide.md).
    *   Do: [Lab 14: Swarms](../src/labs/lab_14_safety_barriers.py) (Consensus and CBFs).
    *   Win: **Consensus Dance** (Verify three-drone formation stability).
16. **Module 15: Deep Perception**
    *   Learn: [Lecture 15](LECTURES.md#lecture-15-deep-perception) and [Study Guide 15](study_guides/Module_15_Study_Guide.md).
    *   Do: [Lab 15: Siamese](../src/labs/lab_15_siamese_perception.py) (Metric learning).
    *   Win: **The Hallucination Challenge** (Verify zero false-positive loop closures).