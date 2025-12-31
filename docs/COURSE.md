# The Squid Drone Integrated Course Guide
**Focus:** Hardware -> Software -> Control -> Estimation -> Intelligence.

This document links the theoretical Lectures to the practical Study Guides and the physical Squid Games challenges. Follow this step-by-step path to master the full robotics stack.

---

### Phase I: The Mechanic (Hardware Foundations)
1.  **Module 0: Systems Engineering**
    *   Learn: [Lecture 0](LECTURES.md#lecture-0-systems-engineering) and [Study Guide 0](study_guides/Module_0_Study_Guide.md).
    *   Do: [Lab 0.1: Jitter Watchdog](../src/labs/lab_0_1_realtime_jitter.py) (Measure loop determinism).
    *   Do: [Module 0 Bench Labs](Module_0_Labs.md) (Power audit, Linux shell, Wi-Fi mapping).
    *   Win: **Morse Heartbeat Challenge** (Verify LED timing jitter).
2.  **Module 1: The Bare Metal API**
    *   Learn: [Lecture 1](LECTURES.md#lecture-1-embedded-communication--numerics) and [Study Guide 1](study_guides/Module_1_Study_Guide.md).
    *   Do: [Lab 1: Mixers](../src/labs/lab_0_6_mixers.py) (Writing the Motor Mixer Matrix).
    *   Win: **Three-Motor Limp** (Verify fault-tolerant mixing logic).

### Phase II: The Test Pilot (Observability)
3.  **Module 2: Telemetry Stack**
    *   Learn: [Lecture 2](LECTURES.md#lecture-2-stochastic-processes) and [Study Guide 2](study_guides/Module_2_Study_Guide.md).
    *   Do: [Lab 0.9: Battery EKF](../src/labs/lab_0_9_battery_ekf.py) (Real-time SoC estimation).
    *   Do: [Dashboard Setup](Module_2_The_Telemetry_Stack.md) (Real-time plotting and CRC validation).
    *   Win: **Lag-Free HUD** (Verify sub-50ms data streaming latency).
4.  **Module 3: Sensing the World**
    *   Learn: [Lecture 3](LECTURES.md#lecture-3-computer-vision-fundamentals) and [Study Guide 3](study_guides/Module_3_Study_Guide.md).
    *   Do: [Lab 3: Calibration](../src/labs/lab_3_calibration.py) (Fisheye rectification).
    *   Do: [Lab 3.5: SITL Bridge](../src/labs/lab_3_5_sitl_bridge.py) (Connecting MSP to PyBullet).
    *   Win: **Virtual Horizon** (Verify IMU-to-Camera extrinsic alignment).

### Phase III: The Engineer (Dynamics and Control)
5.  **Module 4: Signal Processing**
    *   Learn: [Study Guide 4](study_guides/Module_4_Study_Guide.md) and [SysID Lecture](LECTURES.md#lecture-55-system-identification-sysid).
    *   Do: [Lab 4.5: FFT Tuning](../src/labs/lab_4_5_fft_tuning.py) (Resonance analysis).
    *   Win: **The Notch Filter** (Verify 15dB motor vibration reduction).
6.  **Module 5: Control Theory**
    *   Learn: [Lecture 5](LECTURES.md#lecture-5-control-theory) and [Study Guide 5](study_guides/Module_5_Study_Guide.md).
    *   Do: [Lab 5: PID](../src/labs/lab_5_pid.py) and [Lab 5.8: MPC Lite](../src/labs/lab_5_8_mpc_lite.py).
    *   Win: **The Statue Hover Challenge** (Maintain 20cm RMS position error).

### Phase IV: The Architect (Scale and Safety)
7.  **Module 6: ROS 2 Migration**
    *   Learn: [Study Guide 6](study_guides/Module_6_Study_Guide.md) and [Migration Guide](Module_6_ROS2_Migration.md).
    *   Do: [Lab 6: IMU Pre-integration](../src/labs/lab_6_preintegration.py) (Manifold optimization).
    *   Win: **The Latency Hunt** (Verify <5ms transport delay between nodes).
8.  **Module 7: State Estimation**
    *   Learn: [Lecture 7](LECTURES.md#lecture-6--7-state-estimation-the-kalman-filter) and [Study Guide 7](study_guides/Module_7_Study_Guide.md).
    *   Do: [Lab 7: EKF](../src/labs/lab_7_ekf.py) (Fusing IMU, Lidar, and GPS).
    *   Win: **The Paper Trick** (Verify altitude outlier rejection).

### Phase V: The Researcher (Autonomy)
9.  **Module 8: Perception and Mapping**
    *   Learn: [Lecture 15.5](LECTURES.md#lecture-15-deep-perception) and [Study Guide 8](study_guides/Module_8_Study_Guide.md).
    *   Do: [Lab 15.5: Change Detection](../src/labs/lab_15_5_change_detection.py) (Probabilistic diff).
    *   Win: **The Mirror Maze Challenge** (Verify reflection noise filtering).
10. **Module 9: Trajectory Optimization**
    *   Learn: [Lecture 9.5](LECTURES.md#lecture-95-path-planning--search) and [Study Guide 9](study_guides/Module_9_Study_Guide.md).
    *   Do: [Lab 9: Planning](../src/labs/lab_9_planning.py) (A* and Minimum Snap).
    *   Win: **The Maze Runner** (Verify planning optimality and smoothness).

### Phase VI: The Specialist (Tactics and AI)
11. **Module 10: Reinforcement Learning**
    *   Learn: [Study Guide 10](study_guides/Module_10_Study_Guide.md).
    *   Do: [Lab 10.7: LNN Adapter](../src/labs/lab_10_7_liquid_nets.py) (ODE-based gain adaptation).
    *   Win: **Noise Resilience Challenge** (Verify policy survival at 50ms action latency).
12. **Module 11: Aerial Combat**
    *   Learn: [Lecture 12.3](LECTURES.md#lecture-123-pursuit-evasion-games) and [Study Guide 11](study_guides/Module_11_Study_Guide.md).
    *   Do: [Lab 11: Pro-Nav](../src/labs/lab_11_pro_nav.py) and [Lab 12.3: Reachability](../src/labs/lab_12_3_reachability.py).
    *   Win: **Fox Two Lock-on** (Verify <20cm interception miss distance).

### Phase VII: The Frontier (Experimental Research)
13. **Module 13: Visual Inertial Odometry (VIO)**
    *   Learn: [Lecture 13](LECTURES.md#lecture-13-visual-inertial-odometry-vio) and [Study Guide 13](study_guides/Module_13_Study_Guide.md).
    *   Do: [Lab 13: KLT](../src/labs/lab_13_klt_tracker.py).
    *   Win: **The Shadow Hunt** (Verify feature tracking in low-light conditions).
14. **Module 14: Swarm Theory**
    *   Learn: [Lecture 14.3](LECTURES.md#1431-decentralized-collision-avoidance).
    *   Do: [Lab 14.5: RVO](../src/labs/lab_14_5_rvo_collision.py) and [Lab 14: Barriers](../src/labs/lab_14_safety_barriers.py).
    *   Win: **Consensus Dance** (Verify formation stability).
15. **Module 15: Deep Perception**
    *   Learn: [Lecture 15](LECTURES.md#lecture-15-deep-perception).
    *   Do: [Lab 15.7: 3DGS](../src/labs/lab_15_7_3dgs_collision.py) and [Lab 15.2: Quantization](../src/labs/lab_15_2_quantization.py).
    *   Win: **The AI Speed-Run** (Verify 30Hz inference on Pi Zero).
16. **Module 15.6: Audio**
    *   Learn: [Theory 15.6](theory/Theory_15.6_Acoustic_Localization.md).
    *   Do: [Lab 15.6: Audio Loc](../src/labs/lab_15_6_acoustic_loc.py).
    *   Win: **The Echo Test** (Verify sub-10cm wall distance estimation).

---
### Graduation: The Capstones
Choose your final path to becoming a **Systems Architect**:
*   **[Path A: Tactical Architect](FINAL_CHALLENGE.md#path-a-the-tactical-architect)**
*   **[Path B: Swarm Commander](FINAL_CHALLENGE.md#path-b-the-swarm-commander)**
*   **[Path C: AI Researcher](FINAL_CHALLENGE.md#path-c-the-ai-researcher)**
