# The Squid Drone Integrated Course Guide
**Focus:** Hardware -> Software -> Control -> Estimation -> Intelligence.

This document links the theoretical Lectures to the practical Study Guides and the physical Squid Games challenges. Follow this step-by-step path to master the full robotics stack.

---

### Phase I: The Mechanic (Hardware Foundations)
1.  **Module 0: Systems Engineering**
    *   Learn: [Lecture 0](curriculum/support_tools/LECTURES.md#lecture-0-systems-engineering) and [Study Guide 0](curriculum/phase_resources/Module_0_Study_Guide.md).
    *   Do: [Lab 0.1: Jitter Watchdog](src/labs/phase_1/lab_0_1_realtime_jitter.py) (Measure loop determinism).
    *   Do: [Module 0 Bench Labs](curriculum/phase_1_mechanic/Module_0_Labs.md).
    *   Win: **Morse Heartbeat Challenge**.
2.  **Module 1: The Bare Metal API**
    *   Learn: [Lecture 1](curriculum/support_tools/LECTURES.md#lecture-1-embedded-communication--numerics) and [Study Guide 1](curriculum/phase_resources/Module_1_Study_Guide.md).
    *   Do: [Lab 1: Mixers](src/labs/phase_1/lab_0_6_mixers.py).
    *   Win: **Three-Motor Limp**.

### Phase II: The Test Pilot (Observability)
3.  **Module 2: Telemetry Stack**
    *   Learn: [Lecture 2](curriculum/support_tools/LECTURES.md#lecture-2-stochastic-processes) and [Study Guide 2](curriculum/phase_resources/Module_2_Study_Guide.md).
    *   Do: [Lab 0.9: Battery EKF](src/labs/phase_2/lab_0_9_battery_ekf.py).
    *   Win: **Lag-Free HUD**.
4.  **Module 3: Sensing the World**
    *   Learn: [Lecture 3](curriculum/support_tools/LECTURES.md#lecture-3-computer-vision-fundamentals) and [Study Guide 3](curriculum/phase_resources/Module_3_Study_Guide.md).
    *   Do: [Lab 3: Calibration](src/labs/phase_2/lab_3_calibration.py) and [Lab 3.5: SITL Bridge](src/labs/phase_2/lab_3_5_sitl_bridge.py).
    *   Win: **Virtual Horizon**.

### Phase III: The Engineer (Dynamics and Control)
5.  **Module 4: Signal Processing**
    *   Learn: [Study Guide 4](curriculum/phase_resources/Module_4_Study_Guide.md).
    *   Do: [Lab 4.5: FFT Tuning](src/labs/phase_3/lab_4_5_fft_tuning.py).
    *   Win: **The Notch Filter**.
6.  **Module 5: Control Theory**
    *   Learn: [Lecture 5](curriculum/support_tools/LECTURES.md#lecture-5-control-theory) and [Study Guide 5](curriculum/phase_resources/Module_5_Study_Guide.md).
    *   Do: [Lab 5: PID](src/labs/phase_3/lab_5_pid.py) and [Lab 5.8: MPC Lite](src/labs/phase_3/lab_5_8_mpc_lite.py).
    *   Win: **The Statue Hover Challenge**.

### Phase IV: The Architect (Scale and Safety)
7.  **Module 6: ROS 2 Migration**
    *   Learn: [Study Guide 6](curriculum/phase_resources/Module_6_Study_Guide.md).
    *   Do: [Lab 6: IMU Pre-integration](src/labs/phase_4/lab_6_preintegration.py).
    *   Win: **The Latency Hunt**.
8.  **Module 7: State Estimation**
    *   Learn: [Lecture 7](curriculum/support_tools/LECTURES.md#lecture-6--7-state-estimation-the-kalman-filter) and [Study Guide 7](curriculum/phase_resources/Module_7_Study_Guide.md).
    *   Do: [Lab 7: EKF](src/labs/phase_4/lab_7_ekf.py).
    *   Win: **The Paper Trick**.

### Phase V: The Researcher (Autonomy)
9.  **Module 8: Perception and Mapping**
    *   Learn: [Lecture 15.5](curriculum/support_tools/LECTURES.md#lecture-15-deep-perception) and [Study Guide 8](curriculum/phase_resources/Module_8_Study_Guide.md).
    *   Do: [Lab 15.5: Change Detection](src/labs/phase_5/lab_15_5_change_detection.py).
    *   Win: **The Mirror Maze Challenge**.
10. **Module 9: Trajectory Optimization**
    *   Learn: [Lecture 9.5](curriculum/support_tools/LECTURES.md#lecture-95-path-planning--search) and [Study Guide 9](curriculum/phase_resources/Module_9_Study_Guide.md).
    *   Do: [Lab 9: Planning](src/labs/phase_5/lab_9_planning.py).
    *   Win: **The Maze Runner**.

### Phase VI: The Specialist (Tactics and AI)
11. **Module 10: Reinforcement Learning**
    *   Learn: [Study Guide 10](curriculum/phase_resources/Module_10_Study_Guide.md).
    *   Do: [Lab 10.7: LNN Adapter](src/labs/phase_6/lab_10_7_liquid_nets.py).
    *   Win: **Noise Resilience Challenge**.
12. **Module 11: Aerial Combat**
    *   Learn: [Lecture 12.3](curriculum/support_tools/LECTURES.md#lecture-123-pursuit-evasion-games) and [Study Guide 11](curriculum/phase_resources/Module_11_Study_Guide.md).
    *   Do: [Lab 11: Pro-Nav](src/labs/phase_6/lab_11_pro_nav.py) and [Lab 12.3: Reachability](src/labs/phase_6/lab_12_3_reachability.py).
    *   Win: **Fox Two Lock-on**.

### Phase VII: The Frontier (Experimental Research)
13. **Module 13: Visual Inertial Odometry (VIO)**
    *   Learn: [Lecture 13](curriculum/support_tools/LECTURES.md#lecture-13-visual-inertial-odometry-vio) and [Study Guide 13](curriculum/phase_resources/Module_13_Study_Guide.md).
    *   Do: [Lab 13: KLT](src/labs/phase_7/lab_13_klt_tracker.py).
14. **Module 14: Swarm Theory**
    *   Learn: [Lecture 14.3](curriculum/support_tools/LECTURES.md#1431-decentralized-collision-avoidance).
    *   Do: [Lab 14.5: RVO](src/labs/phase_7/lab_14_5_rvo_collision.py) and [Lab 14: Barriers](src/labs/phase_7/lab_14_safety_barriers.py).
15. **Module 15: Deep Perception**
    *   Learn: [Lecture 15](curriculum/support_tools/LECTURES.md#lecture-15-deep-perception).
    *   Do: [Lab 15.7: 3DGS](src/labs/phase_7/lab_15_7_3dgs_collision.py) and [Lab 15.2: Quantization](src/labs/phase_7/lab_15_2_quantization.py).
16. **Module 15.6: Audio**
    *   Learn: [Theory 15.6](curriculum/theory_deep_dives/Theory_15.6_Acoustic_Localization.md).
    *   Do: [Lab 15.6: Audio Loc](src/labs/phase_7/lab_15_6_acoustic_loc.py).

---
### Graduation: The Capstones
Choose your final path to becoming a **Systems Architect**:
*   **[Path A: Tactical Architect](FINAL_CHALLENGE.md)**
*   **[Path B: Swarm Commander](FINAL_CHALLENGE.md)**
*   **[Path C: AI Researcher](FINAL_CHALLENGE.md)**