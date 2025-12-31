# The Squid Drone Integrated Course Guide
**Focus:** Hardware -> Software -> Control -> Estimation -> Intelligence.

This document links the theoretical Lectures to the practical Study Guides and the physical Squid Games challenges. Follow this step-by-step path to master the full robotics stack.

---

### Phase I: The Mechanic (Hardware Foundations)
1.  **Module 0: Systems Engineering**
    *   Learn: [Lecture 0](curriculum/support_tools/LECTURES.md#lecture-0-systems-engineering) and [Study Guide 0](curriculum/phase_resources/Module_0_Study_Guide.md).
    *   Do: [Lab 0.1: Jitter Watchdog](src/labs/phase_1/lab_0_1_realtime_jitter.py) (Bare Pi).
    *   Do: [Lab 0.2: Morse Code](src/labs/phase_1/lab_0_2_morse_code.py) (GPIO check).
    *   Do: [Lab 0.3: Motor Safety](src/labs/phase_1/lab_0_3_motor_safety.py) (Raw motor spins - Props OFF).
    *   Do: [Module 0 Bench Labs](curriculum/phase_1_mechanic/Module_0_Labs.md).
2.  **Module 1: The Bare Metal API**
    *   Learn: [Lecture 1](curriculum/support_tools/LECTURES.md#lecture-1-embedded-communication--numerics) and [Study Guide 1](curriculum/phase_resources/Module_1_Study_Guide.md).
    *   Do: [Lab 1.1: Mixers](src/labs/phase_1/lab_1_1_mixer_matrix.py).
    *   Win: **Three-Motor Limp Challenge**.

### Phase II: The Test Pilot (Observability)
3.  **Module 2: Telemetry Stack**
    *   Learn: [Lecture 2](curriculum/support_tools/LECTURES.md#lecture-2-stochastic-processes) and [Study Guide 2](curriculum/phase_resources/Module_2_Study_Guide.md).
    *   Do: [Lab 2.1: Battery Monitor](src/labs/phase_2/lab_2_3_battery_monitor.py).
    *   Win: **Lag-Free HUD Challenge**.
4.  **Module 3: Sensing the World**
    *   Learn: [Lecture 3](curriculum/support_tools/LECTURES.md#lecture-3-computer-vision-fundamentals) and [Study Guide 3](curriculum/phase_resources/Module_3_Study_Guide.md).
    *   Do: [Lab 3.1: Calibration](src/labs/phase_2/lab_2_1_calibration.py) and [Lab 3.2: SITL Bridge](src/labs/phase_2/lab_2_2_sitl_bridge.py).
    *   Win: **Virtual Horizon Challenge**.

### Phase III: The Engineer (Dynamics and Control)
5.  **Module 4: Signal Processing**
    *   Learn: [Study Guide 4](curriculum/phase_resources/Module_4_Study_Guide.md).
    *   Do: [Lab 4.1: SysID](src/labs/phase_3/lab_3_1_sysid.py) and [Lab 4.2: FFT Tuning](src/labs/phase_3/lab_3_2_fft_tuning.py).
    *   Win: **The Notch Filter Challenge**.
6.  **Module 5: Control Theory**
    *   Learn: [Lecture 5](curriculum/support_tools/LECTURES.md#lecture-5-control-theory).
    *   Do: [Lab 5.1: PID](src/labs/phase_3/lab_3_3_pid.py), [Lab 5.2: MPC](src/labs/phase_3/lab_3_4_mpc_lite.py), and [Lab 5.3: Recovery](src/labs/phase_3/lab_3_5_recovery.py).
    *   Win: **The Statue Hover Challenge**.

### Phase IV: The Architect (Scale and Safety)
7.  **Module 6: Pre-integration**
    *   Do: [Lab 6.1: IMU Pre-integration](src/labs/phase_4/lab_4_3_preintegration.py).
8.  **Module 7: State Estimation**
    *   Learn: [Lecture 7](curriculum/support_tools/LECTURES.md#lecture-6--7-state-estimation-the-kalman-filter) and [Study Guide 7](curriculum/phase_resources/Module_7_Study_Guide.md).
    *   Do: [Lab 7.1: Allan Variance](src/labs/phase_4/lab_4_1_allan_variance.py) and [Lab 7.2: EKF](src/labs/phase_4/lab_4_2_ekf.py).
    *   Win: **The Paper Trick Challenge**.

### Phase V: The Researcher (Autonomy)
9.  **Module 8: Perception**
    *   Do: [Lab 8.1: Occupancy Mapping](src/labs/phase_5/lab_5_1_mapping.py).
10. **Module 9: Planning**
    *   Do: [Lab 9.1: Trajectory Planning](src/labs/phase_5/lab_5_2_planning.py).
    *   Win: **The Maze Runner Challenge**.

### Phase VI: The Specialist (Tactics and AI)
11. **Module 10: Reinforcement Learning**
    *   Do: [Lab 10.1: Sim-to-Real RL](src/labs/phase_6/lab_6_1_sim_to_real.py) and [Lab 10.2: LNN](src/labs/phase_6/lab_6_2_liquid_nets.py).
12. **Module 11: Tactics**
    *   Do: [Lab 11.1: Pro-Nav](src/labs/phase_6/lab_6_3_pro_nav.py) and [Lab 11.2: Search](src/labs/phase_6/lab_6_4_search_heatmap.py).
13. **Module 12: Safety & Docking**
    *   Do: [Lab 12.1: Reachability](src/labs/phase_6/lab_6_5_reachability.py) and [Lab 12.2: Docking](src/labs/phase_6/lab_6_6_docking.py).

### Phase VII: The Frontier (Experimental Research)
14. **Module 13: VIO**
    *   Do: [Lab 13.1: KLT Tracker](src/labs/phase_7/lab_7_1_klt_tracker.py).
15. **Module 14: Swarms**
    *   Do: [Lab 14.1: Barriers](src/labs/phase_7/lab_7_2_safety_barriers.py), [Lab 14.2: RVO](src/labs/phase_7/lab_7_3_rvo_collision.py), and [Lab 14.3: Trap](src/labs/phase_7/lab_7_4_multi_agent_trap.py).
16. **Module 15: Deep Perception**
    *   Do: [Lab 15.1: Siam](src/labs/phase_7/lab_7_5_siamese_perception.py), [Lab 15.2: Opt](src/labs/phase_7/lab_7_6_quantization.py), [Lab 15.3: Change](src/labs/phase_7/lab_7_7_change_detection.py), [Lab 15.4: 3DGS](src/labs/phase_7/lab_7_8_3dgs_collision.py), and [Lab 15.5: Audio](src/labs/phase_7/lab_7_9_acoustic_loc.py).

---
### Graduation: The Capstones
Choose your graduation mission in **[FINAL_CHALLENGE.md](FINAL_CHALLENGE.md)**.
