# The Squid Drone Integrated Course Guide
**Focus:** Hardware -> Software -> Control -> Estimation -> Intelligence.

This document links the theoretical Lectures to the practical Study Guides and the physical Squid Games challenges. Follow this step-by-step path to master the full robotics stack.

---

### Phase I: The Mechanic (Hardware Foundations)
1.  **Module 0: Systems Engineering**
    *   Learn: [Lecture 0](curriculum/Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Lecture.md) and [Study Guide 0](curriculum/Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Study_Guide.md).
    *   Do: [Lab 0.1: Jitter Watchdog](src/Labs/Phase_1_The_Mechanic/Module_00_The_Build/lab_0_1_realtime_jitter.py) (Bare Pi).
    *   Do: [Lab 0.2: Morse Code](src/Labs/Phase_1_The_Mechanic/Module_00_The_Build/lab_0_2_morse_code.py) (GPIO check).
    *   Do: [Lab 0.3: Motor Safety](src/Labs/Phase_1_The_Mechanic/Module_00_The_Build/lab_0_3_motor_safety.py) (Raw motor spins - Props OFF).
    *   Do: [Module 0 Bench Labs](curriculum/Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Labs.md).
2.  **Module 1: The Bare Metal API**
    *   Learn: [Lecture 1](curriculum/Phases/Phase_1_The_Mechanic/Module_01_Bare_Metal_API/Module_01_Lecture.md) and [Study Guide 1](curriculum/Phases/Phase_1_The_Mechanic/Module_01_Bare_Metal_API/Module_01_Study_Guide.md).
    *   Do: [Lab 1.1: Mixers](src/Labs/Phase_1_The_Mechanic/Module_01_Bare_Metal_API/lab_1_1_mixer_matrix.py).
    *   Win: **Three-Motor Limp Challenge**.

### Phase II: The Test Pilot (Observability)
3.  **Module 2: Telemetry Stack**
    *   Learn: [Lecture 2](curriculum/Phases/Phase_2_The_Test_Pilot/Module_02_Telemetry_Stack/Module_02_Lecture.md) and [Study Guide 2](curriculum/Phases/Phase_2_The_Test_Pilot/Module_02_Telemetry_Stack/Module_02_Study_Guide.md).
    *   Do: [Lab 2.1: Battery Monitor](src/Labs/Phase_2_The_Test_Pilot/Module_02_Telemetry_Stack/lab_2_3_battery_monitor.py).
    *   Win: **Lag-Free HUD Challenge**.
4.  **Module 3: Sensing the World**
    *   Learn: [Lecture 3](curriculum/Phases/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/Module_03_Lecture.md) and [Study Guide 3](curriculum/Phases/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/Module_03_Study_Guide.md).
    *   Do: [Lab 3.1: Calibration](src/Labs/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/lab_2_1_calibration.py) and [Lab 3.2: SITL Bridge](src/Labs/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/lab_2_2_sitl_bridge.py).
    *   Win: **Virtual Horizon Challenge**.

### Phase III: The Engineer (Dynamics and Control)
5.  **Module 4: Signal Processing**
    *   Learn: [Study Guide 4](curriculum/Phases/Phase_3_The_Engineer/Module_04_Signal_Processing/Module_04_Study_Guide.md).
    *   Do: [Lab 4.1: SysID](src/Labs/Phase_3_The_Engineer/Module_04_Signal_Processing/lab_3_1_sysid.py) and [Lab 4.2: FFT Tuning](src/Labs/Phase_3_The_Engineer/Module_04_Signal_Processing/lab_3_2_fft_tuning.py).
    *   Win: **The Notch Filter Challenge**.
6.  **Module 5: Control Theory**
    *   Learn: [Lecture 5](curriculum/Phases/Phase_3_The_Engineer/Module_05_Control_Theory/Module_05_Lecture.md).
    *   Do: [Lab 5.1: PID](src/Labs/Phase_3_The_Engineer/Module_05_Control_Theory/lab_3_3_pid.py), [Lab 5.2: MPC](src/Labs/Phase_3_The_Engineer/Module_05_Control_Theory/lab_3_4_mpc_lite.py), and [Lab 5.3: Recovery](src/Labs/Phase_3_The_Engineer/Module_05_Control_Theory/lab_3_5_recovery.py).
    *   Win: **The Statue Hover Challenge**.

### Phase IV: The Architect (Scale and Safety)
7.  **Module 6: Pre-integration**
    *   Do: [Lab 6.1: IMU Pre-integration](src/Labs/Phase_4_The_Architect/Module_06_5_Reliability/lab_4_3_preintegration.py).
8.  **Module 7: State Estimation**
    *   Learn: [Lecture 7](curriculum/Phases/Phase_4_The_Architect/Module_07_State_Estimation/Module_07_Lecture.md) and [Study Guide 7](curriculum/Phases/Phase_4_The_Architect/Module_07_State_Estimation/Module_07_Study_Guide.md).
    *   Do: [Lab 7.1: Allan Variance](src/Labs/Phase_4_The_Architect/Module_07_State_Estimation/lab_4_1_allan_variance.py) and [Lab 7.2: EKF](src/Labs/Phase_4_The_Architect/Module_07_State_Estimation/lab_4_2_ekf.py).
    *   Win: **The Paper Trick Challenge**.

### Phase V: The Researcher (Autonomy)
9.  **Module 8: Perception**
    *   Do: [Lab 8.1: Occupancy Mapping](src/Labs/Phase_5_The_Researcher/Module_08_Perception_and_Mapping/lab_5_1_mapping.py).
10. **Module 9: Planning**
    *   Do: [Lab 9.1: Trajectory Planning](src/Labs/Phase_5_The_Researcher/Module_09_Trajectory_Optimization/lab_5_2_planning.py).
    *   Win: **The Maze Runner Challenge**.

### Phase VI: The Specialist (Tactics and AI)
11. **Module 10: Reinforcement Learning**
    *   Do: [Lab 10.1: Sim-to-Real RL](src/Labs/Phase_6_The_Specialist/Module_10_Reinforcement_Learning/lab_6_1_sim_to_real.py) and [Lab 10.2: LNN](src/Labs/Phase_6_The_Specialist/Module_10_Reinforcement_Learning/lab_6_2_liquid_nets.py).
12. **Module 11: Tactics**
    *   Do: [Lab 11.1: Pro-Nav](src/Labs/Phase_6_The_Specialist/Module_11_Tactics_and_Guidance/lab_6_3_pro_nav.py) and [Lab 11.2: Search](src/Labs/Phase_6_The_Specialist/Module_11_Tactics_and_Guidance/lab_6_4_search_heatmap.py).
13. **Module 12: Safety & Docking**
    *   Do: [Lab 12.1: Reachability](src/Labs/Phase_6_The_Specialist/Module_12_Outdoor_Autonomy/lab_6_5_reachability.py) and [Lab 12.2: Docking](src/Labs/Phase_6_The_Specialist/Module_12_Outdoor_Autonomy/lab_6_6_docking.py).

### Phase VII: The Frontier (Experimental Research)
14. **Module 13: VIO**
    *   Do: [Lab 13.1: KLT Tracker](src/Labs/Phase_7_The_Frontier/Module_13_VIO/lab_7_1_klt_tracker.py).
15. **Module 14: Swarms**
    *   Do: [Lab 14.1: Barriers](src/Labs/Phase_7_The_Frontier/Module_14_Swarm_Theory/lab_7_2_safety_barriers.py), [Lab 14.2: RVO](src/Labs/Phase_7_The_Frontier/Module_14_Swarm_Theory/lab_7_3_rvo_collision.py), and [Lab 14.3: Trap](src/Labs/Phase_7_The_Frontier/Module_14_Swarm_Theory/lab_7_4_multi_agent_trap.py).
16. **Module 15: Deep Perception**
    *   Do: [Lab 15.1: Siam](src/Labs/Phase_7_The_Frontier/Module_15_Deep_Perception/lab_7_5_siamese_perception.py), [Lab 15.2: Opt](src/Labs/Phase_7_The_Frontier/Module_15_5_Edge_AI/lab_7_6_quantization.py), [Lab 15.3: Change](src/Labs/Phase_7_The_Frontier/Module_15_5_Edge_AI/lab_7_7_change_detection.py), [Lab 15.4: 3DGS](src/Labs/Phase_7_The_Frontier/Module_15_5_Edge_AI/lab_7_8_3dgs_collision.py), and [Lab 15.5: Audio](src/Labs/Phase_7_The_Frontier/Module_15_5_Edge_AI/lab_7_9_acoustic_loc.py).

---

### The Squid Games: Validation & Graduation
Don't just complete the labs—win the games.
*   **The Milestones:** Use **[SQUID_GAMES.md](SQUID_GAMES.md)** to verify your skills at the end of every Phase.
*   **The Synthesis:** Combine your knowledge into the four major **[PROJECTS.md](PROJECTS.md)**.
*   **Graduation:** Choose your final mission in **[FINAL_CHALLENGE.md](FINAL_CHALLENGE.md)**.

---
### Graduation: The Capstones
Choose your graduation mission in **[FINAL_CHALLENGE.md](FINAL_CHALLENGE.md)**.
