# The Squid Drone Integrated Course Guide
**Focus:** Hardware -> Software -> Control -> Estimation -> Intelligence.

This document links the theoretical Lectures to the practical Study Guides and the physical Squid Games challenges. Follow this step-by-step path to master the full robotics stack.

---

### Phase I: The Mechanic (Hardware Foundations)
1.  **Module 0: Systems Engineering**
    *   Learn: [Lecture 0](curriculum/Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Lecture.md) and [Study Guide 0](curriculum/Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Study_Guide.md).
    *   Do: [Lab 0.1: Jitter Watchdog](curriculum/Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Labs.md#lab-01-the-power-audit) (Bare Pi).
    *   Do: [Lab 0.2: Morse Code](curriculum/Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Labs.md#lab-02-the-linux-playground) (GPIO check).
    *   Do: [Lab 0.3: Motor Safety](curriculum/Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Labs.md#lab-03-betaflight-sensors-the-ear-test) (Raw motor spins - Props OFF).
    *   Do: [Module 0 Bench Labs](curriculum/Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Labs.md).
2.  **Module 1: The Bare Metal API**
    *   Learn: [Lecture 1](curriculum/Phases/Phase_1_The_Mechanic/Module_01_Bare_Metal_API/Module_01_Lecture.md) and [Study Guide 1](curriculum/Phases/Phase_1_The_Mechanic/Module_01_Bare_Metal_API/Module_01_Study_Guide.md).
    *   Do: [Lab 1.1: Mixers](curriculum/Phases/Phase_1_The_Mechanic/Module_01_Bare_Metal_API/Module_01_Lecture.md#11-the-msp-protocol-motor-control).
    *   Win: **Three-Motor Limp Challenge**.

### Phase II: The Test Pilot (Observability)
3.  **Module 2: Telemetry Stack**
    *   Learn: [Lecture 2](curriculum/Phases/Phase_2_The_Test_Pilot/Module_02_Telemetry_Stack/Module_02_Lecture.md) and [Study Guide 2](curriculum/Phases/Phase_2_The_Test_Pilot/Module_02_Telemetry_Stack/Module_02_Study_Guide.md).
    *   Do: [Lab 2.1: Battery Monitor](curriculum/Phases/Phase_2_The_Test_Pilot/Module_02_Telemetry_Stack/Module_02_Lecture.md#21-networking-udp-sockets).
    *   Win: **Lag-Free HUD Challenge**.
4.  **Module 3: Sensing the World**
    *   Learn: [Lecture 3](curriculum/Phases/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/Module_03_Lecture.md) and [Study Guide 3](curriculum/Phases/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/Module_03_Study_Guide.md).
    *   Do: [Lab 3.1: Calibration](curriculum/Phases/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/Module_03_Lecture.md#31-video-pipeline-hardware-encoding) and [Lab 3.2: SITL Bridge](curriculum/Phases/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/Module_03_Lecture.md#32-augmented-reality-the-hud).
    *   Win: **Virtual Horizon Challenge**.

### Phase III: The Engineer (Dynamics and Control)
5.  **Module 4: Signal Processing**
    *   Learn: [Study Guide 4](curriculum/Phases/Phase_3_The_Engineer/Module_04_Signal_Processing/Module_04_Study_Guide.md).
    *   Do: [Lab 4.1: SysID](curriculum/Phases/Phase_3_The_Engineer/Module_04_Signal_Processing/Module_04_Lecture.md#41-coordinate-frames) and [Lab 4.2: FFT Tuning](curriculum/Phases/Phase_3_The_Engineer/Module_04_Signal_Processing/Module_04_Lecture.md#42-vibration-analysis--filtering).
    *   Win: **The Notch Filter Challenge**.
6.  **Module 5: Control Theory**
    *   Learn: [Lecture 5](curriculum/Phases/Phase_3_The_Engineer/Module_05_Control_Theory/Module_05_Lecture.md).
    *   Do: [Lab 5.1: PID](curriculum/Phases/Phase_3_The_Engineer/Module_05_Control_Theory/Module_05_Lecture.md#51-the-feedback-loop-pid), [Lab 5.2: MPC](curriculum/Phases/Phase_3_The_Engineer/Module_05_Control_Theory/Module_05_Lecture.md#52-implementation-details-the-phd-guard), and [Lab 5.3: Recovery](curriculum/Phases/Phase_3_The_Engineer/Module_05_Control_Theory/Module_05_Lecture.md#53-tuning-the-ziegler-nichols-method).
    *   Win: **The Statue Hover Challenge**.

### Phase IV: The Architect (Scale and Safety)
7.  **Module 6: The ROS 2 Migration**
    *   Learn: [Lecture 6](curriculum/Phases/Phase_4_The_Architect/Module_06_ROS2_Migration/Module_06_Lecture.md).
    *   Do: [Lab 6.1: Nodes & Topics](curriculum/Phases/Phase_4_The_Architect/Module_06_ROS2_Migration/Module_06_Lecture.md#61-architecture-nodes--topics).
8.  **Module 6.5: Safety & Reliability**
    *   Learn: [Lecture 6.5](curriculum/Phases/Phase_4_The_Architect/Module_06_5_Reliability/Module_06_5_Lecture.md).
    *   Do: [Lab 6.5.1: Heartbeat](curriculum/Phases/Phase_4_The_Architect/Module_06_5_Reliability/Module_06_5_Lecture.md#651-the-heartbeat-protocol).
9.  **Module 7: State Estimation**
    *   Learn: [Lecture 7](curriculum/Phases/Phase_4_The_Architect/Module_07_State_Estimation/Module_07_Lecture.md) and [Study Guide 7](curriculum/Phases/Phase_4_The_Architect/Module_07_State_Estimation/Module_07_Study_Guide.md).
    *   Do: [Lab 7.1: Allan Variance](curriculum/Phases/Phase_4_The_Architect/Module_07_State_Estimation/Module_07_Lecture.md#71-time-synchronization) and [Lab 7.2: EKF](curriculum/Phases/Phase_4_The_Architect/Module_07_State_Estimation/Module_07_Lecture.md#72-the-kalman-filter-mathematical-engine).
    *   Win: **The Paper Trick Challenge**.

### Phase V: The Researcher (Autonomy)
10. **Module 8: Perception**
    *   Do: [Lab 8.1: Occupancy Mapping](curriculum/Phases/Phase_5_The_Researcher/Module_08_Perception_and_Mapping/Module_08_Lecture.md#81-the-point-cloud).
11. **Module 9: Planning**
    *   Do: [Lab 9.1: Trajectory Planning](curriculum/Phases/Phase_5_The_Researcher/Module_09_Trajectory_Optimization/Module_09_Lecture.md#91-pathfinding-a).
    *   Win: **The Maze Runner Challenge**.

### Phase VI: The Specialist (Tactics and AI)
12. **Module 10: Reinforcement Learning**
    *   Do: [Lab 10.1: Sim-to-Real RL](curriculum/Phases/Phase_6_The_Specialist/Module_10_Reinforcement_Learning/Module_10_Lecture.md#101-the-simulation-gym) and [Lab 10.2: LNN](curriculum/Phases/Phase_6_The_Specialist/Module_10_Reinforcement_Learning/Module_10_Lecture.md#102-sim-to-real-the-gap).
13. **Module 11: Tactics**
    *   Do: [Lab 11.1: Pro-Nav](curriculum/Phases/Phase_6_The_Specialist/Module_11_Tactics_and_Guidance/Module_11_Lecture.md#111-proportional-navigation-pro-nav) and [Lab 11.2: Search](curriculum/Phases/Phase_6_The_Specialist/Module_11_Tactics_and_Guidance/Module_11_Lecture.md#112-visual-servoing-the-red-balloon).
14. **Module 12: Safety & Docking**
    *   Do: [Lab 12.1: Reachability](curriculum/Phases/Phase_6_The_Specialist/Module_12_Outdoor_Autonomy/Module_12_Lecture.md#121-gps-integration) and [Lab 12.2: Docking](curriculum/Phases/Phase_6_The_Specialist/Module_12_Outdoor_Autonomy/Module_12_Lecture.md#122-hybrid-navigation).

### Phase VII: The Frontier (Experimental Research)
15. **Module 13: VIO**
    *   Do: [Lab 13.1: KLT Tracker](curriculum/Phases/Phase_7_The_Frontier/Module_13_VIO/Module_13_Lecture.md#131-visual-front-end-from-pixels-to-geometry).
16. **Module 14: Swarms**
    *   Do: [Lab 14.1: Barriers](curriculum/Phases/Phase_7_The_Frontier/Module_14_Swarm_Theory/Module_14_Lecture.md#141-graph-theory-the-skeleton-of-the-swarm), [Lab 14.2: RVO](curriculum/Phases/Phase_7_The_Frontier/Module_14_Swarm_Theory/Module_14_Lecture.md#142-distributed-formation-control), and [Lab 14.3: Trap](curriculum/Phases/Phase_7_The_Frontier/Module_14_Swarm_Theory/Module_14_Lecture.md#143-socratic-discussion-the-collective-mind).
17. **Module 15: Deep Perception**
    *   Do: [Lab 15.1: Siam](curriculum/Phases/Phase_7_The_Frontier/Module_15_Deep_Perception/Module_15_Lecture.md#151-cnns-for-visual-landmarks), [Lab 15.2: Opt](curriculum/Phases/Phase_7_The_Frontier/Module_15_Deep_Perception/Module_15_Lecture.md#152-siamese-networks--triplet-loss), [Lab 15.3: Change](curriculum/Phases/Phase_7_The_Frontier/Module_15_Deep_Perception/Module_15_Lecture.md#153-deployment-on-device-inference), [Lab 15.4: 3DGS](curriculum/Phases/Phase_7_The_Frontier/Module_15_Deep_Perception/Module_15_Lecture.md), and [Lab 15.5: Audio](curriculum/Phases/Phase_7_The_Frontier/Module_15_Deep_Perception/Module_15_Lecture.md).

---

### The Squid Games: Validation & Graduation
Don't just complete the labs—win the games.
*   **The Milestones:** Use **[SQUID_GAMES.md](SQUID_GAMES.md)** to verify your skills at the end of every Phase.
*   **The Synthesis:** Combine your knowledge into the four major **[PROJECTS.md](PROJECTS.md)**.
*   **Graduation:** Choose your final mission in **[FINAL_CHALLENGE.md](FINAL_CHALLENGE.md)**.

---
### Graduation: The Capstones
Choose your graduation mission in **[FINAL_CHALLENGE.md](FINAL_CHALLENGE.md)**.
