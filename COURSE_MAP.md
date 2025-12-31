# The Squid Drone Course Map
**A Step-by-Step Guide to Mastery**

Follow the order below. Do not move to the next Phase until you have completed the "Squid Game" challenge for the previous one.

---

### **Phase I: The Mechanic (Hardware Foundations)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **0: Systems** | [Lecture 0](curriculum/Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Lecture.md) | [Lab 0.1: Jitter](src/Labs/Phase_1_The_Mechanic/Module_00_The_Build/lab_0_1_realtime_jitter.py) | [Guide 0](curriculum/Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Study_Guide.md) | **Morse Heartbeat** || **0.5: Bench** | [Hardware 101](curriculum/Support/Hardware/Hardware_Foundations.md) | [Lab 0.2: Morse](src/Labs/Phase_1_The_Mechanic/Module_00_The_Build/lab_0_2_morse_code.py) | [Lab 0.3: Motors](src/Labs/Phase_1_The_Mechanic/Module_00_The_Build/lab_0_3_motor_safety.py) | |
| **1: Drivers** | [Lecture 1](curriculum/Phases/Phase_1_The_Mechanic/Module_01_Bare_Metal_API/Module_01_Lecture.md) | [Lab 1.1: Mixers](src/Labs/Phase_1_The_Mechanic/Module_01_Bare_Metal_API/lab_1_1_mixer_matrix.py) | [Guide 1](curriculum/Phases/Phase_1_The_Mechanic/Module_01_Bare_Metal_API/Module_01_Study_Guide.md) | **The Tethered Levitation** |
### **Phase II: The Test Pilot (Observability)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **2: Telemetry**| [Lecture 2](curriculum/Phases/Phase_2_The_Test_Pilot/Module_02_Telemetry_Stack/Module_02_Lecture.md) | [Lab 2.1: PlotJuggler](src/Labs/Phase_2_The_Test_Pilot/Module_02_Telemetry_Stack/lab_2_1_plotjuggler_udp.py) | [Guide 2](curriculum/Phases/Phase_2_The_Test_Pilot/Module_02_Telemetry_Stack/Module_02_Study_Guide.md) | **The Data Stream** |
| **3: Vision** | [Lecture 3](curriculum/Phases/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/Module_03_Lecture.md) | [Lab 3.1: Calib](src/Labs/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/lab_2_1_calibration.py) | [Guide 3](curriculum/Phases/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/Module_03_Study_Guide.md) | **Virtual Horizon** || **3.5: SITL** | [Lecture 0.3](curriculum/Support/Reference/RESOURCES_AND_GLOSSARY.md#03-discrete-time-physics) | [Lab 3.2: SITL](src/Labs/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/lab_2_2_sitl_bridge.py) | | |

### **Phase III: The Engineer (Dynamics & Control)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **4: Signals** | [Lecture 5.5](curriculum/Phases/Phase_3_The_Engineer/Module_04_Signal_Processing/Module_04_Lecture.md) | [Lab 4.1: SysID](src/Labs/Phase_3_The_Engineer/Module_04_Signal_Processing/lab_3_1_sysid.py) | [Guide 4](curriculum/Phases/Phase_3_The_Engineer/Module_04_Signal_Processing/Module_04_Study_Guide.md) | **The Notch Filter** || **4.5: FFT** | [**DEEP** SysID Theory](curriculum/Library/02_Control_Dynamics/Theory_4.1_Least_Squares_SysID.md) | [Lab 4.2: FFT](src/Labs/Phase_3_The_Engineer/Module_04_Signal_Processing/lab_3_2_fft_tuning.py) | | |
| **5: Control** | [Lecture 5](curriculum/Phases/Phase_3_The_Engineer/Module_05_Control_Theory/Module_05_Lecture.md) | [**DEEP** Lab 5.1: PID](src/Labs/Phase_3_The_Engineer/Module_05_Control_Theory/lab_3_3_pid.py) | [Guide 5](curriculum/Phases/Phase_3_The_Engineer/Module_05_Control_Theory/Module_05_Study_Guide.md) | **The Statue Hover** || **5.5: Elite** | [MPC & iLQR Theory](curriculum/Library/02_Control_Dynamics/Theory_5.8_Model_Predictive_Control.md) | [**DEEP** Lab 5.2: MPC](src/Labs/Phase_3_The_Engineer/Module_05_Control_Theory/lab_3_4_mpc_lite.py) | [Lab 5.3: Recovery](src/Labs/Phase_3_The_Engineer/Module_05_Control_Theory/lab_3_5_recovery.py) | |

### **Phase IV: The Architect (Scale & Safety)**

| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |

| :--- | :--- | :--- | :--- | :--- |

| **6: ROS 2** | [Lecture 6](curriculum/Phases/Phase_4_The_Architect/Module_06_ROS2_Migration/Module_06_Lecture.md) | [SITL Bridge](src/Labs/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/lab_2_2_sitl_bridge.py) | [Guide 6](curriculum/Phases/Phase_4_The_Architect/Module_06_ROS2_Migration/Module_06_Study_Guide.md) | **Node Topology** || **6.5: Safety** | [Lecture 6.5](curriculum/Phases/Phase_4_The_Architect/Module_06_5_Reliability/Module_06_5_Lecture.md) | [Lab 6.1: Pre-int](src/Labs/Phase_4_The_Architect/Module_06_5_Reliability/lab_4_3_preintegration.py) | [Guide 6.5](curriculum/Phases/Phase_4_The_Architect/Module_06_5_Reliability/Module_06_5_Study_Guide.md) | **Heartbeat Loss** |

| **7: State** | [Lecture 7](curriculum/Phases/Phase_4_The_Architect/Module_07_State_Estimation/Module_07_Lecture.md) | [**DEEP** Lab 7.2: EKF](src/Labs/Phase_4_The_Architect/Module_07_State_Estimation/lab_4_2_ekf.py) | [Guide 7](curriculum/Phases/Phase_4_The_Architect/Module_07_State_Estimation/Module_07_Study_Guide.md) | **The Paper Trick** || **7.5: Verify** | [Lecture 7.5](curriculum/Phases/Phase_4_The_Architect/Module_07_5_Forensics/Module_07_5_Lecture.md) | [Regression Tool](tools/regression_test.py) | [Guide 7.5](curriculum/Phases/Phase_4_The_Architect/Module_07_5_Forensics/Module_07_5_Study_Guide.md) | **The Saboteur** |



### **Phase V: The Researcher (Autonomy)**





| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |

| :--- | :--- | :--- | :--- | :--- |

| **8: Mapping** | [Lecture 9](curriculum/Phases/Phase_5_The_Researcher/Module_08_Perception_and_Mapping/Module_08_Lecture.md) | [Lab 8.1: Map](src/Labs/Phase_5_The_Researcher/Module_08_Perception_and_Mapping/lab_5_1_mapping.py) | [Guide 8](curriculum/Phases/Phase_5_The_Researcher/Module_08_Perception_and_Mapping/Module_08_Study_Guide.md) | **The Mirror Maze** || **9: Planning** | [Lecture 9.5](curriculum/Phases/Phase_5_The_Researcher/Module_09_Trajectory_Optimization/Module_09_Lecture.md) | [Lab 9.1: Plan](src/Labs/Phase_5_The_Researcher/Module_09_Trajectory_Optimization/lab_5_2_planning.py) | [Guide 9](curriculum/Phases/Phase_5_The_Researcher/Module_09_Trajectory_Optimization/Module_09_Study_Guide.md) | **[The Maze Runner](PROJECTS.md#project-1-the-labyrinth-navigator)** |

### **Phase VI: The Specialist (Tactics & AI)**

| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |

| :--- | :--- | :--- | :--- | :--- |

| **10: RL** | [Lecture 10.1](curriculum/Phases/Phase_6_The_Specialist/Module_10_Reinforcement_Learning/Module_10_Lecture.md) | [Lab 10.1: RL](src/Labs/Phase_6_The_Specialist/Module_10_Reinforcement_Learning/lab_6_1_sim_to_real.py) | [Guide 10](curriculum/Phases/Phase_6_The_Specialist/Module_10_Reinforcement_Learning/Module_10_Study_Guide.md) | **Noise Resilience** || **10.5: LNN** | [Liquid Nets](curriculum/Library/04_AI_Learning/Theory_10.1_RL_Foundations.md) | [Lab 10.2: LNN](src/Labs/Phase_6_The_Specialist/Module_10_Reinforcement_Learning/lab_6_2_liquid_nets.py) | | |

| **11: Tactics** | [Lecture 12.3](curriculum/Phases/Phase_6_The_Specialist/Module_11_Tactics_and_Guidance/Module_11_Lecture.md) | [Lab 11.1: Nav](src/Labs/Phase_6_The_Specialist/Module_11_Tactics_and_Guidance/lab_6_3_pro_nav.py) | [Guide 11](curriculum/Phases/Phase_6_The_Specialist/Module_11_Tactics_and_Guidance/Module_11_Study_Guide.md) | **[Fox Two Lock-on](PROJECTS.md#project-3-the-ghost-in-the-machine)** || **11.5: Hunt** | [Bayesian Search](curriculum/Library/05_Tactics_Swarms/Theory_11.5_Bayesian_Search_Theory.md) | [Lab 11.2: Heat](src/Labs/Phase_6_The_Specialist/Module_11_Tactics_and_Guidance/lab_6_4_search_heatmap.py) | | |

| **12: Safety** | [Lecture](curriculum/Library/05_Tactics_Swarms/Theory_12.3_Pursuit_Evasion_Games.md) | [Lab 12.1: HJI](src/Labs/Phase_6_The_Specialist/Module_12_Outdoor_Autonomy/lab_6_5_reachability.py) | [Guide 12](curriculum/Phases/Phase_6_The_Specialist/Module_12_Outdoor_Autonomy/Module_12_Study_Guide.md) | **Precision Dock** || **12.5: Dock** | [Autonomous Dock](curriculum/Library/05_Tactics_Swarms/Theory_12.6_Autonomous_Docking_and_Recharging.md) | [Lab 12.2: Dock](src/Labs/Phase_6_The_Specialist/Module_12_Outdoor_Autonomy/lab_6_6_docking.py) | | |



### **Phase VII: The Frontier (Experimental)**



| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |



| :--- | :--- | :--- | :--- | :--- |



| **13: VIO** | [**DEEP** Factor Graphs](curriculum/Library/01_Sensing_Estimation/Theory_13.1_Factor_Graphs_GTSAM.md) | [**DEEP** Lab 13.1: KLT](src/Labs/Phase_7_The_Frontier/Module_13_VIO/lab_7_1_klt_tracker.py) | [Guide 13](curriculum/Phases/Phase_7_The_Frontier/Module_13_VIO/Module_13_Study_Guide.md) | **[The Shadow Hunt](PROJECTS.md#project-4-the-digital-twin)** || **14: Swarms** | [Lecture 14.3](curriculum/Phases/Phase_7_The_Frontier/Module_14_Swarm_Theory/Module_14_Lecture.md) | [Lab 14.1: CBF](src/Labs/Phase_7_The_Frontier/Module_14_Swarm_Theory/lab_7_2_safety_barriers.py) | [Guide 14](curriculum/Phases/Phase_7_The_Frontier/Module_14_Swarm_Theory/Module_14_Study_Guide.md) | **Consensus Dance** || **14.5: RVO** | [RVO Theory](curriculum/Library/05_Tactics_Swarms/Theory_14.5_Multi_Agent_Collision_Avoidance_RVO.md) | [Lab 14.2: RVO](src/Labs/Phase_7_The_Frontier/Module_14_Swarm_Theory/lab_7_3_rvo_collision.py) | [Lab 14.3: Trap](src/Labs/Phase_7_The_Frontier/Module_14_Swarm_Theory/lab_7_4_multi_agent_trap.py) | |






| **15: Deep AI** | [Lecture 15](curriculum/Phases/Phase_7_The_Frontier/Module_15_Deep_Perception/Module_15_Lecture.md) | [Lab 15.1: Siam](src/Labs/Phase_7_The_Frontier/Module_15_Deep_Perception/lab_7_5_siamese_perception.py) | [Guide 15](curriculum/Phases/Phase_7_The_Frontier/Module_15_Deep_Perception/Module_15_Study_Guide.md) | **The AI Speed-Run** || **15.5: Opt** | [Lecture 15.5](curriculum/Phases/Phase_7_The_Frontier/Module_15_5_Edge_AI/Module_15_5_Lecture.md) | [Lab 15.2: Opt](src/Labs/Phase_7_The_Frontier/Module_15_5_Edge_AI/lab_7_6_quantization.py) | [Guide 15.5](curriculum/Phases/Phase_7_The_Frontier/Module_15_5_Edge_AI/Module_15_5_Study_Guide.md) | [Lab 15.3: Change](src/Labs/Phase_7_The_Frontier/Module_15_5_Edge_AI/lab_7_7_change_detection.py) |
| **15.6: Audio** | [Acoustic Theory](curriculum/Library/01_Sensing_Estimation/Theory_15.6_Acoustic_Localization.md) | [Lab 15.5: Audio](src/Labs/Phase_7_The_Frontier/Module_15_5_Edge_AI/lab_7_9_acoustic_loc.py) | | **15.7: 3DGS** | [3DGS Paper](curriculum/Support/Reference/RESOURCES_AND_GLOSSARY.md#5-the-frontier) | [Lab 15.4: 3DGS](src/Labs/Phase_7_The_Frontier/Module_15_5_Edge_AI/lab_7_8_3dgs_collision.py) | | |

---
### **Capstone Specialization**
*   **[Operation Deep Ink](FINAL_CHALLENGE.md#path-a-the-tactical-architect)**
*   **[Operation Hive Mind](FINAL_CHALLENGE.md#path-b-the-swarm-commander)**
*   **[Operation Neural Nest](FINAL_CHALLENGE.md#path-c-the-ai-researcher)**