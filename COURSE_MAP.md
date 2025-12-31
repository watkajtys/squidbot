# The Squid Drone Course Map
**A Step-by-Step Guide to Mastery**

Follow the order below. Do not move to the next Phase until you have completed the "Squid Game" challenge for the previous one.

---

### **Phase I: The Mechanic (Hardware Foundations)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **0: Systems** | [Lecture 0](curriculum/support_tools/LECTURES.md#0-systems-engineering) | [Lab 0.1: Jitter](src/labs/phase_1/lab_0_1_realtime_jitter.py) | [Guide 0](curriculum/phase_resources/Module_0_Study_Guide.md) | **Morse Heartbeat** |
| **0.5: Bench** | [Hardware 101](curriculum/support_tools/Hardware_Foundations.md) | [Lab 0.2: Morse](src/labs/phase_1/lab_0_2_morse_code.py) | [Lab 0.3: Motors](src/labs/phase_1/lab_0_3_motor_safety.py) | |
| **1: Drivers** | [Lecture 1](curriculum/support_tools/LECTURES.md#1-embedded-communication) | [Lab 1.1: Mixers](src/labs/phase_1/lab_1_1_mixer_matrix.py) | [Guide 1](curriculum/phase_resources/Module_1_Study_Guide.md) | **Three-Motor Limp** |

### **Phase II: The Test Pilot (Observability)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **2: Telemetry**| [Lecture 2](curriculum/support_tools/LECTURES.md#2-stochastic-processes) | [Lab 2.1: Battery](src/labs/phase_2/lab_2_3_battery_monitor.py) | [Guide 2](curriculum/phase_resources/Module_2_Study_Guide.md) | **Lag-Free HUD** |
| **3: Vision** | [Lecture 3](curriculum/support_tools/LECTURES.md#3-computer-vision) | [Lab 3.1: Calib](src/labs/phase_2/lab_2_1_calibration.py) | [Guide 3](curriculum/phase_resources/Module_3_Study_Guide.md) | **Virtual Horizon** |
| **3.5: SITL** | [Lecture 0.3](curriculum/support_tools/LECTURES.md#03-discrete-time-physics) | [Lab 3.2: SITL](src/labs/phase_2/lab_2_2_sitl_bridge.py) | | |

### **Phase III: The Engineer (Dynamics & Control)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **4: Signals** | [Lecture 5.5](curriculum/support_tools/LECTURES.md#55-system-identification) | [Lab 4.1: SysID](src/labs/phase_3/lab_3_1_sysid.py) | [Guide 4](curriculum/phase_resources/Module_4_Study_Guide.md) | **The Notch Filter** |
| **4.5: FFT** | [Aerodynamics](curriculum/theory_deep_dives/Theory_0.8_High_Speed_Aerodynamics.md) | [Lab 4.2: FFT](src/labs/phase_3/lab_3_2_fft_tuning.py) | | |
| **5: Control** | [Lecture 5](curriculum/support_tools/LECTURES.md#5-control-theory) | [Lab 5.1: PID](src/labs/phase_3/lab_3_3_pid.py) | [Guide 5](curriculum/phase_resources/Module_5_Study_Guide.md) | **The Statue Hover** |
| **5.5: Elite** | [MPC Theory](curriculum/theory_deep_dives/Theory_5.8_Model_Predictive_Control.md) | [Lab 5.2: MPC](src/labs/phase_3/lab_3_4_mpc_lite.py) | [Lab 5.3: Recovery](src/labs/phase_3/lab_3_5_recovery.py) | |

### **Phase IV: The Architect (Scale & Safety)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **6: Pre-int** | [Lecture 13.3](curriculum/support_tools/LECTURES.md#133-imu-pre-integration) | [Lab 6.1: Pre-int](src/labs/phase_4/lab_4_3_preintegration.py) | | |
| **7: Noise** | [Allan Variance](curriculum/theory_deep_dives/Theory_7.5_Allan_Variance.md) | [Lab 7.1: Allan](src/labs/phase_4/lab_4_1_allan_variance.py) | | |
| **7.5: EKF** | [Lecture 7](curriculum/support_tools/LECTURES.md#6--7-state-estimation) | [Lab 7.2: EKF](src/labs/phase_4/lab_4_2_ekf.py) | [Guide 7](curriculum/phase_resources/Module_7_Study_Guide.md) | **The Paper Trick** |

### **Phase V: The Researcher (Autonomy)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **8: Mapping** | [Lecture 9](curriculum/support_tools/LECTURES.md#9-slam--geometry) | [Lab 8.1: Map](src/labs/phase_5/lab_5_1_mapping.py) | [Guide 8](curriculum/phase_resources/Module_8_Study_Guide.md) | **The Mirror Maze** |
| **9: Planning** | [Lecture 9.5](curriculum/support_tools/LECTURES.md#95-path-planning--search) | [Lab 9.1: Plan](src/labs/phase_5/lab_5_2_planning.py) | [Guide 9](curriculum/phase_resources/Module_9_Study_Guide.md) | **[The Maze Runner](PROJECTS.md#project-1-the-labyrinth-navigator)** |

### **Phase VI: The Specialist (Tactics & AI)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **10: RL** | [Lecture 10.1](curriculum/support_tools/LECTURES.md#101-reinforcement-learning) | [Lab 10.1: RL](src/labs/phase_6/lab_6_1_sim_to_real.py) | [Guide 10](curriculum/phase_resources/Module_10_Study_Guide.md) | **Noise Resilience** |
| **10.5: LNN** | [Liquid Nets](curriculum/theory_deep_dives/Theory_10.1_RL_Foundations.md) | [Lab 10.2: LNN](src/labs/phase_6/lab_6_2_liquid_nets.py) | | |
| **11: Tactics** | [Lecture 12.3](curriculum/support_tools/LECTURES.md#123-pursuit-evasion-games) | [Lab 11.1: Nav](src/labs/phase_6/lab_6_3_pro_nav.py) | [Guide 11](curriculum/phase_resources/Module_11_Study_Guide.md) | **[Fox Two Lock-on](PROJECTS.md#project-3-the-ghost-in-the-machine)** |
| **11.5: Hunt** | [Bayesian Search](curriculum/theory_deep_dives/Theory_11.5_Bayesian_Search_Theory.md) | [Lab 11.2: Heat](src/labs/phase_6/lab_6_4_search_heatmap.py) | | |
| **12: Safety** | [HJI Reachable](curriculum/theory_deep_dives/Theory_12.3_Pursuit_Evasion_Games.md) | [Lab 12.1: HJI](src/labs/phase_6/lab_6_5_reachability.py) | [Guide 12](curriculum/phase_resources/Module_12_Study_Guide.md) | **Precision Dock** |
| **12.5: Dock** | [Autonomous Dock](curriculum/theory_deep_dives/Theory_12.6_Autonomous_Docking_and_Recharging.md) | [Lab 12.2: Dock](src/labs/phase_6/lab_6_6_docking.py) | | |

### **Phase VII: The Frontier (Experimental)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **13: VIO** | [Lecture 13](curriculum/support_tools/LECTURES.md#13-visual-inertial-odometry-vio) | [Lab 13.1: KLT](src/labs/phase_7/lab_7_1_klt_tracker.py) | [Guide 13](curriculum/phase_resources/Module_13_Study_Guide.md) | **[The Shadow Hunt](PROJECTS.md#project-4-the-digital-twin)** |
| **14: Swarms** | [Lecture 14.3](curriculum/support_tools/LECTURES.md#1431-decentralized-collision-avoidance) | [Lab 14.1: CBF](src/labs/phase_7/lab_7_2_safety_barriers.py) | [Guide 14](curriculum/phase_resources/Module_14_Study_Guide.md) | **Consensus Dance** |
| **14.5: RVO** | [RVO Theory](curriculum/theory_deep_dives/Theory_14.5_Multi_Agent_Collision_Avoidance_RVO.md) | [Lab 14.2: RVO](src/labs/phase_7/lab_7_3_rvo_collision.py) | [Lab 14.3: Trap](src/labs/phase_7/lab_7_4_multi_agent_trap.py) | |
| **15: Deep AI** | [Lecture 15](curriculum/support_tools/LECTURES.md#15-deep-perception) | [Lab 15.1: Siam](src/labs/phase_7/lab_7_5_siamese_perception.py) | [Guide 15](curriculum/phase_resources/Module_15_Study_Guide.md) | **The AI Speed-Run** |
| **15.2: Opt** | [Quantization](curriculum/phase_7_frontier/Module_15_2_Edge_AI_Optimization.md) | [Lab 15.2: Opt](src/labs/phase_7/lab_7_6_quantization.py) | [Lab 15.3: Change](src/labs/phase_7/lab_7_7_change_detection.py) | |
| **15.4: 3DGS** | [3DGS Paper](curriculum/support_tools/RESOURCES_AND_GLOSSARY.md#5-the-frontier) | [Lab 15.4: 3DGS](src/labs/phase_7/lab_7_8_3dgs_collision.py) | [Lab 15.5: Audio](src/labs/phase_7/lab_7_9_acoustic_loc.py) | |

---
### **Capstone Specialization**
*   **[Operation Deep Ink](FINAL_CHALLENGE.md#path-a-the-tactical-architect)**
*   **[Operation Hive Mind](FINAL_CHALLENGE.md#path-b-the-swarm-commander)**
*   **[Operation Neural Nest](FINAL_CHALLENGE.md#path-c-the-ai-researcher)**