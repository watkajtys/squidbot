# The Squid Drone Course Map
**A Step-by-Step Guide to Mastery**

Follow the order below. Do not move to the next Phase until you have completed the "Squid Game" challenge for the previous one.

---

### **Phase I: The Mechanic (Hardware Foundations)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **0: Systems** | [Lecture 0](../docs/LECTURES.md#lecture-0-systems-engineering) | [Module 0 Labs](../docs/Module_0_Labs.md) | [Study Guide 0](study_guides/Module_0_Study_Guide.md) | **Morse Heartbeat** |
| *Note:* | *Prerequisite: [Theory 0.1: The Real-Time Budget](../docs/theory/Theory_0.1_The_Real_Time_Budget.md)* | | | |
| **1: Drivers** | [Lecture 1](../docs/LECTURES.md#lecture-1-embedded-communication--numerics) | [Lab 1: Mixers](../src/labs/lab_0_6_mixers.py) | [Study Guide 1](study_guides/Module_1_Study_Guide.md) | **Three-Motor Limp** |

### **Phase II: The Test Pilot (Observability)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **2: Telemetry**| [Lecture 2](../docs/LECTURES.md#lecture-2-stochastic-processes) | [Module 2 Setup](../docs/Module_2_The_Telemetry_Stack.md) | [Study Guide 2](study_guides/Module_2_Study_Guide.md) | **Lag-Free HUD** |
| **3: Vision** | [Lecture 3](../docs/LECTURES.md#lecture-3-computer-vision-fundamentals) | [Lab 3: Calibration](../src/labs/lab_3_calibration.py) | [Study Guide 3](study_guides/Module_3_Study_Guide.md) | **Virtual Horizon** |
| *Note:* | *Optional: Use PlotJuggler to visualize the CSV logs generated in Module 2.* | | | |

### **Phase III: The Engineer (Dynamics & Control)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **4: Signals** | [Lecture 4](../docs/LECTURES.md#lecture-4-rotations--lie-groups-part-i) | [Lab 4: SysID](../src/labs/lab_4_sysid.py) | [Study Guide 4](study_guides/Module_4_Study_Guide.md) | **The Notch Filter** |
| **5: Control** | [Lecture 5](../docs/LECTURES.md#lecture-5-control-theory) | [Lab 5: PID](../src/labs/lab_5_pid.py) | [Study Guide 5](study_guides/Module_5_Study_Guide.md) | **The Statue Hover** |
| *SITL:* | *Use SITL (Software-in-the-Loop) to safely tune your PID gains before flying.* | | | |

### **Phase IV: The Architect (Scale & Safety)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **6: ROS 2** | [Lecture 1](../docs/LECTURES.md#lecture-1-embedded-communication--numerics) | [ROS 2 Migration](../docs/Module_6_ROS2_Migration.md) | [Study Guide 6](study_guides/Module_6_Study_Guide.md) | **The Latency Hunt** |
| **6.5: Safety**| [Real-World Engineering](Real_World_Engineering.md) | **Failsafes & Reliability** | | **The Dead-Man Switch** |
| **7: EKF** | [Lecture 7](../docs/LECTURES.md#lecture-6--7-state-estimation-the-kalman-filter) | [Lab 7: EKF](../src/labs/lab_7_ekf.py) | [Study Guide 7](study_guides/Module_7_Study_Guide.md) | **The Paper Trick** |
| *Tooling:* | *Introduce Foxglove Studio for advanced ROS 2 visualization.* | | | |

### **Phase V: The Researcher (Autonomy)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **8: Mapping** | [Lecture 9](../docs/LECTURES.md#lecture-9-slam--geometry) | [Lab 8: Mapping](../src/labs/lab_8_6_perch_finder.py) | [Study Guide 8](study_guides/Module_8_Study_Guide.md) | **The Mirror Maze** |
| **9: Planning** | [Lecture 9.5](../docs/LECTURES.md#lecture-95-path-planning--search) | [Lab 9: Planning](../src/labs/lab_9_planning.py) | [Study Guide 9](study_guides/Module_9_Study_Guide.md) | **The Maze Runner** |

### **Phase VI: The Specialist (Tactics & AI)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **10: RL** | [Lecture 10.1](../docs/LECTURES.md#lecture-101-reinforcement-learning-foundations) | [Lab 10: RL](../src/labs/lab_10_6_sim_to_real.py) | [Study Guide 10](study_guides/Module_10_Study_Guide.md) | **Noise Resilience** |
| **11: Tactics** | [Lecture 12](../docs/LECTURES.md#lecture-11-reinforcement-learning) | [Lab 11: Guidance](../src/labs/lab_11_pro_nav.py) | [Study Guide 11](study_guides/Module_11_Study_Guide.md) | **Fox Two Lock-on** |
| **12: Outdoor** | [Lecture 12.6](../docs/LECTURES.md#lecture-123-pursuit-evasion-games) | [Lab 12: Docking](../src/labs/lab_12_6_docking.py) | [Study Guide 12](study_guides/Module_12_Study_Guide.md) | **Precision Dock** |
| *Note:* | *Introduce CI/CD (GitHub Actions) for automated testing of advanced AI models.* | | | |

### **Phase VII: The Frontier (Experimental)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **13: VIO** | [Lecture 13](../docs/LECTURES.md#lecture-13-visual-inertial-odometry-vio) | [Lab 13: KLT](../src/labs/lab_13_klt_tracker.py) | [Study Guide 13](study_guides/Module_13_Study_Guide.md) | **The Shadow Hunt** |
| **14: Swarms** | [Lecture 14](../docs/LECTURES.md#lecture-14-swarm-theory) | [Lab 14: Swarms](../src/labs/lab_14_safety_barriers.py) | [Study Guide 14](study_guides/Module_14_Study_Guide.md) | **Consensus Dance** |
| **15: Deep AI** | [Lecture 15](../docs/LECTURES.md#lecture-15-deep-perception) | [Lab 15.2: Edge AI Optimization](Module_15_2_Edge_AI_Optimization.md) | [Study Guide 15](study_guides/Module_15_Study_Guide.md) | **The AI Speed-Run** |
| **16: Ethics** | [Lecture 16](../docs/LECTURES.md#lecture-16-the-ethics-of-autonomous-flight) | | | **The Mirror Test** |
