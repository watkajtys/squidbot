# The Squid Drone Course Map
**A Step-by-Step Guide to Mastery**

Follow the order below. Do not move to the next Phase until you have completed the "Squid Game" challenge for the previous one.

---

### **Phase I: The Mechanic (Hardware Foundations)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **0: Systems** | [Lecture 0](LECTURES.md#lecture-0-systems-engineering) | [Lab 0.1: Jitter](../src/labs/lab_0_1_realtime_jitter.py) | [Study Guide 0](study_guides/Module_0_Study_Guide.md) | **Morse Heartbeat** |
| *Note:* | *Prerequisite: [Theory 0.1: The Real-Time Budget](theory/Theory_0.1_The_Real_Time_Budget.md)* | [Lab 1: Morse](../src/labs/lab_1_morse_code.py) | | |
| **1: Drivers** | [Lecture 1](LECTURES.md#lecture-1-embedded-communication--numerics) | [Lab 1.1: Mixers](../src/labs/lab_0_6_mixers.py) | [Study Guide 1](study_guides/Module_1_Study_Guide.md) | **Three-Motor Limp** |
| *Note:* | *Prerequisite: [Theory 0.6: The Motor Mixer Matrix](theory/Theory_0.6_The_Motor_Mixer_Matrix.md)* | | | |

### **Phase II: The Test Pilot (Observability)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **2: Telemetry**| [Lecture 2](LECTURES.md#lecture-2-stochastic-processes) | [Lab 0.9: Battery EKF](../src/labs/lab_0_9_battery_ekf.py) | [Study Guide 2](study_guides/Module_2_Study_Guide.md) | **Lag-Free HUD** |
| **3: Vision** | [Lecture 3](LECTURES.md#lecture-3-computer-vision-fundamentals) | [Lab 3: Calibration](../src/labs/lab_3_calibration.py) | [Study Guide 3](study_guides/Module_3_Study_Guide.md) | **Virtual Horizon** |
| **3.5: SITL** | [Lecture 0.3](LECTURES.md#03-discrete-time-physics) | [Lab 3.5: SITL Bridge](../src/labs/lab_3_5_sitl_bridge.py) | | |
| *Note:* | *Optional: Use PlotJuggler to visualize the CSV logs generated in Module 2.* | | | |

### **Phase III: The Engineer (Dynamics & Control)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **4: Signals** | [Lecture 5.5](LECTURES.md#lecture-55-system-identification-sysid) | [Lab 4.5: FFT Tuning](../src/labs/lab_4_5_fft_tuning.py) | [Study Guide 4](study_guides/Module_4_Study_Guide.md) | **The Notch Filter** |
| **5: Control** | [Lecture 5](LECTURES.md#lecture-5-control-theory) | [Lab 5.8: MPC Lite](../src/labs/lab_5_8_mpc_lite.py) | [Study Guide 5](study_guides/Module_5_Study_Guide.md) | **The Statue Hover** |
| *SITL:* | *Use SITL (Software-in-the-Loop) to safely tune your PID gains before flying.* | [Lab 5: PID](../src/labs/lab_5_pid.py) | | |

### **Phase IV: The Architect (Scale & Safety)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **6: Pre-int** | [Lecture 13.3](LECTURES.md#133-imu-pre-integration-forster-et-al) | [Lab 6: Pre-int](../src/labs/lab_6_preintegration.py) | | |
| **6.5: Safety**| [Real-World Engineering](Real_World_Engineering.md) | [Lab 14: Barriers](../src/labs/lab_14_safety_barriers.py) | | **The Dead-Man Switch** |
| **7: EKF** | [Lecture 7](LECTURES.md#lecture-6--7-state-estimation-the-kalman-filter) | [Lab 7: EKF](../src/labs/lab_7_ekf.py) | [Study Guide 7](study_guides/Module_7_Study_Guide.md) | **The Paper Trick** |
| *Tooling:* | *Introduce Foxglove Studio for advanced ROS 2 visualization.* | | | |

### **Phase V: The Researcher (Autonomy)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **8: Mapping** | [Lecture 9](LECTURES.md#lecture-9-slam--geometry) | [Lab 15.5: Change Det](../src/labs/lab_15_5_change_detection.py) | [Study Guide 8](study_guides/Module_8_Study_Guide.md) | **The Mirror Maze** |
| **9: Planning** | [Lecture 9.5](LECTURES.md#lecture-95-path-planning--search) | [Lab 9: Planning](../src/labs/lab_9_planning.py) | [Study Guide 9](study_guides/Module_9_Study_Guide.md) | **[The Maze Runner](PROJECTS.md#project-1-the-labyrinth-navigator)** |

### **Phase VI: The Specialist (Tactics & AI)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **10: RL** | [Lecture 10.1](LECTURES.md#lecture-101-reinforcement-learning-foundations) | [Lab 10.7: LNN](../src/labs/lab_10_7_liquid_nets.py) | [Study Guide 10](study_guides/Module_10_Study_Guide.md) | **Noise Resilience** |
| **11: Tactics** | [Lecture 12.3](LECTURES.md#lecture-123-pursuit-evasion-games) | [Lab 12.3: Reachable](../src/labs/lab_12_3_reachability.py) | [Study Guide 11](study_guides/Module_11_Study_Guide.md) | **[Fox Two Lock-on](PROJECTS.md#project-3-the-ghost-in-the-machine-tactical-autonomy)** |
| **12: Outdoor** | [Lecture 12.5](LECTURES.md#lecture-125-bayesian-search-theory) | [Lab 11.5: Search](../src/labs/lab_11_5_search_heatmap.py) | [Study Guide 12](study_guides/Module_12_Study_Guide.md) | **Precision Dock** |
| *Note:* | *Introduce CI/CD (GitHub Actions) for automated testing of advanced AI models.* | | | |

### **Phase VII: The Frontier (Experimental)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **13: VIO** | [Lecture 13](LECTURES.md#lecture-13-visual-inertial-odometry-vio) | [Lab 13: KLT](../src/labs/lab_13_klt_tracker.py) | [Study Guide 13](study_guides/Module_13_Study_Guide.md) | **[The Shadow Hunt](PROJECTS.md#project-4-the-digital-twin-hil-shadowing)** |
| **14: RVO** | [Lecture 14.3](LECTURES.md#1431-decentralized-collision-avoidance) | [Lab 14.5: RVO](../src/labs/lab_14_5_rvo_collision.py) | | |
| **15: Deep AI** | [Lecture 15](LECTURES.md#lecture-15-deep-perception) | [Lab 15.7: 3DGS](../src/labs/lab_15_7_3dgs_collision.py) | [Study Guide 15](study_guides/Module_15_Study_Guide.md) | **[The AI Speed-Run](PROJECTS.md#project-2-the-silent-guardian-stealth--stability)** |
| **15.6: Audio**| [Theory 15.6](theory/Theory_15.6_Acoustic_Localization.md) | [Lab 15.6: Audio](../src/labs/lab_15_6_acoustic_loc.py) | | |

---
### **Capstone Specialization**
Choose your path to graduation:
*   **[Operation Deep Ink](FINAL_CHALLENGE.md#path-a-the-tactical-architect)** (Tactical Architect)
*   **[Operation Hive Mind](FINAL_CHALLENGE.md#path-b-the-swarm-commander)** (Swarm Commander)
*   **[Operation Neural Nest](FINAL_CHALLENGE.md#path-c-the-ai-researcher)** (AI Researcher)