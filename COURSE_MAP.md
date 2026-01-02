# The Squid Drone Course Map
**A Step-by-Step Guide to Mastery**

Follow the order below. Do not move to the next Phase until you have completed the "Squid Game" challenge for the previous one.

---

### **Phase I: The Mechanic (Hardware Foundations)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **0: Systems** | [Lecture 0](curriculum/Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Lecture.md) | [Lab 0.1: Power Audit](curriculum/Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Labs.md#lab-01-the-power-audit) | [Guide 0](curriculum/Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Study_Guide.md) | **Morse Heartbeat** |
| | [**DEEP** Dive](curriculum/Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Deep_Dive_Foundations.md) | [Lab 0.2: CPU Isolation](curriculum/Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Labs.md#lab-021-the-vip-room-cpu-isolation) | [**Wiring Guide**](curriculum/Support/Hardware/WIRING_GUIDE.md) | |
| **1: Drivers** | [Lecture 1](curriculum/Phases/Phase_1_The_Mechanic/Module_01_Bare_Metal_API/Module_01_Lecture.md) | [Lab 1.1: Mixers](src/Labs/Phase_1_The_Mechanic/Module_01_Bare_Metal_API/lab_1_1_mixer_matrix.py) | [Guide 1](curriculum/Phases/Phase_1_The_Mechanic/Module_01_Bare_Metal_API/Module_01_Study_Guide.md) | **The Limp (Failure Sim)** |
| | [Isolcpus Theory](curriculum/Phases/Phase_1_The_Mechanic/Module_01_Bare_Metal_API/Module_01_Lecture.md#2-cpu-isolation-isolcpus) | [Lab 1.5: Virtual Dyno](src/Labs/Phase_1_The_Mechanic/Module_01_Bare_Metal_API/lab_1_5_virtual_dyno.py) | [**Assembly Guide**](curriculum/Support/Hardware/ASSEMBLY_GUIDE.md) | |

### **Phase II: The Test Pilot (Observability)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **2: Telemetry**| [Lecture 2](curriculum/Phases/Phase_2_The_Test_Pilot/Module_02_Telemetry_Stack/Module_02_Lecture.md) | [Lab 2.1: PlotJuggler](curriculum/Phases/Phase_2_The_Test_Pilot/Module_02_Telemetry_Stack/Module_02_Lecture.md) | [Guide 2](curriculum/Phases/Phase_2_The_Test_Pilot/Module_02_Telemetry_Stack/Module_02_Study_Guide.md) | **The Data Stream** |
| **3: Vision** | [Lecture 3](curriculum/Phases/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/Module_03_Lecture.md) | [Lab 3.1: Calib](curriculum/Phases/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/Module_03_Lecture.md) | [Guide 3](curriculum/Phases/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/Module_03_Study_Guide.md) | **Virtual Horizon** |
| **3.5: SITL** | [Lecture 0.3](curriculum/Support/Reference/RESOURCES_AND_GLOSSARY.md#03-discrete-time-physics) | [Lab 3.2: SITL](curriculum/Phases/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/Module_03_Lecture.md) | | |

### **Phase III: The Engineer (Dynamics & Control)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **4: Signals** | [Lecture 5.5](curriculum/Phases/Phase_3_The_Engineer/Module_04_Signal_Processing/Module_04_Lecture.md) | [Lab 4.1: SysID](curriculum/Phases/Phase_3_The_Engineer/Module_04_Signal_Processing/Module_04_Lecture.md) | [Guide 4](curriculum/Phases/Phase_3_The_Engineer/Module_04_Signal_Processing/Module_04_Study_Guide.md) | **The Notch Filter** |
| **4.5: FFT** | [SysID Theory](curriculum/Library/02_Control_Dynamics/Theory_4.1_Least_Squares_SysID.md) | [Lab 4.2: FFT](curriculum/Phases/Phase_3_The_Engineer/Module_04_Signal_Processing/Module_04_Lecture.md) | | |
| **5: Control** | [Lecture 5](curriculum/Phases/Phase_3_The_Engineer/Module_05_Control_Theory/Module_05_Lecture.md) | [Lab 5.1: PID](curriculum/Phases/Phase_3_The_Engineer/Module_05_Control_Theory/Module_05_Lecture.md) | [Guide 5](curriculum/Phases/Phase_3_The_Engineer/Module_05_Control_Theory/Module_05_Study_Guide.md) | **The Statue Hover** |
| **5.5: Elite** | [MPC Theory](curriculum/Library/02_Control_Dynamics/Theory_5.8_Model_Predictive_Control.md) | [Lab 5.2: MPC](curriculum/Phases/Phase_3_The_Engineer/Module_05_Control_Theory/Module_05_Lecture.md) | | |

### **Phase IV: The Architect (Scale & Safety)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **6: ROS 2** | [Lecture 6](curriculum/Phases/Phase_4_The_Architect/Module_06_ROS2_Migration/Module_06_Lecture.md) | [SITL Bridge](src/Labs/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/lab_2_2_sitl_bridge.py) | [Guide 6](curriculum/Phases/Phase_4_The_Architect/Module_06_ROS2_Migration/Module_06_Study_Guide.md) | **Node Topology** |
| **7: State** | [Lecture 7](curriculum/Phases/Phase_4_The_Architect/Module_07_State_Estimation/Module_07_Lecture.md) | [Lab 7.2: EKF](curriculum/Phases/Phase_4_The_Architect/Module_07_State_Estimation/Module_07_Lecture.md) | [Guide 7](curriculum/Phases/Phase_4_The_Architect/Module_07_State_Estimation/Module_07_Study_Guide.md) | **The Paper Trick** |

### **Phase V: The Researcher (Autonomy)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **8: Mapping** | [Lecture 9](curriculum/Phases/Phase_5_The_Researcher/Module_08_Perception_and_Mapping/Module_08_Lecture.md) | [Lab 8.1: Map](curriculum/Phases/Phase_5_The_Researcher/Module_08_Perception_and_Mapping/Module_08_Lecture.md) | [Guide 8](curriculum/Phases/Phase_5_The_Researcher/Module_08_Perception_and_Mapping/Module_08_Study_Guide.md) | **The Mirror Maze** |
| **9: Planning** | [Lecture 9.5](curriculum/Phases/Phase_5_The_Researcher/Module_09_Trajectory_Optimization/Module_09_Lecture.md) | [Lab 9.1: Plan](curriculum/Phases/Phase_5_The_Researcher/Module_09_Trajectory_Optimization/Module_09_Lecture.md) | [Guide 9](curriculum/Phases/Phase_5_The_Researcher/Module_09_Trajectory_Optimization/Module_09_Study_Guide.md) | **Maze Runner** |

### **Phase VI: The Specialist (Tactics & AI)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **10: RL** | [Lecture 10.1](curriculum/Phases/Phase_6_The_Specialist/Module_10_Reinforcement_Learning/Module_10_Lecture.md) | [Lab 10.1: RL](curriculum/Phases/Phase_6_The_Specialist/Module_10_Reinforcement_Learning/Module_10_Lecture.md) | [Guide 10](curriculum/Phases/Phase_6_The_Specialist/Module_10_Reinforcement_Learning/Module_10_Study_Guide.md) | **Noise Resilience** |
| **11: Tactics** | [Lecture 11](curriculum/Phases/Phase_6_The_Specialist/Module_11_Tactics_and_Guidance/Module_11_Lecture.md) | [Lab 11.1: Nav](curriculum/Phases/Phase_6_The_Specialist/Module_11_Tactics_and_Guidance/Module_11_Lecture.md) | [Guide 11](curriculum/Phases/Phase_6_The_Specialist/Module_11_Tactics_and_Guidance/Module_11_Study_Guide.md) | **Fox Two** |
| **12: Safety** | [Lecture 12](curriculum/Phases/Phase_6_The_Specialist/Module_12_Outdoor_Autonomy/Module_12_Lecture.md) | [Lab 12.1: HJI](curriculum/Phases/Phase_6_The_Specialist/Module_12_Outdoor_Autonomy/Module_12_Lecture.md) | [Guide 12](curriculum/Phases/Phase_6_The_Specialist/Module_12_Outdoor_Autonomy/Module_12_Study_Guide.md) | **Precision Dock** |

### **Phase VII: The Frontier (Experimental)**
| Module | Theoretical Lecture | Practical Lab | Study Guide | Challenge |
| :--- | :--- | :--- | :--- | :--- |
| **13: VIO** | [Factor Graphs](curriculum/Library/01_Sensing_Estimation/Theory_13.1_Factor_Graphs_GTSAM.md) | [Lab 13.1: KLT](curriculum/Phases/Phase_7_The_Frontier/Module_13_VIO/Module_13_Lecture.md) | [Guide 13](curriculum/Phases/Phase_7_The_Frontier/Module_13_VIO/Module_13_Study_Guide.md) | **Shadow Hunt** |
| **14: Swarms** | [Lecture 14](curriculum/Phases/Phase_7_The_Frontier/Module_14_Swarm_Theory/Module_14_Lecture.md) | [Lab 14.1: CBF](curriculum/Phases/Phase_7_The_Frontier/Module_14_Swarm_Theory/Module_14_Lecture.md) | [Guide 14](curriculum/Phases/Phase_7_The_Frontier/Module_14_Swarm_Theory/Module_14_Study_Guide.md) | **Consensus Dance** |
| **15: Deep AI** | [Lecture 15](curriculum/Phases/Phase_7_The_Frontier/Module_15_Deep_Perception/Module_15_Lecture.md) | [Lab 15.1: Siam](curriculum/Phases/Phase_7_The_Frontier/Module_15_Deep_Perception/Module_15_Lecture.md) | [Guide 15](curriculum/Phases/Phase_7_The_Frontier/Module_15_Deep_Perception/Module_15_Study_Guide.md) | **Speed-Run** |

---
### **Capstone Specialization**
*   **[Operation Deep Ink](FINAL_CHALLENGE.md#path-a-the-tactical-architect)**
*   **[Operation Hive Mind](FINAL_CHALLENGE.md#path-b-the-swarm-commander)**
*   **[Operation Neural Nest](FINAL_CHALLENGE.md#path-c-the-ai-researcher)**