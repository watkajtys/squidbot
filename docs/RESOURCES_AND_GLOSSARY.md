# Resources & Glossary
**The "Squid" Drone Project Reference Library**

This document supports the Master Curriculum. Refer to this when you encounter new terms or tools.

---

## 📚 The Self-Taught Curriculum (Recommended Reading)

### **1. The Prerequisites (Week 0)**
*   **Linux CLI:** [Linux Journey](https://linuxjourney.com/) (Focus on: Command Line, Permissions, Processes).
*   **Python:** [Real Python](https://realpython.com/) (Focus on: Classes, Threading, Sockets, NumPy).
*   **Git:** [Pro Git Book](https://git-scm.com/book/en/v2) (Focus on: Basic branching and committing).

### **2. Middleware (ROS 2)**
*   **Official Docs:** [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/).
*   **The "Bible":** [Articulated Robotics (YouTube)](https://www.youtube.com/@ArticulatedRobotics). *Watch his "Zero to Hero" series.*
*   **Visualization:** [Foxglove Studio Docs](https://docs.foxglove.dev/docs/introduction).

### **3. The Math (State Estimation & Control)**
*   **Linear Algebra:** [3Blue1Brown - Essence of Linear Algebra](https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab). *Crucial for Kalman Filters.*
*   **Kalman Filters:** [Kalman and Bayesian Filters in Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python). *Free interactive textbook.*
*   **Control Theory:** [Brian Douglas - Control Systems](https://www.youtube.com/user/ControlLectures). *The best explanation of PID/Bode plots.*

### **4. Artificial Intelligence (RL)**
*   **Theory:** [OpenAI Spinning Up](https://spinningup.openai.com/en/latest/). *The best intro to PPO and Policy Gradients.*
*   **Implementation:** [Stable Baselines3 Docs](https://stable-baselines3.readthedocs.io/).
*   **Simulation:** [Gym-PyBullet-Drones Repo](https://github.com/utiasDSL/gym-pybullet-drones).

---

## 📖 Glossary of Terms

### **Hardware & Electronics**
*   **BEC (Battery Eliminator Circuit):** A voltage regulator. Takes 12V from the battery and gives clean 5V to the Pi. Essential for preventing brownouts.
*   **DShot:** A digital protocol for controlling motors. Much faster and more accurate than the old Analog PWM.
*   **UART (Universal Asynchronous Receiver-Transmitter):** Serial communication (TX/RX). Used between the Flight Controller and the Pi.
*   **I2C (Inter-Integrated Circuit):** A bus for connecting sensors (ToF, Mag). Uses two wires (SDA, SCL).

### **Robotics & Control**
*   **Holonomic vs. Non-Holonomic:**
    *   *Holonomic:* Can move in any direction (like a drone).
    *   *Non-Holonomic:* Constrained movement (like a car; it cannot slide sideways).
    *   *Note:* A drone is Holonomic, but we sometimes fly it Non-Holonomically (banking turns) for speed.
*   **PID (Proportional-Integral-Derivative):** The standard control loop.
    *   *P:* Reacts to present error.
    *   *I:* Reacts to past error (accumulation).
    *   *D:* Reacts to predicted future error (rate of change).
*   **EKF (Extended Kalman Filter):** An algorithm that fuses noisy sensor data (IMU + Optical Flow) into a single, accurate estimate of position. It uses Linear Algebra (Jacobians) to handle curved flight paths.

### **Perception & Mapping**
*   **SLAM (Simultaneous Localization and Mapping):** The chicken-and-egg problem of building a map while figuring out where you are inside it.
*   **Voxel:** A "3D Pixel." We use Voxel Grids to represent the room as a Minecraft-like block world.
*   **Point Cloud:** A raw collection of X,Y,Z points returned by the Lidar/ToF sensor.
*   **Visual Servoing:** Controlling a robot's motion based directly on visual feedback (e.g., "Keep the red ball in the center of the frame").

### **AI & Reinforcement Learning**
*   **Agent:** The drone (the thing making decisions).
*   **Policy:** The "Brain." The function (Neural Network) that maps inputs (Sensors) to outputs (Motor Commands).
*   **Sim-to-Real Gap:** The phenomenon where an AI works perfectly in simulation but fails in real life due to friction, noise, or latency.
*   **Domain Randomization:** A technique to fix the Sim-to-Real gap by randomly changing the simulator's physics (gravity, friction) during training, forcing the AI to become robust.
*   **PPO (Proximal Policy Optimization):** A specific RL algorithm. It is stable and reliable, making it the industry standard for robotics.

---

## 🛠 Required Software Stack

1.  **VS Code:** Your Code Editor.
    *   *Extensions:* Python, C/C++, Remote - SSH (for coding on the Pi).
2.  **Docker:** For running ROS 2 containers without breaking your laptop's OS.
3.  **QGroundControl / Betaflight Configurator:** For low-level drone setup.
4.  **OpenSCAD:** For editing the 3D printed mount.
5.  **Foxglove Studio:** The industry-standard web-based visualizer. Used for viewing 3D Coordinate Frames (TFs), Point Clouds, and Telemetry in real-time or from flight logs. Replace standard `print()` debugging with Foxglove layouts.
