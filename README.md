# The Squid: Autonomous Micro-Drone Platform

**Status:** Pre-Alpha (Hardware Acquisition Phase)
**Goal:** Sim-to-Real Reinforcement Learning & Tactical Autonomy on a Pavo20 Frame.

## 📂 Project Structure

*   **`/docs`**: The Curriculum and Lecture notes. Start here.
    *   `COURSE.md`: The step-by-step experiment roadmap.
    *   `LECTURES.md`: The theoretical syllabus.
    *   `RESOURCES_AND_GLOSSARY.md`: Recommended textbooks and terminology.
*   **`/hardware`**: CAD files and schematics.
    *   `avionics_mount.scad`: 3D printable mount for Pi Zero 2 W + Sensors.
*   **`/ros2_ws`**: The Robot Operating System (ROS2) workspace.
    *   Contains the source code for the drone's "Nervous System."
*   **`/simulation`**: Gym-PyBullet-Drones environment.
    *   Where we train the Neural Networks before flying.
*   **`/theory`**: Jupyter Notebooks.
    *   Mathematical derivations for EKF, Splines, and Control Laws.
*   **`/data`**: Flight Logs (MCAP/Rosbags).

## 🚀 Getting Started

1.  **Read the Plan:** Check `docs/COURSE.md` for the Semester 1 roadmap.
2.  **Print the Mount:** Take `hardware/avionics_mount.scad` to your local library.
3.  **Setup Environment:** (Instructions coming in Module 0).

## 🛠 Hardware List
*   **Frame:** BetaFPV Pavo20
*   **Compute:** Raspberry Pi Zero 2 W
*   **Sensors:** VL53L5CX (8x8 ToF), PMW3901 (Flow), Arducam IMX219
*   **Power:** 3S LiPo + Matek BEC
