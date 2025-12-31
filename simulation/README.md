# Simulation Environment
**"Sim-to-Real: The Robotic Proving Ground."**

This folder contains the physical definitions of the Squid Drone used for simulation and visualization.

---

## **1. The URDF (Unified Robot Description Format)**
`quadrotor.urdf` is the "XML DNA" of your drone. It defines:
*   **Geometry:** The size of the frame and the position of the motors.
*   **Inertia:** The weight and center of mass (Critical for PID tuning).
*   **Joints:** The relationship between the camera, Lidar, and the center of the drone.

## **2. How to use this with PyBullet**
We recommend using the **`gym-pybullet-drones`** library (University of Toronto). 

### **Quick Start:**
1.  Install the library: `pip install gym-pybullet-drones`
2.  Load the Squid URDF:
```python
import pybullet as p
p.loadURDF("simulation/quadrotor.urdf", [0, 0, 1])
```

## **3. The Goal: Digital Twin**
By Module 10 (Reinforcement Learning), you should be training your AI in this simulator using the `quadrotor.urdf`. Because the URDF matches your real Pavo20 frame, the AI's "instincts" should transfer to the real world with minimal adjustment.
