[Previous Module](../Module_08_Perception_and_Mapping/Module_08_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../../Phase_6_The_Specialist/Module_10_Reinforcement_Learning/Module_10_Lecture.md)

---

# Module 9: Trajectory Optimization
**"The shortest path is not always the fastest."**

We have a Map (Module 8). We have a Start and an End. How do we get there?
*   **Level 1:** Straight line (Crash into walls).
*   **Level 2:** A* Pathfinding (Jagged, robot-like movement).
*   **Level 3:** Minimum Snap Splines (Smooth, aggressive flight).

---

## **9.1 Pathfinding (A*)**

### **Objective**
Find a collision-free tube.

### **Theory**
*   **Graph Search:** Treat the Voxel Grid as a graph.
*   **A* (A-Star):** Search neighbors, favoring those closer to the goal.
*   **Heuristic:** Distance to Goal (Euclidean).

### **Lab Procedure**
1.  **Code:** Implement A* in Python (on the Voxel Grid from Mod 8).
2.  **Output:** A list of waypoints: `[(0,0,0), (0,1,0), (1,1,1)...]`.
3.  **Problem:** This path is "Zig-Zag". Drones cannot turn 90 degrees instantly.

---

## **9.2 Smoothing: Minimum Snap Splines**

### **Objective**
Turn the Zig-Zag into a Curve.

### **Theory**
*   **Position:** $x(t)$
*   **Velocity:** $x'(t)$
*   **Acceleration:** $x''(t)$ (Motor Force).
*   **Jerk:** $x'''(t)$ (Change in Force).
*   **Snap:** $x''''(t)$ (Motor vibration).

We want to minimize **Snap** (4th derivative). This ensures the motors change speed smoothly, preventing oscillation.

### **Lab Procedure**
1.  **Polynomials:** Fit a 7th-order polynomial between the A* waypoints.
2.  **Constraints:**
    *   Start Velocity = 0.
    *   End Velocity = 0.
    *   Pass through all waypoints.
3.  **Solver:** Use `scipy.optimize` or a quadratic programming solver (`OSQP`).

---

## **9.3 Model Predictive Control (MPC) - Advanced**

### **Objective**
Look ahead, not just behind.

### **Theory**
PID looks at the *past* (Integral) and *present* (Proportional).
MPC solves an optimization problem *every 20ms*:
"What sequence of motor commands over the next 1 second minimizes error while respecting constraints (max tilt, max voltage)?"

*   **Cost Function:** $J = \sum (x_{ref} - x)^2 + u^2$
*   **Constraints:** $u_{min} < u < u_{max}$
*   **Solver:** `ACADOS` or `OSQP` (Runs on Pi Zero 2 W, but barely).

---

## **9.4 Advanced Tracking: Geometric Control on SE(3)**

### **Objective**
Fly upside down (or at least >45 degrees).

### **Theory**
Standard PID uses Euler Angles (Roll/Pitch/Yaw).
**The Problem:** At 90 degrees pitch, Euler angles hit a singularity ("Gimbal Lock"). The math breaks.
**The Solution:** Geometric Control (Lee, Leok, McClamroch). We calculate error directly on the Manifold of Rotation Matrices ($SO(3)$).

### **Lab Procedure**
1.  **Attitude Error:** Instead of `target_roll - current_roll`, we compute:
    $$ e_R = \frac{1}{2} (R_d^T R - R^T R_d)^\vee $$
    (Where $^\vee$ maps a skew-symmetric matrix to a vector).
2.  **The Controller:** Implement the full nonlinear tracking law in your simulation.
3.  **Simulation:** Command a "Loop-the-Loop" trajectory.
    *   **PID:** Will likely flip out and crash at the top of the loop.
    *   **Geometric:** Will track smoothly through the inversion.

### **Deliverable**
*   A simulation video comparing PID vs Geometric Control during an aggressive maneuver.

---

## **Check: The Speed Run**
**Autonomous Navigation.**

1.  **Setup:** Obstacle course (2 chairs).
2.  **Plan:** Compute the trajectory *offline* (on laptop).
3.  **Execute:** Upload the trajectory to the drone.
4.  **Track:** The drone uses its PID/Tracking controller to follow the "Ghost Rabbit" (the Spline).

**Success:**
*   The drone does *not* stop at corners. It banks and turns smoothly.
*   Average speed > 1.0 m/s.

## **Theoretical Foundations**

### Lecture 9: Path Planning & Trajectory Generation

#### **1. Optimal Search & Admissible Heuristics (A*)**
Pathfinding is the search for a collision-free path in a graph $G$.
*   **The Heuristic ($h(n)$):** To find the global optimum, the heuristic must be **Admissible** (it never overestimates the cost to the goal). For a drone, we use Euclidean distance.
*   **Optimal Path:** $f(n) = g(n) + h(n)$. 
*   **Discrete vs. Continuous:** A* only finds a sequence of voxels. It does not account for the drone's inertia or the fact that a drone cannot turn $90^{\circ}$ instantly.

#### **2. Differential Flatness & Polynomial Splines**
Quadrotors possess a mathematical property called **Differential Flatness**. 
*   **The Secret:** The state of the drone (position, velocity, orientation, angular rate) can be mapped entirely to the "flat outputs" ($x, y, z$) and the Yaw angle ($\psi$). 
*   **Minimum Snap:** Because thrust is the 2nd derivative (acceleration) and motor torque is the 3rd (jerk), we minimize the **4th derivative (Snap)** to ensure the most efficient transition of motor power.
*   **QP-Solver:** we represent the path as a 7th-order polynomial $P(t) = a_0 + a_1t + ... + a_7t^7$ and use **Quadratic Programming** to find the coefficients that minimize snap while satisfying waypoint constraints.

#### **3. Geometric Tracking on SE(3)**
Once a trajectory is generated, the drone must "track" it.
*   **The Error Manifold:** We calculate the position error $e_p$ and velocity error $e_v$. 
*   **Non-linear Feedback:** The command $F$ is a combination of the desired acceleration plus a correction term: $F = (k_p e_p + k_v e_v + m g \hat{z}_{W} - m \ddot{x}_d) \cdot R \hat{z}_{B}$.
*   **Globally Stable:** Unlike PID, this tracking law is mathematically proven to converge from any starting orientation, even if the drone is tossed into the air upside down.

**Next Step:** [Phase VI: Module 10 Reinforcement Learning](../../Phase_6_The_Specialist/Module_10_Reinforcement_Learning/Module_10_Lecture.md)

---

## **Professor's Brief: Speaker Notes**

### **The Narrative Hook**
"A* finds the shortest path, but a drone is not a line on a piece of paper. It has weight, it has momentum, and it has limits. To fly fast, we must find the path that respects the laws of physics. We aren't just moving; we are **Slowing Down to Go Faster**. Today, we learn the math used by world-class racing drones to weave through forests at 60 miles per hour."

### **Deep Research Context: The Dynamic Envelope**
In research, we care about the **Feasibility** of a path. A polynomial might look smooth, but it might request more torque than the motors can provide. Explain that a research-grade planner includes a "Feasibility Check": we check if $\max|u(t)| < U_{limit}$ for every motor. If the path is too aggressive, we "Time-Scale" the spline—stretching the flight time until the motors can mathematically handle the forces.

## **Mastery Checklist**
Before moving to the next module, you should be able to:
- [ ] State an Admissible Heuristic for the A* algorithm in 3D space.
- [ ] Explain why the 4th derivative (Snap) is the primary cost for quadrotor spline optimization.
- [ ] Describe the "Differential Flatness" property and how it simplifies control.
- [ ] Differentiate between a discrete Path (A*) and a continuous Trajectory (Spline).

---

## **Further Reading & Bibliography**

### **Trajectory Generation**
*   **Mellinger, D., & Kumar, V. (2011).** *"Minimum snap trajectory generation and control for quadrotors."* IEEE International Conference on Robotics and Automation (ICRA). (The landmark paper).
*   **Richter, C., Bry, A., & Roy, N. (2016).** *"Polynomial trajectory planning for aggressive quadrotor flight."* Springer Tracts in Advanced Robotics.

### **Pathfinding Algorithms**
*   **Hart, P. E., et al. (1968).** *"A Formal Basis for the Heuristic Determination of Minimum Cost Paths."* IEEE Transactions on Systems Science and Cybernetics.

---

[Previous Module](../Module_08_Perception_and_Mapping/Module_08_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../../Phase_6_The_Specialist/Module_10_Reinforcement_Learning/Module_10_Lecture.md)