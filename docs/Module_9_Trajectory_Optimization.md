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

## **Check: The Speed Run**
**Autonomous Navigation.**

1.  **Setup:** Obstacle course (2 chairs).
2.  **Plan:** Compute the trajectory *offline* (on laptop).
3.  **Execute:** Upload the trajectory to the drone.
4.  **Track:** The drone uses its PID/Tracking controller to follow the "Ghost Rabbit" (the Spline).

**Success:**
*   The drone does *not* stop at corners. It banks and turns smoothly.
*   Average speed > 1.0 m/s.

**Submission:** A long-exposure photo (or video overlay) showing the smooth light trail of the drone.
