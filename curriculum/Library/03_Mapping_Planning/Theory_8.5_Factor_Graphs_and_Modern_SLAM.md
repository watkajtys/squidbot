# Theory Deep Dive 8.5: Factor Graphs & Modern SLAM
**"Rewriting History."**

The EKF (Module 7) is "Recursive." It looks at the *now* and the *immediate past*. 
Modern SLAM (Factor Graphs) is "Global." It looks at the **entire flight history** at once.

---

## **1. The Problem with the EKF: The "Forgetful" Robot**
If an EKF makes a small mistake at 10 seconds, that mistake is baked into the math forever. By 10 minutes, the drone thinks it's in the next house.

---

## **2. The Solution: The Graph**
Instead of a single state, we create a **Factor Graph**.
*   **Nodes:** Every position the drone has ever been in.
*   **Factors:** Constraints (e.g., "The IMU says I moved 1m between Node 5 and Node 6").

### **Loop Closure: The "Aha!" Moment**
When the drone sees a landmark it saw 10 minutes ago, it adds a **new factor** between `Node 10` and `Node 600`.
*   **The Optimizer (iSAM2 / GTSAM):** Re-calculates the *entire history* of the flight to make that connection fit. The map "snaps" into place, and the drift is deleted.

---

## **3. The Math: Non-Linear Least Squares**
Factor Graphs are solved by minimizing the error across the entire graph.
*   **Tools:** `GTSAM` (Georgia Tech) or `Ceres Solver` (Google).

**Stanford Link:** This is the core of CS231A (Computer Vision: From 3D Reconstruction to Recognition).
--- [Return to Course Map](../../../COURSE_MAP.md)