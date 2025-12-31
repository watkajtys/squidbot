# Theory Deep Dive 13.1: Factor Graphs & GTSAM
**"Optimization-Based State Estimation."**

While the Extended Kalman Filter (EKF) is a "Recursive" estimator (it forgets the past and only keeps the current mean/covariance), **Factor Graphs** treat state estimation as a global optimization problem.

---

## **1. The Graph Structure**
Imagine a graph where:
*   **Variables (Nodes):** The drone's position, velocity, and orientation at different timestamps ($x_1, x_2, \dots$).
*   **Factors (Edges):** The constraints between those nodes (e.g., an IMU reading between $x_1$ and $x_2$, or a GPS reading at $x_3$).

---

## **2. Probabilistic Least Squares**
Our goal is to find the set of states $X$ that minimizes the error across ALL factors.
$$ \hat{X} = \arg\min_{X} \sum_{i} \| f_i(x_i) - z_i \|^2_{\Sigma_i} $$
This is a large-scale version of the **Least Squares** math from Module 4.

---

## **3. Why Factor Graphs are Better for VIO**
*   **Re-linearization:** In an EKF, if your Jacobian is wrong once, the error is locked in forever. In a Factor Graph, we can re-calculate the Jacobians for the past few seconds to "fix" historical mistakes.
*   **Sparsity:** The relationship between nodes is "sparse" (a node only cares about its neighbors). Libraries like **GTSAM** (Georgia Tech Smoothing and Mapping) use this sparsity to solve for thousands of states in milliseconds on the Pi Zero.
*   **Handling Asynchronicity:** If a camera frame arrives late, we just add a factor to the graph at the correct timestamp. The optimizer will "pull" the other nodes into alignment.

---

## **4. The "Marginalization" Problem**
We can't keep every node forever, or the Pi's RAM will fill up.
*   **Sliding Window:** We keep the last 2-3 seconds of flight in the active graph.
*   **Marginalization:** When a node gets too old, we "squash" its information into a single prior factor for the remaining nodes.

---

## **5. Why This Matters for the Squid**
The Squid moves fast and vibrates heavily. An EKF will often "Diverge" (the math breaks) when the camera blurs or the motors spike. A Factor Graph is more "Forgiving"—it can use the consistency of the entire trajectory to ignore a few frames of bad data.

**Mastery Check:**
- [ ] What is the difference between "Filtering" and "Smoothing"?
- [ ] Define a "Prior Factor" in the context of a Factor Graph.
- [ ] Why does sparsity matter for solving large optimization problems?

---
*Reference: Dellaert, F., & Kaess, M. (2006). Square Root SAM.*
