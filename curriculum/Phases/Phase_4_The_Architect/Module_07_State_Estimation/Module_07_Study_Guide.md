[Return to Course Map](../../../../COURSE_MAP.md)

# Study Guide 7: State Estimation
**Module 7: Finding the Truth**

### Critical Takeaways
1.  **The Sensor Fusion Problem:** No sensor is perfect. GPS is accurate but slow (1Hz). The IMU is fast (1kHz) but drifts. State Estimation is the mathematical process of fusing these noisy, asynchronous signals into a single "best guess" of the drone's position and velocity.
2.  **The Kalman Filter:** This is a recursive optimal estimator. It works in two steps: **Prediction** (using the physics model to guess where the drone went) and **Update** (using sensor measurements to correct that guess).
3.  **Covariance:** The Kalman Filter doesn't just track your position; it tracks your **Uncertainty**. The Covariance Matrix (P) tells the drone how much it should trust its internal model versus its sensors.
4.  **The Jacobian:** In an Extended Kalman Filter (EKF), we must linearize the non-linear equations of motion. The Jacobian matrix is the mathematical "slope" that allows us to perform this linearization at every step.

### Mental Models and Field Notes
*   **The Orthogonality Principle:** In a Kalman Filter, the estimation error is always orthogonal to the measurements. This is a mathematical way of saying that the filter has extracted every single possible drop of "truth" from the noise. There is literally no "information" left in the noise that the filter hasn't already found.
*   **Trust Issues:** State estimation is essentially a mathematical debate between your **Expectations** (the Physics Model) and your **Observations** (the Sensors). If your sensors are lying to you (e.g., GPS multipath near a building), your filter must be "opinionated" enough to ignore them.
*   **The Random Walk:** Imagine a drunk person trying to walk in a straight line. Every step they take is slightly off. Over time, they end up far from the path. This is "Bias Drift" in a gyroscope. The Kalman Filter's job is to "sober up" the data by constantly checking against fixed landmarks or sensors.

### Frontier Facts and Historical Context
*   **The Apollo 11 Code:** The Kalman Filter was originally developed in the 1960s. Rudolf Kalman's math was a key reason why NASA was able to land on the moon. The Apollo Guidance Computer (AGC) used a Kalman Filter to land the Eagle lunar module, managing the same sensor noise problems you are facing today.
*   **Quantum Inertial Sensors:** Traditional MEMS gyroscopes drift by several degrees per hour. Researchers are now testing "Quantum IMUs" that use laser-cooled atoms to measure rotation. These sensors have such low drift that a drone could navigate across an entire continent without GPS and only be off by a few centimeters.
*   **The "Paper Trick":** In the field, we use "Mahalanobis Distance" to reject outliers. If you slide a piece of paper under your altitude sensor, the EKF should realize the "new floor" is impossible based on its physics model and ignore the sensor until the paper is removed.

---

### The Squid Games: Level 7
**The Paper Trick Challenge**
While the drone is estimating its altitude via EKF, slide a piece of paper under the downward-facing Lidar and quickly pull it away.
*   **The Goal:** Observe the altitude estimate. Does it jump up when the paper appears?
*   **Win Condition:** The drone's "estimated" altitude must stay steady. If the estimate jumps, your **Outlier Rejection** is failing.

---

### Module 7 Quiz
1.  **Prediction vs. Update:** If your drone is in a tunnel with no GPS, what happens to the Covariance Matrix (P) over time?
2.  **Sensor Noise (R):** If you tell the Kalman Filter that your GPS is "very noisy" (high R value), will the filter follow the GPS measurements more or less closely?
3.  **Observability:** Why can't you determine your absolute heading (Yaw) using only a 3-axis accelerometer?
4.  **Allan Variance:** What is the "Bias Instability" of a gyroscope, and why is it the "ultimate limit" of how long a drone can fly without GPS?

---
*Reference: Lecture 7 (State Estimation) in docs/LECTURES.md*