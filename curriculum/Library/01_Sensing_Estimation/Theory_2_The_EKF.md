# Theory Deep Dive 2: The Extended Kalman Filter (EKF)
**"Sensing the Unseen."**

In Module 7, you will use an EKF. In university, this is often a 15-week course. Here, we break down the intuition so you can understand the code.

---

## **1. The Philosophical Problem**
You have two sensors:
1.  **Accelerometer:** Tells you "I am moving up at 1.0 m/s." It is fast (1000Hz) but drifts over time.
2.  **Lidar:** Tells you "The floor is 1.02m away." It is slow (30Hz) but never drifts.

**The Question:** What is your *actual* altitude?
*   If you only use the Lidar, your control loop is slow and "choppy."
*   If you only use the Accel, your drone will drift into the ceiling after 10 seconds.

---

## **2. The Recursive Solution**
(See Part 2 for the math).

---

## **Part 2: The Mathematical Engine (The 5 Equations)**
In a university course, you would spend 3 weeks deriving these. For the PhD, you must know them by heart.

### **The State ($x$) and Covariance ($P$)
*   $x$: Our best guess of the drone's position/velocity.
*   $P$: Our "Uncertainty Matrix." (Large numbers = We are lost. Small numbers = We are confident).

### **Step A: Prediction (The Guess)**
1.  **State Predict:** $\hat{x}_{k|k-1} = F_k \hat{x}_{k-1|k-1} + B_k u_k$
    *   *Translation:* "Based on the physics ($F$) and my motors ($u$), I think I am here."
2.  **Covariance Predict:** $P_{k|k-1} = F_k P_{k-1|k-1} F_k^T + Q_k$
    *   *Translation:* "My uncertainty grew because the world is messy ($Q$)."

### **Step B: Correction (The Reality Check)**
3.  **Kalman Gain ($K$):** $K_k = P_{k|k-1} H_k^T (H_k P_{k|k-1} H_k^T + R_k)^{-1}$
    *   *Translation:* "How much do I trust the sensor ($R$) vs. my guess ($P$)?
4.  **State Update:** $\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k (z_k - H_k \hat{x}_{k|k-1})$
    *   *Translation:* "Update my guess using the sensor data ($z$)."
5.  **Covariance Update:** $P_{k|k} = (I - K_k H_k) P_{k|k-1}$
    *   *Translation:* "I am now more confident."

---

## **Part 3: The PhD Nuances (Why Real Robots Crash)**
This is what a 15-week course teaches you: the "Numerical Stability" that standard tutorials skip.

### **3.1 The Joseph Form (Precision)**
Equation #5 ($P = (I-KH)P$) is mathematically perfect but **numerically dangerous**. On a 32-bit computer (like our Pi's flight controller), rounding errors can make the Covariance matrix "Negative," which is physically impossible (like having a negative distance).
*   **The PhD Fix:** Use the **Joseph Form**:
    $ P_k = (I - K_k H_k) P_{k|k-1} (I - K_k H_k)^T + K_k R_k K_k^T $
    It is more code, but it is mathematically guaranteed to stay positive-definite. **This is why professional autopilots don't crash and hobbyist ones do.**

### **3.2 Mahalanobis Distance (Outlier Rejection)**
What if a bird hits your Lidar? It will report a distance of 0.1m when you are at 10m. A basic KF will trust the "Liar" and crash the drone.
*   **The PhD Fix:** Before updating, calculate the **Innovation Covariance ($S$)**.
*   Calculate the **Mahalanobis Distance**: Is this sensor reading "Physically Possible" given our current uncertainty?
*   If the distance > 3.0 (3 standard deviations), we **reject** the measurement. "The sensor is lying; trust the IMU instead."

---

## **4. Why EKFs Fail**1.  **Bad Initial Covariance:** If you tell the filter the Lidar is "Perfect" when it is actually noisy, the filter will "chase the noise" and the drone will twitch.
2.  **Divergence:** If your "Guess" is too far from "Reality," the Jacobians become wrong, the ramps point the wrong way, and the math explodes.

**Study Task:** Look up "Bayesian Filtering." It is the parent of the Kalman Filter.
--- [Return to Course Map](../../../COURSE_MAP.md)