# Module 7: State Estimation (The Truth)
**"Sensors lie. Statistics tell the truth."**

A Lidar says you are at 1.0m. An Accelerometer says you are moving Up. But the Lidar reading just jumped to 5.0m for one frame (glitch). Who do you trust?
The **Extended Kalman Filter (EKF)** answers this question.

---

## **7.1 Time Synchronization**

### **Objective**
Align the timeline.

### **Theory**
*   **IMU:** 1000Hz (Fast, but drifts).
*   **Camera/Lidar:** 30Hz (Slow, but accurate).
*   **The Problem:** When the camera says "I am at X", that happened 33ms ago. The IMU has moved since then.
*   **Buffer:** We must keep a history of IMU states to "rewind" and apply the camera correction at the correct timestamp.

### **Lab Procedure**
1.  **The Stamp:** Ensure every ROS2 message has a `header.stamp`.
2.  **The Interpolator:** Write a function that takes two IMU readings ($t_1, t_2$) and estimates the state at $t_{cam}$ (where $t_1 < t_{cam} < t_2$).

---

## **7.2 The Kalman Filter (Concept)**

### **Objective**
Predict and Correct.

### **Theory**
1.  **Predict (IMU):** "I was at 0. I moved up at 1m/s for 0.1s. I *should* be at 0.1m." (Uncertainty increases).
2.  **Correct (Lidar):** "Measurement says I am at 0.12m." (Uncertainty decreases).
3.  **The Gain ($K$):** How much do we trust the Lidar vs the IMU?
    *   If Lidar Variance is High (Noisy), $K$ is low. Trust IMU.
    *   If Lidar Variance is Low (Precise), $K$ is high. Trust Lidar.

---

## **7.3 Sensor Fusion Implementation**

### **Objective**
Fuse Optical Flow (Velocity) with Lidar (Altitude).

### **Lab Procedure**
1.  **The Tool:** We will not write a full EKF from scratch (that is a PhD thesis). We will use the `robot_localization` ROS2 package.
2.  **Configuration (`ekf.yaml`):**
    *   **Input 1:** IMU (Measure: `accel`, `gyro`).
    *   **Input 2:** Optical Flow (Measure: `vel_x`, `vel_y`).
    *   **Input 3:** Lidar (Measure: `z`).
3.  **Tuning:** Set the covariance matrix values. (Tell the EKF that the Lidar is accurate to +/- 1cm).

### **Deliverable**
*   A plot showing **Raw Lidar** (Noisy) vs **EKF Estimate** (Smooth).

---

## **7.4 Power Modeling: The Battery Estimator**

### **Objective**
Don't trust the voltage.

### **Theory**
Voltage sags under load ($V_{measured} = V_{battery} - I \cdot R_{internal}$). A simple voltage check triggers "Low Battery" during a punch-out.

### **Lab Procedure**
1.  **Model:** Implement a basic Kalman Filter or Complimentary Filter for State-of-Charge (SoC).
2.  **Inputs:** Voltage (V), Current (A) (if available) or Throttle command (as proxy for Current).
3.  **Logic:** `Estimated_V = V_measured + (Throttle * K_sag)`. Use this "Resting Voltage Estimate" to decide when to land.

---

## **Check: The Push Test**
**Robustness Verification.**

1.  **Hover:** Drone at 1.0m.
2.  **Attack:** Cover the Downward Lidar with your hand for 1 second (simulating sensor failure).
3.  **Result:**
    *   **Without EKF:** The drone thinks it hit the ground (Dist=0) and applies Full Throttle (Ceiling Crash).
    *   **With EKF:** The drone sees the jump is "Impossible" (violates physics), ignores the Lidar, and trusts the IMU/Barometer for a few seconds. It holds altitude.

**Submission:** A video of the "Hand Block" test where the drone does *not* freak out.
