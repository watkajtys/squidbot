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
*   Your `ekf.yaml` configuration file.

### **7.3.1 Sub-Lab: The Outlier Rejection Test**
**"Don't believe everything you hear."**

In Theory Deep Dive 2, we discussed Mahalanobis Distance. Now, we test it.

1.  **Setup:** Establish a stable hover at 1.0m.
2.  **The Attack:** Quickly slide a piece of paper 10cm under the Downward Lidar for 1 second, then remove it.
3.  **Observation:**
    *   **Fail:** The drone "jumps" up or down because it trusted the paper.
    *   **Success:** The drone remains steady. The EKF log should show that the Lidar measurement was "Rejected" as an outlier because it violated the laws of physics (you can't teleport 90cm in 0.01 seconds).

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

### **7.5 Calibration: The Compass**

### **Objective**
Stop the "Toilet Bowl Effect."

### **Theory**
A raw magnetometer does not point North. It points at the magnets in your motors and the iron screws in your frame.
*   **Hard Iron Distortion:** A constant magnetic offset (add/subtract vector). Shifts the sphere center away from (0,0,0).
*   **Soft Iron Distortion:** The frame stretches the magnetic field. Turns the sphere into an ellipsoid.

### **Lab Procedure**
1.  **Data Collection:** Write `src/drivers/mag_cal.py` to record `mx, my, mz` while you rotate the drone in all axes (the "Mag Dance").
2.  **Fitting:** Use an Ellipsoid Fitting algorithm (available in `scipy`) to find the Offset (Hard Iron) and Transformation Matrix (Soft Iron).
3.  **Apply:** $Mag_{calibrated} = (Mag_{raw} - Offset) \cdot Matrix$.
4.  **Visualize:** Plot Raw (Ellipsoid) vs Calibrated (Sphere) points.

#### **PhD Note: Tilt-Compensation**
A common mistake is calculating `yaw = atan2(my, mx)`. This only works if the drone is perfectly flat.
*   **The Problem:** When the drone pitches forward to move, the Magnetometer tilts into the Earth's vertical magnetic field. Your "North" will suddenly jump 30 degrees.
*   **The Fix:** You must use your **Accelerometer** data to "de-tilt" the magnetic vector back to the horizontal plane before calculating the heading. 
*   **In ROS 2:** The `imu_filter_madgwick` or `ekf_filter_node` handles this for you, but you must ensure your `gravity_vector` is clean.

---

## **Check: The Push Test**
**Robustness Verification.**

1.  **Hover:** Drone at 1.0m.
2.  **Attack:** Cover the Downward Lidar with your hand for 1 second (simulating sensor failure).
3.  **Result:**
    *   **Without EKF:** The drone thinks it hit the ground (Dist=0) and applies Full Throttle (Ceiling Crash).
    *   **With EKF:** The drone sees the jump is "Impossible" (violates physics), ignores the Lidar, and trusts the IMU/Barometer for a few seconds. It holds altitude.

**Submission:** A video of the "Hand Block" test where the drone does *not* freak out.
