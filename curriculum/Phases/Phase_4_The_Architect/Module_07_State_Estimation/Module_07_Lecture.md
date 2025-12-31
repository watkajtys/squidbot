[Previous Module](../Module_06_5_Reliability/Module_06_5_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_07_5_Forensics/Module_07_5_Lecture.md)

---

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
1.  **The Stamp:** Ensure every ROS 2 message has a `header.stamp`.
2.  **The Interpolator:** Write a function that takes two IMU readings ($t_1, t_2$) and estimates the state at $t_{cam}$ (where $t_1 < t_{cam} < t_2$).

---

## **7.2 The Kalman Filter (Mathematical Engine)**

### **The Fundamental State Vector**
We represent the drone's "Belief" as a 12D state vector $\mathbf{x}$:
$$ \mathbf{x} = [x, y, z, \dot{x}, \dot{y}, \dot{z}, \phi, \theta, \psi, p, q, r]^T $$
*   **The Covariance Matrix ($P$):** A $12 \times 12$ matrix representing the "Confidence" in our guess. The diagonal elements are the variance of each state.

### **The Jacobian Derivation: Linearizing the Chaos**
Because drone physics ($f(x)$) involves sines and cosines, we cannot use a standard Linear Kalman Filter. We must find the **Jacobian** $F$.
*   **The Math:** $F_{ij} = \frac{\partial f_i}{\partial x_j}$.
*   **Intuition:** The Jacobian is the "Best Linear Map" of the drone's movement at a specific millisecond. If the drone is tilted at $30^{\circ}$, the Jacobian tells the EKF exactly how much "Up" thrust will translate into "Forward" acceleration.

### **7.2.1 Advanced Estimation Philosophy**
1.  **EKF (Extended):** High speed, low RAM. Uses Taylor Series (Jacobians). *Squid Standard.*
2.  **UKF (Unscented):** No Jacobians required. It uses "Sigma Points" to sample the distribution. More accurate for highly non-linear maneuvers but uses 3x more CPU.
3.  **PF (Particle Filter):** The "Gold Standard" for mapping. Uses 1000s of "Ghost Drones" to represent the probability. Only used in Module 15 for Global Recognition.

### **7.2.2 Just-In-Time Math: The Drunk and The Sniper**
**"Understanding the Kalman Gain ($K$)"**

In `lab_4_2_ekf.py`, the math can look intimidating. Use this mental model:
*   **The Drunk (Physics/IMU):** "I think I took 5 steps." (High Drift, Fast).
*   **The Sniper (Lidar/GPS):** "You are at position X." (Low Drift, Slow).
*   **$P$ (Covariance):** The "Trust Slider." If $P$ is large, we trust the Sensor. If $P$ is small, we trust the Physics.

**The "Joseph Form" (The PhD Guard):**
You will see a line: `P = (I-KH)P(I-KH)' + KRK'`.
*   **Why?** In standard math, $10.0 - 9.999 = 0.001$. In computers, it might be $-0.00001$.
*   **The Crash:** Negative uncertainty is impossible. It causes the drone to divide by zero. The Joseph Form squares the matrices, ensuring the result is always positive.

**AI Prompt:** "My EKF covariance matrix P became non-positive definite. Check my 'Joseph Form' implementation in the update step."

### **7.2.3 Just-In-Time Math: The Invisible Drone (Observability)**
**"You can't see what doesn't move."**

*   **The Problem:** Some things are "Unobservable." 
*   **The Analogy:** Imagine you are in a pitch-black room with a 1D Lidar pointing Down. You know your Height (Z). But if you slide 1 meter North (X), the Lidar reading doesn't change. You are "Invisible" in the X-direction.
*   **The Lesson:** To find your X and Y, you **must** have a camera or a GPS. If you don't, the EKF covariance matrix $P$ will grow to infinity, even if you are hovering perfectly.

**AI Prompt:** "What is 'Observability' in a Kalman Filter? Give an example of a drone state that becomes unobservable if the GPS is lost."

---

## **7.3 Sensor Fusion Architecture**

### **The Rate Mismatch Problem**
*   **IMU (The Prediction):** 1000Hz. High-frequency, high-drift.
*   **Vision/Lidar (The Correction):** 30Hz. Low-frequency, zero-drift.
*   **The Solution:** We run the **Prediction Step** (Equations 1 & 2) 1000 times a second. We only run the **Correction Step** (Equations 3, 4, 5) when a new pixel/Lidar byte arrives. This allows the drone to react to a gust of wind in $1\text{ms}$ while still knowing its absolute position.

### **7.3.1 Sub-Lab: Innovation & The Residuals**
**"Is the filter healthy?"**

1.  **Innovation ($y$):** The difference between what the sensor says ($z$) and what the filter predicted ($Hx$). 
2.  **Residual Analysis:** In a research paper, we plot the **Residuals**.
    *   **Healthy:** The residuals look like White Noise (centered at 0).
    *   **Unhealthy:** The residuals have a "Bias" (e.g., always positive). This proves your Lidar is mounted at a slight angle or your gravity vector is wrong.

---

## **7.4 Socratic Discussion: The Truth of Sensors**
1.  **Question:** If the Accelerometer says we are at $1\text{m}$ and the Lidar says we are at $1.1\text{m}$, where are we?
    *   **Answer:** We are at the **Weighted Average**. If our Lidar confidence ($R$) is $0.01$ and our Accel confidence ($Q$) is $0.1$, we trust the Lidar $10\text{x}$ more.
2.  **Question:** Why do we ignore the Compass (Magnetometer) during a motor punch-out?
    *   **Answer:** High current creates magnetic fields (**Biot-Savart Law**). A $20\text{A}$ current $2\text{cm}$ from the sensor will create a field stronger than the Earth's North Pole, making the sensor useless.

---

## **7.3 Sensor Fusion Implementation**

### **Objective**
Fuse Optical Flow (Velocity) with Lidar (Altitude).

### **Lab Procedure**
1.  **The Tool:** We will not write a full EKF from scratch (that is a PhD thesis). We will use the `robot_localization` ROS 2 package.
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

## **0.7.4 The "Deep-Dive" Breakout: Scratch EKF**

> **Professor's Note:** The Kalman Filter is often treated as "Voodoo." We are breaking that tradition.

> **Why?** If you use a library, you won't understand how the **Covariance Matrix ($P$)** actually represents the drone's "confidence."

> **The Goal:** You will implement the Predict and Update steps using pure Matrix Multiplication (`numpy.matmul`). You will derive the **Jacobians** for a 6-DOF system on paper before writing a single line of code.



---



## **The "Paper" Test**



---



## **Theoretical Foundations**

### Lecture 6 & 7: State Estimation (The Kalman Filter)

#### **1. Optimal Bayesian Inference (MAP Estimation)**
A State Estimator is a **Recursive Bayesian Filter**. We seek to find the "Maximum A Posteriori" (MAP) estimate of the state $x$.
*   **Prediction (The Physics):** We project the probability distribution forward in time: $P(x_k | z_{1:k-1})$. The "Bell Curve" of our position gets wider (Uncertainty increases).
*   **Correction (The Measurement):** We multiply the Predicted distribution by the Likelihood of the sensor: $P(z_k | x_k)$. 
*   **The Result:** The intersection of these two curves is the "Optimal" estimate—a thinner bell curve that is mathematically closer to the truth than either the sensor or the physics alone.

#### **2. Linearization & The Jacobian Matrix**
The "Extended" in EKF refers to the linearization of non-linear drone physics ($f(x,u)$) and sensor models ($h(x)$).
*   **The Jacobian ($F$ and $H$):** We compute the partial derivatives of the physics relative to the state: $F = \frac{\partial f}{\partial x}$.
*   **The Trap:** If the drone rotates too quickly, the "Taylor Series" approximation used in linearization fails. This causes **Filter Divergence**, where the EKF "hallucinates" its position and crashes the drone.

#### **3. Numerical Integrity & Outlier Rejection**
*   **The Joseph Form:** Computers have rounding errors. If the Covariance matrix ($P$) loses "Positive Definiteness," the EKF math fails. We implement the **Joseph Form Update** ($P_k = (I-KH)P(I-KH)^T + KRK^T$) which is mathematically guaranteed to remain positive.
*   **Mahalanobis Distance ($D_M$):** We check the "Innovation" ($y = z - H\hat{x}$). If $D_M = \sqrt{y^T S^{-1} y} > 3.0$, we assume the sensor is lying (e.g., Lidar hitting a mirror) and **Reject the Measurement**.

### Lecture 7.6: Interacting Multiple Model (IMM) Filters
A drone behaves differently when hovering vs. when racing at $10\text{ m/s}$. 
*   **Multi-Model logic:** We run two EKFs in parallel: one with a "Static" physics model and one with a "Dynamic" model. 
*   **The Mixer:** We compute the probability of each model being correct. When you "punch" the throttle, the IMM instantly "switches" the control priority to the Dynamic model, eliminating the estimation lag found in single-model filters.

**Next Step:** [Phase V: Module 8 Perception & Mapping](../../Phase_5_The_Researcher/Module_08_Perception_and_Mapping/Module_08_Lecture.md)

---

## **Check: Troubleshooting the Truth**

| Symptom | Probable Cause | Physical Reality |
| :--- | :--- | :--- |
| **"The Teleportation"** | Filter Divergence | Your Jacobian was wrong. The EKF calculated that a forward move meant a sideways slide. The math "split" from reality and the drone flew into a wall. |
| **"The Smug Filter"** | Low $Q$ variance | You told the filter your physics model was "Perfect" ($Q \approx 0$). The filter now ignores the real sensors because it thinks its own math is smarter than reality. |
| **"The Paranoid Filter"** | High $R$ variance | You told the filter your sensors are "Lying" ($R \approx \infty$). The drone will ignore the Lidar and drift based on the last IMU noise it felt. |

---

## **Professor's Brief: Speaker Notes**

### **The Narrative Hook**
"Imagine you are walking in a pitch-black room. You take steps (IMU) and you occasionally touch a wall (Lidar). The Kalman Filter is the part of your brain that combines your feeling of motion with your touch to create a stable map. It is the math of intuition. Today, we learn why real robots 'feel' their way through the world with more precision than a human ever could."

### **Deep Research Context: Observability**
In PhD-level research, we must perform an **Observability Analysis**. Not all states can be found. If you only have a 1D Lidar and no camera, you can observe 'Z' (altitude), but 'X' and 'Y' are **Unobservable**. Your EKF will report that it knows where it is, but the covariance will grow to infinity. This is the #1 reason for "Flyaways" in autonomous drones.

## **Mastery Checklist**
Before moving to the next module, you should be able to:
- [ ] Calculate a 1D Jacobian for a simple non-linear sensor model.
- [ ] Explain why the Joseph Form is used for covariance updates in 32-bit systems.
- [ ] Describe the three standard deviations rule for Mahalanobis-based outlier rejection.
- [ ] Identify when a drone state becomes "Unobservable" given a specific sensor suite.

---

## **Further Reading & Bibliography**

### **Estimation Standards**
*   **Kalman, R. E. (1960).** *"A New Approach to Linear Filtering and Prediction Problems."* Journal of Basic Engineering. (The foundation of modern navigation).
*   **Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001).** *Estimation with Applications to Tracking and Navigation.* Wiley. (Deep source for IMM and residual analysis).

### **Robotics-Specific Theory**
*   **Thrun, S., Burgard, W., & Fox, D. (2005).** *Probabilistic Robotics.* MIT Press. (Context for Bayesian filters in spatial logic).

---

[Previous Module](../Module_06_5_Reliability/Module_06_5_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_07_5_Forensics/Module_07_5_Lecture.md)