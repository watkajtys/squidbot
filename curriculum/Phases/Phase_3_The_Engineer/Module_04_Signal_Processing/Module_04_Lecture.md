[Previous Module](../../Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/Module_03_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_05_Control_Theory/Module_05_Lecture.md)

---

# Module 4: Signal Processing & Geometry
**"The map is not the territory."**

We have motors spinning and cameras streaming. But the data is "dirty" (vibration) and "warped" (lens distortion). Before we can write a Control Loop (Module 5), we must clean our inputs.

---

## **4.1 Coordinate Frames**

### **Objective**
Understand where "Down" is.

### **Theory**
*   **Body Frame ($B$):** Attached to the drone. X is "Nose Forward". Y is "Left Wing". Z is "Top".
*   **Inertial Frame ($W$):** Attached to the ground. X is North. Y is East. Z is Up (Gravity inverted).
*   **The Rotation Matrix:** To convert a vector from Body to World, we multiply by $R$.

### **Lab Procedure**
1.  **The IMU Driver:** Write `src/drivers/imu.py` (MPU6050 or BMI270). Read Accel/Gyro raw values.
2.  **The Math:** Implement a function `body_to_world(vector, roll, pitch, yaw)`.
3.  **Experiment:**
    *   Hold the drone flat. Accel Z should be ~9.8 (1G). Accel X/Y should be 0.
    *   Pitch the nose down 90 degrees. Accel X should be 9.8.
    *   **Challenge:** Use your matrix to rotate the vector back to the World Frame. Even if the drone is tilted, the *rotated* Z-vector should still show 9.8.

### **Deliverable**
*   A script that prints the "World Z Acceleration". It should stay near 9.8m/s² even as you tumble the drone in your hands.

### **4.1.1 Just-In-Time Math: The 4D Sphere (Quaternions)**
**"Avoiding the Gimbal Lock"**

In the lecture, we mentioned "Gimbal Lock." This happens when you use Euler Angles (Roll, Pitch, Yaw). If you pitch up 90 degrees, your "Roll" and "Yaw" axes become the same axis. You lose a dimension of control.
*   **The Fix:** Quaternions ($w, x, y, z$).
*   **The Analogy:** Imagine a sphere. To rotate, you pick a point on the surface of a 4D sphere.
    *   It never "Locks" because a sphere has no corners.
    *   **The Math:** To rotate vector $v$ by quaternion $q$: $v_{new} = q \cdot v \cdot q^{-1}$.
    *   **The Trap:** You must always keep the quaternion "normalized" (length = 1). If it shrinks ($0.99$), the drone physics will warp.

**AI Prompt:** "I have a quaternion [w, x, y, z]. How do I convert this to a Rotation Matrix in Python using scipy.spatial.transform?"

### **4.1.2 Just-In-Time Logic: The Reference Frame Trap**
**"Forward is not always Forward."**

When you tell a drone to move "Forward," what do you mean?
*   **Local (Body) Frame:** Move in the direction the camera is currently pointing. If the drone is tilted, "Forward" might be "Down."
*   **Global (World) Frame:** Move "North" relative to where you started.
*   **The Trap:** If you apply a Body-Frame command to a World-Frame goal, the drone will spiral into the ground.
*   **The Solution:** You must rotate your command vector using the **Rotation Matrix** from Module 4.1.

**AI Prompt:** "I have a velocity command [1.0, 0.0, 0.0] in the Body Frame. How do I use a Rotation Matrix R to convert this to the World Frame velocity?"

---

## **4.2 Vibration Analysis & Filtering**

### **Objective**
Kill the noise found in [Theory Lab 0.1](../../../Library/00_Foundations/Theory_0_Concepts.md#1-the-physics-of-vibration-frequency-domain).

### **Theory**
*   **Low Pass Filter (LPF):** Allows slow movements (turning), blocks fast ones (vibration).
*   **Notch Filter:** Surgically removes a specific frequency (e.g., 200Hz motor whine).

### **Lab Procedure**
1.  **Data Collection:**
    *   Tape the drone to the desk (Safety!).
    *   Spin motors to 30% throttle.
    *   Log high-speed IMU data (1kHz) using your Logger from Module 2.
2.  **Analysis:**
    *   Run the FFT notebook (`curriculum/Library/02_Control_Dynamics/01_fft_analysis.ipynb`) on your real data.
    *   Find the peak frequency (e.g., 180Hz).
3.  **Implementation:**
    *   Implement a `NotchFilter` class in Python.
    *   `output = input - k * (bandpass_output)` (Simplified).
    *   Apply it to your live IMU stream.

### **4.2.1 Sub-Lab: The Nyquist Shadow**
**"The Ghost Frequency."**

1.  **Generate Noise:** Spin motors to create a 200Hz vibration.
2.  **Ground Truth:** Log at 1000Hz. You see the 200Hz wave clearly.
3.  **The Mistake:** Downsample the data to 50Hz (Standard Loop Rate) *without* an anti-aliasing filter.
4.  **The Result:** The 200Hz signal disappears. A new, fake 50Hz "wobble" appears. The drone thinks it is rocking, but it is just buzzing. This is why we need hardware filters (LPF).

### **4.2.2 Sub-Lab: Phase Lag Visualization**
**"The Cost of Cleanliness."**

Filters clean the noise, but they steal your time.

1.  **Plot:** Use `tools/plot_log.py` to plot `Raw_Gyro_X` and `Filtered_Gyro_X` on the same axis.
2.  **Zoom:** Zoom into a single oscillation.
3.  **Measure:** Look at the "Peak" of the raw wave vs the "Peak" of the filtered wave.
4.  **The Discovery:** The filtered peak happens later (e.g., 8ms later). 
5.  **The PhD Lesson:** If your total delay (Filter + Wi-Fi + ESC) exceeds the drone's physical response time, you will never achieve a stable hover. This is why we use "Low-Latency" filters like the **Biquad Notch**.

### **Deliverable**
*   "Before" and "After" plots of your Gyroscope noise. The "After" line should be smooth.

### **4.2.3 Just-In-Time Math: The Noise Canceling Headphone**
**"Killing the Whine"**

In the Filtering Lab, you use an FFT and a Notch Filter.
*   **The FFT (Fast Fourier Transform):** Imagine un-baking a cake. The FFT takes the "Cake" (Raw Vibration Log) and tells you the ingredients: "This vibration is 10% Wind, 80% Motor Whine (200Hz), and 10% Propeller Balance."
*   **The Notch Filter:** This is a "Targeted Assassin." It doesn't just turn down the volume (Low Pass); that would make the drone sluggish. Instead, it surgically removes *only* the 200Hz frequency.
*   **The Analogy:** It's like Noise Canceling Headphones that silence the jet engine drone but let you hear the pilot speaking.

**AI Prompt:** "Explain how a Biquad Notch Filter works. Give me the Python code to design a 200Hz notch filter for a system running at 1000Hz sampling rate."

## 4.2.4 The "Deep-Dive" Breakout: Least Squares SysID
> **Professor's Note:** Don't guess your motor constants. Measure them with math.
> **Why?** A simulator is only as good as its parameters. If your thrust-to-weight ratio is wrong by 10%, your MPC controller will never be stable.
> **The Goal:** You will use **Ordinary Least Squares (OLS)** to find the exact $K_t$ and $J$ (Inertia) of your drone using real flight logs. This is the math that bridges the "Reality Gap."

For the derivation of the OLS Normal Equation, see [Theory 4.1: Least Squares SysID](../../../Library/02_Control_Dynamics/Theory_4.1_Least_Squares_SysID.md).

---

## **4.3 Camera Calibration**

### **Objective**
Straighten the fishbowl.

### **Theory**
The Arducam lens distorts the world. We need the **Camera Matrix ($K$)** and **Distortion Coefficients ($D$)**.

### **Lab Procedure**
1.  **Print:** A checkerboard pattern (provided in `assets/checkerboard.pdf`).
2.  **Capture:** Take 20 photos of the board from different angles using the drone camera.
3.  **Solve:**
    *   Write `src/vision/calibrate.py` using `cv2.calibrateCamera`.
    *   Save the resulting matrices to `config/camera_calib.yaml`.
4.  **Verify:** Run `cv2.undistort()` on a live feed. Straight lines should appear straight.

### Deliverable
*   A "Before/After" image of a doorframe or tiled floor.

---

## **4.4 Extrinsic Calibration (The Handshake)**

### **Objective**
Measure the offset between the Eye (Camera) and the Inner Ear (IMU).

### **Theory**
VIO algorithms solve for the Drone's position. But the Camera is mounted 3cm in front of the center.
$ T_{body_cam} = [R | t] $
If this is wrong, the math thinks the drone is "drifting" when it is just rotating. See [Theory 0.3: Calibration](../../../Library/01_Sensing_Estimation/Theory_0.3_Calibration_and_Extrinsics.md).

### **Lab Procedure**
1.  **Translation ($t$):** Use digital calipers. Measure the distance (X, Y, Z) from the center of the Pi Zero (approx IMU location) to the center of the Camera Lens.
    *   *Example:* `[0.03m, 0.00m, -0.01m]` (3cm forward, 1cm down).
2.  **Rotation ($R$):**
    *   The Arducam is mounted flat? Then $R = Identity$.
    *   Is it tilted down 15 degrees? Then you must compute the Rotation Matrix for `pitch = -15`.

### **Deliverable**
*   Add these values to your `config/robot_params.yaml`.

## **4.4.1 The "Deep-Dive" Breakout: SO(3) & Lie Groups**
> **Professor's Note:** We are moving beyond simple Roll/Pitch/Yaw. 
> **Why?** Aerial robotics happens in 360 degrees. Euler angles will fail you during an aggressive recovery or a flip. 
> **The Goal:** You will implement a rotation update using the **Exponential Map**. You'll learn why we do math in the "Flat" Lie Algebra before projecting it back to the "Curved" Manifold.

For the theory behind the Exponential Map, see [Theory 1.1: Lie Groups & SO(3)](../../../Library/00_Foundations/Theory_1.1_Lie_Groups_SO3.md).

---

## **4.5 Magnetometer Calibration**

### **Objective**
Remove magnetic distortion.

### **Theory**
The Compass is sensitive.
*   **Hard Iron:** Permanent magnets (Speakers, Screws) add a constant offset.
*   **Soft Iron:** Metal frames warp the magnetic field into an oval.

### **Lab Procedure**
1.  **Record:** Spin the drone 360 degrees on all axes (The "Calibration Dance").
2.  **Plot:** Plot Mag_X vs Mag_Y.
    *   *Bad:* It looks like an ellipse shifted off-center.
    *   *Good:* It looks like a circle centered at (0,0).
3.  **Fix:** Calculate the offset (bias) and scale factor to turn the ellipse into a circle.

---

## **Check: The Flatline**
**Proof of Stability.**

1.  **Setup:** Drone ON, Motors spinning at "Hover Throttle" (props off or weighed down), Camera ON.
2.  **Metric:**
    *   Gyro noise < 0.5 deg/sec.
    *   Accel noise < 0.1 G.
    *   Video feed shows straight lines (Door frames).
3.  **Test:** If you can achieve this "Flatline" of clean data while the motors are screaming, you are ready for Control Theory.

**Submission:** A plot of your filtered Gyro data during a motor run-up.

---
## **Theoretical Foundations**

### Lecture 4: Signal Processing & Coordinate Geometry

#### **1. Rigid Body Transformations (NED & Body Frames)**
In robotics, we constantly map vectors between coordinate systems.
*   **Body Frame ($B$):** Fixed to the airframe. Right-Hand Rule: X forward, Y left, Z up.
*   **World Frame ($W$):** The "North-East-Down" (NED) standard used in aerospace navigation.
*   **The Rotation Matrix ($R_{WB}$):** A $3 \times 3$ orthogonal matrix where $R^T = R^{-1}$. To find the drone's velocity in the world frame ($v_W$), we compute $v_W = R_{WB} v_B$.

#### **2. The Quaternion Advantage ($S^3$ Manifold)**
Standard **Euler Angles** (Roll, Pitch, Yaw) suffer from **Gimbal Lock**—a mathematical singularity where one degree of freedom is lost at $90^{\circ}$ pitch.
*   **Unit Quaternions ($q$):** A 4D vector $q = [w, x, y, z]$ representing a point on a unit hypersphere ($||q||=1$).
*   **The Hamilton Product:** Successive rotations are calculated via quaternion multiplication, which is more computationally efficient and numerically stable than multiplying $3 \times 3$ matrices.

#### **3. Information Theory: Nyquist-Shannon Sampling**
How frequently must our "Game Loop" run to see the physics?
*   **The Theorem:** To perfectly reconstruct a signal with maximum frequency $B$, you must sample at $f_s > 2B$. 
*   **Aliasing:** If we sample below this rate, high-frequency motor noise "folds" into the low-frequency control band, appearing as a "Wobble" that the PID controller will incorrectly try to fight.
*   **Anti-Aliasing:** We utilize **Digital Low-Pass Filters (LPF)** and **Notch Filters** to suppress frequencies above the Nyquist limit before they hit the control logic.

**Next Step:** [Module 5: Control Theory](../Module_05_Control_Theory/Module_05_Lecture.md)

---

## **Check: Troubleshooting the Geometry**

| Symptom | Probable Cause | Physical Reality |
| :--- | :--- | :--- |
| **"The Ghost Yaw"** | Gimbal Lock | You used Euler angles. When the drone pitches to $90^°$, the math "collapsed" and it can't tell the difference between Roll and Yaw. |
| **"The Twitchy Motor"** | Nyquist Trap | You are sampling at $100\text{Hz}$ but your motors vibrate at $333\text{Hz}$. The noise is "aliasing" into a slow $10\text{Hz}$ wobble that the controller is trying to fight. |
| **"Shrinking Drone"** | Quaternion Drift | You forgot to normalize your Quaternion. Over time, the magnitude $||q||$ became $0.9$, and the physics engine thinks the drone's inertia has changed. |

---

## **Professor's Brief: Speaker Notes**

### **The Narrative Hook**
"Everything vibrates. A drone is just a collection of oscillations looking for an excuse to explode. Signal Processing is the 'Inner Ear' of the robot. If the ear is clogged with noise, the brain cannot balance the body. Today, we learn the language of 3D space: how to describe rotation without ever getting 'lost' in a singularity."

### **Deep Research Context: The "Ghost" in the Math**
In research, we must distinguish between **Passive** and **Active** rotations. A passive rotation changes the coordinate system but keeps the vector fixed; an active rotation moves the vector. Swapping these is the #1 cause of crashes in autonomous systems. Furthermore, quaternions have a **Double-Cover** property: $q$ and $-q$ represent the same physical rotation. If your code doesn't handle this "Sign Flip," your EKF will experience sudden, catastrophic "Jumps" in estimated attitude.

## **Mastery Checklist**
Before moving to the next module, you should be able to:
- [ ] Convert a velocity vector from the ENU (World) frame to the NED (Aviation) frame.
- [ ] Explain why Quaternions are superior to Euler Angles for 3D aerial robotics.
- [ ] Identify the Nyquist Frequency for a sensor running at a specific loop rate.
- [ ] Describe the "Aliasing Ghost" and how digital filtering suppresses it.

---

## **Further Reading & Bibliography**

### **Geometric Math**
*   **Diebel, J. (2006).** *"Representing attitude: Euler angles, unit quaternions, and rotation vectors."* Stanford University Technical Report. (The definitive guide).
*   **Kuipers, J. B. (1999).** *Quaternions and Rotation Sequences.* Princeton University Press. (Deep treatment of the math).

### **Signal Processing**
*   **Oppenheim, A. V., & Schafer, R. W. (1999).** *Discrete-Time Signal Processing.* Prentice Hall. (The standard textbook).

---

[Previous Module](../../Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/Module_03_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_05_Control_Theory/Module_05_Lecture.md)