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

---

## **4.2 Vibration Analysis & Filtering**

### **Objective**
Kill the noise found in Theory Lab 0.1.

### **Theory**
*   **Low Pass Filter (LPF):** Allows slow movements (turning), blocks fast ones (vibration).
*   **Notch Filter:** Surgically removes a specific frequency (e.g., 200Hz motor whine).

### **Lab Procedure**
1.  **Data Collection:**
    *   Tape the drone to the desk (Safety!).
    *   Spin motors to 30% throttle.
    *   Log high-speed IMU data (1kHz) using your Logger from Module 2.
2.  **Analysis:**
    *   Run the FFT notebook (`theory/01_fft_analysis.ipynb`) on your real data.
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
If this is wrong, the math thinks the drone is "drifting" when it is just rotating.

### **Lab Procedure**
1.  **Translation ($t$):** Use digital calipers. Measure the distance (X, Y, Z) from the center of the Pi Zero (approx IMU location) to the center of the Camera Lens.
    *   *Example:* `[0.03m, 0.00m, -0.01m]` (3cm forward, 1cm down).
2.  **Rotation ($R$):**
    *   The Arducam is mounted flat? Then $R = Identity$.
    *   Is it tilted down 15 degrees? Then you must compute the Rotation Matrix for `pitch = -15`.

### **Deliverable**
*   Add these values to your `config/robot_params.yaml`.

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
