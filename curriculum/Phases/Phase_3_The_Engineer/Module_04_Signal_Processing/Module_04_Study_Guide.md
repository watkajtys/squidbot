[Return to Course Map](../../../../COURSE_MAP.md)

# Study Guide 4: Signal Processing and Geometry
**Module 4: Cleaning the Noise**

### Critical Takeaways
1.  **The Nyquist-Shannon Sampling Theorem:** You must sample at least twice as fast as the highest frequency vibration present in the drone's frame. If a motor vibrates at 250Hz and you sample at 400Hz, those 250Hz signals will "alias" and appear as phantom 150Hz oscillations in your PID controller, causing unfixable instability.
2.  **The Phase-Lag Penalty:** Every filter adds a time-delay (Phase Lag). In a high-speed flight controller, a delay of even 10ms can turn a stable drone into a "pendulum" that oscillates itself into the ground. Choosing the right "Cutoff Frequency" is a delicate balance between noise reduction and control delay.
3.  **The Notch Filter:** Drones have specific, narrow-band resonant frequencies caused by motor rotation. A Notch Filter is a "surgical" filter that removes exactly one frequency while leaving the rest of the spectrum untouched. This is essential for flying high-power "racing" drones.
4.  **Rotation Mathematics ($SO(3)$):** Robotics is the study of moving between frames (World to Body, Body to Camera). We use Rotation Matrices and Quaternions because Euler Angles (Roll, Pitch, Yaw) suffer from "Gimbal Lock”—a mathematical singularity where you lose one degree of freedom at a 90-degree pitch.

### Mental Models and Field Notes
*   **The Filter Delay:** Imagine you are steering a car, but there is a 2-second delay between you turning the wheel and the car moving. You would constantly over-correct. This is exactly what happens when you set your Low-Pass Filter frequency too low on a drone.
*   **Sign Errors (The Silent Killer):** As noted in the `Robotics_Debugging_Guide.md`, the #1 cause of first-flight crashes is a sign error. If your Gyro says "Negative 5" for a Right Roll, but your code expects "Positive 5," the PID will try to correct the roll by rolling *even harder* to the right. **Instant Flip.**
*   **The Small Angle Approximation:** In a stable hover, we often assume $\sin(\theta) \approx \theta$. This allows us to use simple Linear control laws. However, the moment your drone performs an aggressive maneuver (> 15 degrees), this "lie" falls apart and you must use full Non-Linear Trigonometry.

### Frontier Facts and Historical Context
*   **Bio-Inspired Sensing:** Dragonflies can hover and maneuver in heavy wind using "Halteres"—tiny vibrating organs that act as biological gyroscopes. Researchers are now building "Vibratory Gyroscopes" that mimic this, allowing for IMUs that are essentially immune to electromagnetic noise.
*   **The "Duct Effect":** The Squid uses a ducted frame. At a hover, these ducts can increase thrust by 20% by "capturing" tip vortices (the Coanda Effect). However, in a crosswind, the duct acts like a sail, introducing a massive "Yaw" disturbance that your filters must handle.
*   **Gimbal Lock and Apollo 11:** During the Apollo 11 moon landing, the astronauts were warned of "Gimbal Lock" in their inertial platform. If they pitched too far, their navigation system would literally forget which way was up. This is why we use Quaternions in the Squid Project—they have no "Gimbal Lock."

---

### The Squid Games: Level 4
**The Statistically Sane Sensor Challenge**
Using your FFT logic from Module 4, identify the dominant vibration frequency of your drone while its motors are spinning at 50% throttle (Props OFF).
*   **The Goal:** Design a digital Notch Filter in Python that reduces the amplitude of that specific vibration by at least 15dB.
*   **Win Condition:** A plot of your "Raw" vs "Filtered" IMU data showing the vibration peak has been successfully removed without adding more than 5ms of delay to the signal.

---

### Module 4 Quiz
1.  **Aliasing:** If your motor vibrates at 600Hz, what is the *minimum* sampling frequency required to avoid aliasing?
2.  **Butterworth Filters:** What is "Roll-off," and why is a 2nd-order filter more aggressive than a 1st-order filter?
3.  **Geometry:** Prove that a 2D Rotation Matrix is orthogonal. Why is this property useful for computer memory?
4.  **Lie Theory:** In `Lecture 4`, we discuss the "Hat" operator. How do you turn a 3-element angular velocity vector $\omega$ into a skew-symmetric matrix $[\omega]_\times$?

---
*Reference: Lecture 4 (Rotations and Lie Groups) in docs/LECTURES.md*