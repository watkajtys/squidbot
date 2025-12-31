# Theory Deep Dive 0.4: System Identification (SysID)
**"Measuring the Soul of the Machine."**

Standard drones are "tuned." Research drones are "identified."
System Identification is the process of using data to find the mathematical parameters of your drone so your simulation matches reality 1:1.

---

## **1. The Thrust-Constant ($K_t$)**
How many Newtons of lift does a motor produce at 50% throttle? 
If you don't know this, your "Gravity Compensation" math is just a guess.

**The Experiment:**
1.  Upside-down drone on a scale.
2.  Increase throttle in 10% increments.
3.  Record the "Weight" change.
4.  **Math:** $Thrust (N) = \Delta Mass (kg) \times 9.81$

---

## **2. The Moment of Inertia ($J$)**
How hard is it to rotate the drone? A heavy battery makes the drone "sluggish." 
*   **The Math:** $\tau = J \alpha$ (Torque = Inertia x Angular Acceleration).
*   In Module 5, you will use your flight logs to solve for $J$ by looking at how fast the drone starts spinning when you give it a "kick" of torque.

---

## **3. The "Black Box" Approach**
If you don't want to measure every screw and wire, you use **Frequency Response**.
1.  Send a "Sine Sweep" (a vibration that goes from 1Hz to 50Hz) to the motors.
2.  Measure the Gyro response.
3.  **The Bode Plot:** This tells you the "Bandwidth" of your drone—how fast it can actually react before physics takes over.

**Research Task:** Look up "Least Squares Regression for Parameter Estimation." This is the math tool used to find $K_t$ and $J$ from noisy data.
