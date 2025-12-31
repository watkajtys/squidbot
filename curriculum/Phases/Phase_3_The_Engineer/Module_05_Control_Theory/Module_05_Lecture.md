[Previous Module](../Module_04_Signal_Processing/Module_04_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../../Phase_4_The_Architect/Module_06_ROS2_Migration/Module_06_Lecture.md)

---

# Module 5: Control Theory
**"Power without control is just a crash."**

You have a drone that can see (Sensors) and act (Motors). But if you connect them directly (`Motor = 1000 * Distance`), it will oscillate and crash. You need a **Controller**.

---

## **5.1 The Feedback Loop (PID)**

### **Objective**
Keep the drone at a fixed altitude (e.g., 0.5m) despite gravity and battery voltage changes.

### **Theory: The Mass-Spring-Damper Analogy**
A PID controller is mathematically equivalent to attaching a virtual spring and damper to your drone.
*   **$K_p$ (The Spring):** Pulls the drone toward the target. Higher $K_p$ makes the "spring" stiffer.
*   **$K_d$ (The Damper):** Acts like a shock absorber. It resists velocity, preventing the drone from overshooting and oscillating forever.
*   **$K_i$ (The Constant Force):** Compensates for gravity. Without $I$, the drone would always hover slightly *below* the target because the "spring" needs a constant error to produce enough lift to fight gravity.

### **The Cascaded Architecture (The Secret of Flight)**
Drones do not have a single PID loop. They use **Nested (Cascaded) Loops**.
1.  **Outer Loop (Position):** Runs at 10Hz. Input: Meters. Output: Desired Angle.
2.  **Middle Loop (Attitude):** Runs at 100Hz. Input: Angle. Output: Desired Rotation Rate.
3.  **Inner Loop (Rate):** Runs at 1000Hz. Input: Deg/Sec. Output: Motor PWM.
*   **Why?** The inner loop must be much faster than the outer loop to ensure that by the time the "Position" loop asks for a move, the "Rate" loop has already stabilized the vibrations.

---

## **5.2 Implementation Details (The PhD Guard)**

### **5.2.1 Feedforward: Predictive Control**
If you know your drone weighs 200g, you know it needs exactly $X$ amount of throttle to stay in the air. 
*   **PID only:** The drone falls, $K_i$ builds up slowly, then it rises. (Sluggish).
*   **Feedforward ($F_f$):** We add a "Gravity Base" to the output immediately.
    $$u = F_f + PID(error)$$
*   **Result:** The PID only has to fight the *wind*, not the weight of the drone. This is how professional racers achieve near-instant response.

### **5.2.2 Stability Analysis: The Bode Plot Intuition**
In a research setting, we analyze the **Open-Loop Gain**.
*   **Phase Margin:** The "Delay Budget." If your Wi-Fi/Processing adds too much delay, the phase of your correction shifts $180^{\circ}$, turning your "Negative Feedback" into "Positive Feedback." Instead of fixing the error, the drone will accelerate *into* the crash.
*   **Gain Margin:** How much you can "crank up" the $P$ term before the system becomes unstable.

### **5.2.3 Just-In-Time Math: The Delay Budget (Phase Margin)**
**"Don't wait until it's too late."**

*   **The Concept:** Imagine steering a boat. You turn the wheel, but the boat turns 2 seconds later. If you keep turning the wheel back and forth quickly, the boat will eventually start turning *left* when you are steering *right*. This is **Instability**.
*   **Phase Margin:** This is your "Safety Buffer." It measures how much *extra* delay your system can handle before it goes crazy.
    *   **Rule of Thumb:** You want $> 45^{\circ}$ of margin.
    *   **The Fix:** If your margin is low, you must **slow down** your controller (reduce Gains) or **speed up** your sensor (increase Hertz).

**AI Prompt:** "Explain Bode Plots and Phase Margin in control theory. Why does a phase lag of 180 degrees turn negative feedback into positive feedback?"

---

## **5.3 Tuning: The Ziegler-Nichols Method**
[... existing Z-N content ...]

---

## **5.4 Socratic Discussion: The Control Gap**
1.  **Question:** Why can't we just set $K_d$ to a massive number to stop all oscillation?
    *   **Answer:** $K_d$ amplifies noise. Because $D = \frac{\Delta e}{\Delta t}$, a tiny $1\text{ms}$ jitter in the sensor becomes a massive spike in motor output, causing the motors to get hot and the frame to vibrate.
2.  **Question:** If the drone is upside down, does the $K_i$ term help or hurt?
    *   **Answer:** It hurts. The $K_i$ term will try to pull the drone "Up" (relative to the drone), which is "Down" relative to the world, accelerating the crash. This is why we need **Geometric Control** (Lecture 5.9).

### **Lab Procedure**
1.  **Modify `pid.py`:** Add `max_i` parameter to clamp the Integral.
2.  **Modify `pid.py`:** Change the D-term logic to use `-(input - prev_input) / dt`.

### **5.2.1 Sub-Lab: The Anti-Windup Guard**
**"Taming the Infinite."**

If your drone's motors are blocked (e.g., by a blade of grass), your PID "I" term will keep growing until it hits the ceiling. This is **Integral Windup**.

1.  **Test (Props OFF):** Hold the drone tilted at 20 degrees. Your code will try to level it.
2.  **Observe:** Watch the `I_term` value in your telemetry. It will climb higher and higher.
3.  **The Fix:** Implement **Clamping** in your `pid.py`:
    ```python
    self.i_term += error * dt
    # The PhD Guard
    if self.i_term > MAX_I: self.i_term = MAX_I
    if self.i_term < -MAX_I: self.i_term = -MAX_I
    ```
4.  **Verification:** Repeat the test. The `I_term` should now hit a "ceiling" and stop, making the drone safe to release.

---

### **5.3 Tuning: The Ziegler-Nichols Method**

### **Objective**
Find the magic numbers ($K_p, K_i, K_d$).

### **5.3.1 Sub-Lab: The Oscillation Hunt**
**"Finding the Edge of Stability."**

1.  **Safety:** Tether the drone. Set $I=0$ and $D=0$.
2.  **Procedure:**
    *   Start with $P=1.0$. Give a small "tap" to the drone.
    *   Increase $P$ in steps of 0.5.
    *   **The Goal:** Find the value of $P$ where the drone stops returning to center and instead starts **oscillating continuously** (but not exploding). This value is $K_u$ (Ultimate Gain).
3.  **Measurement:** Use your Dashboard from Module 2 to measure the time between two peaks of the oscillation. This is $T_u$ (Ultimate Period).
4.  **Math:** Plug these into the Z-N table provided in the theory.

### **Procedure (The Safety Rig)**
1.  **Setup:** Tie the drone down so it can only lift 5cm (use heavy fishing line).
2.  **Step 1 (Find P):** Set I=0, D=0. Increase P until the drone oscillates consistently (Ultimate Gain $K_u$).
3.  **Step 2 (Math):** Use the period of oscillation ($T_u$) to calculate safe P, I, D values using the Z-N table.
    *   $K_p = 0.6 K_u$
    *   $K_i = 2 K_p / T_u$
    *   $K_d = K_p T_u / 8$
4.  **Step 3 (Fine Tune):** Reduce P/D if motors get hot.

---

## **5.4 Advanced SysID: The Bifilar Pendulum**

### **Objective**
Measure the Moment of Inertia ($I_{xx}, I_{yy}, I_{zz}$) physically to validate CAD models.

### **Theory**
$I = \frac{mgT^2d^2}{16\pi^2L}$
Where $T$ is oscillation period, $d$ is string spacing, $L$ is string length.

### **Lab Procedure**
1.  **Rigging:** Hang the drone from two parallel fishing lines.
2.  **Test:** Twist it 10 degrees and let go. Record 10 oscillations.
3.  **Calculate:** Compute $I_{zz}$. Repeat for Pitch ($I_{yy}$) and Roll ($I_{xx}$).
4.  **Update:** Put these exact values into your simulation (`simulation/quadrotor.urdf`).

---

## **5.5 Physics Modeling: The Static Thrust Stand**

### **Objective**
From Amps to Newtons.

### **Lab Procedure**
1.  **Rig:** Mount the drone upside down on a digital kitchen scale.
2.  **Tare:** Zero the scale.
3.  **Run:** Step motors 0% -> 100%. Record Scale Reading (Grams) and Current (Amps).
4.  **Fit:** $Thrust = k_t \cdot \omega^2$.
5.  **Efficiency:** Calculate $g/W$ (Grams per Watt). This predicts your flight time.

## **0.5.4 The "Deep-Dive" Breakout: Scratch PID**
> **Professor's Note:** While libraries exist to handle PID, for this course, we are going "from scratch."
> **Why?** You need to see exactly how a floating-point error accumulates in the Integral term. You need to feel the "Derivative Kick" when a sensor spikes.
> **The Goal:** You will write the `update(error, dt)` function yourself. No black boxes. This is where you learn the "Discrete Calculus" that runs the world.

---

## **The "Statue" Test**

**The Hover Test.**

1.  **Command:** Set Target Altitude = 0.5m.
2.  **Disturbance:** Poke the drone (gently!) with a stick.
3.  **Success Criteria:**
    *   It resists the poke (Stiffness).
    *   It returns to 0.5m without oscillating more than twice (Damping).
    *   It stays within +/- 5cm of the target.

**Submission:** A plot of Altitude vs Time during a 10-second hover.

---
## **Theoretical Foundations**

### Lecture 5: Control Theory & Stability

#### **1. Non-linear Actuator Saturation & Anti-Windup**
PID controllers assume a linear relationship between input and output.
*   **The Nonlinear Limit:** In reality, our motors saturate at $100\text{%}$. If the PID requests $150\text{%}$, the Integral term continues to accumulate "phantom" error ($I = \int e \cdot dt$).
*   **The Guard:** We implement **Clamping Anti-Windup**. We freeze the integral sum if the output is saturated AND the error has the same sign as the integral. This prevents the "Integral Windup" that causes drones to overshoot violently after a period of stuck motors.

#### **2. Optimal Control (The Linear Quadratic Regulator)**
LQR provides the "best possible" gains by minimizing a quadratic cost function $J$.
*   **The Math:** $J = \int_{0}^{\infty} (x^T Q x + u^T R u) dt$.
*   **Weights:** $Q$ (State Penalty) and $R$ (Actuation Penalty) are diagonal matrices. Increasing $Q_{ii}$ makes the drone "stiffer" on that state; increasing $R_{ii}$ makes the drone "lazy" to save battery.
*   **Optimality:** Unlike Z-N tuning, LQR considers the coupling between states (e.g., how Pitch affects Velocity).

### Lecture 5.8: Receding Horizon Control (MPC)
Model Predictive Control (MPC) is a discrete-time optimization problem solved at every tick.
*   **The Finite Horizon:** We predict the state $\hat{x}_{k+1...k+N}$ over a horizon $N$ (e.g., 20 steps).
*   **The Constraint:** We can explicitly include physical constraints: $Tilt < 30^{\circ}$ and $Voltage > 10V$. If a trajectory would violate these, the optimizer finds the "closest safe" path.

### **5.8.1 Just-In-Time Math: The Matrix Trick**
**"Solving the Future in One Shot"**

In the lab `lab_3_4_mpc_lite.py`, you will see code that stacks matrices. Why?
*   **The Hard Way (Recursion):** 
    Step 1: $x_1 = A x_0 + B u_0$
    Step 2: $x_2 = A x_1 + B u_1 = A(A x_0 + B u_0) + B u_1 = A^2 x_0 + AB u_0 + B u_1$
    Step 10: $x_{10} = A^{10} x_0 + \dots$ (This is slow and messy).
*   **The "Batch" Trick:** We stack the equations into one giant matrix operation: $\mathbf{X}_{future} = \mathbf{T} x_0 + \mathbf{S} \mathbf{U}_{future}$.
*   **The Result:** We can solve for the *entire* future path of motor commands ($\mathbf{U}_{future}$) using a standard Least Squares solver (`np.linalg.solve`). This is the "Chess Player" thinking 10 moves ahead instantly.

**AI Prompt:** "Explain how the 'Batch Matrices' (S_bar and T_bar) allow us to solve for all future motor commands in a single step using Least Squares."

### Lecture 5.9: Geometric Control on $SO(3)$
Traditional attitude control uses Euler angles, which fail at $90^{\circ}$ pitch.
*   **The Manifold:** We define error as the distance between the current rotation matrix $R$ and target $R_d$ on the $SO(3)$ manifold.
*   **The Law:** $e_R = \frac{1}{2}(R_d^T R - R^T R_d)^\vee$. This calculation is global—it works upside down, sideways, or during a flip, allowing for "Aggressive Recovery" maneuvers impossible with PID.

**Next Step:** [Phase IV: Module 6 ROS 2 Migration](../../Phase_4_The_Architect/Module_06_ROS2_Migration/Module_06_Lecture.md)

---

## **Check: Troubleshooting the Physics**

| Symptom | Probable Cause | Physical Reality |
| :--- | :--- | :--- |
| **"The Ceiling Magnet"** | Integral Windup | The drone was on the ground with $Throttle=0$, but the error was high. The $I$ term built up to $100\text{%}$. When you bumped the throttle, the $I$ term "unloaded" all at once. |
| **"The Death Spiral"** | Latency / Lag | Your loop frequency dropped below $20\text{Hz}$. Your correction is arriving after the drone has already bounced back, adding energy to the oscillation instead of removing it. |
| **"The Parkinson's Wobble"** | $D$ gain too high | You are taking the derivative of "Dirty" (noisy) sensor data. Tiny jitters are being amplified into massive motor pulses. |

---

## **Professor's Brief: Speaker Notes**

### **The Narrative Hook**
"Stability is a delicate dance between the past, present, and future. PID is reactive (past/present). LQR is optimal (instantaneous). MPC is predictive (future). Today, we move from being a 'slave to the error' to being a 'master of the physics.' We are building a brain that understands that its actions today have consequences one second into the future."

### **Deep Research Context: The Riccati Equation**
In research, LQR is solved using the **Algebraic Riccati Equation (ARE)**: $A^T P + PA - PBR^{-1}B^T P + Q = 0$. Solving this by hand is a rite of passage for robotics PhDs. Mention that for a micro-drone, we assume the system is "Linearized" around the hover point. If you fly very fast (aggressive flight), the $A$ and $B$ matrices change, and your LQR gains will "degrade," which is why we later use LNNs (Module 10) to adapt.

## **Mastery Checklist**
Before moving to the next module, you should be able to:
- [ ] Explain the physical intuition of P, I, and D in a Mass-Spring-Damper model.
- [ ] Implement a "Bumpless" Anti-Windup guard for motor saturation.
- [ ] Define the Cost Function matrices ($Q$ and $R$) for an LQR controller.
- [ ] Describe why SO(3) Geometric Control prevents the "Gimbal Lock" crash.

---

## **Further Reading & Bibliography**

### **Classical Control**
*   **Astrom, K. J., & Murray, R. M. (2010).** *Feedback Systems: An Introduction for Scientists and Engineers.* Princeton University Press. (The standard textbook).

### **Nonlinear & Optimal Control**
*   **Lee, T., Leok, M., & McClamroch, N. H. (2010).** *"Control of complex maneuvers on SO(3)."* IEEE Conference on Decision and Control (CDC). (Groundbreaking quadrotor paper).
*   **Borrelli, F., et al. (2017).** *Predictive Control for Linear and Hybrid Systems.* Cambridge University Press. (The bible of MPC).

---

[Previous Module](../Module_04_Signal_Processing/Module_04_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../../Phase_4_The_Architect/Module_06_ROS2_Migration/Module_06_Lecture.md)