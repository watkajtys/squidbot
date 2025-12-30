# Module 5: Control Theory
**"Power without control is just a crash."**

You have a drone that can see (Sensors) and act (Motors). But if you connect them directly (`Motor = 1000 * Distance`), it will oscillate and crash. You need a **Controller**.

---

## **5.1 The Feedback Loop (PID)**

### **Objective**
Keep the drone at a fixed altitude (e.g., 0.5m) despite gravity and battery voltage changes.

### **Theory**
The **PID Controller** calculates error ($e = Target - Current$) and outputs a correction ($u$).
*   **Proportional ($K_p$):** "I am low, go up." (Reacts to Present).
*   **Integral ($K_i$):** "I have been low for a long time, add more power." (Reacts to Past / Gravity).
*   **Derivative ($K_d$):** "I am going up too fast, slow down." (Reacts to Future / Damping).

$$ u(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt} $$

### **5.1.1 Sub-Lab: The "Drunk" Pilot**
**"Latency kills."**

1.  **Hover:** Establish a stable hover.
2.  **Inject Lag:** Modify your code to store sensor data in a buffer and release it **100ms later**.
3.  **Observe:** The drone will oscillate violently and crash.
4.  **Why:** You destroyed the Phase Margin. The correction is applied *after* the error has already reversed direction, amplifying the error instead of fixing it.

### **Lab Procedure**
1.  **The Class:** Create `src/control/pid.py`.
2.  **The State:** Store `prev_error` and `integral_sum`.
3.  **The Update:** Implement `update(current_value, dt)`.
    *   `error = setpoint - current_value`
    *   `P = Kp * error`
    *   `I += Ki * error * dt`
    *   `D = Kd * (error - prev_error) / dt`
    *   `Output = P + I + D`

### **Deliverable**
*   A Python script `sim_1d.py` that simulates a "Mass on a Spring" and uses your PID class to stop it from bouncing.

---

## **5.2 Implementation Details (Real World)**

### **Objective**
Fix the bugs that math books ignore.

### **Theory**
1.  **Integral Windup:** If the drone is stuck on the ground, the `I` term will grow to infinity. When it finally takes off, it will rocket into the ceiling.
    *   *Fix:* Clamp the `I` term (`if I > max: I = max`).
2.  **Derivative Kick:** If you change the Target instantly (0.5m -> 1.0m), the `error` jumps instantly. The Derivative ($de/dt$) becomes Infinite. The motors spasm.
    *   *Fix:* Calculate D on the *Measurement* ($d(Input)/dt$), not the Error.

### **Lab Procedure**
1.  **Modify `pid.py`:** Add `max_i` parameter to clamp the Integral.
2.  **Modify `pid.py`:** Change the D-term logic to use `-(input - prev_input) / dt`.

---

## **5.3 Tuning: The Ziegler-Nichols Method**

### **Objective**
Find the magic numbers ($K_p, K_i, K_d$).

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

---

## **Check: The Statue**
**The Hover Test.**

1.  **Command:** Set Target Altitude = 0.5m.
2.  **Disturbance:** Poke the drone (gently!) with a stick.
3.  **Success Criteria:**
    *   It resists the poke (Stiffness).
    *   It returns to 0.5m without oscillating more than twice (Damping).
    *   It stays within +/- 5cm of the target.

**Submission:** A plot of Altitude vs Time during a 10-second hover.
