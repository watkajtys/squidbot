# The Squid Drone "Zero-to-PhD" Cheat Sheet
**Quick Reference for Coding & Debugging**

---

## 1. Rotation & Geometry
**"Degrees are for humans. Radians are for robots."**

*   **Degrees to Radians:** $rad = deg \times \frac{\pi}{180}$
*   **Radians to Degrees:** $deg = rad \times \frac{180}{\pi}$
*   **RPM to Radians/Sec:** $\omega = RPM \times \frac{2\pi}{60}$
*   **Small Angle Approximation:** $\sin(\theta) \approx \theta$, $\cos(\theta) \approx 1$ (Valid for $\theta < 10^°$). 

### **The Quaternion Checklist**
1.  **Normalization:** Always ensure $||q|| = 1.0$. If the magnitude drifts, your rotation is no longer "pure."
2.  **The Sign Flip:** $q$ and $-q$ represent the same rotation. Use `if dot(q1, q2) < 0: q2 = -q2` before interpolating.
3.  **Order:** Our standard is **Hamiltonian** $[w, x, y, z]$. *Beware: Some libraries use $[x, y, z, w]$.*

---

## 2. Physics & Dynamics
*   **Gravity ($g$):** $9.80665 \, m/s^2$
*   **Force:** $F = m \cdot a$
*   **Torque:** $\tau = r \times F$
*   **Thrust-to-Weight Ratio (TWR):** $\frac{\text{Max Thrust}}{\text{Total Mass} \times g}$
    *   *Target:* > 2.0 for stable flight. > 4.0 for racing/aggressive AI.

---

## 3. Signal Processing
*   **Sampling Frequency ($f_s$):** $1 / \Delta t$
*   **Nyquist Limit:** $f_{max} = f_s / 2$
*   **Discrete Low-Pass Filter (Alpha):**
    $$ y[k] = \alpha \cdot x[k] + (1 - \alpha) \cdot y[k-1] $$ 
    Where $\alpha = \frac{\Delta t}{RC + \Delta t}$.

---

## 4. Control Theory (PID)
*   **Standard Form:** $u(t) = K_p e(t) + K_i \int e(t)dt + K_d \frac{de}{dt}$
*   **Anti-Windup (PhD Tip):** Always clamp your Integral term to prevent "flyaways" after a collision.
    `if I > limit: I = limit`
*   **Derivative Filtering:** Always apply a Low-Pass Filter to your $D$ term to avoid motor jitter from sensor noise.

---

## 5. State Estimation (EKF)
*   **The Mahalanobis Distance:** $d^2 = \Delta z^T S^{-1} \Delta z$
    *   *Interpretation:* If $d^2 > 9$, there is a 99% chance the sensor is lying (Outlier).

---

## 6. Standard Ports & Pins (Squid Platform)
*   **I2C Bus:** `/dev/i2c-1`
*   **UART:** `/dev/ttyAMA0` (Serial0)
*   **Default ToF Address:** `0x29`
*   **MSP Port:** Usually UART 1 on Betaflight.

---

## 7. Debugging Symptoms
| Symptom | Probable Cause | Fix |
| :--- | :--- | :--- |
| **Warm Motors** | $D$ gain too high or Noisy Gyro | Lower $D$ or add LPF |
| **Slow Oscillations** | $P$ gain too low | Increase $P$ |
| **Fast Oscillations** | $P$ gain too high | Decrease $P$ |
| **Drift on Takeoff** | Accelerometer Bias | Level drone & calibrate |
| **"Toilet Bowl"** | Magnetometer Interference | Move Mag away from power wires |--- [Return to Course Map](../../../COURSE_MAP.md)