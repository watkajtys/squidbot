# Theory Deep Dive 0.7: The Full State-Space Model
**"The 12 Dimensions of Flight."**

To control the drone, you must track 12 variables simultaneously. This is the "State Vector" ($\mathbf{x}$).

---

## **1. The State Vector ($\mathbf{x}$)**
$\mathbf{x} = [p_x, p_y, p_z, v_x, v_y, v_z, \phi, \theta, \psi, \omega_x, \omega_y, \omega_z]^T$
*   **Positions:** $p$ (World Frame)
*   **Velocities:** $v$ (World Frame)
*   **Attitude:** $\phi, \theta, \psi$ (Roll, Pitch, Yaw - World Frame)
*   **Body Rates:** $\omega$ (Angular Velocity - **Body Frame**)

---

## **2. The Nonlinear Dynamics ($\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$)**
This is the heart of your code. How does the state change over time given the motor inputs ($\mathbf{u}$)?

### **Translational Acceleration:**
$$\dot{\mathbf{v}} = \begin{bmatrix} 0 \\ 0 \\ -g \end{bmatrix} + \frac{1}{m} R(\mathbf{q}) \begin{bmatrix} 0 \\ 0 \\ \sum F_{motors} \end{bmatrix}$$
*   *Translation:* Gravity pulls down. The motors pull "Up" in the Body Frame, which we rotate ($R$) into the World Frame.

### **Rotational Acceleration:**
$$\dot{\mathbf{\omega}} = J^{-1} (\mathbf{\tau}_{motors} - \mathbf{\omega} \times (J \mathbf{\omega}))$$
*   *Translation:* The torque from the motors ($M_1-M_4$) creates rotation, but we must subtract the "Gyroscopic Effect" ($\mathbf{\omega} \times J\mathbf{\omega}$) which tries to pull the drone off-axis during high-speed spins.

---

## **3. The Implementation Trap**
Notice that Velocities are in the **World Frame** but Body Rates are in the **Body Frame**.
*   **Common Error:** Adding $\omega$ directly to $\dot{\phi}, \dot{\theta}, \dot{\psi}$. 
*   **The Fix:** You must use the **Angular Velocity Transformation Matrix** to convert Body Rates to Euler Rates.

**Stanford Research:** See the "Mellinger Quadrotor Model" for the full derivation used in high-performance trajectory tracking.
