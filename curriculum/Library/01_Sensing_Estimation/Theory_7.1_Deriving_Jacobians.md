# Theory Deep Dive 7.1: Deriving Jacobians
**"Linearizing the Chaos."**

The EKF requires the **Jacobian Matrix** ($F$). This is the derivative of your 12-state physics model.

---

## **1. What is a Jacobian?**
If $f(x)$ is your physics model, the Jacobian $F$ is:
$$F = \frac{\partial f}{\partial x}$$
It tells the EKF: "If the Roll changes by 0.01 rad, how much does the X-Acceleration change?"

---

## **2. The Matrix Structure ($12 \times 12$)**
Don't be intimidated. Most of the entries are 0 or 1.
*   The top-right $3 \times 3$ block is the Identity Matrix ($I$), because $\frac{\partial position}{\partial velocity} = 1$.
*   The complicated part is the relationship between **Attitude** and **Acceleration**.

### **Example: The "Tilt" Derivative**
If the drone is tilted by $\theta$ (Pitch), the X-acceleration is $g \cdot \sin(\theta)$.
The Jacobian entry for $\frac{\partial \dot{v}_x}{\partial \theta}$ is $g \cdot \cos(\theta)$.

---

## **3. Automatic Differentiation**
In a big lab, we don't always do this by hand anymore.
*   **The Pro Way:** Use **SymPy** (Python) to define your $f(x)$ symbolically and let the computer calculate the Jacobian matrix for you.
*   **The Code:**
    ```python
    import sympy as sp
    # Define symbols
    phi, theta, psi = sp.symbols('phi theta psi')
    # Define rotation matrix
    R = sp.Matrix([...])
    # Compute Jacobian
    F = R.jacobian([phi, theta, psi])
    ```

## **3. Observability: Can we see the Truth?**



Calculating the Jacobian is only half the battle. You must also prove that your sensors provide enough information to estimate your state. This is **Observability Analysis**.



### **The Observability Matrix ($O$)**

For a system with state transition $F$ and measurement Jacobian $H$, the observability matrix is:

$ O = \begin{bmatrix} H \\ HF \\ HF^2 \\ \vdots \\ HF^{n-1} \end{bmatrix} $



### **The Rank Test**

*   **Full Rank:** If the rank of $O$ equals the number of states, the system is **Observable**. You can find your position.

*   **Rank Deficient:** If the rank is lower, you have "Hidden States."

    *   *Example:* If you have an IMU but no GPS or Lidar, your absolute position ($X, Y, Z$) is **Unobservable**. You will drift forever because the sensors only measure "changes," not "absolute truth."



**Student Task:**

 Use the `sympy_jacobian_gen.py` script (to be written in Module 7) to generate your $12 \times 12$ matrix. Do not try to hard-code 144 entries by hand.
--- [Return to Course Map](../../../COURSE_MAP.md)