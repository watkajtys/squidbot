# Theory Deep Dive 4.1: Least Squares System Identification
[Return to Module 4](../../Phases/Phase_3_The_Engineer/Module_04_Signal_Processing/Module_04_Lecture.md) | [Return to Course Map](../../../COURSE_MAP.md)

**"Curve Fitting the Laws of Physics."**

How do we find the exact mass, motor thrust constant, and drag coefficients of our drone using only flight logs? We use the **Ordinary Least Squares (OLS)** method.

---

## **1. The Linear Model**
Most physics can be written as a linear relationship: $y = Ax$.
*   $y$: Our measurements (e.g., Acceleration from the IMU).
*   $A$: Our "Inputs" (e.g., PWM values sent to the motors).
*   $x$: The "Hidden Parameters" we want to find (e.g., the Thrust Constant $K_t$).

---

## **2. The Residual (The Error)**
Since our sensors are noisy, $y$ will never perfectly match $Ax$. There is always an error $e$:
$$ e = y - Ax $$
Our goal is to find the $x$ that makes the **Sum of Squared Errors** as small as possible.

---

## **3. The Normal Equation**
To minimize the error, we take the derivative of the squared error and set it to zero. This gives us the "Golden Equation" of System ID:
$$ \hat{x} = (A^\top A)^{-1} A^\top y $$

**In Python (Numpy):**
```python
import numpy as np
# A = matrix of motor inputs, y = vector of observed accelerations
x_hat = np.linalg.inv(A.T @ A) @ A.T @ y
```

---

## **4. The "Chirp" Test**
To get a good "A" matrix, you can't just hover. You need "Persistent Excitation."
*   **The Chirp:** We send a signal to the motors that slowly increases in frequency. This "shakes" the drone across its entire physical spectrum, ensuring we capture the effects of inertia and vibration.

---

## **5. Why This Matters for the Squid**
In Module 9 (Trajectory Optimization), the computer will try to plan a path that is "Physically Possible." If your thrust constant is off by 20%, the computer will plan a turn that the drone physically cannot make. Least Squares ensures the "Brain" (Software) knows the limits of the "Body" (Hardware).

**Mastery Check:**
- [ ] Why do we square the error instead of just using the raw error?
- [ ] What happens to the Normal Equation if $A^\top A$ is not invertible? (Hint: Singular Value Decomposition).
- [ ] Define "Persistent Excitation" in the context of a flight test.

---
*Reference: Ljung, L. (1999). System Identification: Theory for the User.
