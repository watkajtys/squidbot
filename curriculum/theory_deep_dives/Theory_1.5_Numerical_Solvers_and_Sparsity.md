# Theory Deep Dive 1.5: Numerical Solvers & Sparsity
**"The Engine of Autonomy."**

In robotics, almost every problem eventually becomes: $\mathbf{A}\mathbf{x} = \mathbf{b}$.
*   **A:** What we know about the world (Physics/Constraints).
*   **b:** Our sensor measurements.
*   **x:** The "Truth" we are trying to find.

---

## **1. Don't use `inv()`**
In a big lab, if you write `x = inv(A) @ b`, you will be corrected.
*   **Why:** Matrix inversion is slow and numerically unstable.
*   **The Pro Way:** Use **Decompositions** (LU, Cholesky, or QR). In Python: `x = np.linalg.solve(A, b)`. It is faster and more accurate.

---

## **2. Sparsity**
In **Factor Graphs (Module 8.5)**, your matrix `A` might be 10,000 x 10,000. 
*   However, most of the entries are **Zero** (because Drone Position at Minute 1 doesn't affect the Lidar at Minute 10).
*   **The Secret:** Big labs use **Sparse Matrix Solvers**. They skip the zeros and only do math on the "live" data. This is how a drone can run SLAM in real-time on a small CPU like the Pi Zero.

---

## **3. Condition Numbers ($\kappa$)**
If your matrix `A` is "Ill-conditioned," a tiny bit of sensor noise will cause a massive error in your position.
*   **Intuition:** Imagine trying to find the intersection of two lines that are almost parallel. A tiny wiggle in one line moves the intersection point by miles.
*   **The Fix:** This is why we **Normalize** our data and use **Double Precision** for our EKFs.
