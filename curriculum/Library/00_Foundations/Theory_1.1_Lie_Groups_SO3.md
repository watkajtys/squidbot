# Theory Deep Dive 1.1: Lie Groups & Lie Algebra SO(3)
**"Beyond the Singularity."**

In robotics research, we don't just "rotate" vectors; we operate on a **Manifold**. The set of all valid 3D rotations forms a group called the **Special Orthogonal Group**, or $SO(3)$.

---

## **1. Why Lie Groups?**
Standard vector addition ($A + B$) doesn't work for rotations. If you add two 90-degree rotations, you don't necessarily get 180 degrees in a straight line.
*   **The Problem:** $SO(3)$ is a curved surface (a sphere-like manifold). 
*   **The Lie Solution:** We use **Lie Algebra** ($\mathfrak{so}(3)$) to "flatten" the curve locally. We do our math in the flat space (the Tangent Space) and then "project" it back onto the curve.

---

## **2. The Exponential Map ($\exp$)**
This is the bridge between a **Rotation Vector** ($\omega \in \mathbb{R}^3$) and a **Rotation Matrix** ($R \in SO(3)$).
*   **Rotation Vector ($\omega$):** A 3D vector where the direction is the axis and the length is the angle.
*   **Exponential Map:** Converts $\omega$ into a 3D matrix.
$$ R = \exp([\omega]_\times) $$
Where $[\omega]_\times$ is the "Skew-Symmetric" matrix of $\omega$.

---

## **3. The "Small Angle" Approximation**
In our flight controller, we often deal with tiny changes in angle ($\Delta \theta$).
*   For small $\theta$, $\sin(\theta) \approx \theta$ and $\cos(\theta) \approx 1$.
*   This allows us to treat rotation updates as simple additions in the Lie Algebra before projecting them back. This is the secret to high-speed EKF performance.

---

## **4. Why This Matters for the Squid**
When your drone is performing a high-speed flip, Euler angles will "Flip" their signs (e.g., jumping from +180 to -180). This creates a "Spike" in your PID controller that can snap a motor shaft. By using $SO(3)$ or Quaternions, the math remains "Smooth" across all 360 degrees.

**Mastery Check:**
- [ ] Can you define a Skew-Symmetric matrix given a vector $[x, y, z]$?
- [ ] Why is the determinant of an $SO(3)$ matrix always $+1$?
- [ ] Explain why adding two rotation matrices ($R_1 + R_2$) does not result in a valid rotation.

---
*Reference: Murray, Li, and Sastry (1994). A Mathematical Introduction to Robotic Manipulation.*