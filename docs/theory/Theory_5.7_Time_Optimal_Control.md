# Theory Deep Dive 5.7: Time-Optimal Control
**"The Physics of Aggression."**

In a pursuit, being "smooth" is being "slow." To win, you need to use the **Full Envelope** of the vehicle.

---

## **1. Bang-Bang Control**
Imagine you want to move 10 meters and stop.
*   **PID:** Accelerates slowly, slows down slowly. (Slow).
*   **Bang-Bang:** 100% Throttle until 5 meters, then 100% Brakes until 10 meters. (Fastest possible).

---

## **2. Sliding Mode Control (SMC)**
SMC is a high-performance control method that "slides" the drone along a specific mathematical surface.
*   It is incredibly robust to wind and drag.
*   **The Downside:** It causes "Chatter" (high-frequency motor twitching). 
*   **The Elite Fix:** We use a "Boundary Layer" to smooth the switch, giving us the speed of Bang-Bang with the stability of PID.

---

## **3. Pontryagin’s Minimum Principle (PMP)**
This is the PhD level of GNC. PMP provides the necessary conditions for "Optimal" control.
*   It defines a **Hamiltonian** ($\mathcal{H}$) for the system.
*   By minimizing $\mathcal{H}$, you can prove that your flight path is the mathematical **minimum time** or **minimum energy** path possible.

**Study Task:** Look up "Minimum Snap vs. Minimum Time Trajectories." One is for cinematographers; the other is for interceptors.
