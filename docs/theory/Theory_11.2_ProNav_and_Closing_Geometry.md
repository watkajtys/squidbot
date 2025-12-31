# Theory Deep Dive 11.2: Pro-Nav & Collision Geometry
**"Meeting the Target, not Chasing the Tail."**

If you fly directly at a moving target (Pure Pursuit), you will always end up behind it. To intercept, you must "lead" the target.

---

## **1. The Collision Triangle**
Interception happens when your velocity vector and the target's velocity vector meet at a future point in space.
*   **The Rule:** If the **Line-of-Sight (LOS) angle** between you and the target stays constant, you are on a collision course.
*   **Line-of-Sight Rate ($\dot{\lambda}$):** The speed at which the angle to the target is changing.

---

## **2. Proportional Navigation (Pro-Nav)**
This is the industry standard for interceptors. The command to the drone is:
$$a_n = N \cdot V_c \cdot \dot{\lambda}$$
*   **$a_n$:** Acceleration command (how hard to turn).
*   **$N$:** Navigation Constant (usually 3 to 5).
*   **$V_c$:** Closing Velocity (how fast the distance is shrinking).
*   **$\dot{\lambda}$:** LOS Rate.

**Why it's "GNC" level:** 
Pro-Nav doesn't need to know the target's exact position; it only needs to know how the *angle* is changing. This makes it incredibly robust to sensor noise.

---

## **3. The $t_{go}$ (Time-to-Go)**
$$t_{go} \approx \frac{Range}{V_c}$$
An interceptor uses $t_{go}$ to plan its energy. If $t_{go}$ is small, you use maximum thrust. If it's large, you save energy for the final maneuver.
