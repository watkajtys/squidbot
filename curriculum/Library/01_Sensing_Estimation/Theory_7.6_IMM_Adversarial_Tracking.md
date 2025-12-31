# Theory Deep Dive 7.6: IMM Adversarial Tracking
**"Tracking a Liar."**

If your target is a "Dogfighter," it will not fly in a straight line. If you use a standard EKF, your "Prediction" will always be behind the target's "Reality."

---

## **1. The Interacting Multiple Model (IMM)**
Instead of one EKF, we run a "Bank" of filters:
*   **Filter 1 (CV):** Constant Velocity. (Target is cruising).
*   **Filter 2 (CT):** Constant Turn. (Target is circling).
*   **Filter 3 (CA):** Constant Acceleration. (Target is "punching" it).

---

## **2. The Mixing Step**
At every loop, the IMM calculates the **Likelihood** of each filter. 
*   If the target suddenly pulls a 5G turn, the "Constant Turn" filter's error will be small, and the "Constant Velocity" error will be huge.
*   The IMM shifts the weight to the CT filter instantly.

---

## **3. The Dogfight Edge**
In an intercept, the IMM allows you to "anticipate" the turn. While the target thinks they are shaking you, your IMM has already switched its physics model to match their maneuver.

**Research Task:** Look up "Markov Jump Linear Systems." This is the formal math behind switching between different flight models.
--- [Return to Course Map](../../../COURSE_MAP.md)