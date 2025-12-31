# Theory Deep Dive 10.6: Sim-to-Real Transfer
**"Crossing the Reality Gap."**

The biggest lie in robotics is "It works in simulation."

---

## **1. The Reality Gap**
Physics engines (PyBullet) are approximations. They don't model every air molecule or every loose screw. 

---

## **2. Domain Randomization (DR)**
Instead of making the simulation "Perfect," we make it **Varied**. 
*   During every training episode, we randomly change:
    *   `mass` (+/- 10%)
    *   `thrust_constant` (+/- 5%)
    *   `latency` (0ms to 50ms)
*   **The Result:** The AI learns a policy that works across a "range" of physics. When it hits the real world, the real physics just feels like another "random" episode it has already mastered.

---

## **3. System Identification (Review)**
Remember **Theory 0.4**? The better your SysID numbers are, the "center" of your randomization will be closer to the truth, and the AI will transfer faster.
