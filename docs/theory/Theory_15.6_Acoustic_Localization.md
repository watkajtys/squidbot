# Theory Deep Dive 15.6: Acoustic Localization (Experimental)
**"Hearing the Hunt."**

In a dogfight, the loudest thing in the room is the other drone. 

---

## **1. Propeller Harmonics**
Every motor/propeller combo has a specific "Frequency Signature" (usually between 200Hz and 500Hz).
*   By using a **Microphone** and an **FFT**, your drone can "hear" where the enemy is, even if they are behind a wall where the camera can't see.

---

## **2. TDoA (Time Difference of Arrival)**
If you use multiple microphones on your frame (or across your swarm), you can triangulate the source of the noise.
*   **The Math:** By measuring the 1ms delay between the noise hitting Microphone A and Microphone B, you can calculate the angle to the target.

---

## **3. Stealth Mode**
If your drone can hear but not be heard, you win.
*   **Active Noise Cancellation:** Using the motors to create "Anti-Phase" noise to quiet the drone. (Extreme PhD level).
