# Environment Engineering: Preparing your Flight Space
**"The World is a Sensor Input."**

A robot's performance is 50% code and 50% environment. If you ignore the room, the most advanced VIO algorithm in the world will fail.

---

## **1. Visual Texture (For VIO & Optical Flow)**
Algorithms like KLT (Module 13) and PMW3901 (Optical Flow) work by tracking "Pixels of Interest."
*   **The Enemy:** Flat white walls, glass tables, and solid-colored carpets. To a camera, these look like "Nothing." The drone will think it is stationary when it is actually moving (Drift).
*   **The Fix:** 
    *   **The "Texture" Rug:** Use a rug with a complex, non-repeating pattern. 
    *   **AprilTags:** Tape high-contrast markers (like QR codes) to the walls at varying heights.

---

## **2. Lighting (The Photon Budget)**
*   **Motion Blur:** In a dark room, the camera increases its exposure time. If the drone moves, the image becomes "blurry." Blurred features cannot be tracked.
*   **The Fix:** Fly in a brightly lit room. If you want to fly in the dark, you must add an **IR Floodlight** to the drone (The Arducam can see IR).

---

## **3. Magnetic Interference (The Compass)**
*   **The Enemy:** Steel rebar in concrete floors, large speakers, and the drone's own power wires.
*   **The Test:** Open your Telemetry Dashboard. Place the drone on the floor. Rotate it. Does the North heading skip or "stick" at certain angles? 
*   **The Fix:** Calibrate the compass (Module 7.5) *in the middle of the room*, far away from large metal objects.

---

## **4. Aerodynamics (The Ground Effect)**
*   **The Enemy:** Flying too close to the floor (< 10cm). The "Prop Wash" (air pushed down) bounces off the floor and creates a chaotic "cushion" of air.
*   **The Symptom:** The drone wobbles and feels "slippery" during takeoff.
*   **The Fix:** Program your takeoff sequence to "Punch" to 0.5m immediately to get out of the "Dirty Air" zone.
--- [Return to Course Map](../../../COURSE_MAP.md)