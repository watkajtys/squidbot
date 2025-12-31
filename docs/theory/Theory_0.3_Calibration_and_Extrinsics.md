# Theory Deep Dive 0.3: Calibration & Extrinsics
**"The 1-Centimeter Error."**

In your code, you will calculate the "Drone Position." But your Lidar is not at the center of the drone; it is probably 2cm below it. Your camera is 3cm in front.

If you don't account for these **Extrinsics**, your math will have a "Bias."

---

## **1. The Transform Tree (TF)**
Imagine the drone is a family tree:
*   `world` (Grandparent)
    *   `base_link` (Parent: The center of the drone)
        *   `camera_link` (Child: Offset from center)
        *   `lidar_link` (Child: Offset from center)

Every sensor reading must be "transformed" back to the `base_link` before you use it in the EKF.

---

## **2. Extrinsic vs. Intrinsic**
*   **Intrinsic:** Internal to the sensor (e.g., The focal length of your camera lens).
*   **Extrinsic:** Where the sensor is in the world/on the robot (e.g., The distance between the lens and the battery).

---

## **3. The Multi-Agent Problem**
When you have **three drones**, the calibration becomes a nightmare if you don't use a standard.
*   Each drone must have its own unique `namespace` (e.g., `squid_1`, `squid_2`).
*   The "Base Station" must know the `transform` between `world` and `squid_1`, `world` and `squid_2`, etc.

**Lab Task:** Use a digital caliper to measure your build. Enter those numbers into `src/utils/transforms.py`. **Accuracy here is the difference between a hover and a crash.**
