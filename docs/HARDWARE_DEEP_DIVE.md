# Hardware Component Deep Dive
**"Understanding your nervous system."**

Beyond the wiring, you must understand *how* these components communicate and their mathematical limitations.

---

## 1. The VL53L5CX (8x8 ToF Array)
This is not a camera; it's a multi-zone laser.
*   **Physics:** It emits photons and measures the "Time of Flight" until they return.
*   **The FOV:** It has a 45-degree Field of View. It gives you 64 distance readings (8x8 grid).
*   **Limitations:** Dark surfaces (black carpet) absorb the laser, and mirrors reflect it away. Both result in "Max Range" errors.
*   **The Math:** You must transform these 64 points from the "Sensor Frame" to the "Drone Frame" using the `transforms.py` utility.

## 2. The PMW3901 (Optical Flow)
Think of this as a high-speed mouse sensor looking at the ground.
*   **Physics:** It tracks the movement of texture (pixels) on the floor.
*   **The Catch:** It measures *angular* velocity (pixels/second). To get *linear* velocity (meters/second), you MUST know the height from the Lidar.
*   **Formula:** $V_{real} = V_{pixel} \times Height$.
*   **Blind Spots:** It fails on smooth, featureless floors (like white tiles) or in total darkness.

## 3. The Betaflight F405 (Actuator Server)
We do not use Betaflight for "Control." We use it as a "Slave."
*   **Protocol:** MSP (Multiwii Serial Protocol).
*   **The Pi's Role:** The Pi calculates the required Roll/Pitch/Yaw/Throttle.
*   **The FC's Role:** It takes those commands and runs a 8kHz PID loop to drive the motors.
*   **Prerequisite:** You must configure "MSP Displayport" or "MSP" on UART1 in the Betaflight Configurator.

## 4. The M10Q GPS & Compass
The compass is the most sensitive part of the drone.
*   **Interference:** High current in the motor wires creates magnetic fields (Electromagnetic Interference - EMI). This will make your compass spin.
*   **The Fix:** Mount the GPS/Compass as far away from the battery wires as possible.
*   **Calibration:** You must perform the "6-Sided Dance" to calibrate the magnetometer every time you change the hardware layout.

## 5. The Arducam IMX219
*   **Shutter:** This is a "Rolling Shutter" camera. If the drone vibrates, the image will look like "Jello."
*   **The Fix:** Use the Notch Filters (Module 4) to clean up the physical vibration.
*   **FOV:** The 160-degree lens has significant "Barrel Distortion." You must use the `lab_3_calibration.py` logic to "unflatten" the world for the AI to understand it.
