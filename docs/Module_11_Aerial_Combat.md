# Module 11: Aerial Combat
**"Fox Two."**

Civilian drones avoid obstacles. Military drones intercept them.
In this module, we explore **Guidance Laws**. We will not shoot anything, but we will "Lock On" and chase a target.

---

## **11.1 Proportional Navigation (Pro-Nav)**

### **Objective**
Intercept a moving target.

### **Theory**
*   **Pursuit:** Fly directly at the target. (Bad. You end up in a "Tail Chase").
*   **Pro-Nav:** "Keep the Line-of-Sight angle constant."
    *   If the target is moving right at 10 deg/sec, you turn right at $N \times 10$ deg/sec.
    *   Result: You fly a collision triangle and intercept them *efficiently*.

### **Lab Procedure**
1.  **Visual Servoing:** Use the Arducam to detect a red ball (The "Bogey").
2.  **The Error:** Calculate the pixel offset ($x, y$) of the ball from the center.
3.  **The Controller:** Input this pixel error into the Pro-Nav equation to generate Yaw/Pitch commands.

---

## **11.2 Visual Servoing: The Red Balloon**

### **Objective**
"Ramming Speed."

### **Theory**
*   **HSV Color Space:** RGB is bad for color tracking (shadows change R, G, and B). Hue (H) stays constant in shadow.
*   **Servoing:** We don't need to know "Position (meters)." We just need "Pixel Error."

### **Lab Procedure**
1.  **Target:** Hang a red balloon.
2.  **Vision:** Use OpenCV to threshold Red pixels (`cv2.inRange`). Find the largest contour.
3.  **Control:**
    *   **Yaw:** Minimize `center_x - image_center_x`.
    *   **Pitch:** Minimize `center_y - image_center_y` (or maintain altitude).
    *   **Throttle:** Maintain a specific "Blob Area" (Distance). To ram, set Target Area = Infinity.

---

## **11.3 The Dogfight (Game Theory)**

### **Objective**
Don't get hit.

### **Theory**
*   **Minimax:** Minimize the maximum damage the enemy can do to you.
*   **The Game:**
    *   **Pursuer:** Minimizes distance.
    *   **Evader:** Maximizes distance.

### **Lab Procedure (Simulation)**
1.  **Setup:** Two agents in `gym-pybullet-drones`.
2.  **Red Team:** Uses Pro-Nav.
3.  **Blue Team:** Uses RL (PPO) trained to maximize survival time.
4.  **Result:** The RL agent will learn maneuvers (Barrel Rolls, Split-S) to break the Pro-Nav lock.

---

---

## **11.4 Sub-Lab: The QR Courier**
**"Vision-Based Mission Logic."**

In previous labs, you followed a red ball. In this lab, the robot will follow "Instructions."

1.  **Setup:** Print three QR codes.
    *   QR 1: Contains text `CMD:ORBIT_LEFT`
    *   QR 2: Contains text `CMD:ORBIT_RIGHT`
    *   QR 3: Contains text `CMD:LAND`
2.  **Vision:** Use `opencv` and `pyzbar` to detect and decode the QR code in the camera feed.
3.  **Behavior:**
    *   Calculate the distance to the QR code based on its pixel size.
    *   If the code says `ORBIT`, the drone must maintain a 1m distance and circle the code.
    *   If it says `LAND`, it must approach to 20cm and kill the motors.
4.  **The Knowledge Step-up:** This combines **Perception** (decoding the signal) with **State Logic** (the Behavior Tree choosing a new branch based on the signal).

---

## **Check: The Dark Room Scenario**
**The Thesis Defense.**

1.  **Scenario:** Pitch black room.
2.  **Target:** A flashlight moving randomly.
3.  **Drone:** Uses Lidar for altitude + Camera for tracking.
4.  **Goal:** Keep the flashlight in the center of the frame for 30 seconds.

**Submission:** Video feed from the drone showing the "Lock" indicator.
