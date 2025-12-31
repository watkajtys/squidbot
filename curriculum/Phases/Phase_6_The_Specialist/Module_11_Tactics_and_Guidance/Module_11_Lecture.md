[Previous Module](../Module_10_Reinforcement_Learning/Module_10_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_12_Outdoor_Autonomy/Module_12_Lecture.md)

---

# Module 11: Aerial Combat & Guidance
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

---
## **Theoretical Foundations**

### Lecture 11: Guidance Laws & Interception

#### **1. Proportional Navigation (Pro-Nav) & Collision Triangles**
Interception is not a chase; it is a geometric rendezvous.
*   **The Zero-Effort-Miss (ZEM):** The distance by which the drone would miss the target if it continued at its current velocity.
*   **Closing Velocity ($V_c$):** The rate at which the range $R$ is decreasing: $V_c = -\dot{R}$. 
*   **The Command Law:** $a_n = N \cdot V_c \cdot \dot{\lambda}$. Where $N$ is the Navigation Constant (typically 3-5) and $\dot{\lambda}$ is the Line-of-Sight Rate.
*   **Robustness:** Pro-Nav is preferred in research because it is "Target-Independent." It doesn't need to know the target's mass or acceleration; it only needs the geometric rate of change.

#### **2. Zero-Sum Differential Games**
Aerial pursuit is a **Minimax** problem.
*   **The Objective:** $J = \min_{Pursuer} \max_{Evader} [\text{Final Distance}]$.
*   **Saddle-Point Strategy:** We seek a strategy where neither the pursuer nor the evader can gain an advantage by unilaterally changing their path. This is the foundation of autonomous "Dodge" maneuvers.

### Lecture 11.5: Optimal Search & Bayesian Heatmaps
How do you find a "Lost" drone? 
*   **Prior Probability ($P(H)$):** We define a grid where each cell has a probability of containing the target.
*   **Negative Information:** When we search a cell and see nothing, we use **Bayes' Rule** to update our belief: $P(H | \text{Empty}) = \frac{P(\text{Empty} | H) P(H)}{P(\text{Empty})}$.
*   **Heatmap Redistribution:** The probability doesn't disappear; it "flows" into the unsearched areas. This guides the drone to move from "High-Confidence Empty" zones to "Low-Confidence Unknown" zones.

**Next Step:** [Module 12: Outdoor Autonomy](../../Phase_6_The_Specialist/Module_12_Outdoor_Autonomy/Module_12_Lecture.md)

---

## **Professor's Brief: Speaker Notes**

### **The Narrative Hook**
"Interception is the geometry of meeting. It isn't a race; it's a calculation. If you chase someone, you are always late. If you lead someone, you are already there when they arrive. Today, we turn geometry into a weapon of efficiency. We are teaching our drone not just to 'follow,' but to 'intercept' with the cold, hard logic of a heat-seeking missile."

### **Deep Research Context: The "Tail Chase" Trap**
In amateur guidance, people use "Pure Pursuit" (pointing the nose at the target). In research, we call this the "Tail Chase" trap. As the target turns, the pursuer must turn sharper and sharper until its centripetal force exceeds its motor power ($a > U_{max}$). Pro-Nav avoids this by "Leading" the target, which mathematically minimizes the required acceleration at the end of the maneuver.

## **Mastery Checklist**
Before moving to the next module, you should be able to:
- [ ] Diagram the geometric relationship of Proportional Navigation (Pro-Nav).
- [ ] Explain the role of the Navigation Constant ($N$) in interceptor stability.
- [ ] Define a "Saddle Point" in the context of Pursuit-Evasion games.
- [ ] Describe the recursive Bayesian update process for a Search Heatmap.

---

## **Further Reading & Bibliography**

### **Guidance Laws**
*   **Zarchan, P. (2012).** *Tactical and Strategic Missile Guidance.* Progress in Astronautics and Aeronautics. (The industry standard).
*   **Shneydor, N. A. (1998).** *Proportional Navigation.* Peter Peregrinus Ltd.

### **Optimal Search & Games**
*   **Stone, L. D. (1975).** *Theory of Optimal Search.* Academic Press. (The foundational work on Bayesian search).
*   **Isaacs, R. (1965).** *Differential Games: A Mathematical Theory with Applications.* Wiley.

---

[Previous Module](../Module_10_Reinforcement_Learning/Module_10_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_12_Outdoor_Autonomy/Module_12_Lecture.md)