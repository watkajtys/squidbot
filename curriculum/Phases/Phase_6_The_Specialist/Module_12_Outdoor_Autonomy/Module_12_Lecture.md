[Previous Module](../Module_11_Tactics_and_Guidance/Module_11_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../../Phase_7_The_Frontier/Module_13_VIO/Module_13_Lecture.md)

---

# Module 12: Outdoor Autonomy
**"The Traveler."**

Everything so far has been indoors (Vicon, Optical Flow, Walls). The real world has Wind, GPS Satellites, and Magnetic Interference.

---

## **12.1 GPS Integration**

### **Objective**
Global Positioning.

### **Theory**
*   **WGS84:** The globe (Latitude, Longitude, Altitude).
*   **NED (North-East-Down):** A local flat plane centered on your startup point (meters).
*   **Conversion:** We must project the curved earth onto our flat local map.

### **Lab Procedure**
1.  **Hardware:** Enable the M10Q GPS module.
2.  **Driver:** Parse NMEA sentences (`$GNGGA`) to get Lat/Lon/HDOP.
3.  **Math:** Write a `geo_to_ned(lat, lon, origin_lat, origin_lon)` function.

### **12.1.1 Sub-Lab: The GPS Treasure Hunt**
**"Walking the Math."**

Before you trust the GPS to fly 100 meters, you must trust it to measure 10 meters.

1.  **Setup:** Stand at a starting point (Point A). Save the Lat/Lon as your `origin`.
2.  **Move:** Walk exactly 10 meters North. Save the Lat/Lon as Point B.
3.  **Calculate:** Use your `geo_to_ned` function to convert Point B into meters relative to Point A.
4.  **Verification:** Does your code say `(10.0, 0.0)`? 
    *   If it says `(0.0, 10.0)`, you swapped North and East (a very common PhD-level mistake!).
5.  **The Step-up:** This lab forces you to handle the **Curvature of the Earth**. You'll realize why a simple subtraction of Lat/Lon numbers doesn't work for distance!

---

## **12.2 Hybrid Navigation**

### **Objective**
Seamless switching between Indoor (Flow) and Outdoor (GPS).

### **Theory**
*   **Indoor:** GPS is erratic (multipath reflection). Trust Optical Flow.
*   **Outdoor:** Optical Flow fails (grass is too far away / motion blur). Trust GPS.
*   **The Switch:** Monitor `GPS_HDOP` (Horizontal Dilution of Precision).
    *   If `HDOP < 1.5` and `Sats > 7`: **Outdoor Mode**.
    *   Else: **Indoor Mode**.

---

## **12.3 Advanced Logic: Behavior Trees**

### **Objective**
Beyond `if-else`.

### **Theory**
Finite State Machines (FSMs) get messy ("Spaghetti State").
**Behavior Trees (BTs)** are composable:
*   **Sequence:** "Takeoff -> Fly to A -> Land".
*   **Selector (Fallback):** "Try Fly to A. If Fail, Try RTH. If Fail, Land."

### **Lab Procedure**
1.  **Library:** Use `py_trees` or `BehaviorTree.CPP`.
2.  **Design:**
    *   Root -> Fallback
        *   Sequence (Mission)
            *   Condition: Battery > 30%
            *   Action: Patrol
        *   Action: RTH

---

## **Check: The Mile Run**
**The Graduation Flight.**

1.  **Plan:** A 1km waypoint mission in a park.
2.  **Execute:**
    *   Takeoff (Stabilized).
    *   Fly to Waypoint A (GPS).
    *   Fly to Waypoint B (GPS).
    *   Return to Home (RTH).
    *   Land.
3.  **Constraint:** You cannot touch the sticks unless it's an emergency.

## **Theoretical Foundations**

### Lecture 12: Outdoor Navigation & Geodetic Logic

#### **1. Geo-Projection & Local Tangent Planes (LTP)**
GPS provides coordinates on an ellipsoid (WGS84). To fly, we need a flat map.
*   **The Haversine Formula:** Used to calculate the distance between two points on a sphere: $d = 2r \arcsin(\sqrt{\sin^2(\frac{\Delta \phi}{2}) + \cos \phi_1 \cos \phi_2 \sin^2(\frac{\Delta \lambda}{2})})$. 
*   **NED Projection:** For local flight, we project Lat/Lon onto a **Local Tangent Plane**. We treat the drone's startup point as $(0,0,0)$ and linearize the earth's curvature for the first $1\text{km}$, allowing us to use Euclidean geometry for path planning.

#### **2. Energy-Maneuverability (E-M) Theory**
Outdoor flight is limited by the "Battery Budget" and headwind resistance.
*   **Specific Excess Power ($P_s$):** Defined as $P_s = \frac{T - D}{W} V$, where $T$ is thrust, $D$ is drag, $W$ is weight, and $V$ is velocity. 
*   **The Optimization:** If $P_s > 0$, the drone can climb or accelerate. We use E-M theory to calculate the **Max Endurance Speed** (staying aloft the longest) vs. the **Max Range Speed** (reaching the goal against a headwind).

#### **3. Formal Logic: Behavior Trees (BTs)**
Complex missions fail when Finite State Machines (FSMs) hit an edge case.
*   **Reactivity:** BTs are "Ticked" at high frequency (e.g., 10Hz). Every tick, the tree is re-evaluated from the root.
*   **The Memoryless Advantage:** Unlike FSMs, BTs don't "get stuck" in a state. If the battery suddenly drops, the "Battery Check" node at the top of the tree returns **FAILURE**, instantly halting the mission and forcing a Return-to-Home without requiring complex state-transition logic.

**Next Step:** [Phase VII: Module 13 VIO](../../Phase_7_The_Frontier/Module_13_VIO/Module_13_Lecture.md)

---

## **Professor's Brief: Speaker Notes**

### **The Narrative Hook**
"The real world is messy. It has wind, it has magnetic storms, and it has GPS dropouts. A true autonomous robot must be a 'Hybrid'—capable of trusting the satellites when they are clear, but relying on its own 'Instincts' (Flow/IMU) when the sky goes dark. Today, we leave the safety of the basement and take our drone to the frontier."

### **Deep Research Context: Dilution of Precision (DOP)**
In research, 4 satellites isn't enough. We care about the **Geometric Dilution of Precision (GDOP)**. If all satellites are in a line, your distance calculation is highly unstable. Explain that a PhD-level flight controller checks the **HDOP** (Horizontal) and **VDOP** (Vertical) before enabling autonomous mode. If HDOP > 2.0, we "Force Inhibit" the mission.

## **Mastery Checklist**
Before moving to the next module, you should be able to:
- [ ] Explain the math behind projecting WGS84 (Global) to NED (Local) coordinates.
- [ ] Calculate the "Max Range Speed" of a drone using E-M Theory.
- [ ] Diagram a Behavior Tree with a Fallback (Selector) node for battery safety.
- [ ] Define "Geometric Dilution of Precision" (GDOP) and its impact on GPS reliability.

---

## **Further Reading & Bibliography**

### **Geodetics**
*   **Hofmann-Wellenhof, B., et al. (2007).** *GNSS—Global Navigation Satellite Systems.* Springer-Verlag. (The definitive reference).

### **Robot Logic**
*   **Collett, T. (2018).** *Behavior Trees in Action.* (Practical context for BTs).
*   **Philippides, A., et al. (2011).** *"From birds to robots: The role of behavior trees in autonomous navigation."*

---

[Previous Module](../Module_11_Tactics_and_Guidance/Module_11_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../../Phase_7_The_Frontier/Module_13_VIO/Module_13_Lecture.md)