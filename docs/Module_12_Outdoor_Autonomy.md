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

**Submission:** The `.mcap` log file overlaid on Google Earth.
