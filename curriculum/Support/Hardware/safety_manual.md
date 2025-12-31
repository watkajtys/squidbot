# The Squid Drone Safety Manual
**"Safety Rules are written in blood (or burnt houses)."**

This is an autonomous flying robot with spinning blades and high-energy explosives (LiPos) strapped to it. Treat it with respect.

---

## **1. LiPo Battery Safety**
**The #1 Rule:** NEVER leave a charging battery unattended.

*   **Minimum Voltage:** Never fly below **3.5V per cell**.
    *   3S Battery (3 cells): Land at **10.5V**.
    *   If you hit **9.0V**, the battery is chemically damaged. Throw it away.
*   **Storage:** If you aren't flying for 2 days, discharge/charge the battery to **3.8V per cell** (Storage Voltage). Keeping a full battery (4.2V/cell) for weeks will degrade it (puffing).
*   **Puncture:** If you crash and dent the battery, put it in a fireproof bag outside. It may ignite hours later.

---

## **2. Propeller Safety**
**The "Blender" Rule.**

*   **PROPS OFF ON THE BENCH:**
    *   If the drone is plugged into USB on your desk, the propellers **MUST BE REMOVED**.
    *   Why? If you accidentally upload code that says `motor.set(10000)`, the drone will fly into your face.
*   **The Disarm Switch:**
    *   Always map a physical switch on your radio to "ARM/DISARM".
    *   Keep your finger on this switch *whenever* the drone is powered. It is your "Kill Switch."

---

## **3. The "Fly Away" Protocol**
**What if it stops listening?**

1.  **Disarm:** Hit the Kill Switch. (The drone falls. Parts are cheap. Lawsuits are expensive).
2.  **Failsafe:** Configure Betaflight to "Drop" if it loses radio connection. Never set it to "Return to Home" indoors (it will fly into the ceiling).

---

## **4. Soldering Safety**
*   **Ventilation:** Solder fumes are lead/flux. Open a window.
*   **Short Circuits:**
    *   Carbon Fiber conducts electricity.
    *   If a bare wire touches the frame, it can short the battery.
    *   **Rule:** Use electrical tape or heat shrink on *everything*.
--- [Return to Course Map](../../COURSE_MAP.md)