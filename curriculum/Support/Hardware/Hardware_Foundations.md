# Hardware Foundations: The Workbench
**"Respect the electron."**

## **0. Safety Gear (Mandatory)**
Before you pick up a tool:
1.  **Safety Glasses:** When you clip a wire, the copper flies at 100mph. When a prop shatters, plastic flies faster. **Wear them.**
2.  **Ventilation:** Solder fumes are lead and rosin. Open a window or use a fan.
3.  **Fire Safety:** LiPo batteries burn hot. Charge them in a LiPo bag. Have a fire extinguisher nearby.

## **1. Soldering 101**
Think of electricity like water flowing through pipes.

*   **Voltage ($V$ - Volts):** Pressure. The force pushing the water. (Your battery is 11.1V).
*   **Current ($I$ - Amps):** Flow. How much water is moving. (Your motors draw ~10A).
*   **Resistance ($R$ - Ohms):** Pipe Size. How much the wire resists the flow.
*   **Power ($P$ - Watts):** Total Work. $P = V \times I$.

### **The Golden Rules**
1.  **Short Circuits:** If a 12V wire touches a Ground wire directly, $R$ becomes 0. $I$ becomes infinite. **Fire happens.**
2.  **Grounding:** Everything must share a "Common Ground." The Pi and the Flight Controller must have their Ground pins connected, or they won't be able to "hear" each other's data signals.
3.  **Polarity:** Red is Positive (+), Black is Negative (-). Reversing these will almost always destroy your electronics instantly.

---

## **2. The Multimeter: Your Robot Doctor**
You cannot see electricity. The multimeter is your eyes.

*   **Continuity Mode (The Beep):** Touch the two probes together. It beeps. Use this to check if two wires are touching (bad) or if a solder joint is solid (good).
*   **Voltage Mode:** Measure your battery. A "3S" LiPo should be between 11.1V (empty) and 12.6V (full).
*   **The 5V Test:** Before plugging your $15 Pi into the BEC, measure the BEC output. If it says 12V instead of 5V, you just saved $15.

---

## **3. Soldering: The Art of the "Cold Join"**
Soldering is not "gluing" wires with metal. It is a chemical bond.

### **The Tools**
*   **Soldering Iron:** Needs to reach ~350°C.
*   **Solder:** Use "60/40 Lead-Rosincore" (easier for beginners) or "Lead-Free" (safer but harder to use).
*   **Flux:** The "Secret Sauce." It cleans the metal so the solder sticks. **If it's not sticking, add more flux.**
*   **Brass Sponge:** To keep the iron tip shiny and clean.

### **The Technique: Heat the Joint, Not the Solder**
1.  **Tinning:** Put a tiny bit of solder on the iron tip (makes it shiny).
2.  **Heat:** Touch the iron to the wire AND the pad at the same time for 2 seconds.
3.  **Feed:** Touch the solder to the **wire/pad**, not the iron. It should melt and flow into the joint like water.
4.  **Finish:** Pull the solder away, then the iron. Don't move the wire for 3 seconds while it cools.

**The "Perfect Joint":** Should look like a shiny silver Hershey's Kiss. If it's a dull grey ball, it's a "Cold Joint" and will break mid-flight.

---

## **4. LiPo Battery Safety (CRITICAL)**
Drone batteries (Lithium Polymer) are high-energy density bombs.

*   **Storage:** Never leave a LiPo full or empty for more than a few days. Store them at "Storage Voltage" (~3.8V per cell).
*   **Charging:** Never charge unattended. Use a LiPo-safe bag.
*   **Puffing:** If the battery looks "swollen" or like a pillow, **Stop using it.** It is a fire hazard. Dispose of it safely.
*   **Low Voltage:** If you run a battery below 3.0V per cell, it is permanently damaged. This is why we need a battery alarm/telemetry.

---

## **5. Essential Toolkit**
1.  **Flush Cutters:** For cutting wires and zip ties.
2.  **Wire Strippers:** For removing insulation without cutting the copper.
3.  **Heat Shrink:** The "Rubber Skin" for your wires. Use a lighter or heat gun to shrink it over bare solder joints.
4.  **Blue Loctite:** For the screws. Vibration from motors will unscrew your drone mid-air if you don't use this.

---

## **6. Checklist: Before Your First Power-Up**
- [ ] No "stray hairs" of copper wire touching other pads.
- [ ] Multimeter "Beep" test between 12V and GND shows **NO BEEP**.
- [ ] BEC output confirmed at 5.0V - 5.2V.
- [ ] All bare wires covered in heat shrink or electrical tape.
--- [Return to Course Map](../../COURSE_MAP.md)