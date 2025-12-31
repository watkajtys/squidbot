# Theory Deep Dive 0.9: Battery Dynamics & Power Systems
**"The Flying Chemical Plant."**

A drone is not powered by a magic "5.0V" source. It is powered by a Lithium Polymer (LiPo) battery—a complex chemical system with its own internal resistance, voltage curves, and failure modes.

---

## **1. The Voltage Curve**
A 3S LiPo battery (11.1V Nominal) actually ranges from:
*   **12.6V:** Fully Charged (4.2V per cell).
*   **11.1V:** Nominal/Half Empty (3.7V per cell).
*   **10.5V:** Critical/Empty (3.5V per cell).

**The Impact on Flight:**
Motor RPM is defined by $RPM = V \times KV$.
*   At 12.6V, a 5000KV motor spins at **63,000 RPM**.
*   At 10.5V, the same motor spins at **52,500 RPM**.
*   **Result:** Your drone will feel "punchy" at the start of a flight and "mushy" at the end.

---

## **2. Voltage Sag (Internal Resistance)**
When you demand 20 Amps from a battery, the voltage drops instantly. This is called **Voltage Sag**.
$$V_{actual} = V_{rest} - (I \times R_{internal})$$
*   If your battery has high internal resistance ($R_{internal}$), your voltage will sag more under load.
*   **The Problem:** If you are hovering at 50% throttle and suddenly punch to 100%, the voltage might sag so low that the Raspberry Pi resets (Brownout).

---

## **3. The "Battery-Aware" Controller**
In Phase III (Control Theory), we use **Voltage Compensation**.
Instead of sending a raw PWM signal from 0 to 1000, we scale the output based on the current battery voltage ($V_{batt}$):
$$Output_{compensated} = Output_{desired} \times \frac{V_{nominal}}{V_{batt}}$$
This ensures that the drone "feels" the same at 100% battery as it does at 10%.

---

## **4. The C-Rating (Discharge Rate)**
*   **Capacity ($C$):** Measured in mAh (e.g., 450mAh).
*   **C-Rating:** How fast the battery can safely discharge (e.g., 75C).
*   **Max Current:** $Current_{max} = Capacity \times C$.
*   A 450mAh 75C battery can provide **33.75 Amps**. If your motors draw more than this, the battery will overheat and may catch fire (Thermal Runaway).

---

## **5. Health Monitoring (The "Pre-Flight" Check)**
*   **Imbalance:** Each cell in a 3S battery must be within 0.05V of each other. If Cell 1 is 4.2V and Cell 2 is 3.8V, the battery is unstable.
*   **Storage:** Never store LiPos fully charged. They should be kept at **3.85V per cell**. Storing them at 4.2V causes "Puffing" (gas buildup), which increases internal resistance and ruins the battery.

**Mental Model:** Treat your battery like a fuel tank that gets smaller and leakier every time you use it. Respect the chemicals, or they will disrespect your drone.
## **6. Power Budgeting for the Raspberry Pi**
The Pi Zero 2 W is your "Big Brain," and it is power-hungry when thinking about AI.

### **The Current Draw:**
*   **Idle:** ~100mA
*   **Full CPU (4 Cores):** ~500mA
*   **Full CPU + Camera + AI (TFLite):** ~800mA - 1.2A

### **The "Brownout" Calculation:**
If your BEC (Module 0) is rated for 2.0A, but your motors create a 2.0V voltage sag on the 3S rail, the BEC's efficiency drops.
*   **The Safety Margin:** You should never exceed 80% of your BEC's rated current. 
*   **The Audit:** If the Pi, Camera, and ToF sensors combined draw 1.5A, and your BEC is a 2.0A model, you are in the "Danger Zone" during aggressive maneuvers.

**Pre-Flight Checklist:**
- [ ] Measure total current draw at full AI load.
- [ ] Verify BEC remains < 50°C during bench testing.

---

## **Mastery Check**