# Module 0 Labs: The "Pre-Flight" Drills
**"Confidence is built through testing, not guessing."**

Before you assemble the drone, you must master your tools. These labs are designed to be performed with the components spread out on your desk ("On the Bench") before they are tucked away inside the frame.

---

## **Lab 0.1: The Power Audit**
### **Goal:** Verify your BEC works before it kills your Pi.
1.  **Safety:** Ensure your LiPo battery is at storage voltage (~11.4V).
2.  **Continuity Check:** Set your multimeter to the "Beep" mode. Touch the Red and Black wires of your power lead. **It should NOT beep.** If it beeps, you have a short circuit.
3.  **Voltage Check:** Plug in the battery. Measure the output of your BEC (Battery Eliminator Circuit).
    *   **Success:** Multimeter reads between **4.9V and 5.2V**.
    *   **Failure:** If it reads > 5.5V, **DO NOT** connect the Pi. Your BEC is defective.

---

## **Lab 0.2: The Linux Playground**
### **Goal:** Navigate the drone's brain via terminal.
1.  **SSH into the Pi:** `ssh pi@squid-drone.local`.
2.  **Resource Check:** Run `htop` (or `top`).
    *   Look at your CPU cores. Look at the RAM usage (~512MB).
    *   This is your "Budget." If your Python code uses 90% CPU, your drone will be unstable.
3.  **File Surgery:**
    *   Create a folder: `mkdir test_folder`
    *   Create a file: `nano hello_squid.py`
    *   Type: `print("Squid Online")`
    *   Save (Ctrl+O, Enter) and Exit (Ctrl+X).
    *   Run it: `python3 hello_squid.py`.

---

## **Lab 0.3: Betaflight Sensors (The "Ear" Test)**
### **Goal:** Verify the Flight Controller's "Inner Ear" works.
1.  **Connect:** Plug the Flight Controller (FC) into your laptop via USB.
2.  **Open:** Betaflight Configurator.
3.  **The 3D Model:** Go to the "Setup" tab.
    *   Move the FC in your hand. The 3D drone on screen should mirror your movements exactly.
4.  **The Graph:** Go to the "Sensors" tab.
    *   Select "Gyroscope" and "Accelerometer".
    *   Tap the table next to the FC. You should see "Spikes" in the graph. This is the FC hearing the vibration of your finger.

---

## **Lab 0.4: Wi-Fi Signal Mapping**
### **Goal:** Find your drone's "Safety Bubble."
1.  **Ping:** From your laptop terminal, run `ping squid-drone.local -t` (on Windows) or `ping squid-drone.local` (on Mac/Linux).
2.  **The Walk:** Hold the Pi (powered via battery/BEC) and walk around your room/house.
3.  **Observe:**
    *   **Near Router:** `time=2ms` to `10ms`. (Perfect).
    *   **Behind Wall:** `time=50ms` to `150ms`. (Dangerous).
    *   **In Kitchen (Microwave on):** `Request Timed Out`. (Instant Crash).
**Lesson:** Identify exactly where your Wi-Fi is strong. This is your "Flight Zone."

---

## **Lab 0.5: Motor Safety Test (PROPS OFF)**
### **Goal:** Verify your "Chain of Command" without losing a finger.
1.  **Setup:** Connect FC to Betaflight. Plug in the drone battery (required to power motors).
2.  **Props:** **REMOVE PROPELLERS.** Never test motors with props on the bench.
3.  **Motor Tab:**
    *   Toggle "I understand the risks."
    *   Slide "Motor 1" up slightly.
    *   **Verify:** Does the real Motor 1 spin? Is it spinning in the right direction?
    *   **Direction Check:** Touch the side of the spinning motor with a piece of paper. The way the paper "kicks" tells you the direction.

---

## **Lab 0.6: The Power Curve**
### **Goal:** Understand how your battery affects your thrust.
1.  **Theory:** A drone motor's RPM is determined by $V \times KV$. If your battery voltage drops from 12.6V (Full) to 11.1V (Low), your max thrust drops by ~20%.
2.  **Exercise:** Use a multimeter to monitor your battery voltage while you spin up the motors (Props OFF) to 100% in Betaflight.
    *   **Observe:** Does the voltage "sag" when the motors are at 100%?
    *   **The Math:** Calculate the percentage loss in potential RPM between Full and Low battery. This is why our controllers will need "Voltage Scaling" in Phase III.
- [ ] BEC output is 5V.
- [ ] Can SSH and run a Python script.
- [ ] 3D Model in Betaflight matches my hand movements.
- [ ] Ping time is < 20ms in the Flight Zone.
- [ ] All 4 motors spin (Props Off).
