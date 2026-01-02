[Return to Course Map](../../../../COURSE_MAP.md)

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

> **Pro Tip: The Modern Workflow (VS Code Remote)**
> Editing code with `nano` is painful.
> 1.  Install **VS Code** on your laptop.
> 2.  Install the **Remote - SSH** extension.
> 3.  Connect to `pi@squid-drone.local`.
> Now you can edit files on the Pi using a full IDE, drag-and-drop files, and run the integrated terminal. This is how professionals work.

### **Lab 0.2.1: The VIP Room (CPU Isolation)**
**"The Chef Analogy."**
Imagine a Chef (CPU Core) chopping onions (Flight Loop). If the phone rings (WiFi Interrupt), the Chef stops chopping to answer. In a kitchen, dinner is late. In a drone, the motors stop updating for 20ms, and you flip over.

**The Goal:** We will fire the "Scheduler" from **Core 3** and dedicate that core exclusively to our flight code.

1.  **The Baseline:**
    *   Run `htop`. You see 4 bars (0, 1, 2, 3) dancing with random tasks.
2.  **The Kernel Isolation (isolcpus):**
    *   `isolcpus=3` tells the OS: "Do not schedule user programs (like SSH or Python) on Core 3 unless forced."
    *   **Action:** Add `isolcpus=3` to `/boot/cmdline.txt`.
3.  **The Interrupt Routing (SMP Affinity):**
    *   Even with `isolcpus`, the hardware wires (IRQs) for WiFi/USB can still fire on Core 3. We must "mask" them.
    *   **The Bitmask:** We have 4 cores. We represent them as binary bits: `3 2 1 0`.
    *   We want Cores 0, 1, 2 to handle interrupts. That is Binary `0111`.
    *   Binary `0111` = Hex `7`.
    *   **Action (Temporary):** `echo 7 | sudo tee /proc/irq/default_smp_affinity`.
    *   **Action (Permanent):** To make this survive a reboot, you must add `echo 7 > /proc/irq/default_smp_affinity` to your `/etc/rc.local` file (before the `exit 0` line).
    *   *Note:* The `setup_pi.sh` script does this for you automatically.
4.  **The Enemy (irqbalance):**
    *   Linux has a daemon called `irqbalance` that tries to undo our work to "maximize throughput."
    *   **Action:** `sudo systemctl stop irqbalance`.
5.  **The Verification:**
    *   Reboot. Run `htop`. Core 3 should be dead silent (0.0%).
6.  **The VIP Pass (`taskset`):**
    *   To run code in the VIP room, you need a pass.
    *   Run: `taskset -c 3 python3 hello_squid.py`.
    *   This forces your script to run ONLY on Core 3.

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
