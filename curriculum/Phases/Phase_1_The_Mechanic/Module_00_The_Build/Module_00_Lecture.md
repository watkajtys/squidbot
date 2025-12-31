[Previous: Syllabus](../../../../SYLLABUS.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_01_Bare_Metal_API/Module_01_Lecture.md)

---

# Module 0: The Build & Legal Compliance
**"Hardware is the body. Linux is the soul."**

Before we write code, we must build the machine. This module covers the physical assembly of the Squid drone and the configuration of its operating system.

---

## **0.1 Anatomy & Power**

### **The Schematics**
We are grafting a "Big Brain" (Raspberry Pi Zero 2 W) onto a "Fast Spinal Cord" (Betaflight Flight Controller).

*   **The Problem:** The Pi needs 5V. The Drone battery is 12V (3S).
*   **The Danger:** If you plug 12V into the Pi, it dies immediately.
*   **The Solution:** The BEC (Battery Eliminator Circuit). It steps 12V down to a clean 5V/3A.

### **Wiring Diagram**
**STOP.** Open [hardware_reference.md](../../../Support/Hardware/hardware_reference.md) for the exact pinout and soldering guide. Do not guess.

1.  **Power:** Battery Pads $\to$ BEC Input (12V) $\to$ BEC Output (5V) $\to$ Pi GPIO 5V (Pin 2/4) & GND (Pin 6).
2.  **Data:** FC UART TX $\to$ Pi UART RX (GPIO 15).
3.  **Data:** FC UART RX $\to$ Pi UART TX (GPIO 14).
4.  **Ground:** **CRITICAL.** You MUST connect a common Ground (GND) wire between the FC and the Pi.

## **0.1.1 The "Deep-Dive" Breakout: Power Budgeting**
> **Professor's Note:** The Pi Zero 2 W is a "Brownout" waiting to happen.
> **Why?** When you run AI and 4 motors at full speed, the voltage sags. If your BEC isn't rock-solid, the Pi will reboot mid-air.
> **The Goal:** You will calculate the **Total Current Draw** for every component. You will ensure your power rail has at least a 20% safety margin before your first takeoff.

### **0.1.2 Just-In-Time Logic: The Wiring Language**
**"How chips talk."**

*   **UART (Universal Asynchronous Receiver-Transmitter):**
    *   **The Analogy:** A phone call. You speak (TX), I listen (RX).
    *   **The Rule:** TX must go to RX. (If you wire TX-to-TX, two chips shout at each other and nobody listens).
*   **I2C (Inter-Integrated Circuit):**
    *   **The Analogy:** A classroom. The Teacher (Master) points at Student #29 (Slave Address) and asks for an answer.
    *   **The Rule:** All students share the same wire (SDA/SCL). Each must have a unique Name Tag (Address).

---

## **0.2 Assembly**

### **The Mount**
1.  Print `hardware/avionics_mount.scad`.
2.  Mount the Pi Zero 2 W using M2.5 nylon standoffs.
3.  Secure the Camera (Arducam) to the front slot.
4.  **Vibration Damping:** Use soft silicone gummies between the mount and the drone frame. This is crucial for Module 4 (Signal Processing).

### **The Camera Ribbon**
*   Thread the CSI ribbon cable carefully. It is fragile.
*   **Orientation:** The blue tape on the cable usually faces *away* from the board on the Pi Zero side, but check your specific connector.

### **The Flight Controller Setup**
Before you close everything up, you must configure the "Spinal Cord" (Betaflight).
1.  **Install:** Betaflight Configurator on your Laptop.
2.  **Config:** Follow the [Betaflight Passthrough Guide](../../../Support/Setup/Betaflight_Passthrough_Config.md) to prepare the FC for external control.
3.  **Ports:** In the "Ports" tab, ensure the UART you soldered to is set to **MSP** at 115200 baud.

---

## **0.3 Systems: Headless Linux**

### **Objective**
Set up the Pi without a monitor or keyboard.

### **Procedure**
1.  **Flash the OS:**
    *   Download **Raspberry Pi OS Lite (64-bit)**. (Do not use Desktop version; it wastes RAM).
    *   Use "Raspberry Pi Imager".
2.  **Configuring settings (The "Gear" Icon):**
    *   **Hostname:** `squid-drone`.
    *   **User:** `pi` (or your name).
    *   **Wi-Fi:** Enter your router's SSID and Password.
    *   **Enable SSH:** This is mandatory.
3.  **First Boot:**
    *   Insert SD card. Power on.
    *   Wait 2 minutes.
    *   On your laptop terminal: `ssh pi@squid-drone.local`.
    *   **Password:** The one you set in step 2.
4.  **Run the Automation Script:**
    *   Instead of manual configuration, run our setup script:
    *   `curl -sSL https://raw.githubusercontent.com/[REPLACE_WITH_REPO]/main/tools/setup_pi.sh | sudo bash`
    *   Or locally: `sudo ./tools/setup_pi.sh` from the project root.

### **Troubleshooting: "Could not resolve hostname"**
If you are on Windows and `squid-drone.local` fails:
1.  **Install Bonjour Print Services:** This enables `.local` domain discovery.
2.  **Or, find the IP:**
    *   Log into your WiFi Router's admin page.
    *   Look for a device named `squid-drone` or `raspberrypi`.
    *   Use the IP address: `ssh pi@192.168.1.X` (Replace X with the actual number).

### **Environment Setup**
Once logged in:
```bash
# Update everything
sudo apt update && sudo apt upgrade -y

# Install dependencies for Module 1 & 2
sudo apt install -y python3-pip git i2c-tools python3-smbus

# Enable Hardware Interfaces
sudo raspi-config
# -> Interface Options -> I2C -> Enable
# -> Interface Options -> Serial Port -> Login Shell: NO, Hardware Enabled: YES
```

### **0.3.1 Just-In-Time Workflow: Remote SSH**
**"Stop using Nano."**

Editing code inside a terminal is painful.
1.  **The Pro Tool:** Install **VS Code** on your laptop.
2.  **The Extension:** Install the "Remote - SSH" extension by Microsoft.
3.  **The Connection:**
    *   Click the green button (bottom left) -> "Connect to Host".
    *   Enter `pi@squid-drone.local`.
    *   Enter your password.
4.  **The Result:** You can now edit files on the drone as if they were on your laptop. You get IntelliSense, Autocomplete, and a built-in terminal.

**AI Prompt:** "How do I set up SSH Key-Based Authentication so I don't have to type my password every time I connect to my Raspberry Pi?"

### **0.3.2 Just-In-Time Hardware: Know Your Compute**
**"It's not a toy. It's a satellite."**

The Raspberry Pi Zero 2 W is technically a **System-on-Chip (SiP)**.
*   **The Brain (CPU):** Quad-core Cortex-A53 @ 1GHz. This is the same architecture used in the original Raspberry Pi 3, but squeezed into a tiny package. It has 4 cores, meaning it can do 4 things at once (Vision, Control, Comm, OS).
*   **The Muscle (GPU):** VideoCore IV. We don't use this for gaming. We use it for **H.264 Encoding**. It can compress camera video in real-time without touching the CPU.
*   **The Bottleneck (RAM):** 512MB LPDDR2. This is your constraint. If you load a massive AI model, the Pi will "swap" to the SD card and freeze. We must write efficient code.

**AI Prompt:** "What is the difference between the 'User Space' and 'Kernel Space' in Linux, and why does accessing GPIO pins via Python (User Space) introduce latency?"

### **0.3.3 Just-In-Time Workflow: The Time Machine (Git)**
**"Save your game."**

You will break your code. You will delete a file by accident.
1.  **Init:** Run `git init` in your `squid_drone` folder.
2.  **Commit:** Every time you finish a lab and it works, run:
    ```bash
    git add .
    git commit -m "Lab 1.1 Complete: Motors spin!"
    ```
3.  **The Safety Net:** If you break everything tomorrow, you can revert to today's commit. It is the "Undo" button for your entire project.

**AI Prompt:** "Explain the difference between 'git add', 'git commit', and 'git push'. How do I revert my last commit if I made a mistake?"

---

## **0.4 Legal & Safety Compliance**
**"Don't go to jail."**

*   **US (FAA):**
    *   **Registration:** If >250g (Squid is ~180g, so likely exempt, but check battery weight).
    *   **TRUST:** You *must* pass the "The Recreational UAS Safety Test" (Free online).
    *   **Remote ID:** Required for >250g or Part 107.
*   **EU (EASA):**
    *   **Class C0:** <250g. Read user manual.
    *   **Insurance:** Often mandatory even for toy drones if they have a camera.
*   **The Golden Rules:**
    *   Visual Line of Sight (VLOS) only.
    *   < 400ft AGL.
    *   Never over people.

---

## **Check: The Smoke Test**

**Stop. Do not skip this.**

1.  **Continuity Check:** Use a multimeter. Measure resistance between 5V and GND on the Pi. It should NOT be 0 (Short circuit).
2.  **The Power Up:**
    *   Plug in the battery.
    *   **Look:** For Magic Smoke.
    *   **Smell:** For burning silicon.
    *   **Listen:** The Flight Controller should beep (ESC initialization).
3.  **The Ping:**
    *   Open your laptop terminal.
    *   `ping squid-drone.local`
    *   If you see replies, your drone is alive.

---

## **0.5 Hands-On: Bench Testing**

Before you zip-tie the Pi to the frame and tuck away the wires, you must perform your "Bench Drills." It is 10x easier to fix a soldering error or a network issue while the components are spread out on your desk than when they are trapped inside the drone.

**Action:** Complete all exercises in **[Module 0 Labs](Module_00_Labs.md)**.

Do not proceed to Module 1 until your "Smoke Test" and "Ping Test" are both passing.

---

## **Deliverable**
*   A photo of your wiring (specifically the UART and BEC connections).
*   A screenshot of your terminal showing `pi@squid-drone:~ $` prompt.

---
## **Theoretical Foundations**

### Lecture 0: Systems Engineering & Determinism

#### **1. The Real-Time Budget & Jitter Math**
In robotics, "on time" is a functional requirement. 
*   **The Problem:** The Raspberry Pi runs a Linux kernel designed for throughput, not latency. A background process (e.g., `systemd-journald`) can preempt your Python script, causing **Jitter** ($\Delta t_{actual} - \Delta t_{target}$).
*   **The Math:** If your control loop runs at $50\text{Hz}$ ($\Delta t = 20\text{ms}$), and your computation takes $5\text{ms}$, but jitter adds $16\text{ms}$, you have missed your window. This causes "Phase Lag" in the controller, leading to instability.
*   **The Solution:** We utilize the `PREEMPT_RT` patch logic or set high process priority (`nice -20`) and CPU affinity (`taskset`) to isolate the flight loop. We implement **Watchdog Timers**: a hardware-level counter that triggers a processor reset or "Emergency Land" if the software fails to "pet" the watchdog within the allocated $20\text{ms}$ window.

#### **2. Discrete Fourier Analysis & The Physics of Vibration**
The motors on a micro-drone spin at $20,000\text{--}40,000\text{ RPM}$, creating high-frequency acoustic noise that propagates through the frame.
*   **The Fast Fourier Transform (FFT):** We use the FFT to decompose the IMU's time-domain signal $x(t)$ into its frequency components $X(f)$. 
*   **Resonance:** If the frame's natural frequency matches the motor's PWM frequency, we get constructive interference.
*   **Digital Filtering:** We implement **Cascaded Notch Filters**. A Notch filter is a band-stop filter with a narrow range, defined by the transfer function $H(s) = \frac{s^2 + \omega_z^2}{s^2 + \frac{\omega_z}{Q}s + \omega_z^2}$. We dynamically center $\omega_z$ on the motor noise peaks identified by the FFT.

#### **3. Network Topology: UDP vs. TCP in Low-Bandwidth Channels**
*   **The OSI Model:** Drone communication happens at the Transport Layer.
*   **TCP (Transmission Control Protocol):** Uses a three-way handshake and cumulative ACKs. If a packet is lost, the "Sliding Window" stops (Head-of-Line Blocking) to retransmit. For a drone, a $500\text{ms}$ retransmission delay results in a physical displacement of several meters—a guaranteed crash.
*   **UDP (User Datagram Protocol):** A connectionless protocol. We treat the state of the drone as "ephemeral." If packet $N$ is lost, we don't care because packet $N+1$ contains the *current* truth. We prioritize **Freshness over Completeness**.

**Next Step:** [Module 1: The Bare Metal API](../Module_01_Bare_METAL_API/Module_01_Lecture.md)

---

## **Professor's Brief: Speaker Notes**

### **The Narrative Hook**
"Imagine trying to perform brain surgery while riding a roller coaster. That is the fundamental challenge of Edge Robotics. We are asking a General Purpose Operating System (Linux) to manage a high-speed physics problem. The success of this entire course hinges on one variable: **Determinism**. If we cannot guarantee that our code runs exactly every 20 milliseconds, the most advanced AI in the world won't save this drone from a wall."

### **Deep Research Context: The "Double-Interrupt" Problem**
In research, we must account for the "Interrupt Latency" of the ARM Cortex-A53 processor. When a hardware interrupt (like a UART byte arrival) occurs, the CPU must stop its current task, save the state, and jump to the Interrupt Service Routine (ISR). On a Pi under heavy load, this "Context Switch" can vary wildly. This is why we avoid heavy libraries like `Pandas` or `Scikit-Learn` in the main flight loop; they create unpredictable memory allocations (Garbage Collection) that destroy our real-time guarantees.

## **Mastery Checklist**
Before moving to the next module, you should be able to:
- [ ] Explain why a BEC is a safety-critical component for the Raspberry Pi.
- [ ] Calculate the frequency of motor vibration given an RPM value.
- [ ] Articulate the difference between Head-of-Line Blocking (TCP) and State-Freshness (UDP).
- [ ] Identify your drone's "Ping Jitter" and explain its impact on control latency.

---

## **Further Reading & Bibliography**

### **Foundational Texts**
*   **Butenhof, D. R. (1997).** *Programming with POSIX Threads.* Addison-Wesley. (Standard reference for real-time concurrency).
*   **Abbott, D. (2006).** *Linux for Embedded and Real-Time Applications.* Newnes. (Deep dive into kernel preemption).

### **Historical Papers**
*   **Nyquist, H. (1928).** *"Certain topics in Telegraph Transmission Theory."* Transactions of the American Institute of Electrical Engineers. (The birth of digital signal sampling).

---

[Previous: Syllabus](../../../../SYLLABUS.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_01_Bare_Metal_API/Module_01_Lecture.md)