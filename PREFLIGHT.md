# PRE-FLIGHT: Zero Hour
**"While you wait for the mail."**

You have ordered the parts. They are in a box somewhere. But you want to start *now*.
This module is designed to be completed on your laptop, today, with **zero hardware**.

---

## **1. The Ground Station (Laptop Setup)**
Your laptop is the "Ground Control Station" (GCS). It needs to be ready.

### **The Toolbox**
1.  **Code Editor:** Install **[VS Code](https://code.visualstudio.com/)**.
    *   *Extension:* Install "Remote - SSH" (You will need this later).
    *   *Extension:* Install "Python" (Microsoft).
2.  **The Brain:** Install **[Python 3.10+](https://www.python.org/)**.
3.  **The Scope:** Install **[PlotJuggler](https://github.com/facontidavide/PlotJuggler)**.
    *   This is your oscilloscope. We use it to see math.
    *   *Windows:* Download the Installer.
    *   *Mac/Linux:* Follow build instructions.

---

## **2. The Mental Model**
**"What are we building?"**

Most drones are "Monolithic" (One chip does everything).
The Squid is "Bi-Cameral" (Two Brains).

### **Brain 1: The Lizard Brain (Flight Controller)**
*   **Hardware:** Betaflight F405 (STM32 Chip).
*   **Job:** "Keep us upright."
*   **Speed:** 8,000Hz (Ultra Fast).
*   **Thinking:** None. It just reacts to gravity.

### **Brain 2: The Cortex (Raspberry Pi Zero 2 W)**
*   **Hardware:** Linux Computer.
*   **Job:** "Where am I? Where should I go?"
*   **Speed:** 50Hz (Thoughtful).
*   **Thinking:** Runs the AI, Vision, and Planning.

**The Handshake:**
The Pi sends "Intentions" (Move Forward). The FC converts "Intentions" into "Electricity" (Motor RPM).

---

## **3. The Simulation (Instant Flight)**
You don't need the drone to fly the code. We use a **Digital Twin**.

### **Step 1: Install the Physics Engine**
We use `gym-pybullet-drones`, the gold standard for RL research.
```bash
pip install gym-pybullet-drones numpy matplotlib
```

### **Step 2: Run "Hello World"**
We have pre-packaged a simulation script for you.
1.  Open your terminal in the `squid_drone` folder.
2.  Run:
    ```bash
    python tools/sim_hello_world.py
    ```
3.  **The Result:** A window should pop up with a quadrotor hovering.
    *   **Keys:** Use `Arrow Keys` (Pitch/Roll) and `W/S` (Throttle) `A/D` (Yaw) to fly.
    *   *Note: Click on the simulation window to focus it.*

### **Step 3: The "Matrix" Test**
While flying in the sim, look at your terminal. You will see numbers streaming.
`Alt: 1.0m | Roll: 0.1 rad | Motor 1: 15000 RPM`
*   **The Insight:** This is exactly what the Raspberry Pi will see. Your code won't know the difference between this Simulation and Reality.

---

## **4. The Math Refresher (Optional)**
If you felt intimidated by the "Just-In-Time" math notes, now is the time to prime your brain.
*   **Vectors:** Watch a 5-minute video on "Dot Products."
*   **Matrices:** Watch a video on "Matrix Multiplication."
*   **Derivatives:** Remember that "Velocity is the Derivative of Position."

---

## **Ready?**
Once your hardware arrives:
1.  Go to **[Module 0: The Build](curriculum/Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Lecture.md)**.
2.  Start soldering.
