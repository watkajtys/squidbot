[Previous Module](../../Phase_1_The_Mechanic/Module_01_Bare_Metal_API/Module_01_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_03_FPV_and_HUD/Module_03_Lecture.md)

---

# Module 2: The Telemetry Stack
**"If you can't see it, you can't fix it."**

In Module 1, the drone was a black box. Now, we will make it transparent. You will build a real-time dashboard to visualize the drone's "thoughts" and a black-box recorder to analyze its failures.

---

## **2.1 Networking: UDP Sockets**

### **Objective**
Stream sensor data from the Drone (Pi Zero) to your Laptop (Ground Station) over Wi-Fi.

### **Theory**
*   **TCP:** "Did you get that? No? I'll resend." (Too slow for flight).
*   **UDP:** "Here's data! Here's data! Here's data!" (Fast, low latency).
We use UDP. If a packet drops, we don't care. The next one is coming in 20ms.

### **Lab Procedure**
1.  **Drone Side (`src/comms/transmitter.py`):**
    *   Pack data into a JSON string or binary struct.
    *   Send via `socket.sendto()` to your Laptop's IP.
2.  **Laptop Side (`ground_station/receiver.py`):**
    *   Bind a socket to `0.0.0.0` and a specific port (e.g., 5005).
    *   Loop and print received data.

### **Deliverable**
*   A script that streams the Lidar distance from Module 1 to your laptop terminal at 50Hz.

### **2.1.1 Sub-Lab: The Packet Loss Simulation**
**"When Wi-Fi goes bad."**

In the lab, Wi-Fi is perfect. In the field, it is not. You must know how your drone handles "Gaps" and "Jumps" in data.

1.  **Drop Test:** Modify your `transmitter.py` to intentionally "drop" 10% of packets.
2.  **Jitter Test:** Add a random delay to your loop:
```python
import time, random
time.sleep(random.uniform(0, 0.05)) # Delay up to 50ms
```
3.  **Observe:** Look at your dashboard. 
    *   **The Problem:** The data looks "jerky." If this was a control signal, the drone would "twitch" because it's reacting to stale information.
    *   **The PhD Lesson:** We use **Circular Buffers** and **Time-Alignment** (Interpolation) to smooth out this jitter before the math sees it.

### **2.1.2 Just-In-Time Logic: The Translator (Serialization)**
**"Talking Fast vs. Talking Clear"**

In the lab, we send data as text: `"lidar:1.23"`.
*   **The Good:** You can read it with your eyes.
*   **The Bad:** The computer has to turn the number `1.23` into the characters `'1'`, `'.'`, `'2'`, `'3'`. This costs CPU.
*   **The Pro Way (Binary):** Later in ROS 2, we will send the raw 32-bit float (4 bytes). It looks like garbage to humans (`@^#&`) but it is instant for computers.
*   **The Analogy:** Text is like reading a speech. Binary is like telepathy.

**AI Prompt:** "Explain the difference between ASCII serialization (JSON) and Binary serialization (Struct/Protobuf). Write a Python snippet to pack two floats into a binary byte array."

---

## **2.2 Just-In-Time Tooling: PlotJuggler**

### **Objective**
Don't build a dashboard. Build a stream.

### **The Solution**
**PlotJuggler** is the "Gold Standard" for open-source time-series analysis. It is powerful, lightweight, and requires **zero** frontend code.

### **Lab Procedure**
1.  **The Stream:** Modify `src/comms/transmitter.py`.
    *   Instead of complex JSON, we will send a simple "Key/Value" UDP packet.
    *   Example: `timestamp:1234,lidar:0.5,accel_z:9.81`
2.  **The UI:**
    *   Install **PlotJuggler** (Windows/Mac/Linux).
    *   Open the "UDP Server" plugin (Port 9870).
    *   **The Win:** Click "Start." Drag `lidar` into the center area.
    *   **The "Aha!" Moment:** Right-click the graph -> "Apply Function" -> "Derivative." You are now seeing the drone's **Velocity** calculated in real-time from the Lidar distance.

### **Deliverable**
*   A screenshot of PlotJuggler showing both your raw Lidar data and its calculated Derivative (Velocity) on the same graph.

---

## **2.3 The Logger**

### **Objective**
Wi-Fi is not perfect. It cuts out. We need onboard logging to debug crashes that happen when the dashboard is frozen.

### **Theory**
Writing to the SD card is slow (blocking I/O). If we write every loop, the drone will stutter.
**Solution:** The Buffer.
Store 1000 lines in RAM, then dump to disk in one chunk (or use a separate thread).

### **Lab Procedure**
1.  Create `src/utils/logger.py`.
2.  Format: `Timestamp, Lidar_Dist, Motor_Out, Loop_Time`.
3.  Implement a `flush()` method that writes the buffer to a CSV file.
4.  Call `flush()` only when the drone is Disarmed (safe) or use a background thread.

### **Deliverable**
*   A `.csv` file generated after a run. Open it in Excel/Pandas and plot the data.

### **2.3.1 Sub-Lab: CSV to Plot**
**"The Autopsy."**

When the drone crashes, the log is the only witness.

1.  **Code:** Create `tools/plot_log.py`.
2.  **Logic:**
    *   Use `pandas` to read your `flight_log.csv`.
    *   Use `matplotlib` to plot `Altitude` vs `Time`.
3.  **Analysis:** Look for "Spikes" in the loop time. If your loop time jumps from 20ms to 100ms, your code is "blocking" (probably waiting for a slow sensor).

---

## **Check: The Flight Recorder**
**Forensics 101.**

1.  Run the "Reflex" logic from Module 1.
2.  Wave your hand rapidly to stress the sensor.
3.  Stop the code.
4.  Pull the CSV logs.
5.  **Analysis:**
    *   Plot `Timestamp` vs `Lidar_Dist`.
    *   Calculate the **Standard Deviation** of the loop time. Is it stable?
    *   Did the motor command lag behind the sensor reading? (Calculate the delay).

**Submission:** A screenshot of your plot and the calculated "System Latency" (ms).

---
## **Theoretical Foundations**

### Lecture 2: Stochastic Processes & Observability

#### **1. The Gaussian Distribution (Thermal Noise)**
Electronic sensors (IMUs, Lidars) are subject to **Johnson-Nyquist Noise**. This white noise is modeled as a **Gaussian (Normal) Distribution** $\mathcal{N}(\mu, \sigma^2)$.
*   **The Mean ($\mu$):** Represents the "true" state of the system plus any static bias.
*   **The Variance ($\sigma^2$):** Quantifies the uncertainty. 
*   **Central Limit Theorem:** Even if individual noise sources are not Gaussian, the sum of many independent noise sources (like those in a drone frame) tends toward a Gaussian distribution. This assumption allows us to use **Least Squares** optimization in Phase IV.

#### **2. Random Walks & Integration Drift**
When we integrate a noisy signal (Acceleration $\to$ Velocity $\to$ Position), we create a **Random Walk**.
*   **Brownian Motion:** If the noise is white, the variance of the integrated signal grows linearly with time: $\sigma_{pos}^2(t) \propto t$. 
*   **Angle Random Walk (ARW):** In a Gyroscope, a $0.1^{\circ}/s$ noise bias doesn't just jitter; it causes the drone's "Belief" of its angle to drift away from reality. Without an absolute reference, the drone will eventually think "Up" is "Sideways."

#### **3. Allan Variance: The Fingerprint of Chaos**
To build a professional EKF (Module 7), we must know the exact noise characteristics of our specific hardware. We use **Allan Variance (AVAR)**.
*   **Formula:** $\sigma^2(\tau) = \frac{1}{2(N-1)} \sum_{i=1}^{N-1} (\bar{y}_{i+1} - \bar{y}_i)^2$. 
*   By calculating variance over different "time bins" ($\tau$), we can separate **White Noise** (High frequency) from **Bias Instability** (Slow, temperature-related drift). This allows us to "tune" our filters to ignore the jitter but track the drift.

**Next Step:** [Module 3: FPV & HUD](../Module_03_FPV_and_HUD/Module_03_Lecture.md)

---

## **Professor's Brief: Speaker Notes**

### **The Narrative Hook**
"In the old days of flight testing, pilots used their 'seat-of-the-pants' intuition. In the age of autonomy, the robot doesn't have pants. It only has data. If you can't graph it, it didn't happen. Telemetry is the window into the robot's soul. Today, you aren't just a coder; you are a data archeologist, digging through the noise to find the physical truth of the flight."

### **Deep Research Context: The Serialization Bottleneck**
In research-grade telemetry, we avoid JSON. Converting a Python dictionary to a string takes significant CPU cycles (Marshalling). High-performance systems like the Mars Ingenuity helicopter use **CCSDS standards** or **Protocol Buffers (Protobuf)**. We use UDP because we care about the **Markov Property**: the future depends only on the *present* state, not the past. Retransmitting a lost packet from 100ms ago is mathematically useless for a flight controller.

## **Mastery Checklist**
Before moving to the next module, you should be able to:
- [ ] Explain the relationship between Mean, Variance, and Gaussian Noise in electronic sensors.
- [ ] Describe how "Integration Drift" turns a small Gyroscope bias into a large attitude error.
- [ ] Interpret an Allan Deviation (ADEV) plot to find the Noise Floor of a sensor.
- [ ] List the three types of IMU noise (White, Bias Instability, Rate Random Walk).

---

## **Further Reading & Bibliography**

### **Signal Theory**
*   **Shannon, C. E. (1948).** *"A Mathematical Theory of Communication."* Bell System Technical Journal. (The foundation of all telemetry).
*   **Allan, D. W. (1966).** *"Statistics of atomic frequency standards."* Proceedings of the IEEE. (The original AVAR paper).

### **Estimation Foundations**
*   **Bar-Shalom, Y., et al. (2001).** *Estimation with Applications to Tracking and Navigation.* Wiley. (Deep context for stochastic noise modeling).

---

[Previous Module](../../Phase_1_The_Mechanic/Module_01_Bare_Metal_API/Module_01_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_03_FPV_and_HUD/Module_03_Lecture.md)