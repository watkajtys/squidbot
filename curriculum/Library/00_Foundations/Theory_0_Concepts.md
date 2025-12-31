# Module 0 Handbook: Signal Processing & Foundations
**"The Robot is a Lie. Reality is Noise."**

This handbook covers the theoretical prerequisites for building a stable autonomous drone. Before you can control the drone, you must filter the world.

---

## **1. The Physics of Vibration (Frequency Domain)**

### **The Problem**
Your Pavo20 has spinning motors. Motors vibrate.
If your motors spin at 20,000 RPM, they create a vibration at:
`20,000 / 60 = 333 Hz`

If your IMU (Gyroscope) listens to this vibration, it will think the drone is "shaking" 333 times a second. Your PID controller will try to fight this, causing the motors to twitch, which creates *more* vibration. This is a positive feedback loop (Resonance) that melts motors.

### **The Solution: The Frequency Domain**
We cannot see this noise in a time-graph. We must convert Time -> Frequency using the **Fast Fourier Transform (FFT)**.

#### **Python Lab: The FFT**
You will eventually run this on flight logs. Here is the concept:

```python
import numpy as np
import matplotlib.pyplot as plt

# Simulate a drone hovering (DC signal) + Motor Noise (High Freq)
t = np.linspace(0, 1, 1000)
signal = 0.0 + np.sin(2 * np.pi * 333 * t)  # 333Hz noise

# Compute FFT
fft_vals = np.fft.fft(signal)
freqs = np.fft.fftfreq(len(signal))

# The graph will show a massive spike at 333Hz.
# YOUR JOB: Configure the Flight Controller's "Notch Filter" to kill 333Hz.
```

### **Sampling Theorem (Nyquist)**
To see a 333Hz vibration, you must sample at least **2x** that speed (666Hz).
*   **Pavo20 Gyro Rate:** Usually 3.2kHz or 8kHz.
*   **Result:** We are safe. We can see the noise.

---

## **2. Camera Models & Calibration**

### **The Problem**
Your Arducam has a lens. Real lenses are not perfect. They curve light like a fishbowl ("Radial Distortion").
If the drone sees a straight wall as curved, it will try to fly "around" the curve and crash into the wall.

### **The Math: Pinhole Camera Model**
We model the camera as a matrix $K$:
$$
\begin{bmatrix}
 f_x & 0 & c_x \\
 0 & f_y & c_y \\
 0 & 0 & 1
\end{bmatrix}
$$
*   $f_x, f_y$: Focal Length (Zoom level).
*   $c_x, c_y$: Optical Center (Where is the middle of the image?).

### **The Fix: Distortion Coefficients**
We must find the parameters $k_1, k_2, k_3$ that describe the curve.
You will use OpenCV to "undistort" the image.

```python
import cv2
# You will take 10 photos of a checkerboard.
# OpenCV finds the corners and solves the math for you.
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
```

---

## **3. Real-Time Systems & Latency**

### **The Problem**
You send a command: "Stop!"
*   Wi-Fi takes 10ms.
*   OS Kernel takes 2ms.
*   Python takes 5ms.
*   **Total:** 17ms.

If the drone is flying at 10 m/s, it traveled **17cm** before it even *started* to stop.
Worse, Wi-Fi is "Jittery." Sometimes it takes 10ms, sometimes 500ms.

### **The Solution: Determinism & Watchdogs**
1.  **UDP > TCP:** TCP retries lost packets (Slow). UDP just sends them. If a packet is lost, ignore it. We only care about the *newest* data.
2.  **The Watchdog Timer:**
A safety mechanism that "bites" (kills motors) if it isn't "fed" (received a packet) recently.

**Code Concept:**
```python
last_packet_time = time.time()

while True:
    if time.time() - last_packet_time > 0.5: # 500ms timeout
        EMERGENCY_LAND()
```

---

## **4. Failure Mode & Effects Analysis (FMEA)**

Before we fly, we must list how we die.

| Component | Failure Mode | Symptom | Mitigation |
| :--- | :--- | :--- | :--- |
| **Wi-Fi** | Disconnect | High Latency | Auto-Disarm (Watchdog) |
| **Camera** | Frame Freeze | Old Image Data | Check Timestamp vs System Time |
| **Motor** | Desync | Violent Spin | Vibration Failsafe (Accel > 10G) |
| **Battery** | Voltage Sag | Logic Brownout | **External BEC** (Hardware Fix) |

---

## **5. From Theory to Practice: The MSP Protocol**

### **The Bridge**
You have learned about sampling, latency, and noise. Now, how do we actually spin the motors?

We cannot connect the motors directly to the Raspberry Pi.
*   **Reason:** Linux is not "Real-Time." If Linux decides to run a system update, your motor signal will pause for 100ms, and the drone will flip.

### **The Solution: The Flight Controller (STM32)**
We delegate the fast, dangerous work to the Flight Controller (FC).
1.  **Pi Zero (The Brain):** Decides "Go Up" (High Level).
2.  **MSP Protocol:** The language the Pi uses to tell the FC "Go Up".
3.  **Flight Controller (The Spinal Cord):** Converts "Go Up" into precise microsecond pulses (DSHOT) for the ESCs.

In **Module 1**, you will write the Python driver to speak this language.

---

## **Module 0 Checklist**
Before moving to Module 1, you must:
- [ ] Understand why we need a BEC (Power stability).
- [ ] Be able to explain why we use UDP instead of TCP.
- [ ] Have Python, VS Code, and Docker installed on your Laptop.
- [ ] Have successfully SSH'd into your Raspberry Pi.--- [Return to Course Map](../../../COURSE_MAP.md)