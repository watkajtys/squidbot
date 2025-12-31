# Theory Deep Dive 0.1: The Real-Time Budget
**"Latency is a physical force."**

In robotics, a late answer is a wrong answer. If your drone is traveling at 5m/s and your control loop is delayed by 100ms, the drone has moved **0.5 meters** before it even realizes it should have turned.

---

## **1. The "20ms" Law**
Most autonomous drones run their "High-Level" control at **50Hz** (20ms per loop).
Inside that 20ms, the Pi Zero must:
1.  **Read Sensors:** (I2C/UART) -> 2-5ms.
2.  **Estimate State:** (EKF Math) -> 3-7ms.
3.  **Plan Path:** (A* or MPC) -> 5-10ms.
4.  **Send Command:** (UART to FC) -> 1-2ms.

**The Danger:** If the sum of these exceeds 20ms, you "Drop a Frame." The drone will start to oscillate and eventually crash.

---

## **2. Python's "Enemy": The Garbage Collector**
Python is not a "Real-Time" language. Occasionally, it pauses everything to clean up unused memory (**Garbage Collection**). 
*   **The Symptom:** Your loop usually takes 10ms, but every 5 seconds, it takes 45ms.
*   **The Fix:** 
    *   Pre-allocate your arrays (use `numpy`). 
    *   Avoid creating new objects inside the `while True` loop.
    *   Manually trigger `gc.collect()` during "Safe" times (e.g., when the drone is disarmed).

---

## **3. Profiling: Finding the Bottleneck**
You cannot optimize what you have not measured. We use the **Squid Profiler** (a simple wrapper) to measure how long each block of code takes.

```python
import time

start = time.perf_counter()
do_complex_math()
duration = (time.perf_counter() - start) * 1000 # Convert to ms

if duration > 15:
    print(f"WARNING: Math took {duration}ms! Budget exceeded.")
```

---

## **4. Real-Time Priority (`chrt`)**
Linux is a multi-tasking OS. By default, the Pi might decide that a "Background Update" is more important than your Drone script.
*   **The Fix:** We run our script with **Round-Robin Real-Time Priority**.
```bash
sudo chrt -r 99 python3 main_control.py
```
This tells the Linux kernel: "Do not interrupt this script for anything except the most critical system tasks."

**Mental Model:** You are a chef with a 20-minute timer for a dish that takes 19 minutes to cook. You have zero room for distractions.
--- [Return to Course Map](../../../COURSE_MAP.md)