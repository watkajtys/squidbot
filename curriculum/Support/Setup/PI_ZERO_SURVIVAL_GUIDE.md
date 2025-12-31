# Pi Zero 2 W Survival Guide
**"Making the most of 512MB RAM and 1GHz."**

The Raspberry Pi Zero 2 W is a miracle of engineering, but it is not a server. For the Squid project, we are pushing it to its absolute limits. Follow these rules to prevent crashes, overheating, and "Ghost" bugs.

---

## 0. Turbo Mode (Safe Optimization)
Before you write a single line of code, let's unlock the free performance hidden in the silicon.
1.  **Edit Config:** `sudo nano /boot/config.txt`
2.  **Add/Change Lines:**
    ```ini
    # Safe Overclock (Most Zero 2 Ws can handle this)
    arm_freq=1200
    over_voltage=2
    
    # Free up RAM (We run Headless, so we don't need 64MB for GPU)
    gpu_mem=16
    ```
3.  **Reboot:** `sudo reboot`
4.  **Verify:** Run `vcgencmd measure_clock arm`. You should see `1200000000`.
    *   *Note: If it crashes, lower freq to 1100.*

---

## 1. Thermal Management (The Heat Problem)
Running an EKF, a Vision node, and an LNN at the same time generates significant heat.
*   **Throttling:** When the CPU hits 80°C, it will automatically drop its speed from 1GHz to 600MHz. Your PID loop will lag, and the drone will crash.
*   **The Fix:**
    *   **Heatsinks:** A small copper or aluminum heatsink is mandatory.
    *   **Airflow:** The Pavo20 ducts provide some airflow, but the Pi is often shielded. Ensure your mount doesn't "blanket" the CPU.
    *   **Monitoring:** Run `vcgencmd measure_temp` regularly during bench tests.

## 2. SD Card Corruption & Speed
Linux writes a lot of logs. Sudden power loss (crashes/battery unplug) can corrupt the SD card.
*   **The Speed Trap:** A slow SD card causes the entire OS to "freeze" while writing a log file. This adds 100ms of latency to your control loop.
*   **The Fix:**
    *   **Buy the Right Card:** Look for the **"A2"** symbol (Application Performance Class 2) or **"U3"**. Do not use generic Class 10 cards.
    *   **Overlay File System:** Once your code is stable, enable the "Read-Only" overlay in `raspi-config`.
    *   **High Endurance:** Use "Industrial" cards (e.g., SanDisk High Endurance) to survive thousands of log writes.
    *   **Power Off Safely:** Always run `sudo shutdown -h now` when on the bench.

## 3. Memory Optimization (The 512MB Wall)
512MB is tiny. Python is memory-heavy.
*   **Disable the Desktop:** Always boot to CLI (Console) mode.
*   **Swap Space:** Do NOT increase swap space on the SD card (it is too slow). Instead, keep your memory footprint small.
*   **Shared Memory:** ROS 2 uses shared memory for transport. If you run out of RAM, nodes will simply stop talking to each other.

## 4. Power Stability (The "Brownout")
The Pi Zero 2 W draws ~1.5A to 2A at peak load.
*   **The Symptom:** Random reboots during high-throttle maneuvers.
*   **The Cause:** Voltage sag. When the motors pull 40A, the battery voltage drops, and a cheap 5V regulator might drop to 4.5V. The Pi will reboot.
*   **The Fix:** Use a high-quality BEC (Battery Eliminator Circuit) rated for at least 3A constant. Add a 470uF capacitor to the 5V rail if reboots persist.

## 5. Wireless Latency (The SSH Lag)
2.4GHz Wi-Fi is crowded.
*   **The Problem:** Your SSH terminal lags right when you need to send a 'Kill' command.
*   **The Fix:**
    *   **Antenna Mod:** (Advanced) Solder a small u.FL connector for an external antenna.
    *   **Power Management:** Run `sudo iw dev wlan0 set power_save off` to prevent the Wi-Fi from "sleeping" mid-flight.
--- [Return to Course Map](../../../COURSE_MAP.md)