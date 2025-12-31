# Pi Zero 2 W Survival Guide
**"Making the most of 512MB RAM and 1GHz."**

The Raspberry Pi Zero 2 W is a miracle of engineering, but it is not a server. For the Squid project, we are pushing it to its absolute limits. Follow these rules to prevent crashes, overheating, and "Ghost" bugs.

---

## 1. Thermal Management (The Heat Problem)
Running an EKF, a Vision node, and an LNN at the same time generates significant heat.
*   **Throttling:** When the CPU hits 80°C, it will automatically drop its speed from 1GHz to 600MHz. Your PID loop will lag, and the drone will crash.
*   **The Fix:**
    *   **Heatsinks:** A small copper or aluminum heatsink is mandatory.
    *   **Airflow:** The Pavo20 ducts provide some airflow, but the Pi is often shielded. Ensure your mount doesn't "blanket" the CPU.
    *   **Monitoring:** Run `vcgencmd measure_temp` regularly during bench tests.

## 2. SD Card Corruption
Linux writes a lot of logs. Sudden power loss (crashes/battery unplug) can corrupt the SD card.
*   **The Symptoms:** The Pi won't boot, or weird "File not found" errors appear.
*   **The Fix:**
    *   **Overlay File System:** Once your code is stable, enable the "Read-Only" overlay in `raspi-config`. This prevents writes to the SD card.
    *   **High Endurance Cards:** Use "Industrial" or "Endurance" rated cards (e.g., SanDisk High Endurance).
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
--- [Return to Course Map](../../COURSE_MAP.md)