[Return to Course Map](../../../../COURSE_MAP.md) | [Phase I Index](../../Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Lecture.md)

---

# SQUID DRONE: EMERGENCY QUICK REFERENCE
=======================================

1. HARDWARE PINOUT (GPIO)
-------------------------
[Pi Pin] | [Function] | [Connects To]
---------|------------|--------------
02       | 5V Power   | BEC Output (+)
06       | Ground     | BEC Output (-) / FC Ground
03       | SDA (I2C)  | Lidar Bus (Shared)
05       | SCL (I2C)  | Lidar Bus (Shared)
14       | TX (UART)  | FC RX Pad
15       | RX (UART)  | FC TX Pad
17       | GPIO 17    | Front Lidar XSHUT
27       | GPIO 27    | Down Lidar XSHUT

2. MSP PROTOCOL (BETAFLIGHT)
----------------------------
- Header:  $M<  (0x24 0x4d 0x3c)
- API Ver: Type 1
- Set Mot: Type 214 (MSP_SET_MOTOR)
- Range:   1000 (Idle) to 2000 (Full Throttle)
- Arming:  AUX 1 (Usually > 1500)

3. I2C SENSORS (VL53L1X)
-----------------------
- Default Addr: 0x29
- Secondary:    0x30 (Assigned in Lab 1.2)
- Command:      i2cdetect -y 1

4. MATH CONVERSIONS
-------------------
- Degrees to Radians: deg * 0.01745
- Radians to Degrees: rad * 57.295
- Loop DT (50Hz):     0.02 seconds
- Loop DT (400Hz):    0.0025 seconds
- G to m/s^2:         9.81

5. TERMINAL COMMANDS
--------------------
- Resource Check: htop
- Performance:    echo "performance" | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
- Run VIP Core:   taskset -c 3 python3 your_script.py
- Log Reader:     tail -f flight_log.csv | column -t -s,

6. SAFETY CHECKLIST
-------------------
- PROPS OFF? (Bench test only)
- SMOKE STOPPER PLUGGED IN?
- VOLTAGE > 3.5V per cell? (3S = 10.5V)
- CAPACITOR POLARITY CORRECT? (Stripe to GND)
=======================================
