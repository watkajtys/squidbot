# Module 1: The Bare Metal API
**"Hardware is innocent. Software is guilty."**

In this module, you will build the "Nervous System" of the drone. You will write the drivers that talk to the Flight Controller (spinal cord) and the Sensors (eyes/ears).

---

## **1.0 Lab: The Heartbeat (Your First Success)**

### **Objective**
Blink an LED without buying an LED.

### **Theory**
The Raspberry Pi Zero 2 W has a tiny green LED onboard labeled `ACT`. We can control this through the Linux filesystem. In Linux, "Everything is a file," including hardware.

### **Lab Procedure**
1.  **SSH into your Pi.**
2.  **Create `heartbeat.py`**:
```python
import time
import os

# Path to the onboard LED
LED_PATH = "/sys/class/leds/ACT/brightness"

def set_led(state):
    with open(LED_PATH, "w") as f:
        f.write(str(state))

print("Starting Heartbeat... Press Ctrl+C to stop.")
try:
    while True:
        set_led(1) # ON
        time.sleep(0.5)
        set_led(0) # OFF
        time.sleep(0.5)
except KeyboardInterrupt:
    set_led(0)
    print("Stopped.")
```
3.  **Run it:** `sudo python3 heartbeat.py` (Note: Accessing hardware files usually requires `sudo`).

**Success:** If the green light on your Pi is blinking, your software environment is working perfectly.

---

## **1.1 The MSP Protocol (Motor Control)**

### **Objective**
The Flight Controller (Betaflight) handles the high-speed electronic speed controllers (ESCs). We need to tell it what to do. We use the **MultiWii Serial Protocol (MSP)**.

### **Theory**
MSP is a binary protocol over UART. It's not JSON. It's not XML. It is raw bytes.
Structure: `[Header] [Size] [Type] [Checksum] [Payload]`

#### **Concept: The Chain of Command**
Wait—doesn't Betaflight already have an autopilot?
**Yes, but we are hijacking it.**
*   **Betaflight:** Acts as the **Actuator Server**. It handles the high-speed math (DShot) to spin motors.
*   **The Pi (Your Code):** Acts as the **Commander**. It sends a list of 4 numbers (Motor 1, 2, 3, 4) to Betaflight via MSP.

Think of Betaflight as a "translator" that turns your Python numbers into real-world electricity.

### **Lab Procedure**
1.  **Connect UART:** Connect the Pi Zero's UART pins (TX/RX) to the Flight Controller's UART pads.
2.  **Configure Betaflight:** Enable "MSP" on that UART port in the Ports tab.
3.  **The Driver:**
    *   Create `src/drivers/msp.py`.
    *   Implement `send_motor_command(motor_values)`.
    *   **Challenge:** Calculate the Checksum (XOR of size, type, and payload).

```python
# Pseudo-code for MSP Checksum
def calculate_checksum(data):
    xor_sum = 0
    for byte in data:
        xor_sum ^= byte
    return xor_sum
```

### **Deliverable**
*   A script `spin_test.py` that spins Motor 1 at 5% throttle for 2 seconds.
*   **Safety:** PROPS OFF.

### **1.1.1 Sub-Lab: The UART Sanity Check**
**"Is anybody out there?"**

Before we try to spin motors, let's see if the Flight Controller can hear us. We will send a request for the **MSP API Version** (Command Type 1).

1.  **Code:** Create `src/drivers/check_fc.py`.
2.  **Logic:**
    *   Send the byte sequence for `MSP_API_VERSION`: `$M<` (Header) + `\x00` (Size) + `\x01` (Type 1) + `\x01` (Checksum).
    *   Read 6 bytes back.
3.  **The Test:**
    *   Run `python3 check_fc.py`.
    *   **Success:** You see `MSP API Version: 1.XX`. This proves your TX, RX, and Ground wires are soldered correctly.

### **1.1.2 Sub-Lab: The MSP Hex Dump**
**"Decoding the Matrix."**

Computers don't send "numbers"; they send voltage pulses that we interpret as bytes. If your driver isn't working, you need to see the raw data.

1.  **Action:** Modify `check_fc.py` to print the raw response.
```python
response = ser.read(6)
print(f"Raw Bytes: {[hex(b) for b in response]}")
```

#### **The Checksum Challenge**
**"Don't trust the library, trust the math."**
MSP uses an XOR checksum. The last byte of every packet is the XOR result of the `Size`, `Type`, and `Payload`.

1.  **Capture:** Find a 6-byte packet in your hex dump (e.g., `0x24 0x4d 0x3e 0x00 0x01 0x01`).
2.  **Calculate:** In a Python terminal, run: `print(0x00 ^ 0x01)`.
3.  **Verify:** Does it match the last byte (`0x01`)?
4.  **Experiment:** Unplug a wire mid-transmission. Does the checksum still match? (This is how your code knows to ignore "garbage" data).

---

## **1.2 I2C Drivers & The Address Conflict**

### **Objective**
We have two VL53L1X Time-of-Flight sensors (Down + Up).
**The Problem:** They both ship with the *same* default I2C address (0x29). If you plug them both in, the bus crashes.

### **The Solution: XSHUT Pins**
The VL53L1X has an `XSHUT` (Shutdown) pin. We can turn one off, change the address of the other, and then turn the first one back on.

### **Lab Procedure**
1.  **Wiring:** Connect the XSHUT pins to the Pi as per [hardware_reference.md](hardware_reference.md).
    *   Up Lidar XSHUT $\to$ **GPIO 17**.
    *   Down Lidar XSHUT $\to$ **GPIO 27**.
2.  **The Sequence:**
    *   Pull both XSHUT low (Turn both OFF).
    *   Pull Sensor A XSHUT high (Turn A ON).
    *   Write to register `I2C_SLAVE_DEVICE_ADDRESS` on Sensor A to change it to `0x30`.
    *   Pull Sensor B XSHUT high (Turn B ON). It is still at `0x29`.
3.  **Verification:** Run `i2cdetect -y 1`. You should see `0x29` and `0x30`.

### **Deliverable**
*   `src/drivers/tof_array.py` which initializes both sensors and returns two distances.

### **1.2.1 Sub-Lab: The I2C Scanner**
**"Who is on the bus?"**

Before writing a Python driver, use the Linux system tools to "scan" the wires.
1.  **Command:** `i2cdetect -y 1`.
2.  **Identify:** 
    *   If you see `29`, your Lidar is connected.
    *   If you see `empty`, your wiring (SDA/SCL) is swapped or loose.
3.  **Python Version:** Write `src/drivers/i2c_scan.py` using `smbus2` to try and write a "0" to address 0x29. If it doesn't crash, the sensor is alive.

---

## **1.3 The Game Loop**

### **Objective**
Robots don't wait. They loop. We need a main loop that runs at a consistent frequency (e.g., 50Hz).

### **Theory**
`time.sleep(0.02)` is not accurate enough. Linux is not a Real-Time OS. We need to measure "dt" (delta time) dynamically.

### **Lab Procedure**
1.  Create `src/main.py`.
2.  Implement a `while True` loop.
3.  Measure the time start at the top of the loop.
4.  Do work (Read Sensors -> Calc Logic -> Write Motors).
5.  Sleep for `TARGET_DT - (now - start)`.
6.  **Log Jitter:** Print how much the actual dt deviates from the target.

---

## **Check: The Reflex**
**The "Hello World" of Robotics.**

Combine 1.1, 1.2, and 1.3.
1.  Read the Downward Lidar distance.
2.  **Logic:**
    *   If `distance < 0.2m`: Motors OFF.
    *   If `distance > 0.2m` and `distance < 0.5m`: Motors IDLE (spin slow).
    *   If `distance > 0.5m`: Motors PULSE (visual feedback).

**Goal:** Move your hand in front of the sensor. The motors should react instantly.
