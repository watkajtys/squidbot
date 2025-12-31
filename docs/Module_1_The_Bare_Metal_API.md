# Module 1: The Bare Metal API
**"Hardware is innocent. Software is guilty."**

In this module, you will build the "Nervous System" of the drone. You will write the drivers that talk to the Flight Controller (spinal cord) and the Sensors (eyes/ears). By the end, you will have a drone that can spin its motors in reaction to a sensor reading.

---

## **1.1 The MSP Protocol (Motor Control)**

### **Objective**
The Flight Controller (Betaflight) handles the high-speed electronic speed controllers (ESCs). We need to tell it what to do. We use the **MultiWii Serial Protocol (MSP)**.

### **Theory**
MSP is a binary protocol over UART. It's not JSON. It's not XML. It is raw bytes.
Structure: `[Header] [Size] [Type] [Checksum] [Payload]`

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

### **1.1.1 Sub-Lab: The Logic Analyzer**
**"Trust but Verify."**

Software is a liar. It says it sent a pulse. Did the wire actually go high?
1.  **Hardware:** Connect a Saleae Logic Analyzer (Channel 0) to the Motor 1 Signal Pin.
2.  **Trigger:** Set trigger to "Rising Edge."
3.  **Capture:** Send `50% Throttle`. Measure the pulse width (in microseconds).
4.  **Verify:** Does the width match the DSHOT/PWM spec exactly? Or is your timer clock slightly off?

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
