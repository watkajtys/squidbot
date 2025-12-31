# Hardware Reference & Pinout
**"The Map of the Machine."**

This document is the Single Source of Truth for wiring your Squid Drone. Use this when connecting the Raspberry Pi Zero 2 W to the Flight Controller and Sensors.

---

## **1. Raspberry Pi Zero 2 W GPIO Header**
We use the standard 40-pin header.

**WARNING: 3.3V LOGIC ONLY.**
The Raspberry Pi GPIO pins are **3.3V tolerant**.
*   **Do NOT** connect 5V logic signals (like from an Arduino Uno) directly to the Pi. You will fry the CPU.
*   **The Good News:** Modern Flight Controllers (STM32 F405) also use 3.3V logic, so direct wire connection is safe.

| Pin # | Function | Description | Connected To |
| :--- | :--- | :--- | :--- |
| **01** | 3.3V | Power | (Unused) |
| **02** | **5V** | **Power Input** | **BEC 5V Output** (Critical) |
| **03** | **SDA** | I2C Data | VL53L1X Lidar (Up & Down) |
| **04** | 5V | Power Input | (Alternative Power) |
| **05** | **SCL** | I2C Clock | VL53L1X Lidar (Up & Down) |
| **06** | **GND** | **Ground** | **BEC GND / FC GND** (Common Ground) |
| **...**| ... | ... | ... |
| **14** | **TX** | UART Transmit | Flight Controller **RX** Pad |
| **15** | **RX** | UART Receive | Flight Controller **TX** Pad |
| **...**| ... | ... | ... |
| **17** | **GPIO 17** | Digital Out | **Up Lidar XSHUT** |
| **27** | **GPIO 27** | Digital Out | **Down Lidar XSHUT** |

---

## **2. Flight Controller (Pavo20 / F405 AIO)**
*Note: Pad locations vary by manufacturer. Check your specific wiring diagram.*

| Pad Name | Function | Connected To | Betaflight Config |
| :--- | :--- | :--- | :--- |
| **TX1 / T1** | UART 1 TX | Pi **RX** (Pin 15) | Port 1: MSP ON |
| **RX1 / R1** | UART 1 RX | Pi **TX** (Pin 14) | Port 1: MSP ON |
| **GND** | Ground | Pi **GND** (Pin 6) | - |
| **BAT+** | Battery Voltage | BEC Input (+) | - |
| **BAT-** | Battery Ground | BEC Input (-) | - |

---

## **3. Sensor Array Wiring**

### **VL53L1X Lidars (x2)**
These sensors share the I2C bus (SDA/SCL). To prevent address conflicts, we use the XSHUT pins.

*   **Up Lidar:**
    *   VCC -> 3.3V (Pi Pin 1)
    *   GND -> GND
    *   SDA -> SDA (Pi Pin 3)
    *   SCL -> SCL (Pi Pin 5)
    *   **XSHUT -> GPIO 17**
*   **Down Lidar:**
    *   VCC -> 3.3V (Pi Pin 1)
    *   GND -> GND
    *   SDA -> SDA (Pi Pin 3)
    *   SCL -> SCL (Pi Pin 5)
    *   **XSHUT -> GPIO 27**

### **Arducam**
*   **Ribbon Cable:** Connects to the CSI Camera Port on the Pi.
*   **Note:** Ensure the "silver" contacts on the cable face the correct way (usually towards the board on the Pi Zero, but check the connector latch).

---

## **4. Power Distribution**
*   **Battery (3S LiPo):** Provides 11.1V - 12.6V.
*   **BEC (Regulator):**
    *   **Input:** Soldered to the main battery pads on the Flight Controller.
    *   **Output:** 5V / 3A wired to Pi Pins 2 & 6.
*   **Warning:** NEVER plug a USB cable into the Pi's power port *while* the drone battery is plugged in. You might back-feed power and damage your laptop or the regulator. **Data USB is safe if the 5V rail is isolated, but be careful.**
--- [Return to Course Map](../../../COURSE_MAP.md)