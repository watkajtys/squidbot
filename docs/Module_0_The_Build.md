# Module 0: The Build & Setup
**"Hardware is the body. Linux is the soul."**

Before we write code, we must build the machine. This module covers the physical assembly of the Squid drone and the configuration of its operating system.

---

## **0.1 Anatomy & Power**

### **The Schematics**
We are grafting a "Big Brain" (Raspberry Pi Zero 2 W) onto a "Fast Spinal Cord" (Betaflight Flight Controller).

*   **The Problem:** The Pi needs 5V. The Drone battery is 12V (3S).
*   **The Danger:** If you plug 12V into the Pi, it dies immediately.
*   **The Solution:** The BEC (Battery Eliminator Circuit). It steps 12V down to a clean 5V/3A.

### **Wiring Diagram**
**STOP.** Open [hardware_reference.md](hardware_reference.md) for the exact pinout and soldering guide. Do not guess.

1.  **Power:** Battery Pads $\to$ BEC Input (12V) $\to$ BEC Output (5V) $\to$ Pi GPIO 5V (Pin 2/4) & GND (Pin 6).
2.  **Data:** FC UART TX $\to$ Pi UART RX (GPIO 15).
3.  **Data:** FC UART RX $\to$ Pi UART TX (GPIO 14).
4.  **Ground:** **CRITICAL.** You MUST connect a common Ground (GND) wire between the FC and the Pi.

---

## **0.2 Assembly**

### **The Mount**
1.  Print `hardware/avionics_mount.scad`.
2.  Mount the Pi Zero 2 W using M2.5 nylon standoffs.
3.  Secure the Camera (Arducam) to the front slot.
4.  **Vibration Damping:** Use soft silicone gummies between the mount and the drone frame. This is crucial for Module 4 (Signal Processing).

### **The Camera Ribbon**
*   Thread the CSI ribbon cable carefully. It is fragile.
*   **Orientation:** The blue tape on the cable usually faces *away* from the board on the Pi Zero side, but check your specific connector.

### **The Flight Controller Setup**
Before you close everything up, you must configure the "Spinal Cord" (Betaflight).
1.  **Install:** Betaflight Configurator on your Laptop.
2.  **Connect:** Plug the Flight Controller into your Laptop via USB.
3.  **Ports Tab:**
    *   Find the UART port where you soldered the Pi (usually UART 1 or 2).
    *   Toggle **"MSP"** (Configuration/MSP) to **ON**.
    *   Click **Save and Reboot**.
    *   *Note:* Remember this UART number. You will need it later.

---

## **0.3 Systems: Headless Linux**

### **Objective**
Set up the Pi without a monitor or keyboard.

### **Procedure**
1.  **Flash the OS:**
    *   Download **Raspberry Pi OS Lite (64-bit)**. (Do not use Desktop version; it wastes RAM).
    *   Use "Raspberry Pi Imager".
2.  **Configuring settings (The "Gear" Icon):**
    *   **Hostname:** `squid-drone`.
    *   **User:** `pi` (or your name).
    *   **Wi-Fi:** Enter your router's SSID and Password.
    *   **Enable SSH:** This is mandatory.
3.  **First Boot:**
    *   Insert SD card. Power on.
    *   Wait 2 minutes.
    *   On your laptop terminal: `ssh pi@squid-drone.local`.
    *   **Password:** The one you set in step 2.

### **Troubleshooting: "Could not resolve hostname"**
If you are on Windows and `squid-drone.local` fails:
1.  **Install Bonjour Print Services:** This enables `.local` domain discovery.
2.  **Or, find the IP:**
    *   Log into your WiFi Router's admin page.
    *   Look for a device named `squid-drone` or `raspberrypi`.
    *   Use the IP address: `ssh pi@192.168.1.X` (Replace X with the actual number).

### **Environment Setup**
Once logged in:
```bash
# Update everything
sudo apt update && sudo apt upgrade -y

# Install dependencies for Module 1 & 2
sudo apt install -y python3-pip git i2c-tools python3-smbus

# Enable Hardware Interfaces
sudo raspi-config
# -> Interface Options -> I2C -> Enable
# -> Interface Options -> Serial Port -> Login Shell: NO, Hardware Enabled: YES
```

---

## **0.4 Legal & Safety Compliance**
**"Don't go to jail."**

*   **US (FAA):**
    *   **Registration:** If >250g (Squid is ~180g, so likely exempt, but check battery weight).
    *   **TRUST:** You *must* pass the "The Recreational UAS Safety Test" (Free online).
    *   **Remote ID:** Required for >250g or Part 107.
*   **EU (EASA):**
    *   **Class C0:** <250g. Read user manual.
    *   **Insurance:** Often mandatory even for toy drones if they have a camera.
*   **The Golden Rules:**
    *   Visual Line of Sight (VLOS) only.
    *   < 400ft AGL.
    *   Never over people.

---

## **Check: The Smoke Test**

**Stop. Do not skip this.**

1.  **Continuity Check:** Use a multimeter. Measure resistance between 5V and GND on the Pi. It should NOT be 0 (Short circuit).
2.  **The Power Up:**
    *   Plug in the battery.
    *   **Look:** For Magic Smoke.
    *   **Smell:** For burning silicon.
    *   **Listen:** The Flight Controller should beep (ESC initialization).
3.  **The Ping:**
    *   Open your laptop terminal.
    *   `ping squid-drone.local`
    *   If you see replies, your drone is alive.

---

## **0.5 Hands-On: Bench Testing**

Before you zip-tie the Pi to the frame and tuck away the wires, you must perform your "Bench Drills." It is 10x easier to fix a soldering error or a network issue while the components are spread out on your desk than when they are trapped inside the drone.

**Action:** Complete all exercises in **[Module 0 Labs](Module_0_Labs.md)**.

Do not proceed to Module 1 until your "Smoke Test" and "Ping Test" are both passing.

---

## **Deliverable**
*   A photo of your wiring (specifically the UART and BEC connections).
*   A screenshot of your terminal showing `pi@squid-drone:~ $` prompt.
