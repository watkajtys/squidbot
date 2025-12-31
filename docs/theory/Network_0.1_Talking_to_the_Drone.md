# Mini-Lecture 0.1: Network Survival Guide
**"How to fix code you can't touch."**

Your drone has no screen and no keyboard. You control it through a "Headless" connection. If the network fails, you are blind.

---

## **1. The IP Address (The Drone's House Number)**
When your Pi connects to Wi-Fi, the router gives it an IP (e.g., `192.168.1.15`).
*   **The Problem:** Every time the Pi reboots, the router might give it a *new* number. You will be trying to SSH into `.15` while the drone is now at `.22`.
*   **The Fix:** Use `.local` (mDNS). The Pi broadcasts its name. `ssh pi@squid-drone.local` will find the drone even if the IP changes.

---

## **2. SSH: The Remote Brain**
SSH (Secure Shell) allows your laptop terminal to *become* the Pi's terminal.
*   Anything you type in an SSH window is happening **on the drone**.
*   **Pro Tip:** Use **VS Code Remote SSH** extension. It allows you to edit files on the Pi as if they were on your own computer.

---

## **3. Latency & Jitter (The Drone's Nemesis)**
Wi-Fi is made of radio waves. Radio waves can be blocked by walls or interfered with by microwaves.
*   **Latency:** The delay (e.g., 50ms).
*   **Jitter:** The *change* in delay. (5ms, then 200ms, then 10ms).

**Why this kills drones:**
If you are sending "Move Left" commands over Wi-Fi, and a 500ms jitter spike happens, the drone will keep moving left for half a second longer than you intended. **CRASH.**

---

## **4. The "Ping" Test**
If the drone feels "sluggish," run this on your laptop:
`ping squid-drone.local`
*   If `time=` is > 100ms, do not fly.
*   If `packet loss` is > 0%, do not fly.

**Environment Tip:** Always fly your drone in the same room as your Wi-Fi router for the best connection.
