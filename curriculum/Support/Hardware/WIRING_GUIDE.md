# The Hacker's Guide to Wiring
**"Soldering is just hot glue for metal."**

You admitted you don't know what a "Wiring Harness" is. That's good. It means you haven't learned bad habits yet.

---

## 1. What is a "Wiring Harness"?
If you run individual loose wires from your Pi to your Flight Controller, they will:
1.  Vibrate loose.
2.  Get sliced by the propellers.
3.  Act as antennas, picking up radio noise that crashes your drone.

A **Harness** is simply those same wires, but **Measured, Twisted, and Bundled** into a single cohesive unit. It looks like a "cable" rather than a "rat's nest."

---

## 2. The Golden Rules of Drone Wiring

### Rule #1: Silicone vs. PVC
*   **PVC Wire (Bad):** Stiff plastic insulation. If you bend it, it springs back. It transmits vibration to your sensors. It melts if you touch it with the soldering iron. **Do not use this.**
*   **Silicone Wire (Good):** Soft, rubbery insulation. It flops like a cooked noodle. It is heat resistant (up to 400°C). It absorbs vibration.
    *   **Size:** Use **30 AWG** (very thin) for Signals (UART/I2C). Use **22-26 AWG** (thicker) for Power (BEC to Pi).
    *   *Note:* The Pi Zero draws ~1 Amp. **26 AWG** handles 2.2 Amps, so it is plenty safe. **22 AWG** is just extra insurance. Do NOT use 30 AWG for power.

### Rule #2: The "Twist" (Interference Rejection)
*   **The Problem:** Every wire carrying current creates a magnetic field. This field confuses your Compass and adds noise to your Video.
*   **The Fix:** **Twist your wire pairs.**
    *   Twist **5V** and **GND** together.
    *   Twist **TX** and **RX** together.
    *   Twist **SDA** and **SCL** together.
*   **Why:** When twisted, the magnetic fields cancel each other out ($+B$ and $-B$). This is why Ethernet cables are "Twisted Pair."

### Rule #3: Measure Twice, Cut Once (The "Loom" Method)
Don't solder one end and then guess the length.
1.  **Mockup:** Place your Pi and Flight Controller on the desk exactly how they will sit in the frame.
2.  **Route:** Run the wire from Pad A to Pad B, leaving a **10% loop** of slack (stress relief).
3.  **Cut:** Cut the wire.
4.  **Repeat:** Do this for all wires.
5.  **Bundle:** Use tiny zip-ties, dental floss, or "Tesa Tape" (fabric tape) to wrap the bundle.
6.  **Solder:** Now, solder the harness to the boards.

---

## 3. The "Squid" Harness (Step-by-Step)

You need to build **three** distinct mini-harnesses.

### Harness A: The Umbilical (Power)
*   **Wires:** 2 (Red/Black, **24-26 AWG**).
*   **From:** Flight Controller Battery Pads.
*   **To:** BEC Input.
*   **Then:** BEC Output (5V/GND) to Pi GPIO (Pin 2/6).

---

## 4. The Critical Extras (Do Not Skip)

### 4.1 The Capacitor (The "Noise Killer")
**Physics:** Motors create massive voltage spikes (30V+) when braking. These spikes kill video feeds and fry 5V regulators (like the Pi's).
**The Fix:** Solder a **Low ESR Capacitor** (35V, 470uF or 1000uF) to the main battery pads.
*   **Polarity:** The side with the **Stripe** is Negative (-). **Reversing this causes an explosion.**
*   **Mounting:** If space is tight, leave the legs long enough to tuck the capacitor body into the frame, but insulate the legs with heat shrink.

### 4.2 The "Pad Sandwich" (Soldering Technique)
You need to solder THREE things to the main battery pads:
1.  **Bottom:** The Capacitor Legs.
2.  **Middle:** The XT30 Battery Pigtail.
3.  **Top:** The BEC Power Wires.
*   **The Problem:** The Ground pad sucks heat.
*   **The Solution:** Max heat (400°C). Large Tip. Flux. Pre-tin everything. Melt them into one single shiny blob. Do not "cold solder" (dull grey blob).

### 4.3 The "Smoke Stopper" (The Engineer's Fuse)
**Rule:** NEVER plug a battery in for the first time without this.
*   It limits current. If you have a short (e.g., a solder ball bridging 5V/GND), the bulb glows and saves your electronics.
*   If the Smoke Stopper trips (Light off/Bulb Bright), **STOP**. Find the short.

---

## 5. Mechanical Integrity (The "Vibe Check")
A drone is a vibration machine.
1.  **Heat Shrink:** Every splice needs it. Use 3:1 ratio if possible.
2.  **Service Loops:** Leave 10% slack. A tight wire snaps in a crash.
3.  **Connector Maintenance:** XT30 pins get loose. Gently spread the male prongs with a knife to ensure a tight friction fit.

---

## 6. Soldering Tips for the Terrified

1.  **The Flux Pen:** Buy a "No-Clean Flux Pen." Draw on the pads before you solder. It makes the solder flow like water instead of sticking like gum.
2.  **The "Inline Splice" (The Window Cut):** For the I2C Y-Harness, don't cut the wire. Use your iron to melt a small ring of insulation mid-wire, then pick it off. Wrap the second wire around the exposed copper.
3.  **The "Tug Test":** After the joint cools (10 seconds), grab the wire and pull it. Hard. If it breaks, it was a "Cold Joint." Better it breaks now than in the sky.

---

## 7. Visual Guide
*(Sketch this out on paper before you start)*

```text
[ PI ZERO 2 ]                 [ FLIGHT CONTROLLER ]
   Pin 14 (TX)  ----twist----->  RX Pad
   Pin 15 (RX)  ----twist----->  TX Pad

   Pin 02 (5V)  <---twist------  BEC 5V Output
   Pin 06 (GND) <---twist------  BEC GND Output
                                      ^
                                      |
                                  [ BATTERY PADS ]
```

**Go slow. A pretty harness is a safe drone.**
