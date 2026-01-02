# The Mechanic's Handbook: Assembly & Mechanics
**"If it vibrates, it will loosen. If it spins, it will cut."**

Wiring is only half the battle. The physical assembly of a drone determines if it survives the first crash.

---

## 1. The Capacitor "L-Bend" Technique
Capacitors are tall. Frames are flat. To fit the capacitor inside the Pavo20 frame:

1.  **Strain Relief:** Grip the capacitor legs with needle-nose pliers about **2mm** away from the body.
2.  **The Bend:** Bend the legs 90 degrees *against the pliers*. **DO NOT** bend against the capacitor body (you will break the seal).
3.  **Result:** The capacitor now lays flat (horizontal), parallel to the Flight Controller, instead of standing up like a tower.
4.  **Insulate:** Ensure the exposed legs are covered in heat shrink so they don't touch the carbon fiber frame (Carbon conducts electricity!).

---

## 2. Blue Loctite (The "Vibe Killer")
**Rule:** Every metal screw going into a metal hole needs **Blue Threadlocker (Loctite 243)**.
*   **Why:** Motors vibrate at 20,000 RPM. Without Loctite, screws back out. Motors fall off.
*   **Where:** Motor screws, Standoff screws, Camera screws.
*   **WARNING:** **Never** use Loctite on plastic (like the Pavo20 frame hoops or nylon nuts). The chemical reaction melts plastic, causing it to crack instantly.

---

## 3. Bind BEFORE You Build
**The Trap:** The "Bind Button" on your receiver (ELRS/TBS) is tiny.
**The Reality:** Once you mount the Raspberry Pi on top of the Flight Controller, that button is buried under a sandwich of silicon.
**The Fix:**
1.  Power up the Flight Controller via USB.
2.  Bind your Radio.
3.  Verify the connection in Betaflight (Receiver Tab).
4.  *Then* assemble the drone.

---

## 4. Propeller Orientation (The "High Five")
Props are not identical. There are **CW (Clockwise)** and **CCW (Counter-Clockwise)** props.
*   **The Scoop:** Look at the blade. The "High Edge" is the leading edge. It scoops the air.
*   **The Logic:**
    *   If the motor spins CW, use a CW prop.
    *   If you mismatch them, the drone pushes air *up* and sucks itself into the floor.
*   **"Props Out" Configuration:**
    *   Most Cinewhoops (like Squid) run "Props Out."
    *   Front Left & Rear Right: Spin **Counter-Clockwise**.
    *   Front Right & Rear Left: Spin **Clockwise**.
    *   *Why?* It prevents the front props from throwing grass/dirt onto the camera lens.

---

## 5. The USB Problem
**The Trap:** The Pavo20 frame hoop often blocks the USB-C port on the Flight Controller.
**The Fix:**
*   **Option A:** Use a Right-Angle USB-C adapter.
*   **Option B (Pro):** Use a **Magnetic USB Cable**. Leave the magnetic "Tip" plugged into the Flight Controller permanently. This protects the port from dirt and wear.

---

## 6. Soft Mounting (The "Gummies")
**The Component:** Those little rubber rings that came with your Flight Controller.
**The Rule:** USE THEM.
*   **Why:** The Gyroscope is sensitive. Hard-mounting the board transfers frame resonance (noise) to the sensor.
*   **Result:** The drone thinks it's shaking when it isn't, causing "Hot Motors" and poor battery life.
*   **Assembly:** Screw -> Frame -> **Rubber Gummy** -> FC Board -> Nut.

---

## 7. The Smoke Stopper (Final Warning)
Before you plug in the battery for the first time after assembly:
1.  Inspect all solder joints with a magnifying glass. Look for "Solder Balls" (tiny splashes of lead).
2.  Shake the drone upside down.
3.  Use the **Smoke Stopper**.
